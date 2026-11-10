/*******************************************************************************
 * Copyright (c) 2026 Synecdoque
 *
 * The software is licensed under the MIT License. See the LICENSE file in this
 * repository for details.
 ***************************************************************************SDG*/

package main

import (
	"fmt"
	"log"
	"math"
	"time"
)

// mission decides what the chauffeur does, cycle by cycle, kept apart from I/O
// so it can be tested and simulated.
//
// Anything that makes the route unsafe to follow is a hold: velocity zero, keep
// sending, keep control. A hold ends only when a person says so, by starting
// again. Keeping control is why this is not a stop: a stop can be undone only
// with the pad.
//
// What the mission believes about control is only a belief, as in the gamer's
// pilot. Each change starts a new epoch, and a reply from an older one is
// ignored, so a command refused before a handover cannot end the next mission.

// Code is what the chauffeur is doing, and why. The values are the drivingState
// service's, so they are a contract with whoever reads it: add to them, do not
// renumber them.
type Code int

const (
	codeIdle    Code = 0 // nothing to do
	codeTaking  Code = 1 // asking the loader for control
	codeDriving Code = 2 // following the route
	codeArrived Code = 3 // reached the end of the route
	codeBraking Code = 4 // coming to rest before releasing control

	codeHoldObstacle     Code = 10 // something is too close ahead
	codeHoldBlind        Code = 11 // no clearance reading to go on
	codeHoldPose         Code = 12 // no recent pose
	codeHoldFrame        Code = 13 // the route and the pose are in different maps
	codeHoldRoute        Code = 14 // no route, or an old one
	codeHoldOffRoute     Code = 15 // too far from the route
	codeHoldSteering     Code = 16 // the loader refused a curvature
	codeHoldRouteChanged Code = 17 // the route now goes somewhere else, or much further

	codeStoppedTooClose Code = 20 // stopped the vehicle; the pad must take it back
	codeLostControl     Code = 21 // the loader refused: stopped, or someone else has control
)

func (c Code) String() string {
	switch c {
	case codeIdle:
		return "idle"
	case codeTaking:
		return "asking for control"
	case codeDriving:
		return "driving"
	case codeArrived:
		return "arrived"
	case codeBraking:
		return "coming to rest"
	case codeHoldObstacle:
		return "holding: something is too close ahead"
	case codeHoldBlind:
		return "holding: no clearance reading"
	case codeHoldPose:
		return "holding: no recent pose from the cartographer"
	case codeHoldFrame:
		return "holding: the route and the pose are in different maps"
	case codeHoldRoute:
		return "holding: no current route from the navigator"
	case codeHoldOffRoute:
		return "holding: too far from the route"
	case codeHoldSteering:
		return "holding: the loader refused a curvature"
	case codeHoldRouteChanged:
		return "holding: the route has changed"
	case codeStoppedTooClose:
		return "stopped: something came too close; take and release control with the pad"
	case codeLostControl:
		return "lost control: the vehicle is stopped or someone else is driving"
	}
	return "unknown"
}

func (c Code) holding() bool { return c >= codeHoldObstacle && c <= codeHoldRouteChanged }

// active is whether the chauffeur has, or is asking for, control.
func (c Code) active() bool {
	return c == codeTaking || c == codeDriving || c == codeBraking || c.holding()
}

// tuning is how the mission behaves, from the configuration.
type tuning struct {
	cruise, lookahead          float64
	holdClearance, stopClear   float64
	arrive, offRoute           float64
	maxPoseAge, maxRouteAge    time.Duration
	maxClearanceAge            time.Duration
	brakeFor                   time.Duration
	routeEndMoved, routeLonger float64

	// feedforward steers by the route's own curvature (see pursuit.go), and
	// otherwise by pure pursuit. maxArtAge is how old the articulation may be and
	// still be allowed for. logSteering logs each new pose's steering.
	feedforward bool
	ff          ffParams
	maxArtAge   time.Duration
	logSteering bool
}

// observation is everything known at the start of a cycle.
type observation struct {
	now time.Time

	havePose   bool
	x, y, th   float64
	poseFrame  string
	poseAt     time.Time
	route      *route
	haveClear  bool
	clearance  float64
	clearAt    time.Time
	curveLimit float64

	haveArt bool
	art     float64 // the measured articulation, radians, positive to the left
	artAt   time.Time
}

// action is what one cycle sends.
type action struct {
	take, release, stop bool
	drive               bool
	velocity, curvature float64
	epoch               int
}

// stopRepeats is how many cycles a stop is sent, so that one lost request does
// not matter.
const stopRepeats = 5

// settleAfterStart is how long after setting off the vehicle counts as not yet
// moving. Something within stopClear then is a reason to hold, and not one to
// stop the vehicle that has only just been told to go.
const settleAfterStart = time.Second

// routeGoneArrival is how near the end of the route the vehicle may be when the
// route goes away and still count it as arrived. The navigator drops a route
// when the vehicle is within its own arriveMetres, half a meter, of the goal.
const routeGoneArrival = 0.6

type mission struct {
	cfg   tuning
	code  Code
	epoch int

	// The route being followed. nil means the next one seen is accepted as it is,
	// which is what starting again means: a person has looked at it.
	route *route
	index int
	at    place // where on the route, last cycle

	started   time.Time
	curvature float64 // last sent; kept through a hold, so that the joint stays put

	brakeUntil time.Time
	after      Code // what the code becomes once control is released
	askedAt    time.Time
	stopsOwed  int

	artLost  bool      // feedforward is steering without the articulation, and has said so
	loggedAt time.Time // the pose last logged, with logSteering
}

// start is a person saying go: from rest, or on from a hold.
func (m *mission) start(now time.Time) {
	switch {
	case m.code.holding():
		log.Printf("chauffeur: told to go on, from %q", m.code)
		m.code, m.started, m.route, m.index = codeDriving, now, nil, 0
	case !m.code.active():
		m.code, m.askedAt, m.route, m.index = codeTaking, time.Time{}, nil, 0
		m.epoch++
	}
}

// cancel is a person saying that is enough. The vehicle comes to rest and control
// is released, so that it can be started again without the pad.
func (m *mission) cancel(now time.Time) {
	if m.code.active() && m.code != codeBraking {
		m.brake(now, codeIdle)
	}
}

func (m *mission) brake(now time.Time, after Code) {
	m.code, m.after, m.brakeUntil, m.askedAt = codeBraking, after, now.Add(m.cfg.brakeFor), time.Time{}
}

func (m *mission) hold(c Code) {
	m.code = c
}

// granted is the loader's answer to a take made in epoch e.
func (m *mission) granted(e int, now time.Time) {
	if e != m.epoch || m.code != codeTaking {
		return
	}
	m.code, m.started = codeDriving, now
	m.epoch++
}

// released is the loader's confirmation of a release made in epoch e.
func (m *mission) released(e int) {
	if e == m.epoch && m.code == codeBraking {
		m.finish()
	}
}

func (m *mission) finish() {
	m.code = m.after
	m.epoch++
}

// refused is the loader's 409: someone stopped the vehicle or has control. It is
// never retried, because a stop stays a stop until a person undoes it.
func (m *mission) refused(e int, reason string) {
	if e != m.epoch || !m.code.active() {
		return
	}
	// Releasing, and refused: a command already on its way overtook the release,
	// or nobody has control any more. Either way this mission is over.
	if m.code == codeBraking && !m.askedAt.IsZero() {
		m.finish()
		return
	}
	log.Printf("chauffeur: control lost: %s", reason)
	m.code = codeLostControl
	m.epoch++
}

// steeringRefused is a curvature the loader cannot follow: a 503, usually a waist
// that is not calibrated. It says nothing about who has control.
func (m *mission) steeringRefused(e int, reason string) {
	if e == m.epoch && m.code == codeDriving {
		log.Printf("chauffeur: the loader refused a curvature: %s", reason)
		m.hold(codeHoldSteering)
	}
}

// step is one cycle.
func (m *mission) step(o observation) action {
	a := action{}
	switch m.code {
	case codeTaking:
		if o.now.Sub(m.askedAt) >= time.Second {
			a.take, m.askedAt = true, o.now
		}
	case codeDriving:
		m.driveOn(o, &a)
	case codeBraking:
		switch {
		case o.now.Before(m.brakeUntil):
			m.rest(&a)
		case m.askedAt.IsZero():
			a.release, m.askedAt = true, o.now
		case o.now.Sub(m.askedAt) > 2*time.Second:
			log.Println("chauffeur: the loader did not confirm the release")
			m.finish()
		case o.now.Sub(m.askedAt)%(250*time.Millisecond) < 50*time.Millisecond:
			a.release = true
		}
	default:
		if m.code.holding() {
			m.rest(&a)
		}
	}
	if m.stopsOwed > 0 {
		m.stopsOwed--
		a.stop = true
	}
	a.epoch = m.epoch
	return a
}

// rest keeps the vehicle where it is: it goes on hearing from its pilot, and the
// joint stays as it was rather than swinging straight under a stopped vehicle.
func (m *mission) rest(a *action) {
	a.drive, a.velocity, a.curvature = true, 0, m.curvature
}

func (m *mission) driveOn(o observation, a *action) {
	// What is in front of the vehicle comes first.
	switch {
	case !o.haveClear || o.now.Sub(o.clearAt) > m.cfg.maxClearanceAge:
		m.hold(codeHoldBlind)
	case o.clearance < m.cfg.stopClear && o.now.Sub(m.started) >= settleAfterStart:
		log.Printf("chauffeur: STOP: something at %.2f m", o.clearance)
		m.code, m.stopsOwed = codeStoppedTooClose, stopRepeats
		m.epoch++
		return
	case o.clearance < m.cfg.holdClearance:
		log.Printf("chauffeur: something at %.2f m", o.clearance)
		m.hold(codeHoldObstacle)
	case !o.havePose || o.now.Sub(o.poseAt) > m.cfg.maxPoseAge:
		m.hold(codeHoldPose)
	case o.route == nil:
		// The navigator drops a route once the vehicle is near enough for it, which
		// may be before this mission thinks so.
		if m.route != nil && m.at.remaining <= routeGoneArrival {
			m.brake(o.now, codeArrived)
			m.rest(a)
			return
		}
		m.hold(codeHoldRoute)
	case o.now.Sub(o.route.stamp) > m.cfg.maxRouteAge:
		m.hold(codeHoldRoute)
	case o.route.frame != o.poseFrame:
		m.hold(codeHoldFrame)
	case o.route != m.route:
		if m.route != nil && m.changedTooMuch(o.route) {
			m.hold(codeHoldRouteChanged)
			break
		}
		m.route, m.index = o.route, 0
	}
	if m.code != codeDriving {
		m.rest(a)
		return
	}

	m.at = m.route.locate(m.index, o.x, o.y)
	m.index = m.at.index
	if m.at.offset > m.cfg.offRoute {
		log.Printf("chauffeur: %.2f m from the route", m.at.offset)
		m.hold(codeHoldOffRoute)
		m.rest(a)
		return
	}
	// The navigator's routes end within a quarter of a meter of the goal, and it
	// checks for arrival only when it replans. So the end of the route is judged
	// here, and the vehicle does not depend on being told.
	if m.at.remaining <= m.cfg.arrive {
		m.brake(o.now, codeArrived)
		m.rest(a)
		return
	}
	m.curvature = m.steer(o)
	a.drive, a.velocity, a.curvature = true, m.cfg.cruise, m.curvature
}

// steer is the curvature for this cycle, by feedforward or by pure pursuit.
func (m *mission) steer(o observation) float64 {
	haveArt := o.haveArt && o.now.Sub(o.artAt) <= m.cfg.maxArtAge
	var s steer
	if m.cfg.feedforward {
		switch {
		case !haveArt && !m.artLost:
			log.Println("chauffeur: no recent articulation from the loader; steering without allowing for the waist")
			m.artLost = true
		case haveArt && m.artLost:
			log.Println("chauffeur: the articulation is back")
			m.artLost = false
		}
		s = m.route.pursueFeedforward(m.at, o.x, o.y, o.th, o.art, haveArt, m.cfg.ff, o.curveLimit)
	} else {
		s.curvature = m.route.pursue(m.at, o.x, o.y, o.th, m.cfg.lookahead, o.curveLimit)
		s.offset, s.heading = m.route.errorsAt(m.at, o.x, o.y, o.th)
		s.planned = m.route.curvatureOver(m.at.along, m.cfg.ff.window)
	}
	if m.cfg.logSteering && o.poseAt.After(m.loggedAt) {
		m.loggedAt = o.poseAt
		gamma := "-"
		if haveArt {
			gamma = fmt.Sprintf("%+.1f°", o.art*180/math.Pi)
		}
		log.Printf("chauffeur: steer at %.2f m: off %+.3f m, heading %+.1f°, route %+.3f, sent %+.3f /m, waist %s",
			m.at.along, s.offset, s.heading*180/math.Pi, s.planned, s.curvature, gamma)
	}
	return s.curvature
}

// changedTooMuch is whether a replanned route is one the person did not look at:
// it ends somewhere else, or is much further than what was left of the old one.
// Replans that only tidy the same route up go through.
func (m *mission) changedTooMuch(next *route) bool {
	ox, oy := m.route.end()
	nx, ny := next.end()
	return math.Hypot(nx-ox, ny-oy) > m.cfg.routeEndMoved || next.length() > m.at.remaining+m.cfg.routeLonger
}
