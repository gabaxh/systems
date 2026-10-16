/*******************************************************************************
 * Copyright (c) 2026 Synecdoque
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, subject to the following conditions:
 *
 * The software is licensed under the MIT License. See the LICENSE file in this
 * repository for details.
 *
 * Contributors:
 *   Jan A. van Deventer, Luleå - initial implementation
 ***************************************************************************SDG*/

package main

import (
	"math"
	"testing"
	"time"
)

// A waist as the angle loop sees it: a self-locking motor that does nothing
// until its effort passes the breakaway, keeps turning on less once it moves,
// and coasts when let go; a chain with slack between it and the joint; and a
// sensor that reads whole counts with a count of noise.
//
// Until 8 October 2026 the simulated motor broke away and kept turning at the
// same effort, and nothing but the effort moved it. The angle loop settled on
// that motor and hunted on the vehicle, standing on blocks with the stick
// centred.
type simWaist struct {
	cfg WaistConfig

	breakaway           float64 // percent of effort that starts a motor at rest
	kinetic             float64 // percent of effort that a turning motor needs to keep turning
	degPerSecPerPercent float64
	tau                 float64 // s, how long the motor and the joint coast
	slack               float64 // degrees of motor travel before the chain bites
	// centring is the tyres' pull back towards straight while rolling, as the
	// share of the articulation a second it would undo. The gearbox is
	// self-locking, so it moves the joint only within the chain's slack.
	centring float64

	motor, speed, joint float64 // degrees, degrees a second, degrees
	turning             bool
	noise               uint32
}

// stopSpeed is how slowly the simulated motor turns before it sticks.
const stopSpeed = 0.05

func (w *simWaist) advance(effortPercent, dt float64) {
	a := math.Abs(effortPercent)
	if !w.turning && a > w.breakaway {
		w.turning = true
	}
	if w.turning {
		drive := 0.0
		if a > w.kinetic {
			drive = math.Copysign((a-w.kinetic)*w.degPerSecPerPercent, effortPercent)
		}
		w.speed += (drive - w.speed) * dt / w.tau
		w.motor += w.speed * dt
		if drive == 0 && math.Abs(w.speed) < stopSpeed {
			w.turning, w.speed = false, 0
		}
	}
	w.joint -= w.joint * w.centring * dt
	switch {
	case w.motor-w.joint > w.slack/2:
		w.joint = w.motor - w.slack/2
	case w.joint-w.motor > w.slack/2:
		w.joint = w.motor + w.slack/2
	}
}

// at puts the motor and the joint at deg degrees, at rest, the chain taut on
// the side it would have reached deg from straight.
func (w *simWaist) at(deg float64) *simWaist {
	w.joint = deg
	w.motor = deg + math.Copysign(w.slack/2, deg)
	return w
}

// raw is what the sensor reports for the joint now, with a count of noise
// either way now and then and two counts more rarely, as measured on the
// vehicle.
func (w *simWaist) raw() int {
	w.noise = w.noise*1664525 + 1013904223
	n := 0
	switch w.noise >> 28 {
	case 0:
		n = -2
	case 1, 2:
		n = -1
	case 13, 14:
		n = 1
	case 15:
		n = 2
	}
	return w.cfg.StraightCount + int(math.Round(w.joint*w.cfg.countsPerDegree())) + n
}

// artitraxWaist is the waist configuration on the vehicle on 1 October 2026,
// with the angle loop's own defaults in place of its P loop, and the breakaway
// measured then.
func artitraxWaist() WaistConfig {
	w := defaultWaist()
	w.StraightCount, w.CalibrationCount, w.CalibrationDegrees = 471, 951, 57.91
	w.LimitDegrees, w.LimitMarginDegrees = 50, 2
	w.EffortTurnsLeft = -1
	w.RampPercentPerSecond = 100
	w.BreakawayPercent = 22
	applyWaistDefaults(&w)
	return w
}

// steerTo drives the simulated waist towards target degrees by curvature, at
// 50 Hz with the loader's own steering path and ramp, and returns the joint's
// angle every cycle.
func steerTo(t *testing.T, w *simWaist, target float64, seconds float64) []float64 {
	t.Helper()
	trace, _ := steerThrough(t, w, func(float64) float64 { return target }, seconds)
	return trace
}

// steerThrough is steerTo with a target that changes: target(s) is the target
// s seconds in. It returns the effort applied every cycle as well, in percent.
func steerThrough(t *testing.T, w *simWaist, target func(float64) float64, seconds float64) (trace, efforts []float64) {
	t.Helper()
	d, _, _ := drivingDrivetrain(t, w.cfg)
	steer, _ := d.steeringMotor()
	d.byAngle = true
	step := d.cfg.Waist.rampStep(d.cfg.CommandHz)
	now := time.Unix(1_000_000, 0)
	for c := 0; c < int(seconds*50); c++ {
		d.curvature = d.cfg.Geometry.frontCurvature(target(float64(c)/50) * math.Pi / 180)
		raw := w.raw()
		d.followJointLocked(raw, now, true)
		desired := d.steeringRawLocked(steer, raw, now, true, now)
		d.last[steer.NodeID] = steerLimit(d.last[steer.NodeID], desired, step)
		effort := float64(d.last[steer.NodeID]) / fullScale * 100 * d.waist.rawEffortSign()
		for ms := 0; ms < 20; ms++ {
			w.advance(effort, 0.001)
		}
		now = now.Add(20 * time.Millisecond)
		trace = append(trace, w.joint)
		efforts = append(efforts, effort)
	}
	return trace, efforts
}

// pushes counts the times the effort goes from nothing to something: each one
// is the motor taking up the chain's slack and breaking away again.
func pushes(efforts []float64, from int) int {
	n := 0
	for i := from; i < len(efforts); i++ {
		before := 0.0
		if i > 0 {
			before = efforts[i-1]
		}
		if before == 0 && efforts[i] != 0 {
			n++
		}
	}
	return n
}

// newSimWaist is a waist rolling on the ground: the tyres pull the joint back
// towards straight, as far as the chain's slack lets them. The breakaway is the
// 22% measured on the vehicle on 1 October 2026; the rest is a guess at its
// numbers.
func newSimWaist(cfg WaistConfig) *simWaist {
	w := onBlocks(cfg)
	w.centring = 0.5
	return w
}

// onBlocks is the waist with the wheels in the air: nothing but the motor
// moves the joint.
func onBlocks(cfg WaistConfig) *simWaist {
	return &simWaist{cfg: cfg, breakaway: 22, kinetic: 14, degPerSecPerPercent: 1.0, tau: 0.1, slack: 3, noise: 1}
}

// swing is how far the joint wanders either side of target over the trace's
// last part, and how many times it crosses the target there.
func swing(trace []float64, target float64, from int) (worst float64, crossings int) {
	for i := from; i < len(trace); i++ {
		worst = math.Max(worst, math.Abs(trace[i]-target))
		if i > from && (trace[i-1]-target)*(trace[i]-target) < 0 {
			crossings++
		}
	}
	return worst, crossings
}

// The waist as configured on the vehicle settles on its target, and stays
// there, rather than swinging about it.
func TestTheWaistSettles(t *testing.T) {
	for _, target := range []float64{20, -20, 5, 0} {
		w := newSimWaist(artitraxWaist())
		trace := steerTo(t, w, target, 12)
		worst, crossings := swing(trace, target, 6*50)
		t.Logf("target %v°: over the last 6 s within %.2f° of it, crossing it %d times", target, worst, crossings)
		if worst > 1 || crossings > 2 {
			t.Errorf("target %v°: over the last 6 s the joint wandered %.2f° from it and crossed it %d times; "+
				"want within 1° and settled", target, worst, crossings)
		}
	}
}

// A reversal from 20° left to 20° right settles as well, without swinging
// about the new target.
func TestTheWaistSettlesAfterAReversal(t *testing.T) {
	w := newSimWaist(artitraxWaist())
	trace, _ := steerThrough(t, w, func(s float64) float64 {
		if s < 6 {
			return 20
		}
		return -20
	}, 18)
	worst, crossings := swing(trace, -20, 12*50)
	t.Logf("20° to -20°: over the last 6 s within %.2f° of it, crossing it %d times", worst, crossings)
	if worst > 1 || crossings > 2 {
		t.Errorf("after the reversal the joint wandered %.2f° from -20° and crossed it %d times; want within 1° and settled",
			worst, crossings)
	}
}

// Full stick, from straight, either way: the effort sits at the ceiling for
// most of the way, and the joint stops at the target the loop aims for, inside
// the limit, rather than being carried through it.
func TestFullLockStaysInsideTheLimit(t *testing.T) {
	cfg := artitraxWaist()
	lock := cfg.targetLimitDegrees()
	for _, side := range []float64{1, -1} {
		w := newSimWaist(cfg)
		trace := steerTo(t, w, side*(cfg.LimitDegrees+10), 12)
		furthest := 0.0
		for _, deg := range trace {
			furthest = math.Max(furthest, side*deg)
		}
		worst, crossings := swing(trace, side*lock, 6*50)
		t.Logf("full lock to the %s: furthest %.2f°, then within %.2f° of %.0f°, crossing it %d times",
			sideName(int(side)), furthest, worst, side*lock, crossings)
		if furthest >= cfg.LimitDegrees {
			t.Errorf("full lock to the %s reached %.2f°, past the %.0f° limit", sideName(int(side)), furthest, cfg.LimitDegrees)
		}
		if worst > 1 || crossings > 2 {
			t.Errorf("full lock to the %s: within %.2f° of %.0f°, crossing %d times; want within 1° and settled",
				sideName(int(side)), worst, side*lock, crossings)
		}
	}
}

// The integral finds the breakaway the P loop needed a measured minimum
// effort for: 22%, as on the vehicle on 1 October 2026, is more than the gain
// asks for a few degrees from the target.
func TestTheIntegratorFindsTheBreakaway(t *testing.T) {
	w := newSimWaist(artitraxWaist())
	w.breakaway = 22
	trace := steerTo(t, w, 5, 8)
	arrived := -1
	for i, deg := range trace {
		if math.Abs(deg-5) < 1 {
			arrived = i
			break
		}
	}
	over := 0.0
	for _, deg := range trace {
		over = math.Max(over, deg-5)
	}
	worst, crossings := swing(trace, 5, 4*50)
	t.Logf("5° against a 22%% breakaway: within 1° after %.2f s, overshoot %.2f°; over the last 4 s within %.2f°, crossing %d times",
		float64(arrived)/50, over, worst, crossings)
	if arrived < 0 || arrived > 3*50 {
		t.Errorf("5° against a 22%% breakaway: within 1° of it after %.2f s; want within 3 s", float64(arrived)/50)
	}
	if over > 1 || worst > 1 || crossings > 2 {
		t.Errorf("overshoot %.2f°, then within %.2f° crossing %d times; want under 1°, within 1°, settled", over, worst, crossings)
	}
}

// On blocks with the stick centred, a joint left a little off straight is
// brought back once and then left alone. On 8 October 2026 it was not: the
// integral wound up to the breakaway, the joint lurched past straight on the
// lower effort a turning motor needs, and the integral, still loaded, carried
// it on; then the same the other way, across the chain's slack each time.
func TestOnBlocksTheWaistStaysPut(t *testing.T) {
	for _, start := range []float64{0.8, 1.5, 3, -2} {
		w := onBlocks(artitraxWaist()).at(start)
		trace, efforts := steerThrough(t, w, func(float64) float64 { return 0 }, 30)
		worst, crossings := swing(trace, 0, 10*50)
		late := pushes(efforts, 10*50)
		t.Logf("from %v°: %d pushes in all, %d after 10 s; over the last 20 s within %.2f° of straight, crossing it %d times",
			start, pushes(efforts, 0), late, worst, crossings)
		if late > 0 || crossings > 0 {
			t.Errorf("from %v°: after 10 s the motor still pushed %d times and the joint crossed straight %d times; "+
				"want it left alone", start, late, crossings)
		}
		if worst > w.cfg.HoldDegrees+0.25 {
			t.Errorf("from %v°: the joint rests %.2f° off straight; want within holdDegrees, %v°", start, worst, w.cfg.HoldDegrees)
		}
	}
}

// A target that wanders by less than holdDegrees, as a stick near its centre
// does, does not move the joint at all.
func TestASmallWanderingTargetMovesNothing(t *testing.T) {
	w := onBlocks(artitraxWaist()).at(10)
	_, efforts := steerThrough(t, w, func(s float64) float64 { return 10 + 0.6*math.Sin(2*math.Pi*s/1.3) }, 15)
	if n := pushes(efforts, 0); n > 0 || efforts[0] != 0 {
		t.Errorf("a target wandering ±0.6° about the joint's angle pushed the motor %d times; want none", n)
	}
}

// Small steps start at once: the loop does not wait for its integral to wind
// up to the breakaway, which took seconds for a degree or two. A step back the
// way the joint came crosses the chain's slack first, 3° of the motor at about
// 8° a second, and is given that long. A move may stop short and be finished
// by a second push the same way, never by one back.
func TestSmallStepsStartAtOnce(t *testing.T) {
	for _, c := range []struct {
		step, within float64
	}{{1.5, 0.5}, {2, 0.5}, {5, 0.5}, {-2, 0.9}} {
		w := onBlocks(artitraxWaist()).at(0.001) // the chain taut to the left
		trace, efforts := steerThrough(t, w, func(float64) float64 { return c.step }, 8)
		started := -1
		for i, deg := range trace {
			if math.Abs(deg) > 0.2 {
				started = i
				break
			}
		}
		over := 0.0
		for _, deg := range trace {
			over = math.Max(over, math.Copysign(1, c.step)*(deg-c.step))
		}
		worst, crossings := swing(trace, c.step, 4*50)
		t.Logf("step of %v°: the joint moved after %.2f s, overshoot %.2f°, %d pushes; over the last 4 s within %.2f°, crossing %d times",
			c.step, float64(started)/50, over, pushes(efforts, 0), worst, crossings)
		if started < 0 || float64(started)/50 > c.within {
			t.Errorf("step of %v°: the joint moved after %.2f s; want within %v s", c.step, float64(started)/50, c.within)
		}
		back := 0
		for _, e := range efforts {
			if e*c.step < 0 {
				back++
			}
		}
		if over > 0.5 || back > 0 || pushes(efforts, 0) > 2 || worst > w.cfg.HoldDegrees {
			t.Errorf("step of %v°: overshoot %.2f°, %d pushes, %d cycles pushing back, resting %.2f° off; "+
				"want under 0.5°, at most a second push the same way, none back, resting within holdDegrees",
				c.step, over, pushes(efforts, 0), back, worst)
		}
	}
}
