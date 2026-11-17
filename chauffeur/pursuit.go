/*******************************************************************************
 * Copyright (c) 2026 Synecdoque
 *
 * The software is licensed under the MIT License. See the LICENSE file in this
 * repository for details.
 ***************************************************************************SDG*/

package main

import (
	"fmt"
	"math"
	"sort"
	"time"

	"github.com/sdoque/mbaigo/forms"
)

// Following a route by pure pursuit: aim at a point a fixed distance further
// along it, and drive the circle that passes through that point.
//
// It suits this vehicle because each half of an articulated loader is a rigid
// body on an axle that does not slide sideways. The front axle's center moves
// along the way the front half faces, on the curvature the loader is told, and
// that is all pure pursuit assumes.

// searchWindow is how far along the route, in meters, the vehicle may have moved
// since it was last placed on it. The corridors loop, as a B or an 8, so a route
// can pass close to itself, and the nearest point overall could be a stretch the
// vehicle is not on.
const searchWindow = 5.0

// route is the navigator's path, with the distance along it worked out.
type route struct {
	x, y, along []float64
	frame       string
	stamp       time.Time
}

// newRoute checks a path and makes a route of it.
func newRoute(p *forms.PathA_v1a) (*route, error) {
	if len(p.X) == 0 || len(p.X) != len(p.Y) {
		return nil, fmt.Errorf("a path of %d x and %d y", len(p.X), len(p.Y))
	}
	if p.DistanceUnit != "" && p.DistanceUnit != unitMetre {
		return nil, fmt.Errorf("a path in %s; it must be in meters", p.DistanceUnit)
	}
	r := &route{x: p.X, y: p.Y, along: make([]float64, len(p.X)), frame: p.Frame, stamp: p.Timestamp}
	for i := 1; i < len(r.x); i++ {
		r.along[i] = r.along[i-1] + math.Hypot(r.x[i]-r.x[i-1], r.y[i]-r.y[i-1])
	}
	return r, nil
}

func (r *route) length() float64 { return r.along[len(r.along)-1] }

func (r *route) end() (float64, float64) { return r.x[len(r.x)-1], r.y[len(r.y)-1] }

// place is where on the route the vehicle is.
type place struct {
	index     int     // the segment it is on, where the next search starts
	along     float64 // meters along the route
	offset    float64 // meters from the route
	remaining float64 // meters along the route to its end
}

// locate finds the nearest point of the route to (x, y), searching forward from
// segment from, no further than searchWindow.
func (r *route) locate(from int, x, y float64) place {
	last := len(r.x) - 1
	if from > last {
		from = last
	}
	best := place{index: from, along: r.along[from], offset: math.Hypot(x-r.x[from], y-r.y[from])}
	for i := from; i < last && r.along[i]-r.along[from] <= searchWindow; i++ {
		sx, sy := r.x[i+1]-r.x[i], r.y[i+1]-r.y[i]
		seg := sx*sx + sy*sy
		if seg == 0 {
			continue
		}
		t := math.Max(0, math.Min(1, ((x-r.x[i])*sx+(y-r.y[i])*sy)/seg))
		if d := math.Hypot(x-(r.x[i]+t*sx), y-(r.y[i]+t*sy)); d < best.offset {
			best = place{index: i, along: r.along[i] + t*math.Sqrt(seg), offset: d}
		}
	}
	best.remaining = r.length() - best.along
	return best
}

// pointAt is the point a distance s along the route. Past the end it carries on
// in the direction of the last segment, so the aim point stays a full lookahead
// away and the steering does not swing as the vehicle closes on the end.
func (r *route) pointAt(s float64) (float64, float64) {
	last := len(r.x) - 1
	if last == 0 {
		return r.x[0], r.y[0]
	}
	i := 0
	for i < last-1 && r.along[i+1] < s {
		i++
	}
	seg := r.along[i+1] - r.along[i]
	if seg == 0 {
		return r.x[i], r.y[i]
	}
	t := (s - r.along[i]) / seg
	if t < 0 {
		t = 0
	}
	return r.x[i] + t*(r.x[i+1]-r.x[i]), r.y[i] + t*(r.y[i+1]-r.y[i])
}

// pursue is the curvature that takes a vehicle at (x, y) facing th through the
// point lookahead beyond its place on the route, no tighter than limit. Positive
// is to the left (ISO 8855), as the loader takes it.
func (r *route) pursue(at place, x, y, th, lookahead, limit float64) float64 {
	tx, ty := r.pointAt(at.along + lookahead)
	dx, dy := tx-x, ty-y
	dist2 := dx*dx + dy*dy
	if dist2 < 1e-6 {
		return 0
	}
	sideways := -math.Sin(th)*dx + math.Cos(th)*dy
	return math.Max(-limit, math.Min(limit, 2*sideways/dist2))
}

//-------------------------------------Feedforward

// Following a route by feedforward: steer the curvature the route itself has
// just ahead, and correct only gently for being off it.
//
// Pure pursuit reacts to geometry alone, and on an articulated loader that is
// a trap: steering is bending the joint, and bending the joint turns the front
// half, where the scanner and the steering's reference point are. The loader's
// kinematics give the front half's turn rate as
//
//	(v·sin γ + L2·γ̇) / (L1·cos γ + L2)
//
// and the second term is that turn: about half the waist's own movement, at any
// speed. Pursuit reads it as a heading error and steers back, the swing back
// turns the front the other way, and it overshoots. So the heading error is
// judged here as it will be once the waist has reached the angle the route
// asks for: the part of the front's heading that is only the waist on its way
// there is not an error.

// ffParams is how feedforward steers.
type ffParams struct {
	lead       float64 // meters ahead of the vehicle the route's curvature is read
	window     float64 // meters of route it is averaged over
	correction float64 // meters: the feedback is a pursuit this far ahead, gentle when long
	l1, l2     float64 // meters from the joint to the front and the rear axle
}

// steer is one cycle's steering, and what it was worked out from.
type steer struct {
	curvature float64 // what is sent
	planned   float64 // the route's curvature ahead
	offset    float64 // meters left of the route, negative to the right
	heading   float64 // radians the front half points left of the route, as judged
}

// segAt is the segment that s meters along the route lies on.
func (r *route) segAt(s float64) int {
	last := len(r.x) - 1
	i := sort.Search(len(r.along), func(k int) bool { return r.along[k] > s }) - 1
	return max(0, min(i, last-1))
}

// segHeading is the direction of segment i, skipping any of no length.
func (r *route) segHeading(i int) float64 {
	last := len(r.x) - 1
	for i < last-1 && r.along[i+1] == r.along[i] {
		i++
	}
	if i < 0 || i >= last {
		return 0
	}
	return math.Atan2(r.y[i+1]-r.y[i], r.x[i+1]-r.x[i])
}

// curvatureOver is the route's mean curvature from s to s+w meters along it: how
// far it turns, divided by how far it goes. The turn is measured between the
// middles of the segments at either end, which is exact on the navigator's
// arcs whatever w is. Past the end the route goes straight on, as pointAt has it.
func (r *route) curvatureOver(s, w float64) float64 {
	last := len(r.x) - 1
	if last < 2 || s >= r.length() {
		return 0
	}
	i, j := r.segAt(s), r.segAt(math.Min(s+w, r.length()))
	if j == i {
		if i+1 > last-1 {
			return 0
		}
		j = i + 1
	}
	mid := func(k int) float64 { return (r.along[k] + r.along[k+1]) / 2 }
	d := mid(j) - mid(i)
	if d <= 0 {
		return 0
	}
	return wrapAngle(r.segHeading(j)-r.segHeading(i)) / d
}

// headingAt is the route's direction s meters along it. A segment's own
// direction is the arc's only at its middle, and on the tightest arc it is 1.7°
// out at either end; so it is interpolated between the middles of segments,
// which is exact on the navigator's arcs. Otherwise the correction would jump
// every 15 cm, by enough to move the waist.
func (r *route) headingAt(s float64) float64 {
	last := len(r.x) - 1
	if last < 1 {
		return 0
	}
	i := r.segAt(s)
	mid := func(k int) float64 { return (r.along[k] + r.along[k+1]) / 2 }
	j := i + 1
	if s < mid(i) {
		i, j = i-1, i
	}
	if i < 0 || j > last-1 || mid(j) <= mid(i) {
		return r.segHeading(r.segAt(s))
	}
	f := math.Max(0, math.Min(1, (s-mid(i))/(mid(j)-mid(i))))
	hi := r.segHeading(i)
	return hi + f*wrapAngle(r.segHeading(j)-hi)
}

// errorsAt is how far left of the route a vehicle at (x, y) facing th is, and
// how far left of the route's direction it points, at its place on the route.
func (r *route) errorsAt(at place, x, y, th float64) (offset, heading float64) {
	if len(r.x) < 2 {
		return 0, 0
	}
	px, py := r.pointAt(at.along)
	h := r.headingAt(at.along)
	return -math.Sin(h)*(x-px) + math.Cos(h)*(y-py), wrapAngle(th - h)
}

// pursueFeedforward is the curvature that holds a vehicle on the route: the
// route's own curvature ahead, and a gentle correction for being off it. art is
// the measured articulation in radians, positive to the left; without it
// (haveArt false) the waist's turn of the front half is not allowed for.
func (r *route) pursueFeedforward(at place, x, y, th, art float64, haveArt bool, p ffParams, limit float64) steer {
	if len(r.x) < 2 {
		return steer{}
	}
	s := steer{planned: r.curvatureOver(at.along+p.lead, p.window)}
	s.offset, s.heading = r.errorsAt(at, x, y, th)
	if haveArt {
		s.heading -= p.l2 / (p.l1*math.Cos(art) + p.l2) * (art - articulationFor(s.planned, p.l1, p.l2))
	}
	L := p.correction
	k := s.planned - 2*s.offset/(L*L) - 2*s.heading/L
	s.curvature = math.Max(-limit, math.Min(limit, k))
	return s
}

// maxArticulation bounds the search in articulationFor. Beyond it the joint does
// not go, and the loader limits it more tightly still.
const maxArticulation = 60 * math.Pi / 180

// articulationFor is the joint angle, in radians, that puts the front axle on a
// path of curvature kappa: the inverse of sin γ / (L1·cos γ + L2), by bisection.
// A copy of the loader's Geometry.articulationFor (loader/kinematics.go); the
// loader is a module of its own, so it is not imported.
func articulationFor(kappa, l1, l2 float64) float64 {
	front := func(g float64) float64 { return math.Sin(g) / (l1*math.Cos(g) + l2) }
	if kappa >= front(maxArticulation) {
		return maxArticulation
	}
	if kappa <= front(-maxArticulation) {
		return -maxArticulation
	}
	lo, hi := -maxArticulation, maxArticulation
	for i := 0; i < 60; i++ {
		mid := (lo + hi) / 2
		if front(mid) < kappa {
			lo = mid
		} else {
			hi = mid
		}
	}
	return (lo + hi) / 2
}

func wrapAngle(a float64) float64 {
	return math.Remainder(a, 2*math.Pi)
}
