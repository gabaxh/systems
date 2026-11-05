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
