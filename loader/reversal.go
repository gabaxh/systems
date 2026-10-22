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
	"fmt"
	"strings"
	"time"
)

// Where the time goes when the steering is reversed.
//
// On 22 October 2026, standing on the ground, the waist took two to three
// seconds to answer the pad's stick going from one side to the other. The
// command crosses the network well inside the helm's silence limit, and the
// ramps are a tenth of a second, so the time is spent in one of three places:
// the angle loop not yet pushing, the wheels holding the joint against the
// waist motor, or the motor and the chain themselves. The loader cannot tell
// them apart from the log as it was. Each reversal now ends with one line that
// says when the effort went out the new way, what held it until then, when the
// joint started to move, and what the wheels were doing meanwhile.

// reversalTimeout is how long a reversal is followed before the joint is
// reported as not having moved.
const reversalTimeout = 5 * time.Second

// reversalCycle is one command cycle as the reversal trace sees it.
type reversalCycle struct {
	now    time.Time
	demand int     // the side the pilot asks for: 1 left, -1 right, 0 none
	effort float64 // the steering effort sent, percent, positive to the left
	held   string  // why no effort was asked of the motor this cycle, if none was

	raw      int  // the waist count
	fresh    bool // and whether it is a reading to go by
	leftward int  // the direction the count moves as the joint turns left
	counts   int  // how far the count must move for the joint to be moving

	wheelPercent float64 // the largest wheel command sent, percent of full scale
	wheels       string  // what the wheels are doing: the standstill's mode, or open loop
}

// reversalTrace follows one reversal from the stick to the joint.
type reversalTrace struct {
	lastSide int // the last side asked for, kept through the stick's centre

	active   bool
	side     int
	since    time.Time
	from     int // the waist count when it began
	haveFrom bool
	effortAt time.Time
	held     string
	wheelMax float64
	wheels   []string // the wheels' modes, in the order they were seen
}

// observe moves the trace on by a cycle, and returns a line for the log when a
// reversal has been followed to its end.
func (r *reversalTrace) observe(c reversalCycle) string {
	var line string
	if c.demand != 0 && c.demand != r.lastSide {
		if r.active {
			line = r.report(c.now, "reversed again before the joint moved")
		}
		if r.lastSide != 0 {
			*r = reversalTrace{active: true, side: c.demand, since: c.now}
		}
		r.lastSide = c.demand
	}
	if !r.active {
		return line
	}

	if c.fresh && !r.haveFrom {
		r.from, r.haveFrom = c.raw, true
	}
	if r.effortAt.IsZero() {
		if sign(c.effort) == r.side {
			r.effortAt = c.now
		} else if c.held != "" {
			r.held = c.held
		}
	}
	if c.wheelPercent > r.wheelMax {
		r.wheelMax = c.wheelPercent
	}
	if c.wheels != "" && (len(r.wheels) == 0 || r.wheels[len(r.wheels)-1] != c.wheels) {
		r.wheels = append(r.wheels, c.wheels)
	}

	switch {
	case c.fresh && r.haveFrom && (c.raw-r.from)*c.leftward*r.side >= c.counts:
		return r.report(c.now, fmt.Sprintf("joint moving after %d ms", c.now.Sub(r.since).Milliseconds()))
	case c.now.Sub(r.since) >= reversalTimeout:
		return r.report(c.now, fmt.Sprintf("joint did not move within %d ms", reversalTimeout.Milliseconds()))
	}
	return line
}

// report is the line for the reversal followed so far, ending with outcome,
// and ends the trace.
func (r *reversalTrace) report(now time.Time, outcome string) string {
	effort := "no effort the new way"
	if !r.effortAt.IsZero() {
		effort = fmt.Sprintf("effort after %d ms", r.effortAt.Sub(r.since).Milliseconds())
	}
	if r.held != "" {
		effort += " (held: " + r.held + ")"
	}
	wheels := "wheels " + strings.Join(r.wheels, " then ")
	if len(r.wheels) == 0 {
		wheels = "wheels not commanded"
	}
	r.active = false
	return fmt.Sprintf("steering reversal to the %s: %s, %s; %s, up to %.0f%% effort",
		sideName(r.side), effort, outcome, wheels, r.wheelMax)
}

func (m standMode) String() string {
	switch m {
	case holding:
		return "holding"
	case resting:
		return "resting"
	}
	return "driving"
}
