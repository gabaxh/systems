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

// A waist as the angle loop sees it: a motor that does nothing below its
// breakaway and coasts when let go, a chain with slack between it and the
// joint, and a sensor that reads whole counts with a count of noise.
type simWaist struct {
	cfg WaistConfig

	breakaway           float64 // percent of effort that turns nothing
	degPerSecPerPercent float64
	tau                 float64 // s, how long the motor and the joint coast
	slack               float64 // degrees of motor travel before the chain bites
	// centring is the tyres' push back towards straight while rolling, in
	// percent of effort per degree of articulation.
	centring float64

	motor, speed, joint float64 // degrees, degrees a second, degrees
	noise               uint32
}

func (w *simWaist) advance(effortPercent, dt float64) {
	push := effortPercent - w.centring*w.joint
	drive := 0.0
	if a := math.Abs(push); a > w.breakaway {
		drive = math.Copysign((a-w.breakaway)*w.degPerSecPerPercent, push)
	}
	w.speed += (drive - w.speed) * dt / w.tau
	w.motor += w.speed * dt
	switch {
	case w.motor-w.joint > w.slack/2:
		w.joint = w.motor - w.slack/2
	case w.joint-w.motor > w.slack/2:
		w.joint = w.motor + w.slack/2
	}
}

// raw is what the sensor reports for the joint now, with a count of noise
// either way now and then, as measured on the vehicle.
func (w *simWaist) raw() int {
	w.noise = w.noise*1664525 + 1013904223
	n := 0
	switch w.noise >> 29 {
	case 0:
		n = -1
	case 7:
		n = 1
	}
	return w.cfg.StraightCount + int(math.Round(w.joint*w.cfg.countsPerDegree())) + n
}

// artitraxWaist is the waist configuration on the vehicle on 1 October 2026.
func artitraxWaist() WaistConfig {
	w := defaultWaist()
	w.StraightCount, w.CalibrationCount, w.CalibrationDegrees = 471, 951, 57.91
	w.LimitDegrees, w.LimitMarginDegrees = 50, 2
	w.EffortTurnsLeft = -1
	w.GainPercentPerDegree, w.MaxEffortPercent, w.DeadbandDegrees, w.MinEffortPercent = 4, 50, 0.5, 22
	w.RampPercentPerSecond = 100
	w.TakeUpEffortPercent, w.TakeUpCounts, w.TakeUpMaxMs = 40, 4, 1000
	applyWaistDefaults(&w)
	return w
}

// steerTo drives the simulated waist towards target degrees by curvature, at
// 50 Hz with the loader's own steering path and ramp, and returns the joint's
// angle every cycle.
func steerTo(t *testing.T, w *simWaist, target float64, seconds float64) []float64 {
	t.Helper()
	d, _, _ := drivingDrivetrain(t, w.cfg)
	steer, _ := d.steeringMotor()
	d.curvature, d.byAngle = d.cfg.Geometry.frontCurvature(target*math.Pi/180), true
	step := d.cfg.Waist.rampStep(d.cfg.CommandHz)
	now := time.Unix(1_000_000, 0)
	var trace []float64
	for c := 0; c < int(seconds*50); c++ {
		raw := w.raw()
		desired := d.steeringRawLocked(steer, raw, true, now)
		d.last[steer.NodeID] = rateLimit(d.last[steer.NodeID], desired, step, step)
		effort := float64(d.last[steer.NodeID]) / fullScale * 100 * d.waist.rawEffortSign()
		for ms := 0; ms < 20; ms++ {
			w.advance(effort, 0.001)
		}
		now = now.Add(20 * time.Millisecond)
		trace = append(trace, w.joint)
	}
	return trace
}

// newSimWaist is a waist rolling on the ground: the tyres ease the joint's
// friction and push it back towards straight. Not measured — a guess at the
// vehicle's numbers, chosen as one the angle loop swung about its target on
// until 1 October 2026, crossing it every second or so; standing on blocks, the
// same loop settled.
func newSimWaist(cfg WaistConfig) *simWaist {
	return &simWaist{cfg: cfg, breakaway: 8, degPerSecPerPercent: 1.0, tau: 0.1, slack: 3, centring: 0.3, noise: 1}
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
