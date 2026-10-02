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
	"math"
	"time"
)

// The waist: what its sensor means, how far it may go, and what drives it.
//
// The waist motor turns the joint through a bicycle chain, and there is no hard
// stop and no limit switch anywhere in the travel. Past about 40° either way the
// motor is simply driving the joint into itself, and something gives — the
// chain, the sprocket or the motor. The only limit is the one in this file.
//
// So steering effort is never sent blind. Every cycle it passes a guard:
//
//   - no fresh reading from the waist sensor, no steering at all;
//   - beyond the limit, no effort that pushes further out, and none until the
//     joint is back inside by a margin; effort back towards straight is always
//     allowed;
//   - while nobody has said which way the motor turns the joint, no effort at
//     all beyond the limit, since "back" is not known;
//   - a watchdog on the effort and the angle together: effort held and the
//     joint not moving is a jumped or broken chain, or a stalled motor; effort
//     one way and the joint moving the other is a sign configured wrongly,
//     which would turn the limit into the opposite of a limit. Either stops the
//     vehicle, and the loader says which.
//
// A refused effort takes the steering motor to zero at once, skipping the ramp:
// the ramp's worth of travel is exactly what the limit is there to prevent.

// WaistConfig is the sensor's calibration, the limit, and the angle loop.
type WaistConfig struct {
	// StraightCount is the sensor's reading with the joint straight.
	StraightCount int `json:"straightCount"`

	// CalibrationCount is the reading at a measured articulation of
	// CalibrationDegrees, positive to the LEFT (ISO 8855). Until both are set
	// the loader is uncalibrated: it will not steer to an angle, and its limit
	// is UncalibratedWindowCounts either side of straight.
	CalibrationCount   int     `json:"calibrationCount"`
	CalibrationDegrees float64 `json:"calibrationDegrees"`

	// LimitDegrees is the software limit, either side. The joint's range is
	// about ±40° with nothing to stop it beyond that; the default leaves a
	// margin for the travel between one reading and the next.
	LimitDegrees float64 `json:"limitDegrees"`

	// LimitMarginDegrees is how far back inside the limit the joint must come
	// before effort outward is allowed again. Without it, a joint resting on
	// the limit — the chain's slack, the tyres springing back, a count of
	// sensor noise — dropped inside by one count, was pushed out again, and the
	// motor kept nudging past the limit for as long as the stick was held.
	LimitMarginDegrees float64 `json:"limitMarginDegrees"`

	// UncalibratedWindowCounts is the limit before calibration, in raw counts
	// either side of straight. Small, because before calibration nobody knows
	// what a count is worth: at 11 counts a degree, 40 counts is under 4°; if
	// the reference's (raw-450)/150 were radians, it would be 15°. Either is
	// safe, and 100 counts would not have been under the second reading.
	UncalibratedWindowCounts int `json:"uncalibratedWindowCounts"`

	// EffortTurnsLeft says what a positive raw effort on the steering motor
	// does to the vehicle: 1 if it turns it left, -1 if right, 0 if nobody has
	// looked yet. The first thing to find out on the vehicle.
	EffortTurnsLeft int `json:"effortTurnsLeft"`

	// StaleMs is how old a reading may be and still steer. It is much shorter
	// than for any other measurement, because it is what the limit stands on.
	StaleMs int `json:"staleMs"`

	// The angle loop: effort in percent per degree of error, its ceiling, and
	// a deadband inside which the joint is left alone. Not tuned on the vehicle.
	GainPercentPerDegree float64 `json:"gainPercentPerDegree"`
	MaxEffortPercent     float64 `json:"maxEffortPercent"`
	DeadbandDegrees      float64 `json:"deadbandDegrees"`
	// MinEffortPercent is the least effort the angle loop applies outside the
	// deadband: what it takes to get the joint moving at all, against the chain
	// and the tyres. Below it the loop asks for effort that turns nothing, and
	// the last few degrees take seconds. 0 until measured on the vehicle.
	MinEffortPercent float64 `json:"minEffortPercent"`

	// RampPercentPerSecond is how fast the steering effort may change. The
	// wheels' ramp (accelStep) is in RPM and far too slow for the waist: at 24%
	// a second the motor sat below its breakaway for most of a second before
	// the joint moved. Per second rather than per cycle, so that a change of
	// commandHz does not retune it.
	RampPercentPerSecond float64 `json:"rampPercentPerSecond"`

	// The take-up, which crosses the slack in the chain. When the effort asked
	// for turns the other way from the side the chain is loaded on, the motor
	// winds through the slack before the joint moves at all, and a small effort
	// takes a long time to do it. Until the joint has moved TakeUpCounts that
	// way, or TakeUpMaxMs has passed, the effort is at least
	// TakeUpEffortPercent. 0 turns it off; measure it on the ground, where the
	// tyres are part of the load.
	//
	// TakeUpCounts is 4 because the sensor at rest reads ±1 and now and then ±2
	// (measured on the vehicle, 30 September 2026): 2 would end a take-up on
	// noise.
	TakeUpEffortPercent float64 `json:"takeUpEffortPercent"`
	TakeUpCounts        int     `json:"takeUpCounts"`
	TakeUpMaxMs         int     `json:"takeUpMaxMs"`
	// TakeUpMinErrorDegrees is how far from its target the angle loop must be
	// for a take-up to be worth it. Every correction of an overshoot turns the
	// other way from the side the chain is loaded on, and boosting each one to
	// TakeUpEffortPercent threw the joint past its target again: the waist
	// swung about the target, wider the longer it went on. A new stick
	// position is a move worth boosting; a degree of overshoot is not.
	TakeUpMinErrorDegrees float64 `json:"takeUpMinErrorDegrees"`

	// The watchdog: effort of at least StallEffortPercent held for StallMs must
	// move the joint by at least StallCounts.
	StallEffortPercent float64 `json:"stallEffortPercent"`
	StallMs            int     `json:"stallMs"`
	StallCounts        int     `json:"stallCounts"`
}

func defaultWaist() WaistConfig {
	return WaistConfig{
		StraightCount:            450,
		LimitDegrees:             35,
		LimitMarginDegrees:       2,
		UncalibratedWindowCounts: 40,
		StaleMs:                  200,
		GainPercentPerDegree:     4,
		MaxEffortPercent:         50,
		DeadbandDegrees:          0.5,
		RampPercentPerSecond:     500, // 0 to 50% in 0.1 s
		TakeUpCounts:             4,
		TakeUpMaxMs:              300,
		TakeUpMinErrorDegrees:    2,
		StallEffortPercent:       30,
		StallMs:                  1500,
		StallCounts:              3,
	}
}

func applyWaistDefaults(w *WaistConfig) {
	d := defaultWaist()
	if w.StraightCount == 0 {
		w.StraightCount = d.StraightCount
	}
	if w.LimitDegrees <= 0 {
		w.LimitDegrees = d.LimitDegrees
	}
	if w.LimitMarginDegrees <= 0 {
		w.LimitMarginDegrees = d.LimitMarginDegrees
	}
	// A margin as wide as the limit would hold the joint until it was past
	// straight.
	w.LimitMarginDegrees = math.Min(w.LimitMarginDegrees, w.LimitDegrees/2)
	if w.UncalibratedWindowCounts <= 0 {
		w.UncalibratedWindowCounts = d.UncalibratedWindowCounts
	}
	if w.StaleMs <= 0 {
		w.StaleMs = d.StaleMs
	}
	if w.GainPercentPerDegree <= 0 {
		w.GainPercentPerDegree = d.GainPercentPerDegree
	}
	if w.MaxEffortPercent <= 0 {
		w.MaxEffortPercent = d.MaxEffortPercent
	}
	if w.DeadbandDegrees <= 0 {
		w.DeadbandDegrees = d.DeadbandDegrees
	}
	if w.MinEffortPercent < 0 {
		w.MinEffortPercent = 0
	}
	if w.RampPercentPerSecond <= 0 {
		w.RampPercentPerSecond = d.RampPercentPerSecond
	}
	if w.TakeUpEffortPercent < 0 {
		w.TakeUpEffortPercent = 0
	}
	if w.TakeUpCounts <= 0 {
		w.TakeUpCounts = d.TakeUpCounts
	}
	if w.TakeUpMaxMs <= 0 {
		w.TakeUpMaxMs = d.TakeUpMaxMs
	}
	if w.TakeUpMinErrorDegrees <= 0 {
		w.TakeUpMinErrorDegrees = d.TakeUpMinErrorDegrees
	}
	if w.StallEffortPercent <= 0 {
		w.StallEffortPercent = d.StallEffortPercent
	}
	if w.StallMs <= 0 {
		w.StallMs = d.StallMs
	}
	if w.StallCounts <= 0 {
		w.StallCounts = d.StallCounts
	}
}

// calibrated reports whether the reading can be turned into degrees.
func (w WaistConfig) calibrated() bool {
	return w.CalibrationDegrees != 0 && w.CalibrationCount != w.StraightCount
}

// countsPerDegree is signed: negative when the count falls as the joint turns
// left.
func (w WaistConfig) countsPerDegree() float64 {
	return float64(w.CalibrationCount-w.StraightCount) / w.CalibrationDegrees
}

// degrees is the articulation, positive to the left. Only meaningful when
// calibrated.
func (w WaistConfig) degrees(raw int) float64 {
	return float64(raw-w.StraightCount) / w.countsPerDegree()
}

// targetLimitDegrees is the furthest the angle loop aims, either side: inside
// the limit by its margin. A target on the limit itself is one the guard cuts
// the effort at, and the joint hunted there for as long as the stick was held —
// over the limit, held, back inside by the margin, pushed out again.
func (w WaistConfig) targetLimitDegrees() float64 {
	return w.LimitDegrees - w.LimitMarginDegrees
}

// leftward is the direction the count moves as the joint turns left: the
// calibration's answer when there is one, and until then the reference's
// convention, that lower counts are to the left. Guessed or not, the watchdog
// checks it against what the joint actually does.
func (w WaistConfig) leftward() int {
	if w.calibrated() {
		return sign(w.countsPerDegree())
	}
	return -1
}

// beyond says whether a reading is past the limit: 1 beyond it on the left, -1
// on the right, 0 within it.
func (w WaistConfig) beyond(raw int) int {
	if w.calibrated() {
		deg := w.degrees(raw)
		switch {
		case deg > w.LimitDegrees:
			return 1
		case deg < -w.LimitDegrees:
			return -1
		}
		return 0
	}
	offset := raw - w.StraightCount
	if abs(offset) <= w.UncalibratedWindowCounts {
		return 0
	}
	// Which side of straight this is, in the vehicle's terms.
	return sign(float64(offset)) * w.leftward()
}

// uncalibratedMarginCounts is LimitMarginDegrees before calibration, when a
// degree has no count to be converted to: a few counts, well clear of one
// count's noise and small against the 40-count window.
const uncalibratedMarginCounts = 5

// clearOf reports whether a reading is back inside the limit on side (1 left,
// -1 right) by the margin.
func (w WaistConfig) clearOf(raw, side int) bool {
	if w.calibrated() {
		return float64(side)*w.degrees(raw) <= w.LimitDegrees-w.LimitMarginDegrees
	}
	leftOfStraight := (raw - w.StraightCount) * w.leftward()
	return side*leftOfStraight <= w.UncalibratedWindowCounts-uncalibratedMarginCounts
}

// waistState is the guard and the watchdog's memory.
type waistState struct {
	cfg WaistConfig

	fault string // set by the watchdog; steering is refused until it is cleared

	// held is the side whose limit the joint has reached, 1 left or -1 right,
	// and 0 once it is back inside by the margin. Outward effort on that side
	// is refused until then.
	held int

	// engaged is the side the chain is loaded on, 1 left or -1 right, 0 when
	// not known — as at start-up, so the first move takes up the slack too.
	// takingUp is the direction of a take-up in progress, 0 when none.
	engaged     int
	takingUp    int
	takeUpSince time.Time
	takeUpFrom  int

	// settled is set once the angle loop has brought the joint inside its
	// deadband, and holds it there until the error is twice the deadband.
	settled bool

	watching   int // the sign of the effort being watched, 0 when none
	watchSince time.Time
	watchFrom  int

	reported map[string]time.Time // for saying things once in a while, not every cycle
}

func newWaistState(cfg WaistConfig) *waistState {
	return &waistState{cfg: cfg, reported: make(map[string]time.Time)}
}

// guard is the effort, in percent with positive to the left, that may actually
// be applied this cycle, and why not if it is less.
func (s *waistState) guard(effort float64, raw int, fresh bool) (float64, string) {
	// Every fresh reading moves the latch, whether or not anything is being
	// asked of the motor this cycle.
	if fresh {
		s.latch(raw)
	}
	if effort == 0 {
		return 0, ""
	}
	if s.fault != "" {
		return 0, s.fault
	}
	if !fresh {
		return 0, "no fresh reading from the waist sensor — steering is not driven blind"
	}
	side := s.held
	if side == 0 {
		return effort, ""
	}
	if s.cfg.EffortTurnsLeft == 0 {
		return 0, "the waist is past its limit and effortTurnsLeft is not set, so there is no telling which way is back"
	}
	if sign(effort) == side {
		if s.cfg.beyond(raw) == side {
			return 0, fmt.Sprintf("the waist is past its limit on the %s; only effort back towards straight is allowed", sideName(side))
		}
		return 0, fmt.Sprintf("the waist reached its limit on the %s and is not yet back inside it by the margin; "+
			"only effort back towards straight is allowed", sideName(side))
	}
	return effort, ""
}

// latch notes the side whose limit the joint has passed, and lets go only once
// the joint is back inside it by the margin.
func (s *waistState) latch(raw int) {
	if side := s.cfg.beyond(raw); side != 0 {
		s.held = side
		return
	}
	if s.held != 0 && s.cfg.clearOf(raw, s.held) {
		s.held = 0
	}
}

// takeUp is the effort, in percent with positive to the left, that crosses the
// slack in the chain before the loop's own effort is left to move the joint.
// A request the other way from the side the chain is loaded on, however small,
// gets at least TakeUpEffortPercent until the joint has moved TakeUpCounts that
// way, or TakeUpMaxMs has passed. ended is set on the cycle a take-up finishes,
// and says how long it took, for tuning it.
//
// With boost false the effort is left as it is and only the side the chain is
// loaded on is followed: a person steering by effort gets what the stick says,
// and the angle loop that takes over later starts from where the chain really
// is.
//
// An idle motor leaves the chain where it is, so zero effort keeps the side it
// is engaged on; it does abandon a take-up half done, which starts again from
// the beginning the next time it is asked for.
func (s *waistState) takeUp(effort float64, raw int, fresh bool, now time.Time, boost bool) (e float64, ended string) {
	if s.cfg.TakeUpEffortPercent <= 0 {
		return effort, ""
	}
	d := sign(effort)
	if d == 0 || d == s.engaged {
		s.takingUp = 0
		return effort, ""
	}
	if s.takingUp != d {
		s.takingUp, s.takeUpSince, s.takeUpFrom = d, now, raw
	}
	took := now.Sub(s.takeUpSince)
	moved := 0
	if fresh {
		moved = (raw - s.takeUpFrom) * s.cfg.leftward() * d
	}
	switch {
	case moved >= s.cfg.TakeUpCounts:
		ended = fmt.Sprintf("took up the slack to the %s in %d ms", sideName(d), took.Milliseconds())
	case took >= time.Duration(s.cfg.TakeUpMaxMs)*time.Millisecond:
		ended = fmt.Sprintf("the waist did not move within takeUpMaxMs (%d ms) of taking up the slack to the %s; "+
			"carrying on as if it had", s.cfg.TakeUpMaxMs, sideName(d))
	case boost:
		return float64(d) * math.Max(math.Abs(effort), s.cfg.TakeUpEffortPercent), ""
	default:
		return effort, ""
	}
	s.engaged, s.takingUp = d, 0
	if !boost {
		return effort, ""
	}
	return effort, ended
}

// watch compares the effort applied with what the joint did, and reports a
// fault when they disagree. applied is in percent, positive to the left.
func (s *waistState) watch(applied float64, raw int, fresh bool, now time.Time) (fault string, observation string) {
	if !fresh || math.Abs(applied) < s.cfg.StallEffortPercent {
		s.watching = 0
		return "", ""
	}
	dir := sign(applied)
	if dir != s.watching {
		s.watching, s.watchSince, s.watchFrom = dir, now, raw
		return "", ""
	}
	held := now.Sub(s.watchSince)
	if held < time.Duration(s.cfg.StallMs)*time.Millisecond {
		return "", ""
	}
	moved := raw - s.watchFrom
	s.watchSince, s.watchFrom = now, raw

	if abs(moved) < s.cfg.StallCounts {
		return fmt.Sprintf("steering fault: %.0f%% effort for %d ms moved the waist %d counts — "+
			"check the chain, the waist motor and the sensor", applied, held.Milliseconds(), moved), ""
	}
	if s.cfg.EffortTurnsLeft == 0 {
		return "", fmt.Sprintf("effort to the %s (raw sign %+d) moved the waist count %s, %d to %d; "+
			"if the vehicle turned left, set effortTurnsLeft to %d, otherwise to %d",
			sideName(dir), dir, upDown(moved), raw-moved, raw, dir, -dir)
	}
	// Effort to the left should move the count in the leftward direction.
	if sign(float64(moved)) != dir*s.cfg.leftward() {
		return fmt.Sprintf("steering fault: effort to the %s moved the waist count %s, the opposite of what "+
			"effortTurnsLeft and the calibration say — one of them is wrong, and with it the limit", sideName(dir), upDown(moved)), ""
	}
	return "", ""
}

// angleEffort is the angle loop: the effort, in percent with positive to the
// left, that moves the joint from measured towards target, both in degrees.
//
// The deadband has hysteresis. Once inside it the loop lets go, and starts
// again only when the error is twice the deadband. Outside the deadband the
// effort is at least MinEffortPercent, so without it a count of sensor noise at
// the deadband's edge was a kick of the breakaway's effort, and another back.
func (s *waistState) angleEffort(target, measured float64) float64 {
	err := target - measured
	band := s.cfg.DeadbandDegrees
	if s.settled {
		band *= 2
	}
	if math.Abs(err) < band {
		s.settled = true
		return 0
	}
	s.settled = false
	e := math.Copysign(math.Max(s.cfg.MinEffortPercent, s.cfg.GainPercentPerDegree*math.Abs(err)), err)
	return math.Max(-s.cfg.MaxEffortPercent, math.Min(s.cfg.MaxEffortPercent, e))
}

// rampStep is how far, in raw counts, the steering effort may move in one
// cycle at commandHz.
func (w WaistConfig) rampStep(commandHz int) int16 {
	step := w.RampPercentPerSecond / 100 * fullScale / float64(commandHz)
	return int16(math.Max(1, math.Min(fullScale, step)))
}

// rawEffortSign is what to multiply an effort towards the left by to get the
// sign the motor wants. Until someone has looked, positive is sent as positive.
func (s *waistState) rawEffortSign() float64 {
	if s.cfg.EffortTurnsLeft < 0 {
		return -1
	}
	return 1
}

// every reports whether key has not been said for at least interval, and
// notes that it is being said now.
func (s *waistState) every(key string, interval time.Duration, now time.Time) bool {
	if last, ok := s.reported[key]; ok && now.Sub(last) < interval {
		return false
	}
	s.reported[key] = now
	return true
}

func sign(v float64) int {
	switch {
	case v > 0:
		return 1
	case v < 0:
		return -1
	}
	return 0
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

func sideName(s int) string {
	if s > 0 {
		return "left"
	}
	return "right"
}

func upDown(moved int) string {
	if moved > 0 {
		return "up"
	}
	return "down"
}
