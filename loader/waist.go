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
	"encoding/json"
	"fmt"
	"math"
	"slices"
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
//     allowed; and the angle loop's integral is forgotten as the limit is
//     reached;
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

	// The angle loop holds, or moves the joint; see anglePI. Holding, it gives
	// the motor nothing: the gearbox is self-locking, and holds the joint by
	// itself. It moves the joint once it is HoldDegrees from its target, with
	// a PI, and lets go once the joint will coast to within SettleDegrees of
	// it, CoastSeconds of its rate from now.
	//
	// GainPercentPerDegree is kp and IntegralPercentPerDegreeSecond ki, both in
	// percent of effort, and MaxEffortPercent the ceiling. BreakawayPercent is
	// the effort that starts the joint from rest, measured with waistkick.py:
	// a move starts just below it, and the integral finds the rest. 0 starts a
	// move from the gain alone. Tuned in the simulation (waistsim_test.go), not
	// on the vehicle.
	GainPercentPerDegree           float64 `json:"gainPercentPerDegree"`
	IntegralPercentPerDegreeSecond float64 `json:"integralPercentPerDegreeSecond"`
	MaxEffortPercent               float64 `json:"maxEffortPercent"`
	BreakawayPercent               float64 `json:"breakawayPercent"`
	HoldDegrees                    float64 `json:"holdDegrees"`
	SettleDegrees                  float64 `json:"settleDegrees"`
	CoastSeconds                   float64 `json:"coastSeconds"`

	// RampPercentPerSecond is how fast the steering effort may change. The
	// wheels' ramp (accelStep) is in RPM and far too slow for the waist: at 24%
	// a second the motor sat below its breakaway for most of a second before
	// the joint moved. Per second rather than per cycle, so that a change of
	// commandHz does not retune it.
	RampPercentPerSecond float64 `json:"rampPercentPerSecond"`

	// JointFilterSeconds is how much the joint's angle and rate are smoothed
	// before the wheels are set from them; see jointEstimate. IgnoreJointRate
	// sets the wheels from the steady-state model and the raw angle, as before
	// 7 October 2026, for comparing the two on the vehicle.
	JointFilterSeconds float64 `json:"jointFilterSeconds"`
	IgnoreJointRate    bool    `json:"ignoreJointRate"`

	// The watchdog: effort of at least StallEffortPercent held for StallMs must
	// move the joint by at least StallCounts.
	StallEffortPercent float64 `json:"stallEffortPercent"`
	StallMs            int     `json:"stallMs"`
	StallCounts        int     `json:"stallCounts"`
}

func defaultWaist() WaistConfig {
	return WaistConfig{
		StraightCount:                  450,
		LimitDegrees:                   35,
		LimitMarginDegrees:             2,
		UncalibratedWindowCounts:       40,
		StaleMs:                        200,
		GainPercentPerDegree:           4,
		IntegralPercentPerDegreeSecond: 4,
		MaxEffortPercent:               50,
		HoldDegrees:                    1,
		SettleDegrees:                  0.3,
		CoastSeconds:                   0.1,
		RampPercentPerSecond:           500, // 0 to 50% in 0.1 s
		JointFilterSeconds:             0.15,
		StallEffortPercent:             30,
		StallMs:                        1500,
		StallCounts:                    3,
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
	if w.IntegralPercentPerDegreeSecond <= 0 {
		w.IntegralPercentPerDegreeSecond = d.IntegralPercentPerDegreeSecond
	}
	if w.BreakawayPercent < 0 {
		w.BreakawayPercent = 0
	}
	w.BreakawayPercent = math.Min(w.BreakawayPercent, w.MaxEffortPercent)
	if w.HoldDegrees <= 0 {
		w.HoldDegrees = d.HoldDegrees
	}
	if w.SettleDegrees <= 0 {
		w.SettleDegrees = d.SettleDegrees
	}
	// Letting go no closer than it starts would let go at once.
	w.SettleDegrees = math.Min(w.SettleDegrees, w.HoldDegrees/2)
	if w.CoastSeconds <= 0 {
		w.CoastSeconds = d.CoastSeconds
	}
	if w.RampPercentPerSecond <= 0 {
		w.RampPercentPerSecond = d.RampPercentPerSecond
	}
	if w.JointFilterSeconds <= 0 {
		w.JointFilterSeconds = d.JointFilterSeconds
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

// removedWaistKeys are the angle loop's settings that are gone: the P loop's
// until 7 October 2026, and the deadzone's until 8 October.
var removedWaistKeys = []string{
	"deadbandDegrees", "minEffortPercent",
	"takeUpEffortPercent", "takeUpCounts", "takeUpMaxMs", "takeUpMinErrorDegrees",
	"deadzonePercent", "integralBleedSeconds",
}

// staleWaistKeys lists the removed settings a configuration's traits still
// carry. They are ignored, and a tuning someone measured should not vanish
// without a word.
func staleWaistKeys(traits []byte) []string {
	var t struct {
		Waist map[string]json.RawMessage `json:"waist"`
	}
	if json.Unmarshal(traits, &t) != nil {
		return nil
	}
	var stale []string
	for _, k := range removedWaistKeys {
		if _, ok := t.Waist[k]; ok {
			stale = append(stale, k)
		}
	}
	slices.Sort(stale)
	return stale
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
	// is refused until then. Set, it zeroes the angle loop's integral.
	held int

	// The angle loop: the way it is moving the joint, 1 left or -1 right, and
	// 0 while it holds; its integral, in percent of effort; and the time of the
	// reading it last learned from: it learns once per reading, not per cycle.
	// letGo is when it last let go, until the joint has come to rest, and
	// letGoTarget the target it was then.
	moving      int
	integral    float64
	piAt        time.Time
	letGo       time.Time
	letGoTarget float64

	// joint is the angle and rate the wheels are set from.
	joint jointEstimate

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
// the joint is back inside it by the margin. Reaching the limit also forgets
// the angle loop's integral: what it learned was a push to here.
func (s *waistState) latch(raw int) {
	if side := s.cfg.beyond(raw); side != 0 {
		if side != s.held {
			s.integral = 0
		}
		s.held = side
		return
	}
	if s.held != 0 && s.cfg.clearOf(raw, s.held) {
		s.held = 0
	}
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

// breakawayPreload is the share of BreakawayPercent a move starts from: a
// little short of it, so that a breakaway measured high, or one that has
// eased, does not throw the joint past its target; the integral finds the rest
// within a fraction of a second.
const breakawayPreload = 0.9

// settleWait is the longest the angle loop waits, after letting go, for the
// joint to come to rest: a joint still moving after that is being moved by
// something else, the tyres or the wheels, and the loop takes it on.
//
// The wait is for the target the loop let go at. A target that has since moved
// by more than HoldDegrees ends it: on 22 October 2026, swinging the stick from
// one side to the other standing on the ground, every reversal waited the full
// second — the joint, just let go at 30°/s, was still coasting — with the
// target already 70° away on the other side, and the waist answered the stick
// a second and a half late.
const settleWait = time.Second

// anglePI is the angle loop: the effort, in percent with positive to the
// left, that moves the joint from measured towards target, both in degrees.
// measured is the reading as it is; the joint estimate, which the caller keeps
// up to date, gives the joint's rate and a smoothed angle. at is when the
// reading was taken; the integral learns once per reading, over the time since
// the last, and not across a gap the reading would be stale after.
//
// The loop holds or it moves. Holding, the effort is zero and so is the
// integral, whatever the error, until the joint is HoldDegrees from its target:
// a count of noise, a target wandering with a stick near its centre, or a
// fraction of a degree is no reason to cross the chain's slack, and the
// self-locking gearbox holds the joint without help. Where the joint is, for
// that, is the smoothed angle, and where it will come to rest: noise and a joint
// coasting after a move are no reason either.
//
// Moving, it is a PI, pushing one way only. It starts from just below the
// breakaway, rather than waiting for the integral to wind up to it, which took
// two seconds for a step of two degrees; and it lets go, back to holding, once
// the joint will coast to within SettleDegrees of the target, or past it. A
// push one way that turned into a push the other way would take up the slack
// in the chain both ways, and that is what the joint hunted on.
//
// Until 8 October 2026 the loop was a PI with a deadzone on its effort. It
// settled in the simulation, and hunted on the vehicle on blocks with the stick
// centred: the integral wound up to the breakaway, a motor that has broken away
// keeps turning on less, so the joint lurched past its target, and the
// integral, still loaded, carried it on.
//
// At the ceiling the integral is frozen. Until 16 October 2026 it was clamped
// to the ceiling's headroom instead: as the joint closed in and the gain's share
// fell, the integral grew into what the gain left, and kept the effort at the
// ceiling for as long as it could keep up; and a target far enough off for the
// gain alone to reach the ceiling discarded it altogether.
func (s *waistState) anglePI(target, measured float64, at time.Time) float64 {
	c := s.cfg
	dt := 0.0
	if at.After(s.piAt) {
		if gap := at.Sub(s.piAt); !s.piAt.IsZero() && gap <= time.Duration(c.StaleMs)*time.Millisecond {
			dt = gap.Seconds()
		}
		s.piAt = at
	}
	err := target - measured
	p := c.GainPercentPerDegree * err
	rate, smoothed := 0.0, measured
	if s.joint.ok {
		rate, smoothed = s.joint.rate(), s.joint.deg
	}
	if s.moving == 0 {
		// Just let go, the joint is left to come to rest before it is judged:
		// coasting, it is neither where it will stop nor anything the loop can
		// help, and a push back would only take up the chain's slack. Unless
		// the pilot has asked for somewhere else since.
		if !s.letGo.IsZero() {
			if rate != 0 && at.Sub(s.letGo) < settleWait && math.Abs(target-s.letGoTarget) <= c.HoldDegrees {
				return 0
			}
			s.letGo = time.Time{}
		}
		// Whether to move at all is judged on the smoothed angle: a joint
		// resting near the edge of HoldDegrees is not to be pushed by a count
		// of noise.
		resting := target - smoothed - rate*c.CoastSeconds
		if math.Abs(resting) <= c.HoldDegrees {
			return 0
		}
		s.moving = sign(resting)
		s.integral = float64(s.moving) * math.Max(0, breakawayPreload*c.BreakawayPercent-math.Abs(p))
	}
	// Everything from here is in the direction of the move. It ends where the
	// joint will come to rest if let go now, CoastSeconds of its rate on.
	dir := float64(s.moving)
	if dir*(err-rate*c.CoastSeconds) <= c.SettleDegrees {
		s.moving, s.integral, s.letGo, s.letGoTarget = 0, 0, at, target
		return 0
	}
	pushing := dir * s.integral
	if dir*p+pushing < c.MaxEffortPercent {
		// Below the ceiling: learn, but no further than the ceiling.
		pushing = math.Min(c.MaxEffortPercent-dir*p, pushing+c.IntegralPercentPerDegreeSecond*dir*err*dt)
	}
	// At the ceiling the integral is frozen: neither learned nor discarded.
	pushing = math.Max(0, pushing)
	s.integral = dir * pushing
	return dir * math.Max(0, math.Min(c.MaxEffortPercent, dir*p+pushing))
}

// resetPI forgets the angle loop's integral, and that it was moving the joint,
// whenever the loop is not the one steering: no curvature command, no reading
// to steer by, or the guard refusing what it asked for. What it learned against
// a refusal is a push waiting for the refusal to end.
func (s *waistState) resetPI() {
	s.moving, s.integral, s.piAt, s.letGo = 0, 0, time.Time{}, time.Time{}
}

// jointEstimate is the joint's angle and rate as the wheels see them: an
// alpha-beta filter on the waist sensor, learning once per reading.
//
// While the joint swings, the two halves turn relative to each other and the
// wheels have to roll with them. Set from the steady-state model alone, the
// wheels' speed loops held them at speeds that did not allow the swing, and
// pushed back against the waist motor; standing still, they held all four at
// zero. The waist motor and the wheels fought, and the vehicle oscillated. The
// wheels need the joint's rate for that, and one count of the sensor between
// two readings is several degrees a second: hence the filter. The angle loop
// steers by the raw reading, since its feedback must not lag; it takes the
// rate from here to know when to let go, and the smoothed angle to decide
// whether to move at all.
type jointEstimate struct {
	ok      bool
	at      time.Time // the reading last learned from
	deg     float64   // degrees, positive to the left
	rateDeg float64   // degrees a second
}

// jointRestDegPerSec is the rate below which the joint is taken to be still: a
// count of noise must not set a resting vehicle's wheels turning.
const jointRestDegPerSec = 0.5

// update learns from a reading of deg degrees taken at at. The filter is the
// critically damped one with a memory of tau seconds: g = 1 − θ², h = (1 − θ)²,
// θ = e^(−dt/tau). A gap the reading would be stale after starts it afresh.
func (j *jointEstimate) update(deg float64, at time.Time, tau float64, stale time.Duration) {
	if j.ok && !at.After(j.at) {
		return
	}
	dt := at.Sub(j.at)
	if !j.ok || dt > stale {
		*j = jointEstimate{ok: true, at: at, deg: deg}
		return
	}
	sec := dt.Seconds()
	theta := math.Exp(-sec / tau)
	g, h := 1-theta*theta, (1-theta)*(1-theta)
	predicted := j.deg + j.rateDeg*sec
	residual := deg - predicted
	j.deg = predicted + g*residual
	j.rateDeg += h * residual / sec
	j.at = at
}

// rate is the joint's rate in degrees a second, zero while it is still.
func (j *jointEstimate) rate() float64 {
	if math.Abs(j.rateDeg) < jointRestDegPerSec {
		return 0
	}
	return j.rateDeg
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
