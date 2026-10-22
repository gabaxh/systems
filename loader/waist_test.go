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
	"net/http"
	"strconv"
	"strings"
	"testing"
	"time"
)

// calibrated is the waist as the students might measure it: straight at 450,
// 20° to the left at 230, so 11 counts a degree with the count falling to the
// left.
func calibrated() WaistConfig {
	w := defaultWaist()
	w.CalibrationCount, w.CalibrationDegrees = 230, 20
	w.EffortTurnsLeft = 1
	return w
}

func TestCalibrationIsISO(t *testing.T) {
	w := calibrated()
	if !w.calibrated() {
		t.Fatal("a two-point calibration was not taken")
	}
	for _, c := range []struct {
		raw int
		deg float64
	}{{450, 0}, {230, 20}, {560, -10}} {
		if got := w.degrees(c.raw); math.Abs(got-c.deg) > 1e-9 {
			t.Errorf("count %d = %v°, want %v° (positive left)", c.raw, got, c.deg)
		}
	}
	if w.leftward() != -1 {
		t.Error("the count falls to the left, and leftward says otherwise")
	}
	// Measured on the other side, the same sensor gives the same answer.
	w.CalibrationCount, w.CalibrationDegrees = 670, -20
	if got := w.degrees(230); math.Abs(got-20) > 1e-9 {
		t.Errorf("calibrated on the right, count 230 = %v°, want 20", got)
	}
}

func TestUncalibratedByDefault(t *testing.T) {
	if defaultWaist().calibrated() {
		t.Error("the default waist claims a calibration nobody made")
	}
}

func TestBeyondTheLimit(t *testing.T) {
	w := calibrated() // limit 35°: counts 65 to 835
	for _, c := range []struct{ raw, side int }{{450, 0}, {70, 0}, {54, 1}, {846, -1}} {
		if got := w.beyond(c.raw); got != c.side {
			t.Errorf("calibrated, count %d: beyond = %d, want %d", c.raw, got, c.side)
		}
	}
	u := defaultWaist() // uncalibrated: 40 counts either side, lower is left
	for _, c := range []struct{ raw, side int }{{450, 0}, {490, 0}, {491, -1}, {409, 1}} {
		if got := u.beyond(c.raw); got != c.side {
			t.Errorf("uncalibrated, count %d: beyond = %d, want %d", c.raw, got, c.side)
		}
	}
}

func TestGuard(t *testing.T) {
	s := newWaistState(calibrated())
	const pastLeft, pastRight = 40, 860

	if e, _ := s.guard(30, 450, true); e != 30 {
		t.Errorf("inside the limit, effort was cut to %v", e)
	}
	if e, why := s.guard(30, 450, false); e != 0 || !strings.Contains(why, "fresh") {
		t.Errorf("with a stale reading: %v (%q), want no steering", e, why)
	}
	if e, _ := s.guard(30, pastLeft, true); e != 0 {
		t.Error("past the left limit, effort further left was allowed")
	}
	if e, _ := s.guard(-30, pastLeft, true); e != -30 {
		t.Error("past the left limit, effort back to the right was refused")
	}
	if e, _ := s.guard(-30, pastRight, true); e != 0 {
		t.Error("past the right limit, effort further right was allowed")
	}
	if e, _ := s.guard(30, pastRight, true); e != 30 {
		t.Error("past the right limit, effort back to the left was refused")
	}

	unknown := newWaistState(defaultWaist())
	if e, _ := unknown.guard(30, 450, true); e != 30 {
		t.Error("with the effort direction unknown, steering inside the window was refused; it is how it gets found")
	}
	if e, _ := unknown.guard(-30, 400, true); e != 0 {
		t.Error("with the effort direction unknown, effort beyond the window was allowed")
	}

	s.fault = "steering fault: test"
	if e, _ := s.guard(-10, 450, true); e != 0 {
		t.Error("steering was allowed with a fault standing")
	}
}

// A joint resting on the limit is not pushed out again the moment it drops a
// count inside it; outward effort waits until it is back by the margin.
func TestGuardHoldsUntilBackInsideByTheMargin(t *testing.T) {
	s := newWaistState(calibrated()) // 11 counts a degree, lower is left: limit 35° at 65, 33° at 87
	for _, c := range []struct {
		raw          int
		effort, want float64
		what         string
	}{
		{70, 30, 30, "inside, never at the limit"},
		{64, 30, 0, "past the limit"},
		{66, 30, 0, "a count inside, not yet by the margin"},
		{66, -30, -30, "back towards straight, while held"},
		{80, 30, 0, "still within the margin"},
		{87, 30, 30, "back inside by the margin"},
		{66, 30, 30, "released: inside the limit is inside again"},
	} {
		if got, _ := s.guard(c.effort, c.raw, true); got != c.want {
			t.Errorf("%s (count %d, effort %v): allowed %v, want %v", c.what, c.raw, c.effort, got, c.want)
		}
	}

	// The latch moves with every fresh reading, asked for effort or not.
	s = newWaistState(calibrated())
	s.guard(0, 64, true)
	if got, _ := s.guard(30, 66, true); got != 0 {
		t.Error("a limit passed with the motor idle was forgotten")
	}

	// Before calibration the margin is in counts: the window is 40 either side
	// of 450, lower is left.
	w := defaultWaist()
	w.EffortTurnsLeft = 1
	u := newWaistState(w)
	for _, c := range []struct {
		raw  int
		want float64
		what string
	}{
		{409, 0, "past the window on the left"},
		{412, 0, "inside, not yet by the margin"},
		{415, 30, "back inside by the margin"},
	} {
		if got, _ := u.guard(30, c.raw, true); got != c.want {
			t.Errorf("uncalibrated, %s (count %d): allowed %v, want %v", c.what, c.raw, got, c.want)
		}
	}
}

// Held against the limit with the stick hard over, the motor stays off: it
// used to nudge outward every time the reading dropped a count inside.
func TestNoNudgingAtTheLimit(t *testing.T) {
	d, _, steering := drivingDrivetrain(t, calibrated())
	put(t, steering.setpointService, "gamer", "50") // hard left
	withWaist(d, 70)
	d.writeCycle()
	if d.last[5] <= 0 {
		t.Fatal("the steering never started")
	}
	for _, raw := range []int{64, 65, 66, 64, 65, 66, 66, 65} {
		withWaist(d, raw)
		d.writeCycle()
		if d.last[5] != 0 {
			t.Fatalf("at count %d, against the left limit, the steering motor was given %d", raw, d.last[5])
		}
	}
}

func TestWatchdog(t *testing.T) {
	cfg := calibrated()
	cfg.StallMs = 1000
	start := time.Now()
	at := func(ms int) time.Time { return start.Add(time.Duration(ms) * time.Millisecond) }

	// Effort held, joint not moving: the chain.
	s := newWaistState(cfg)
	s.watch(40, 450, true, at(0))
	if f, _ := s.watch(40, 451, true, at(500)); f != "" {
		t.Fatalf("a fault before the watch period was over: %s", f)
	}
	if f, _ := s.watch(40, 451, true, at(1000)); !strings.Contains(f, "chain") {
		t.Errorf("a second of effort and one count of movement: %q, want a stall", f)
	}

	// Effort left and the count falling, as calibrated: all well.
	s = newWaistState(cfg)
	s.watch(40, 450, true, at(0))
	if f, _ := s.watch(40, 420, true, at(1000)); f != "" {
		t.Errorf("the joint moving the right way was a fault: %s", f)
	}

	// Effort left and the count rising: a sign is wrong, and with it the limit.
	s = newWaistState(cfg)
	s.watch(40, 450, true, at(0))
	if f, _ := s.watch(40, 480, true, at(1000)); !strings.Contains(f, "opposite") {
		t.Errorf("the joint moving the wrong way: %q, want a direction fault", f)
	}

	// Small efforts are not watched: holding the joint is not a stall.
	s = newWaistState(cfg)
	s.watch(10, 450, true, at(0))
	if f, _ := s.watch(10, 450, true, at(5000)); f != "" {
		t.Errorf("a holding effort was taken for a stall: %s", f)
	}

	// Direction unknown: say what happened, fault nothing.
	u := defaultWaist()
	u.StallMs = 1000
	s = newWaistState(u)
	s.watch(40, 450, true, at(0))
	f, obs := s.watch(40, 420, true, at(1000))
	if f != "" || !strings.Contains(obs, "effortTurnsLeft") {
		t.Errorf("with the direction unknown: fault %q, observation %q", f, obs)
	}
}

// Holding, the loop gives nothing until the joint is holdDegrees from its
// target; moving, it is the gain, at most the ceiling, and one way only.
func TestAngleLoopHolds(t *testing.T) {
	w := calibrated() // 4% a degree, hold 1°, ceiling 50%, no breakaway set
	w.IntegralPercentPerDegreeSecond = 0
	at := time.Unix(1_000_000, 0)
	for _, c := range []struct {
		target, want float64
		why          string
	}{
		{0.5, 0, "half a degree: holding"},
		{-1, 0, "a degree: still holding"},
		{1.5, 6, "a degree and a half: moving, at the gain"},
		{-1.5, -6, "the same to the right"},
		{10, 40, "ten degrees"},
		{30, 50, "thirty: the ceiling"},
	} {
		s := newWaistState(w)
		if got := s.anglePI(c.target, 0, at); math.Abs(got-c.want) > 1e-9 {
			t.Errorf("%s: effort %v; want %v", c.why, got, c.want)
		}
	}
}

// A move starts just below the breakaway: the integral is preloaded, rather
// than left to wind up to it.
func TestAngleLoopStartsAtTheBreakaway(t *testing.T) {
	w := calibrated()
	w.BreakawayPercent = 22
	s := newWaistState(w)
	at := time.Unix(1_000_000, 0)
	if got, want := s.anglePI(2, 0, at), 0.9*22; math.Abs(got-want) > 1e-9 {
		t.Errorf("2° off from rest: effort %v; want %v, just below the breakaway", got, want)
	}
	s = newWaistState(w)
	if got := s.anglePI(-8, 0, at); math.Abs(got+32) > 1e-9 {
		t.Errorf("8° off to the right: effort %v; want -32, the gain alone, past the breakaway already", got)
	}
}

// Moving, the loop lets go once the joint will coast to within settleDegrees
// of its target, and it never pushes the other way: past the target it holds.
func TestAngleLoopLetsGoBeforeItCoastsPast(t *testing.T) {
	s := newWaistState(calibrated()) // settle 0.3°, coast 0.1 s
	at := time.Unix(1_000_000, 0)
	ms := func(n int) time.Time { return at.Add(time.Duration(n) * time.Millisecond) }
	if s.anglePI(5, 0, at) <= 0 {
		t.Fatal("5° off, the loop did not move the joint")
	}
	s.joint = jointEstimate{ok: true, deg: 4, rateDeg: 5}
	if e := s.anglePI(5, 4, ms(20)); e <= 0 {
		t.Errorf("a degree short at 5°/s, coasting to half a degree short: effort %v; want still pushing", e)
	}
	s.joint = jointEstimate{ok: true, deg: 4.2, rateDeg: 6}
	if e := s.anglePI(5, 4.2, ms(40)); e != 0 || s.moving != 0 {
		t.Errorf("0.8° short at 6°/s, coasting to 0.2° short: effort %v, moving %d; want let go", e, s.moving)
	}
	s.joint = jointEstimate{ok: true, deg: 6.5, rateDeg: 4}
	if e := s.anglePI(5, 6.5, ms(60)); e != 0 {
		t.Errorf("just let go and still coasting, 1.5° past: effort %v; want left to come to rest", e)
	}
	s.joint = jointEstimate{ok: true, deg: 5.5}
	if e := s.anglePI(5, 5.5, ms(80)); e != 0 || s.moving != 0 {
		t.Errorf("at rest half a degree past: effort %v, moving %d; want holding, not pushing back", e, s.moving)
	}
	if e := s.anglePI(5, 6.3, ms(100)); e != 0 {
		t.Errorf("a reading 1.3° past with the smoothed angle half a degree past: effort %v; want none, it is noise", e)
	}
	s.joint = jointEstimate{ok: true, deg: 6.5}
	if e := s.anglePI(5, 6.5, ms(120)); e >= 0 {
		t.Errorf("at rest 1.5° past: effort %v; want a push back", e)
	}
}

// The integral learns once per reading, over the time since the last, and is
// bounded so that the effort stays within the ceiling.
func TestAngleLoopIntegral(t *testing.T) {
	s := newWaistState(calibrated()) // ki 4% a degree-second
	at := time.Unix(1_000_000, 0)
	s.anglePI(5, 0, at) // the first reading: nothing to learn over yet
	if s.integral != 0 {
		t.Fatalf("the first reading taught the integral %v", s.integral)
	}
	s.anglePI(5, 0, at.Add(100*time.Millisecond))
	if math.Abs(s.integral-2) > 1e-9 {
		t.Errorf("5° off for 0.1 s: integral %v; want 2", s.integral)
	}
	s.anglePI(5, 0, at.Add(100*time.Millisecond)) // the same reading again
	if math.Abs(s.integral-2) > 1e-9 {
		t.Errorf("the same reading taught it again: %v", s.integral)
	}
	s.anglePI(5, 0, at.Add(time.Second)) // a gap the reading would be stale after
	if math.Abs(s.integral-2) > 1e-9 {
		t.Errorf("it learned across a gap: %v", s.integral)
	}
	for i := 1; i <= 200; i++ {
		s.anglePI(5, 0, at.Add(time.Second+time.Duration(i)*20*time.Millisecond))
	}
	if want := 50 - 20.0; math.Abs(s.integral-want) > 1e-9 {
		t.Errorf("5° off for 4 s: integral %v; want it held at %v, where the effort is at its ceiling", s.integral, want)
	}
	s.resetPI()
	if s.integral != 0 || !s.piAt.IsZero() || s.moving != 0 {
		t.Errorf("reset left %v at %v, moving %d", s.integral, s.piAt, s.moving)
	}
}

// At the ceiling the integral is frozen: it neither learns nor is discarded,
// and learns again once the effort is back below the ceiling.
func TestAngleLoopFreezesAtTheCeiling(t *testing.T) {
	s := newWaistState(calibrated()) // kp 4, ki 4, ceiling 50%, no breakaway set
	at := time.Unix(1_000_000, 0)
	ms := func(n int) time.Time { return at.Add(time.Duration(n) * time.Millisecond) }
	s.anglePI(5, 0, at)
	s.integral = 30
	if e := s.anglePI(5, 0, ms(20)); e != 50 || s.integral != 30 {
		t.Errorf("5° off with 30 learned, at the ceiling: effort %v, integral %v; want 50 and 30, frozen", e, s.integral)
	}
	if e := s.anglePI(20, 0, ms(40)); e != 50 || s.integral != 30 {
		t.Errorf("the target 20° off, the gain alone past the ceiling: effort %v, integral %v; want 50 and 30, not discarded", e, s.integral)
	}
	e := s.anglePI(20, 17, ms(60))
	if want := 30 + 4*3*0.02; math.Abs(s.integral-want) > 1e-9 || math.Abs(e-(12+want)) > 1e-9 {
		t.Errorf("3° off, below the ceiling: effort %v, integral %v; want %v and %v, learning again", e, s.integral, 12+want, want)
	}
}

// Reaching the limit, on either side, forgets the integral: once, as the limit
// is reached, not on every reading past it.
func TestTheLimitResetsTheIntegral(t *testing.T) {
	s := newWaistState(calibrated())
	const pastLeft, pastRight = 40, 860
	s.moving, s.integral = 1, 20
	s.guard(0, 450, true)
	if s.integral != 20 {
		t.Fatalf("inside the limit the integral went to %v", s.integral)
	}
	s.guard(0, pastLeft, true)
	if s.integral != 0 {
		t.Errorf("reaching the left limit left the integral at %v", s.integral)
	}
	s.integral = 5
	s.guard(0, pastLeft, true)
	if s.integral != 5 {
		t.Errorf("a second reading past the left limit reset the integral again, to %v", s.integral)
	}
	s.moving, s.integral = -1, -20
	s.guard(0, pastRight, true)
	if s.integral != 0 {
		t.Errorf("reaching the right limit left the integral at %v", s.integral)
	}
}

// A refusal by the guard resets the integral: it must not wind up against it.
func TestARefusalResetsTheAngleLoop(t *testing.T) {
	d, _, _ := drivingDrivetrain(t, calibrated())
	steer, _ := d.steeringMotor()
	d.curvature, d.byAngle = d.curvatureLimit(), true
	at := time.Now()
	d.steeringRawLocked(steer, waistAt(30), at, true, at)
	d.steeringRawLocked(steer, waistAt(30), at.Add(20*time.Millisecond), true, at)
	if d.waist.integral <= 0 {
		t.Fatalf("3° short of its target the loop learned %v", d.waist.integral)
	}
	d.waist.fault = "the watchdog's"
	d.steeringRawLocked(steer, waistAt(30), at.Add(40*time.Millisecond), true, at)
	if d.waist.integral != 0 || d.waist.moving != 0 {
		t.Errorf("the guard refused and the integral is still %v, moving %d", d.waist.integral, d.waist.moving)
	}
}

//-------------------------------------Through the drivetrain

// withWaist puts a fresh reading into the feedback.
func withWaist(d *drivetrain, raw int) {
	d.fb.mu.Lock()
	d.fb.waistRaw, d.fb.waistAt, d.fb.waistFresh = raw, time.Now(), true
	d.fb.mu.Unlock()
}

func artitraxMotors() []MotorSpec {
	return []MotorSpec{
		{Name: "FrontLeft", NodeID: 1, Kind: "wheel", Axle: "front", Side: "left"},
		{Name: "FrontRight", NodeID: 2, Kind: "wheel", Axle: "front", Side: "right"},
		{Name: "BackLeft", NodeID: 3, Kind: "wheel", Axle: "back", Side: "left"},
		{Name: "BackRight", NodeID: 4, Kind: "wheel", Axle: "back", Side: "right"},
		{Name: "Steering", NodeID: 5, Kind: "steering"},
	}
}

func drivingDrivetrain(t *testing.T, w WaistConfig) (*drivetrain, *Traits, *Traits) {
	t.Helper()
	d := testDrivetrain(t, true)
	d.cfg.Motors = artitraxMotors()
	d.cfg.MaxWheelRPM = 120
	d.cfg.Waist = w
	d.waist = newWaistState(w)
	vehicle := &Traits{Name: "Vehicle", Kind: "vehicle", dt: d, encoderIndex: -1}
	steering := &Traits{Name: "Steering", NodeID: 5, Kind: "steering", dt: d, encoderIndex: -1}
	if w := put(t, vehicle.controlService, "gamer", "1"); w.Code != http.StatusOK {
		t.Fatalf("taking control: %d %s", w.Code, w.Body)
	}
	return d, vehicle, steering
}

// A velocity through a bend sets the wheels from the kinematics, with the
// inner wheels slower.
func TestVelocityDrivesTheWheelsThroughTheKinematics(t *testing.T) {
	d, vehicle, _ := drivingDrivetrain(t, calibrated())
	withWaist(d, 230) // 20° to the left
	if w := put(t, vehicle.velocityService, "gamer", "1.0"); w.Code != http.StatusOK {
		t.Fatalf("velocity: %d %s", w.Code, w.Body)
	}
	d.writeCycle()
	left, right := d.setpoint[1], d.setpoint[2]
	if !(left > 0 && right > left) {
		t.Errorf("at 20° left, front wheels at %.1f and %.1f RPM; want the left slower", left, right)
	}
	want := artitrax.wheelRPMs(artitrax.wheelSpeeds(1, deg(20)), 120)
	if math.Abs(left-want[frontLeft]) > 1e-6 || math.Abs(d.setpoint[4]-want[backRight]) > 1e-6 {
		t.Errorf("setpoints %v, want the kinematics' %v", d.setpoint, want)
	}
}

func TestVelocityNeedsControl(t *testing.T) {
	d := testDrivetrain(t, true)
	vehicle := &Traits{Name: "Vehicle", Kind: "vehicle", dt: d, encoderIndex: -1}
	if w := put(t, vehicle.velocityService, "gamer", "1"); w.Code != http.StatusConflict {
		t.Errorf("velocity without control: %d", w.Code)
	}
}

func TestCurvatureNeedsACalibration(t *testing.T) {
	d, vehicle, _ := drivingDrivetrain(t, defaultWaist())
	withWaist(d, 450)
	w := put(t, vehicle.curvatureService, "gamer", "0.2")
	if w.Code != http.StatusServiceUnavailable || !strings.Contains(w.Body.String(), "calibrated") {
		t.Errorf("curvature on an uncalibrated waist: %d %q, want 503 saying why", w.Code, w.Body)
	}
	if !d.helm.holds(pad) {
		t.Error("a refused curvature cost the pilot control")
	}
}

// A curvature command steers the joint towards the matching angle.
func TestCurvatureSteersTowardsItsAngle(t *testing.T) {
	d, vehicle, _ := drivingDrivetrain(t, calibrated())
	withWaist(d, 450) // straight
	k := artitrax.frontCurvature(deg(10))
	if w := put(t, vehicle.curvatureService, "gamer", formatFloat(k)); w.Code != http.StatusOK {
		t.Fatalf("curvature: %d %s", w.Code, w.Body)
	}
	d.writeCycle()
	if d.last[5] <= 0 {
		t.Errorf("asked to turn left from straight, the steering motor got %d; want positive effort", d.last[5])
	}
	// Reversed motor: the same request, the opposite raw effort.
	w := calibrated()
	w.EffortTurnsLeft = -1
	d2, v2, _ := drivingDrivetrain(t, w)
	withWaist(d2, 450)
	put(t, v2.curvatureService, "gamer", formatFloat(k))
	d2.writeCycle()
	if d2.last[5] >= 0 {
		t.Errorf("with the motor reversed, the raw effort was %d; want negative", d2.last[5])
	}
}

// The waist starts turning at once. Under the wheels' ramp, a turn asked for
// from straight reached 2.4% effort in 100 ms, below what moves the joint.
func TestCurvatureReachesItsEffortQuickly(t *testing.T) {
	d, vehicle, _ := drivingDrivetrain(t, calibrated())
	withWaist(d, 450) // straight
	k := artitrax.frontCurvature(deg(20))
	if w := put(t, vehicle.curvatureService, "gamer", formatFloat(k)); w.Code != http.StatusOK {
		t.Fatalf("curvature: %d %s", w.Code, w.Body)
	}
	for i := 0; i < 5; i++ { // 100 ms at 50 Hz
		d.writeCycle()
	}
	if want := int16(0.4 * fullScale); d.last[5] < want {
		t.Errorf("after 100 ms the steering motor is at %d; want at least %d", d.last[5], want)
	}
}

// The steering's ramp does not speed up the wheels'.
func TestTheWheelsKeepTheirRamp(t *testing.T) {
	d, _, _ := drivingDrivetrain(t, calibrated())
	withWaist(d, 450)
	wheel := &Traits{Name: "FrontLeft", NodeID: 1, Kind: "wheel", dt: d, encoderIndex: -1}
	if w := put(t, wheel.setpointService, "gamer", "60"); w.Code != http.StatusOK {
		t.Fatalf("setpoint: %d %s", w.Code, w.Body)
	}
	d.writeCycle()
	if d.last[1] != int16(d.cfg.AccelStep) {
		t.Errorf("after one cycle the wheel is at %d; want one accelStep, %d", d.last[1], d.cfg.AccelStep)
	}
}

// Past the limit the steering motor stops at once, not down the ramp.
func TestTheLimitStopsTheSteeringAtOnce(t *testing.T) {
	d, _, steering := drivingDrivetrain(t, calibrated())
	withWaist(d, 450)
	put(t, steering.setpointService, "gamer", "50") // hard left
	for i := 0; i < 20; i++ {
		d.writeCycle()
	}
	if d.last[5] <= 0 {
		t.Fatal("the steering never started")
	}
	withWaist(d, 40) // now past the left limit
	d.writeCycle()
	if d.last[5] != 0 {
		t.Errorf("past the limit, the steering motor is still at %d", d.last[5])
	}
	// Back towards straight is allowed.
	put(t, steering.setpointService, "gamer", "-30")
	d.writeCycle()
	if d.last[5] >= 0 {
		t.Errorf("effort back towards straight was refused: %d", d.last[5])
	}
}

// A chain that has jumped stops the vehicle, and the fault clears only when
// someone takes control again.
func TestAStallStopsTheVehicle(t *testing.T) {
	w := calibrated()
	w.StallMs = 30
	d, vehicle, steering := drivingDrivetrain(t, w)
	put(t, steering.setpointService, "gamer", "60")
	deadline := time.Now().Add(2 * time.Second)
	for !d.helm.stopped && time.Now().Before(deadline) {
		withWaist(d, 450) // the joint never moves
		d.writeCycle()
		time.Sleep(5 * time.Millisecond)
	}
	if !d.helm.stopped || !strings.Contains(d.helm.why, "watchdog") {
		t.Fatalf("a steering motor pushing a joint that never moves did not stop the vehicle (stopped=%v, %q)", d.helm.stopped, d.helm.why)
	}
	if e, _ := d.waist.guard(-20, 450, true); e != 0 {
		t.Error("the fault did not hold the steering")
	}
	if w := put(t, vehicle.controlService, "gamer", "1"); w.Code != http.StatusOK {
		t.Fatalf("taking control after the fault: %d %s", w.Code, w.Body)
	}
	if d.waist.fault != "" {
		t.Error("taking control again did not clear the fault")
	}
}

// Nobody in control: the steering does not return to straight by itself.
func TestReleaseLeavesTheSteeringAlone(t *testing.T) {
	d, vehicle, _ := drivingDrivetrain(t, calibrated())
	withWaist(d, 450)
	put(t, vehicle.curvatureService, "gamer", formatFloat(artitrax.frontCurvature(deg(15))))
	put(t, vehicle.controlService, "gamer", "0")
	withWaist(d, 300) // well off straight
	d.writeCycle()
	if d.byAngle || d.curvature != 0 {
		t.Error("a release left a curvature command standing")
	}
}

func formatFloat(v float64) string { return strconv.FormatFloat(v, 'f', -1, 64) }

// waistAt is the raw count for an articulation on the calibrated() waist.
func waistAt(degrees float64) int { return 450 - int(math.Round(11*degrees)) }

// Full stick aims inside the limit by its margin, so the guard never has to
// cut the loop off, and the curvature limit says so.
func TestTheLoopAimsInsideTheLimit(t *testing.T) {
	d, _, _ := drivingDrivetrain(t, calibrated()) // limit 35°, margin 2°
	if got, want := d.curvatureLimit(), artitrax.frontCurvature(deg(33)); math.Abs(got-want) > 1e-12 {
		t.Errorf("curvature limit %v; want %v, that of 33°", got, want)
	}
	steer, _ := d.steeringMotor()
	d.curvature, d.byAngle = 10, true // far tighter than the waist can turn
	if out := d.steeringRawLocked(steer, waistAt(33), time.Now(), true, time.Now()); out != 0 {
		t.Errorf("at 33° with the tightest curve asked for, the steering got %d; want 0, the target reached", out)
	}
}

// swingAtAStandstill has the gamer in control with the stick centred while the
// joint swings left at 10° a second for a second, and then rests at 10° with a
// count of noise; it returns the wheels' setpoints during the swing and at
// rest, on simulated time.
func swingAtAStandstill(t *testing.T, w WaistConfig) (swinging, resting map[int]float64, d *drivetrain) {
	t.Helper()
	d, vehicle, _ := drivingDrivetrain(t, w)
	if w := put(t, vehicle.velocityService, "gamer", "0"); w.Code != http.StatusOK {
		t.Fatalf("velocity: %d %s", w.Code, w.Body)
	}
	now := time.Unix(1_000_000, 0)
	cycle := func(deg float64) {
		now = now.Add(20 * time.Millisecond)
		d.fb.mu.Lock()
		d.fb.waistRaw, d.fb.waistAt, d.fb.waistFresh = waistAt(deg), now, true
		d.fb.mu.Unlock()
		d.helm.heard = now
		d.writeCycleAt(now)
	}
	copyOf := func() map[int]float64 {
		m := map[int]float64{}
		for k, v := range d.setpoint {
			m[k] = v
		}
		return m
	}
	for c := 0; c < 50; c++ {
		cycle(float64(c) / 5)
	}
	swinging = copyOf()
	for c := 0; c < 100; c++ {
		cycle(10 + []float64{0, 0.09, 0, -0.09}[c%4])
	}
	return swinging, copyOf(), d
}

// Standing still with the joint swinging, the wheels roll with it rather than
// hold it: held at zero, they fought the waist motor.
func TestTheWheelsFollowTheJoint(t *testing.T) {
	swinging, resting, d := swingAtAStandstill(t, calibrated())
	if !(swinging[1] < 0 && swinging[2] > 0 && swinging[3] > 0 && swinging[4] < 0) {
		t.Errorf("swinging left at a standstill, the wheels are set to %v; want the front pair turning the front half "+
			"left and the rear pair the rear half right", swinging)
	}
	want := artitrax.wheelRPMs(artitrax.wheelSpeedsMoving(0, deg(9.8), deg(10)), 120)
	if math.Abs(swinging[2]-want[frontRight]) > 0.25*math.Abs(want[frontRight]) {
		t.Errorf("front right at %.3f RPM; want about %.3f", swinging[2], want[frontRight])
	}
	for node := 1; node <= 4; node++ {
		if resting[node] != 0 {
			t.Errorf("at rest with a count of noise, wheel %d is set to %v; want 0", node, resting[node])
		}
	}
	if d.stand.mode == driving {
		t.Error("at rest the vehicle is still driving; want the standstill holding it")
	}
}

// ignoreJointRate is the wheels as they were: held while the joint swings.
func TestIgnoreJointRate(t *testing.T) {
	w := calibrated()
	w.IgnoreJointRate = true
	swinging, _, _ := swingAtAStandstill(t, w)
	for node := 1; node <= 4; node++ {
		if swinging[node] != 0 {
			t.Errorf("with ignoreJointRate, wheel %d is set to %v while the joint swings; want 0", node, swinging[node])
		}
	}
}

// A configuration still carrying the P loop's settings, or the deadzone's, is
// told they are gone.
func TestStaleWaistKeys(t *testing.T) {
	traits := []byte(`{"waist": {"straightCount": 471, "minEffortPercent": 22, "takeUpEffortPercent": 40, "deadzonePercent": 3}}`)
	if got := staleWaistKeys(traits); strings.Join(got, ",") != "deadzonePercent,minEffortPercent,takeUpEffortPercent" {
		t.Errorf("stale keys %v", got)
	}
	if got := staleWaistKeys([]byte(`{"commandHz": 50}`)); len(got) != 0 {
		t.Errorf("with no waist, stale keys %v", got)
	}
}
