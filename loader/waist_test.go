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

func takingUp() WaistConfig {
	w := calibrated() // lower counts are left
	w.TakeUpEffortPercent = 40
	return w
}

func TestTakeUp(t *testing.T) {
	start := time.Now()
	at := func(ms int) time.Time { return start.Add(time.Duration(ms) * time.Millisecond) }
	s := newWaistState(takingUp())
	step := func(effort float64, raw, ms int, want float64, ended bool, what string) {
		t.Helper()
		got, note := s.takeUp(effort, raw, true, at(ms), true)
		if got != want || (note != "") != ended {
			t.Errorf("%s: takeUp(%v, count %d) = %v, %q; want %v, ended %v", what, effort, raw, got, note, want, ended)
		}
	}
	step(4, 450, 0, 40, false, "from start-up, a small request left")
	step(4, 448, 20, 40, false, "two counts is noise")
	step(4, 446, 40, 4, true, "four counts left: the chain is tight")
	step(4, 446, 60, 4, false, "engaged left")
	step(0, 446, 80, 0, false, "idle")
	step(4, 446, 100, 4, false, "idle kept the side")
	step(-4, 446, 120, -40, false, "a reversal takes up again")
	step(-4, 450, 140, -4, true, "four counts right")
	step(4, 450, 160, 40, false, "back left")
	step(4, 456, 180, 40, false, "moving the wrong way does not end it")
	step(4, 456, 470, 4, true, "takeUpMaxMs ends it")

	off := newWaistState(calibrated())
	if got, note := off.takeUp(4, 450, true, at(0), true); got != 4 || note != "" {
		t.Errorf("with takeUpEffortPercent 0: %v, %q; want the effort untouched", got, note)
	}

	// Without the boost, the side is still followed.
	s = newWaistState(takingUp())
	if got, note := s.takeUp(4, 450, true, at(0), false); got != 4 || note != "" {
		t.Errorf("without the boost: %v, %q; want the effort untouched", got, note)
	}
	s.takeUp(4, 446, true, at(40), false)
	if got, _ := s.takeUp(4, 446, true, at(60), true); got != 4 {
		t.Errorf("after the joint moved left under effort, the loop took up the slack again: %v", got)
	}
}

// The push that crossed the slack goes as soon as the joint moves, not down
// the ramp: through the ramp it outlived the slack, and the joint lunged.
func TestTakeUpDropsAtOnce(t *testing.T) {
	d, vehicle, _ := drivingDrivetrain(t, takingUp())
	withWaist(d, 450)
	put(t, vehicle.curvatureService, "gamer", formatFloat(artitrax.frontCurvature(deg(4))))
	for i := 0; i < 5; i++ {
		d.writeCycle()
	}
	if d.last[5] < int16(0.35*fullScale) {
		t.Fatalf("asked for 4° from straight, the take-up reached only %d", d.last[5])
	}
	withWaist(d, 446) // the joint moves: the chain is tight
	d.writeCycle()
	if d.last[5] <= 0 || d.last[5] > fullScale*16/100 {
		t.Errorf("once the joint moved, the steering motor is at %d; want the loop's 4%% a degree at once", d.last[5])
	}
}

// Steering by effort gets what the stick says.
func TestNoTakeUpByEffort(t *testing.T) {
	d, _, steering := drivingDrivetrain(t, takingUp())
	withWaist(d, 450)
	put(t, steering.setpointService, "gamer", "10")
	for i := 0; i < 5; i++ {
		d.writeCycle()
	}
	if want := int16(0.1 * fullScale); d.last[5] != want {
		t.Errorf("a 10%% setpoint gave %d; want %d, with no take-up", d.last[5], want)
	}
}

// The limit applies to the take-up as to anything else.
func TestTakeUpStopsAtTheLimit(t *testing.T) {
	d, vehicle, _ := drivingDrivetrain(t, takingUp())
	put(t, vehicle.curvatureService, "gamer", formatFloat(d.curvatureLimit())) // as far left as allowed
	withWaist(d, 64)                                                           // past the left limit
	d.writeCycle()
	withWaist(d, 80) // back inside, not yet by the margin; the loop asks for more left
	d.writeCycle()
	if d.last[5] != 0 {
		t.Errorf("within the margin of the left limit, a take-up to the left gave %d", d.last[5])
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

func TestAngleLoop(t *testing.T) {
	s := newWaistState(calibrated())
	if e := s.angleEffort(10, 0); e != 40 {
		t.Errorf("10° to the left of the target: effort %v, want 40", e)
	}
	if e := s.angleEffort(-30, 0); e != -50 {
		t.Errorf("30° off: effort %v, want the -50 ceiling", e)
	}
	if e := s.angleEffort(10, 9.8); e != 0 {
		t.Errorf("inside the deadband: effort %v, want 0", e)
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

func TestTheAngleLoopOvercomesBreakaway(t *testing.T) {
	w := calibrated()
	w.MinEffortPercent = 20
	s := newWaistState(w)
	for _, c := range []struct {
		target, measured, want float64
	}{
		{0.2, 0, 0},  // inside the deadband
		{1, 0, 20},   // 4% by the gain, lifted to the breakaway
		{-1, 0, -20}, // the same, to the right
		{8, 0, 32},   // above the breakaway, the gain's
		{30, 0, 50},  // clamped to the ceiling
		{-30, 0, -50},
	} {
		if got := s.angleEffort(c.target, c.measured); math.Abs(got-c.want) > 1e-9 {
			t.Errorf("angleEffort(%v, %v) = %v; want %v", c.target, c.measured, got, c.want)
		}
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

// Once inside the deadband the loop lets go, and starts again only at twice
// the deadband: a count of noise at its edge is not a kick.
func TestTheDeadbandHasHysteresis(t *testing.T) {
	s := newWaistState(calibrated()) // 0.5° deadband, 4% a degree
	for _, c := range []struct {
		measured, want float64
		why            string
	}{
		{9.0, 4, "1° short, outside the deadband"},
		{9.6, 0, "inside the deadband"},
		{9.2, 0, "0.8° short after settling: inside twice the deadband"},
		{8.9, 4.4, "1.1° short: past twice the deadband, so the loop starts again"},
		{9.3, 2.8, "0.7° short while moving: the deadband is 0.5° again"},
	} {
		if got := s.angleEffort(10, c.measured); math.Abs(got-c.want) > 1e-9 {
			t.Errorf("%s: effort %v; want %v", c.why, got, c.want)
		}
	}
}

// waistAt is the raw count for an articulation on the calibrated() waist.
func waistAt(degrees float64) int { return 450 - int(math.Round(11*degrees)) }

// Correcting a small overshoot turns the other way from the side the chain is
// loaded on, and is not boosted to the take-up's effort; a real move is.
func TestOnlyRealMovesAreTakenUp(t *testing.T) {
	w := calibrated()
	w.TakeUpEffortPercent = 40
	for _, c := range []struct {
		joint, want float64
		why         string
	}{
		{11, 4, "1° past a 10° target"},
		{15, 40, "5° past it"},
	} {
		d, _, _ := drivingDrivetrain(t, w)
		steer, _ := d.steeringMotor()
		d.curvature, d.byAngle = artitrax.frontCurvature(deg(10)), true
		d.waist.engaged = 1 // the chain is loaded on the left
		out := d.steeringRawLocked(steer, waistAt(c.joint), true, time.Now())
		if got := -float64(out) / fullScale * 100; math.Abs(got-c.want) > 0.01 {
			t.Errorf("%s: effort %.2f%% to the right; want %v%%", c.why, got, c.want)
		}
	}
}

// Full stick aims inside the limit by its margin, so the guard never has to
// cut the loop off, and the curvature limit says so.
func TestTheLoopAimsInsideTheLimit(t *testing.T) {
	d, _, _ := drivingDrivetrain(t, calibrated()) // limit 35°, margin 2°
	if got, want := d.curvatureLimit(), artitrax.frontCurvature(deg(33)); math.Abs(got-want) > 1e-12 {
		t.Errorf("curvature limit %v; want %v, that of 33°", got, want)
	}
	steer, _ := d.steeringMotor()
	d.curvature, d.byAngle = 10, true // far tighter than the waist can turn
	if out := d.steeringRawLocked(steer, waistAt(33), true, time.Now()); out != 0 {
		t.Errorf("at 33° with the tightest curve asked for, the steering got %d; want 0, the target reached", out)
	}
}

// When the angle loop turns the other way, the effort it was applying stops
// at once; a person steering by effort still gets the ramp.
func TestTheLoopCutsItsEffortWhenItTurns(t *testing.T) {
	d, _, _ := drivingDrivetrain(t, calibrated())
	steer, _ := d.steeringMotor()
	d.curvature, d.byAngle = artitrax.frontCurvature(deg(10)), true
	d.last[steer.NodeID] = int16(0.4 * fullScale) // pushing left
	d.steeringRawLocked(steer, waistAt(12), true, time.Now())
	if l := d.last[steer.NodeID]; l != 0 {
		t.Errorf("2° past its target the loop's old effort is still %d; want it cut to 0", l)
	}

	d2, _, s2 := drivingDrivetrain(t, calibrated())
	withWaist(d2, waistAt(12))
	put(t, s2.setpointService, "gamer", "-30")
	d2.last[steer.NodeID] = int16(0.4 * fullScale)
	d2.steeringRawLocked(steer, waistAt(12), true, time.Now())
	if l := d2.last[steer.NodeID]; l != int16(0.4*fullScale) {
		t.Errorf("steering by effort, the ramp's memory became %d; want it left for the ramp", l)
	}
}
