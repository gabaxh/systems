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
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/sdoque/mbaigo/components"
)

// The controller's scale is the one fact the reference implementation states
// outright: 120 RPM is 0x7800. Everything the vehicle does rides on it.
func TestWheelScale(t *testing.T) {
	d := &drivetrain{
		cfg:      LoaderConfig{MaxWheelRPM: 120},
		setpoint: map[int]float64{1: 120, 2: -120, 3: 0},
	}
	wheel := func(node int) MotorSpec { return MotorSpec{NodeID: node, Kind: "wheel"} }

	if got := d.rawFor(wheel(1)); got != 30720 {
		t.Errorf("120 RPM = %d; want 30720 (0x7800)", got)
	}
	if got := d.rawFor(wheel(2)); got != -30720 {
		t.Errorf("-120 RPM = %d; want -30720", got)
	}
	if got := d.rawFor(wheel(3)); got != 0 {
		t.Errorf("0 RPM = %d; want 0", got)
	}
}

// A 12 inch wheel turns 17.41 times a minute at 1 km/h. The command that
// carries is what the vehicle will actually be asked for first.
func TestOneKilometrePerHour(t *testing.T) {
	const rpmAt1kmh = 17.405 // 1 km/h / (pi * 0.3048 m) * 60
	d := &drivetrain{
		cfg:      LoaderConfig{MaxWheelRPM: 120},
		setpoint: map[int]float64{1: rpmAt1kmh},
	}
	got := d.rawFor(MotorSpec{NodeID: 1, Kind: "wheel"})
	if got < 4450 || got > 4460 {
		t.Errorf("1 km/h = %d counts; want about 4456", got)
	}
}

// Steering is an effort, not an angle, so its setpoint is a percentage of what
// the drive can push. Full deflection must reach full scale and no further.
func TestSteeringIsPercentOfEffort(t *testing.T) {
	d := &drivetrain{setpoint: map[int]float64{5: 100, 6: -50}}
	if got := d.rawFor(MotorSpec{NodeID: 5, Kind: "steering"}); got != 30720 {
		t.Errorf("100%% = %d; want 30720", got)
	}
	if got := d.rawFor(MotorSpec{NodeID: 6, Kind: "steering"}); got != -15360 {
		t.Errorf("-50%% = %d; want -15360", got)
	}
}

// Nothing may leave the integer range the drive accepts, whatever is asked for.
func TestClampNeverOverflows(t *testing.T) {
	for _, v := range []float64{1e9, -1e9, 40000, -40000} {
		got := clampToScale(v)
		if got > fullScale || got < -fullScale {
			t.Errorf("clampToScale(%v) = %d, outside the drive's range", v, got)
		}
	}
}

// The rate limiter walks towards the request. Braking gets the larger step,
// because stopping should never be slower than starting.
func TestRateLimit(t *testing.T) {
	cases := []struct {
		name                string
		last, desired, want int16
		accel, brake        int16
	}{
		{"from rest, capped by the accel step", 0, 10000, 30, 30, 100},
		{"already there, no movement", 500, 500, 500, 30, 100},
		{"small step taken whole", 100, 110, 110, 30, 100},
		{"slowing uses the brake step", 1000, 0, 900, 30, 100},
		{"reversing uses the brake step", 1000, -1000, 900, 30, 100},
	}
	for _, c := range cases {
		if got := rateLimit(c.last, c.desired, c.accel, c.brake); got != c.want {
			t.Errorf("%s: rateLimit(%d,%d) = %d; want %d", c.name, c.last, c.desired, got, c.want)
		}
	}
}

// A command far outside the range must not wrap around into full reverse.
func TestRateLimitDoesNotOverflow(t *testing.T) {
	if got := rateLimit(32760, 32767, 30, 100); got < 0 {
		t.Errorf("rateLimit near the int16 ceiling wrapped to %d", got)
	}
	if got := rateLimit(-32760, -32767, 30, 100); got > 0 {
		t.Errorf("rateLimit near the int16 floor wrapped to %d", got)
	}
}

// The encoder decoding is transcribed from the artitrax bridge, so it is worth
// pinning: a wrong scale here becomes a map of the wrong size.
func TestDecodeWheelScaling(t *testing.T) {
	// One full output revolution is 20 x 4096 = 81 920 counts.
	var f canFrame
	f.ID = encoderBaseID + 1 // front right: index 1, not sign-flipped
	f.DLC = 6
	counts := uint32(81920)
	f.Data[0] = byte(counts)
	f.Data[1] = byte(counts >> 8)
	f.Data[2] = byte(counts >> 16)

	// One output revolution per second is 81 920 counts per second, so 409.6
	// counts fall in each 5 ms window. 410 is therefore just over 60 RPM.
	raw := int16(410)
	f.Data[4] = byte(uint16(raw))
	f.Data[5] = byte(uint16(raw) >> 8)

	i, r, ok := decodeWheel(f)
	if !ok {
		t.Fatal("frame rejected")
	}
	if i != 1 {
		t.Errorf("index = %d, want 1", i)
	}
	if math.Abs(r.revolutions-1.0) > 1e-9 {
		t.Errorf("revolutions = %v, want 1.0", r.revolutions)
	}
	if math.Abs(r.rpm-60) > 0.1 {
		t.Errorf("rpm = %v, want about 60", r.rpm)
	}
}

// The left-hand encoders are mounted facing the other way. If that sign is
// dropped, a vehicle driving straight reads as one spinning on the spot.
func TestDecodeWheelFlipsTheLeftSide(t *testing.T) {
	mk := func(id uint32) wheelReading {
		var f canFrame
		f.ID = id
		f.DLC = 6
		f.Data[0] = 0x00
		f.Data[1] = 0x40 // 16384 counts
		raw := int16(1000)
		f.Data[4] = byte(uint16(raw))
		f.Data[5] = byte(uint16(raw) >> 8)
		_, r, _ := decodeWheel(f)
		return r
	}
	left := mk(encoderBaseID + 0)  // front left
	right := mk(encoderBaseID + 1) // front right
	if left.rpm >= 0 || right.rpm <= 0 {
		t.Errorf("left rpm %v and right rpm %v should have opposite signs", left.rpm, right.rpm)
	}
	if left.rpm != -right.rpm {
		t.Errorf("left %v is not the negation of right %v", left.rpm, right.rpm)
	}
}

func TestDecodeWheelRejectsForeignFrames(t *testing.T) {
	var f canFrame
	f.ID = 0x601 // a motor command, not an encoder
	f.DLC = 8
	if _, _, ok := decodeWheel(f); ok {
		t.Error("a motor command frame was decoded as an encoder reading")
	}
}

// A steering motor has no wheel encoder and a wheel has no articulation sensor.
// Registering a service that cannot answer is worse than not offering it.
func TestServicesForMotorKind(t *testing.T) {
	configured := []components.Service{
		{Definition: "setpoint", SubPath: "setpoint"},
		{Definition: "speed", SubPath: "speed"},
		{Definition: "travel", SubPath: "travel"},
		{Definition: "waist", SubPath: "waist"},
	}
	wheel := servicesFor("wheel", configured)
	if _, ok := wheel["waist"]; ok {
		t.Error("a wheel offers the articulation sensor")
	}
	if _, ok := wheel["speed"]; !ok {
		t.Error("a wheel does not offer its measured speed")
	}
	steering := servicesFor("steering", configured)
	if _, ok := steering["speed"]; ok {
		t.Error("the steering motor offers a wheel speed it cannot measure")
	}
	if _, ok := steering["waist"]; !ok {
		t.Error("the steering motor does not offer the articulation sensor")
	}
}

// A stale encoder must not be served as a measurement.
func TestSpeedServiceRefusesStaleFeedback(t *testing.T) {
	fb := newFeedback(100 * time.Millisecond)
	fb.wheels[0] = wheelReading{rpm: 42, at: time.Now().Add(-time.Second)}
	tr := &Traits{Name: "FrontLeft", Kind: "wheel", encoderIndex: 0,
		dt: &drivetrain{fb: fb}}

	w := httptest.NewRecorder()
	tr.speedService(w, httptest.NewRequest(http.MethodGet, "/speed", nil))
	if w.Code != http.StatusServiceUnavailable {
		t.Errorf("a one-second-old encoder reading was served as current (%d)", w.Code)
	}
}

// The encoder's counter wraps every 204.8 revolutions; travel must not. A
// wheel driving forward across the wrap keeps counting up, on both sides.
func TestTravelIsUnwrapped(t *testing.T) {
	near := uint32(1<<24 - 8192) // 0.1 of a revolution before the wrap
	fb := newFeedback(time.Second)
	a := fb.unwrapLocked(1, wheelReading{count: near})
	b := fb.unwrapLocked(1, wheelReading{count: 8192})
	if d := b.revolutions - a.revolutions; math.Abs(d-0.2) > 1e-9 {
		t.Errorf("right wheel across the wrap moved %v revolutions, want 0.2", d)
	}
	a = fb.unwrapLocked(0, wheelReading{count: 8192})
	b = fb.unwrapLocked(0, wheelReading{count: near})
	if d := b.revolutions - a.revolutions; math.Abs(d-0.2) > 1e-9 {
		t.Errorf("left wheel across the wrap moved %v revolutions, want 0.2", d)
	}
	// And a thousand revolutions later it is still counting.
	fb = newFeedback(time.Second)
	count := uint32(0)
	var last wheelReading
	for step := 0; step < 1000*10; step++ {
		count = (count + 8192) & (1<<24 - 1) // 0.1 revolution per frame
		last = fb.unwrapLocked(1, wheelReading{count: count})
	}
	if math.Abs(last.revolutions-1000) > 1e-6 {
		t.Errorf("after 10 000 frames of 0.1 revolution: %v, want 1000", last.revolutions)
	}
	if got := distanceForm(last, 1.335).Value; math.Abs(got-1335) > 1e-6 {
		t.Errorf("1000 revolutions of a 1.335 m wheel = %v m, want 1335", got)
	}
}

// frame feeds one encoder frame through the unwrapping as the listener does.
func frame(fb *feedback, i int, count uint32, at time.Time) wheelReading {
	fb.mu.Lock()
	defer fb.mu.Unlock()
	r := fb.unwrapLocked(i, wheelReading{count: count, at: at})
	fb.wheels[i] = r
	return r
}

// An encoder that went quiet and came back counting from somewhere else —
// reset, most likely — leaves travel where it was rather than jumping.
func TestTravelCarriesOnAcrossASilence(t *testing.T) {
	fb := newFeedback(time.Second)
	start := time.Unix(1_000_000, 0)
	var r wheelReading
	for k := 0; k <= 10; k++ { // a revolution, in ten frames
		r = frame(fb, 1, uint32(k*8192), start.Add(time.Duration(k)*50*time.Millisecond))
	}
	before := r.revolutions
	// Two seconds of nothing, then the counter at a third of its range.
	back := start.Add(2550 * time.Millisecond)
	r = frame(fb, 1, 1<<22, back)
	if r.revolutions != before {
		t.Errorf("after the silence travel went from %v to %v revolutions; want it unchanged", before, r.revolutions)
	}
	r = frame(fb, 1, 1<<22+8192, back.Add(50*time.Millisecond))
	if d := r.revolutions - before; math.Abs(d-0.1) > 1e-9 {
		t.Errorf("the first frame after it moved travel %v revolutions; want 0.1", d)
	}
	// A short gap is not a silence: a lost frame or two still differences.
	r = frame(fb, 1, 1<<22+3*8192, back.Add(200*time.Millisecond))
	if d := r.revolutions - before; math.Abs(d-0.3) > 1e-9 {
		t.Errorf("across a 150 ms gap travel is %v revolutions on; want 0.3", d)
	}
}

// The wheels that have said nothing for a second, or ever, are the ones to
// restart.
func TestQuietWheels(t *testing.T) {
	fb := newFeedback(time.Second)
	now := time.Now()
	fb.wheels[0].at = now.Add(-100 * time.Millisecond)
	fb.wheels[1].at = now.Add(-1500 * time.Millisecond)
	fb.wheels[3].at = now
	got := fb.quietWheels(now)
	if len(got) != 2 || got[0] != "FrontRight" || got[1] != "BackLeft" {
		t.Errorf("quiet wheels %v; want FrontRight (1.5 s) and BackLeft (never)", got)
	}
	for i := range fb.wheels {
		fb.wheels[i].at = now
	}
	if got := fb.quietWheels(now); len(got) != 0 {
		t.Errorf("with every wheel reporting, %v are quiet", got)
	}
}

// stalledWheel is a drivetrain with control taken, FrontLeft asked for target
// RPM and given effort percent of full scale, its encoder fresh and reporting
// rpm.
func stalledWheel(t *testing.T, target, effort, rpm float64) (*drivetrain, *Traits, map[int]int16) {
	t.Helper()
	d := testDrivetrain(t, true)
	vehicle := &Traits{Name: "Vehicle", Kind: "vehicle", dt: d, encoderIndex: -1}
	if w := put(t, vehicle.controlService, "gamer", "1"); w.Code != http.StatusOK {
		t.Fatalf("taking control: %d %s", w.Code, w.Body)
	}
	d.setpoint[1] = target
	d.fb.wheels[0] = wheelReading{rpm: rpm, at: time.Now()}
	return d, vehicle, map[int]int16{1: int16(effort / 100 * fullScale)}
}

// A wheel driven hard that does not turn stops the vehicle after
// wheelStallMs, and not before; taking control again clears the fault.
func TestAStalledWheelStopsTheVehicle(t *testing.T) {
	d, vehicle, sent := stalledWheel(t, 60, 50, 0)
	start := time.Now()
	d.watchWheelsLocked(start, sent)
	d.watchWheelsLocked(start.Add(1900*time.Millisecond), sent)
	if d.helm.stopped {
		t.Fatal("stopped after 1.9 s; the watchdog allows 2")
	}
	d.watchWheelsLocked(start.Add(2000*time.Millisecond), sent)
	if !d.helm.stopped || !strings.Contains(d.helm.why, "wheel watchdog") || !strings.Contains(d.wheelFault, "FrontLeft") {
		t.Fatalf("after 2 s of a stalled wheel: stopped %v (%q), fault %q", d.helm.stopped, d.helm.why, d.wheelFault)
	}
	if w := put(t, vehicle.controlService, "gamer", "1"); w.Code != http.StatusOK {
		t.Fatalf("taking control after the fault: %d %s", w.Code, w.Body)
	}
	if d.wheelFault != "" {
		t.Error("taking control again did not clear the wheel fault")
	}
}

// What is not a stall: a wheel that turns, one driven gently, one asked to
// hold still on a hill, and one whose encoder has gone quiet.
func TestWhatIsNotAStall(t *testing.T) {
	for _, c := range []struct {
		why                 string
		target, effort, rpm float64
		stale               bool
	}{
		{"turning", 60, 50, 30, false},
		{"driven gently", 20, 17, 0, false},
		{"holding at zero on a hill", 0, 45, 0, false},
		{"an encoder gone quiet", 60, 50, 0, true},
	} {
		d, _, sent := stalledWheel(t, c.target, c.effort, c.rpm)
		if c.stale {
			d.fb.wheels[0].at = time.Now().Add(-5 * time.Second)
		}
		start := time.Now()
		for ms := 0; ms <= 5000; ms += 20 {
			d.watchWheelsLocked(start.Add(time.Duration(ms)*time.Millisecond), sent)
		}
		if d.helm.stopped {
			t.Errorf("%s: the watchdog stopped the vehicle: %s", c.why, d.wheelFault)
		}
	}
}
