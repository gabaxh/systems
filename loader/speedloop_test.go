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
	"testing"
	"time"
)

// wheelOnASlope is a wheel motor as the loop sees it: open loop, full scale
// turns it at 120 RPM on the flat, a slope takes load RPM off that (negative
// downhill), and the wheel answers with a fifth of a second's lag.
type wheelOnASlope struct {
	load  float64 // RPM
	rpm   float64
	rev   float64
	clock time.Duration // since the simulation started
}

func (w *wheelOnASlope) advance(commandRPM, dt float64) {
	command := math.Max(-120, math.Min(120, commandRPM)) // the drive's full scale
	const tau = 0.2
	w.rpm += (command - w.load - w.rpm) * dt / tau
	w.rev += w.rpm / 60 * dt
}

// drive runs the loop against the wheel for the given time: the loop at 50 Hz,
// the encoder reporting every 50 ms, as on the vehicle. It returns the wheel's
// speed at the end and the fastest it went.
func drive(s *speedLoop, cfg SpeedLoopConfig, w *wheelOnASlope, target func(time.Duration) float64, d time.Duration) (final, peak float64) {
	start := time.Unix(1_000_000, 0)
	var frame wheelReading
	command := 0.0
	for end := w.clock + d; w.clock < end; w.clock += time.Millisecond {
		ms := w.clock
		w.advance(command, 0.001)
		peak = math.Max(peak, w.rpm)
		if ms%(50*time.Millisecond) == 0 {
			frame = wheelReading{revolutions: w.rev, rpm: w.rpm, at: start.Add(ms)}
		}
		if ms%(20*time.Millisecond) == 0 {
			s.ramp(target(ms), 0, 150, 500)
			if cfg.OpenLoop {
				command = s.reference
			} else {
				command, _ = s.step(cfg, frame, true, 120)
			}
		}
	}
	return w.rpm, peak
}

func steady(rpm float64) func(time.Duration) float64 {
	return func(time.Duration) float64 { return rpm }
}

// The point of the loop: the same target, the same speed, whatever the slope.
// Open loop, the slope is the error.
func TestTheSpeedHoldsOnASlope(t *testing.T) {
	for _, c := range []struct {
		name string
		load float64
	}{{"flat", 0}, {"uphill", 15}, {"downhill", -15}} {
		w := &wheelOnASlope{load: c.load}
		got, _ := drive(&speedLoop{}, defaultSpeedLoop(), w, steady(40), 8*time.Second)
		if math.Abs(got-40) > 0.8 {
			t.Errorf("%s: the wheel settled at %.1f RPM; want 40 within 2%%", c.name, got)
		}

		open := defaultSpeedLoop()
		open.OpenLoop = true
		w = &wheelOnASlope{load: c.load}
		got, _ = drive(&speedLoop{}, open, w, steady(40), 8*time.Second)
		if want := 40 - c.load; math.Abs(got-want) > 0.8 {
			t.Errorf("%s, open loop: the wheel settled at %.1f RPM; want %.1f, the slope's error", c.name, got, want)
		}
	}
}

// Uphill at full scale the motor can do no more, and an integral that kept
// growing would throw the vehicle forward when the slope ended.
func TestNoWindupAtFullScale(t *testing.T) {
	w := &wheelOnASlope{load: 30}
	s := &speedLoop{}
	cfg := defaultSpeedLoop()
	drive(s, cfg, w, steady(110), 6*time.Second)
	if w.rpm > 95 {
		t.Fatalf("uphill at full scale the wheel turned %.1f RPM; the model allows 90", w.rpm)
	}
	w.load = 0 // over the top
	_, peak := drive(s, cfg, w, steady(110), 4*time.Second)
	if peak > 110*1.05 {
		t.Errorf("over the top the wheel reached %.1f RPM; want no more than 5%% past 110", peak)
	}
}

// A wheel whose encoder goes quiet is driven as it was before the loop, and
// what the loop had learned is forgotten.
func TestAQuietEncoderIsOpenLoop(t *testing.T) {
	cfg := defaultSpeedLoop()
	s := &speedLoop{}
	w := &wheelOnASlope{load: 15}
	drive(s, cfg, w, steady(40), 4*time.Second)
	if s.integral < 10 {
		t.Fatalf("uphill the integral is %.1f; the loop has not learned the slope", s.integral)
	}
	command, note := s.step(cfg, wheelReading{}, false, 120)
	if command != s.reference || command != 40 {
		t.Errorf("with no encoder the command is %.1f; want the target, 40", command)
	}
	if note == "" {
		t.Error("falling back to open loop said nothing")
	}
	if s.integral != 0 {
		t.Errorf("the integral %.1f survived the encoder going quiet", s.integral)
	}
	if _, again := s.step(cfg, wheelReading{}, false, 120); again != "" {
		t.Errorf("a second quiet cycle said %q again", again)
	}
}

// A wrong reading costs at most maxCorrectionRPM.
func TestTheCorrectionIsBounded(t *testing.T) {
	cfg := defaultSpeedLoop()
	s := &speedLoop{}
	s.ramp(20, 20, 150, 500)
	at := time.Unix(1_000_000, 0)
	// The encoder says the wheel is standing still, frame after frame.
	command := 0.0
	for i := 0; i < 200; i++ {
		command, _ = s.step(cfg, wheelReading{revolutions: 0, at: at.Add(time.Duration(i) * 50 * time.Millisecond)}, true, 120)
	}
	if want := 20 + cfg.MaxCorrectionRPM; command > want+1e-9 {
		t.Errorf("a wheel that never moves got %.1f RPM; want no more than %.1f", command, want)
	}
}

//-------------------------------------Through the drivetrain

// withWheel puts a fresh reading for one wheel into the feedback.
func withWheel(d *drivetrain, index int, revolutions float64, at time.Time) {
	d.fb.mu.Lock()
	d.fb.wheels[index] = wheelReading{revolutions: revolutions, at: at}
	d.fb.mu.Unlock()
}

// Driving by velocity with the wheel slower than asked, the loop pushes harder
// than the open-loop command would.
func TestVelocityClosesTheLoop(t *testing.T) {
	d, vehicle, _ := drivingDrivetrain(t, calibrated())
	d.cfg.SpeedLoop = defaultSpeedLoop()
	withWaist(d, 450) // straight
	if w := put(t, vehicle.velocityService, "gamer", "0.5"); w.Code != http.StatusOK {
		t.Fatalf("velocity: %d %s", w.Code, w.Body)
	}
	target := d.cfg.Geometry.wheelRPMs(d.cfg.Geometry.wheelSpeeds(0.5, 0), 120)[frontLeft]
	// Ramp up with the encoder saying nothing: that is open loop, the target.
	for i := 0; i < 100; i++ {
		d.writeCycle()
	}
	if want := clampToScale(target * rpmToCommand); d.last[1] != want {
		t.Fatalf("with no encoder the wheel got %d; want the open-loop %d", d.last[1], want)
	}
	// The encoder reports the wheel at half its target.
	now := time.Now()
	withWheel(d, 0, 0, now.Add(-50*time.Millisecond))
	d.writeCycle()
	withWheel(d, 0, target/2/60*0.05, now)
	d.writeCycle()
	if d.last[1] <= clampToScale(target*rpmToCommand) {
		t.Errorf("a wheel at half its speed got %d; want more than the open-loop %d", d.last[1], clampToScale(target*rpmToCommand))
	}
}

// A stop is zero at once, and the loop starts again from nothing.
func TestAStopResetsTheLoop(t *testing.T) {
	d, vehicle, _ := drivingDrivetrain(t, calibrated())
	d.cfg.SpeedLoop = defaultSpeedLoop()
	withWaist(d, 450)
	put(t, vehicle.velocityService, "gamer", "0.5")
	for i := 0; i < 50; i++ {
		d.writeCycle()
	}
	d.loopFor(1).integral = 30
	put(t, vehicle.stopService, "gamer", "1")
	d.writeCycle()
	if d.last[1] != 0 {
		t.Errorf("after a stop the wheel got %d; want 0", d.last[1])
	}
	if s := d.loopFor(1); s.integral != 0 || s.active {
		t.Errorf("after a stop the loop kept integral %.1f, active %v", s.integral, s.active)
	}
}

// A wheel commanded directly is open loop, whatever its encoder says.
func TestADirectSetpointIsOpenLoop(t *testing.T) {
	d, _, _ := drivingDrivetrain(t, calibrated())
	d.cfg.SpeedLoop = defaultSpeedLoop()
	wheel := &Traits{Name: "FrontLeft", NodeID: 1, Kind: "wheel", dt: d, encoderIndex: 0}
	if w := put(t, wheel.setpointService, "gamer", "20"); w.Code != http.StatusOK {
		t.Fatalf("setpoint: %d %s", w.Code, w.Body)
	}
	now := time.Now()
	for i := 0; i < 60; i++ { // standing still, frame after frame
		withWheel(d, 0, 0, now.Add(time.Duration(i-60)*50*time.Millisecond))
		d.writeCycle()
	}
	if want := clampToScale(20 * rpmToCommand); d.last[1] != want {
		t.Errorf("a direct 20 RPM sent %d; want the open-loop %d", d.last[1], want)
	}
}

// With the loop off, a velocity drives the wheels exactly as before it.
func TestOpenLoopIsTheOldRamp(t *testing.T) {
	d, vehicle, _ := drivingDrivetrain(t, calibrated())
	d.cfg.SpeedLoop = defaultSpeedLoop()
	d.cfg.SpeedLoop.OpenLoop = true
	withWaist(d, 450)
	put(t, vehicle.velocityService, "gamer", "1.0")
	d.writeCycle()
	if d.last[1] != int16(d.cfg.AccelStep) {
		t.Errorf("after one cycle the wheel is at %d; want one accelStep, %d", d.last[1], d.cfg.AccelStep)
	}
}

// Closed, the ramp is on the target, at the same rate.
func TestTheLoopKeepsTheRamp(t *testing.T) {
	d, vehicle, _ := drivingDrivetrain(t, calibrated())
	d.cfg.SpeedLoop = defaultSpeedLoop()
	withWaist(d, 450)
	put(t, vehicle.velocityService, "gamer", "1.0")
	d.writeCycle()
	if got := d.loopFor(1).reference; math.Abs(got-float64(d.cfg.AccelStep)/rpmToCommand) > 1e-9 {
		t.Errorf("after one cycle the target is %.3f RPM; want one accelStep, %.3f", got, float64(d.cfg.AccelStep)/rpmToCommand)
	}
}

func TestSpeedLoopDefaults(t *testing.T) {
	var s SpeedLoopConfig
	applySpeedLoopDefaults(&s)
	if s != defaultSpeedLoop() {
		t.Errorf("an empty configuration became %+v; want the defaults %+v", s, defaultSpeedLoop())
	}
	s = SpeedLoopConfig{IntegralGainPerSecond: 2}
	applySpeedLoopDefaults(&s)
	if s.ProportionalGain != 0 || s.IntegralGainPerSecond != 2 {
		t.Errorf("an integral-only loop became %+v", s)
	}
}
