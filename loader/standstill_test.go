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

// hillWheel is a wheel on a slope: the slope pulls it back with load RPM's
// worth of command, and the drivetrain's friction — the gearbox, the drive —
// holds up to friction RPM's worth before it turns at all. With friction above
// the load the vehicle stays put unpowered; below it, it rolls back.
type hillWheel struct {
	load, friction float64
	rpm, rev       float64
}

func (w *hillWheel) advance(commandRPM, dt float64) {
	net := math.Max(-120, math.Min(120, commandRPM)) - w.load
	drive := 0.0
	if math.Abs(net) > w.friction {
		drive = net - math.Copysign(w.friction, net)
	}
	w.rpm += (drive - w.rpm) * dt / 0.2
	w.rev += w.rpm / 60 * dt
}

// hill is the loader standing on a slope with the stick centred, run on
// simulated time: the loop at 50 Hz, the encoders every 50 ms.
type hill struct {
	t      *testing.T
	d      *drivetrain
	wheels [4]*hillWheel
	now    time.Time
	quiet  bool // the encoders have stopped reporting

	lowest    [4]float64 // the furthest back each wheel has been, metres
	stoppedAt float64    // where the front left wheel was when the stick was centred
}

func newHill(t *testing.T, cfg SpeedLoopConfig, load, friction float64) *hill {
	t.Helper()
	d, _, _ := drivingDrivetrain(t, calibrated())
	applySpeedLoopDefaults(&cfg)
	d.cfg.SpeedLoop = cfg
	h := &hill{t: t, d: d, now: time.Unix(1_000_000, 0)}
	d.fb.clock = func() time.Time { return h.now }
	for i := range h.wheels {
		h.wheels[i] = &hillWheel{load: load, friction: friction}
	}
	// Driving up the slope, as a vehicle does before it stops on one; then the
	// stick centred, by someone still in control.
	d.velocity, d.byVelocity = 0.2, true
	h.run(3)
	d.velocity = 0
	for i := range h.lowest {
		h.lowest[i] = h.distance(i)
	}
	h.stoppedAt = h.distance(0)
	return h
}

// rolledBack is how far the vehicle has gone back down the slope, at worst,
// since the stick was centred.
func (h *hill) rolledBack() float64 {
	return h.stoppedAt - h.lowest[0]
}

func (h *hill) run(seconds float64) {
	for ms := 0; ms < int(seconds*1000); ms++ {
		for i, w := range h.wheels {
			w.advance(float64(h.d.last[i+1])/rpmToCommand, 0.001)
			h.lowest[i] = math.Min(h.lowest[i], h.distance(i))
		}
		h.now = h.now.Add(time.Millisecond)
		at := h.now.UnixMilli()
		if at%50 == 0 && !h.quiet {
			h.d.fb.mu.Lock()
			for i, w := range h.wheels {
				h.d.fb.wheels[i] = wheelReading{revolutions: w.rev, rpm: w.rpm, at: h.now}
			}
			h.d.fb.mu.Unlock()
		}
		if at%20 == 0 {
			h.d.writeCycleAt(h.now)
		}
	}
}

func (h *hill) distance(i int) float64 {
	return h.wheels[i].rev * h.d.cfg.Geometry.WheelCircumference
}

// effort is the largest wheel command now, in percent of full scale.
func (h *hill) effort() float64 {
	e := 0.0
	for node := 1; node <= 4; node++ {
		e = math.Max(e, math.Abs(float64(h.d.last[node]))/fullScale*100)
	}
	return e
}

// With resting off — the default — the loop holds the vehicle on the slope for
// as long as it stands there.
func TestHoldingOnAHill(t *testing.T) {
	h := newHill(t, defaultSpeedLoop(), 15, 5) // rolls back if let go
	h.run(20)
	if h.d.stand.mode != holding {
		t.Errorf("after 20 s on the slope the standstill is %v; want holding", h.d.stand.mode)
	}
	if back := h.rolledBack(); back > 0.05 {
		t.Errorf("the vehicle rolled back %.1f cm before it was held; want under 5", back*100)
	}
	if rpm := h.wheels[0].rpm; math.Abs(rpm) > 0.5 {
		t.Errorf("held, the wheel turns at %.2f RPM", rpm)
	}
	if e := h.effort(); e < 5 {
		t.Errorf("holding against the slope with %.1f%% effort; want the slope's", e)
	}
}

// A drivetrain that holds the vehicle by itself is let do it: the effort goes
// to zero and stays there, and the vehicle does not move.
func TestRestingWhenTheDrivetrainHolds(t *testing.T) {
	cfg := defaultSpeedLoop()
	cfg.HoldRelaxAfterSeconds = 3
	h := newHill(t, cfg, 15, 20)
	h.run(2)
	if h.d.stand.mode != holding {
		t.Fatalf("after 2 s the standstill is %v; want holding", h.d.stand.mode)
	}
	h.run(2)
	if h.d.stand.mode != resting {
		t.Fatalf("after 4 s the standstill is %v; want resting", h.d.stand.mode)
	}
	at := h.distance(0)
	h.run(20)
	if h.d.stand.mode != resting || len(h.d.stand.catches) != 0 {
		t.Errorf("after 24 s: %v, %d catches; want resting, none", h.d.stand.mode, len(h.d.stand.catches))
	}
	if e := h.effort(); e != 0 {
		t.Errorf("resting, the wheels still get %.1f%%", e)
	}
	if moved := math.Abs(h.distance(0) - at); moved > 0.001 {
		t.Errorf("resting, the vehicle moved %.1f mm", moved*1000)
	}
}

// A vehicle that rolls when let go is caught within about creepMetres, and
// after maxCatches the slope is held for good.
func TestCatchingAVehicleThatRolls(t *testing.T) {
	cfg := defaultSpeedLoop()
	cfg.HoldRelaxAfterSeconds = 2
	h := newHill(t, cfg, 15, 5)
	h.run(40)
	if n := len(h.d.stand.catches); n != cfg.MaxCatches {
		t.Errorf("%d catches in 40 s; want %d, and then no more", n, cfg.MaxCatches)
	}
	if !h.d.stand.permanent || h.d.stand.mode != holding {
		t.Errorf("after the catches: permanent %v, %v; want holding for good", h.d.stand.permanent, h.d.stand.mode)
	}
	// Each catch costs the creep and the moment it takes the loop to stop the
	// wheel again, and they add up: the vehicle is held where it was caught.
	if back, most := h.rolledBack(), float64(cfg.MaxCatches)*(cfg.CreepMetres+0.02); back > most {
		t.Errorf("the vehicle rolled back %.1f cm in all; want no more than %.0f cm, 2 cm over each creep", back*100, most*100)
	}
}

// Driving off uphill from a hold starts from the effort the hold learned, and
// rolls back less than from nothing.
func TestDrivingOffFromAHold(t *testing.T) {
	rollback := func(cold bool) float64 {
		h := newHill(t, defaultSpeedLoop(), 15, 5)
		h.run(2)
		if cold {
			h.d.loops = nil
			for node := range h.d.last {
				h.d.last[node] = 0
			}
		}
		start := h.distance(0)
		h.lowest[0] = start
		h.d.velocity = 0.2
		h.run(3)
		if h.distance(0) <= start {
			h.t.Fatalf("cold %v: driving off at 0.2 m/s, the vehicle did not go up the slope", cold)
		}
		return start - h.lowest[0]
	}
	warm, cold := rollback(false), rollback(true)
	if warm >= cold {
		t.Errorf("driving off from the hold rolled back %.1f mm, from nothing %.1f mm; want less from the hold",
			warm*1000, cold*1000)
	}
}

// With an encoder quiet, a creep could not be seen: the loader stops resting.
func TestAQuietEncoderEndsTheRest(t *testing.T) {
	cfg := defaultSpeedLoop()
	cfg.HoldRelaxAfterSeconds = 2
	h := newHill(t, cfg, 15, 20)
	h.run(3)
	if h.d.stand.mode != resting {
		t.Fatalf("after 3 s the standstill is %v; want resting", h.d.stand.mode)
	}
	h.quiet = true
	h.run(1.5)
	if h.d.stand.mode != holding {
		t.Errorf("with the encoders quiet for 1.5 s the standstill is %v; want holding", h.d.stand.mode)
	}
}

// A stop ends the standstill like everything else.
func TestAStopEndsTheStandstill(t *testing.T) {
	cfg := defaultSpeedLoop()
	cfg.HoldRelaxAfterSeconds = 2
	h := newHill(t, cfg, 15, 20)
	h.run(3)
	vehicle := &Traits{Name: "Vehicle", Kind: "vehicle", dt: h.d, encoderIndex: -1}
	if w := put(t, vehicle.stopService, "gamer", "1"); w.Code != http.StatusOK {
		t.Fatalf("stop: %d %s", w.Code, w.Body)
	}
	h.run(0.1)
	if h.d.stand.mode != driving || h.effort() != 0 {
		t.Errorf("after a stop: %v at %.1f%%; want the standstill gone and every wheel at zero", h.d.stand.mode, h.effort())
	}
}
