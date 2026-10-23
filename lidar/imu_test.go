/*******************************************************************************
 * Copyright (c) 2026 Synecdoque
 *
 * The software is licensed under the MIT License. See the LICENSE file in this
 * repository for details.
 *
 * Contributors:
 *   Gabriel Axheim Gustafsson, Luleå - initial implementation
 ***************************************************************************SDG*/

package main

import (
	"math"
	"testing"
	"time"
)

func testConfig() LidarConfig {
	cfg := LidarConfig{}
	applyDefaults(&cfg)
	return cfg
}

func testEstimator(t *testing.T, cfg LidarConfig) *estimator {
	t.Helper()
	e, err := newEstimator(cfg)
	if err != nil {
		t.Fatal(err)
	}
	e.logf = t.Logf
	return e
}

func scripted(cfg LidarConfig, script ...motion) *simulatedSource {
	s := newSimulatedSource(cfg)
	s.script = script
	return s
}

// feed runs the script from t0 to t1 seconds into the estimator, stamping the
// samples from start, and returns the last stamp.
func feed(e *estimator, s *simulatedSource, start time.Time, t0, t1 float64) time.Time {
	var at time.Time
	for k := int(math.Round(t0 / s.period.Seconds())); float64(k)*s.period.Seconds() <= t1; k++ {
		at = start.Add(time.Duration(k) * s.period)
		e.add(s.sample(float64(k)*s.period.Seconds(), at))
	}
	return at
}

// The reason u exists. Tilted 30° forward, the sensor's own z reads every turn
// 13 % short; the rate about the measured vertical reads all of it, and a turn
// to the left is positive.
func TestYawIsTheTurnAboutTheVerticalOnATiltedMount(t *testing.T) {
	cfg := testConfig()
	e := testEstimator(t, cfg)
	s := scripted(cfg, motion{seconds: 3}, motion{seconds: 12, yawRate: 30}, motion{seconds: 3})
	feed(e, s, time.Unix(1000, 0), 0, 18)

	r := e.reading()
	if !r.ready {
		t.Fatal("not ready after a three-second standstill")
	}
	if math.Abs(r.yaw-360) > 0.1 {
		t.Errorf("a full turn to the left read %.3f°, want 360 ± 0.1", r.yaw)
	}
	if tilt := tiltDegrees(e.up); math.Abs(tilt-30) > 0.5 {
		t.Errorf("the measured up vector implies a tilt of %.2f°, want 30", tilt)
	}
	// What the z gyro alone would have said.
	if z := 360 * math.Cos(radians(30)); math.Abs(r.yaw-z) < 40 {
		t.Errorf("yaw %.1f° is close to the z gyro's %.1f°; the tilt was not taken out", r.yaw, z)
	}
}

// Rocking from side to side on a tilted mount puts part of the roll rate on
// the sensor's z. About the vertical, it is no turn at all.
func TestRockingIsNotTurning(t *testing.T) {
	cfg := testConfig()
	e := testEstimator(t, cfg)
	s := scripted(cfg, motion{seconds: 3}, motion{seconds: 8, rollDeg: 3, rollHz: 0.5}, motion{seconds: 3})
	feed(e, s, time.Unix(1000, 0), 0, 14)

	if r := e.reading(); math.Abs(r.yaw) > 0.1 {
		t.Errorf("rocking ±3° read as %.3f° of turn, want under 0.1", r.yaw)
	}
}

// The defining property of a bias estimate: a gyro biased by half a degree a
// second, left standing for a minute, has not turned.
func TestTheBiasIsRemovedAtAStandstill(t *testing.T) {
	cfg := testConfig()
	cfg.SimBiasDegPerSec = 0.5
	e := testEstimator(t, cfg)
	feed(e, scripted(cfg, motion{seconds: 70}), time.Unix(1000, 0), 0, 62)

	r := e.reading()
	if math.Abs(r.yaw) > 0.1 {
		t.Errorf("standing still for a minute read %.3f° of turn, want under 0.1", r.yaw)
	}
	// The simulator's bias is (b, −b, b); about the vertical it is b·(u0 − u1 + u2).
	want := 0.5 * (e.up[0] - e.up[1] + e.up[2])
	if math.Abs(r.bias-want) > 0.01 {
		t.Errorf("bias about the vertical = %.4f°/s, want %.4f", r.bias, want)
	}
}

// A steady slow turn has as small a variance as a standstill. It must not be
// taken for the bias, or it would vanish from the yaw.
func TestASlowSteadyTurnIsNotTheBias(t *testing.T) {
	cfg := testConfig()
	cfg.SimNoiseDegPerSec = 0.01
	e := testEstimator(t, cfg)
	s := scripted(cfg, motion{seconds: 3}, motion{seconds: 30, yawRate: 1})
	feed(e, s, time.Unix(1000, 0), 0, 2.5)
	before := e.reading().bias
	feed(e, s, time.Unix(1000, 0), 2.505, 33)

	r := e.reading()
	if math.Abs(r.bias-before) > 1e-9 {
		t.Errorf("the bias moved from %.4f to %.4f°/s during a 1°/s turn", before, r.bias)
	}
	if math.Abs(r.yaw-30) > 0.2 {
		t.Errorf("30 s at 1°/s read %.3f°, want 30 ± 0.2", r.yaw)
	}
	if e.offBias == 0 {
		t.Error("no window was counted as still but off the bias")
	}
}

// Without a standstill there is no bias, and a yaw with an unknown bias in it
// is not a yaw anyone should use.
func TestNotReadyBeforeAStandstill(t *testing.T) {
	cfg := testConfig()
	e := testEstimator(t, cfg)
	feed(e, scripted(cfg, motion{seconds: 10, yawRate: 20}), time.Unix(1000, 0), 0, 10)
	if r := e.reading(); r.ready || r.biased {
		t.Errorf("ready = %v, biased = %v after turning since start-up; want neither", r.ready, r.biased)
	}
}

// A hole in the stream loses the turn made during it. It is not integrated
// across, and the reading says so for long enough that a poller sees it.
func TestAGapIsNotIntegratedAcross(t *testing.T) {
	cfg := testConfig()
	e := testEstimator(t, cfg)
	s := scripted(cfg, motion{seconds: 3}, motion{seconds: 10, yawRate: 90})
	start := time.Unix(1000, 0)
	feed(e, s, start, 0, 3)
	// 200 ms missing, a quarter of a second into the turn.
	feed(e, s, start, 3.2, 3.25)
	if r := e.reading(); !r.recovering {
		t.Error("just after a 200 ms gap, the reading does not say it is recovering")
	}
	feed(e, s, start, 3.255, 4)
	r := e.reading()
	if r.recovering {
		t.Error("still recovering 800 ms after the gap")
	}
	if want := 90 * 0.8; math.Abs(r.yaw-want) > 0.5 {
		t.Errorf("yaw = %.2f°, want about %.1f: the turn during the gap should be lost, not interpolated", r.yaw, want)
	}
	if e.gaps != 1 {
		t.Errorf("%d gaps counted, want 1", e.gaps)
	}
}

func TestAConfiguredUpVectorIsNormalizedAndChecked(t *testing.T) {
	cfg := testConfig()
	cfg.UpVector = []float64{0, 0, 1.2}
	e, err := newEstimator(cfg)
	if err != nil {
		t.Fatal(err)
	}
	if !e.haveUp || e.up != [3]float64{0, 0, 1} {
		t.Errorf("up = %v (known %v), want [0 0 1]", e.up, e.haveUp)
	}
	for _, bad := range [][]float64{{0, 0, 0}, {0, 1}, {0, 0, 5}} {
		cfg.UpVector = bad
		if _, err := newEstimator(cfg); err == nil {
			t.Errorf("upVector %v was accepted", bad)
		}
	}
}

// The rate is the mean over the rate window, about the vertical.
func TestYawRateIsAboutTheVertical(t *testing.T) {
	cfg := testConfig()
	e := testEstimator(t, cfg)
	feed(e, scripted(cfg, motion{seconds: 3}, motion{seconds: 5, yawRate: -25}), time.Unix(1000, 0), 0, 5)
	if r := e.reading(); math.Abs(r.rate+25) > 0.2 {
		t.Errorf("yawRate = %.3f°/s, want −25", r.rate)
	}
}
