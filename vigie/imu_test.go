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
	"math/rand"
	"testing"
	"time"
)

const deg = math.Pi / 180

// The Artitrax's Mid-360 as measured (gyro characterization, October 2026):
// up, in the lidar's own frame, with the vehicle level. It sits turned 90° on
// its stick and tilted about 11° to see the floor.
var measuredUp = [3]float64{0.0134, 0.1908, 0.9815}

// vehicleMount is the mount that measuredUp implies, with the yaw a person
// reads off the stick.
func vehicleMount() Mount {
	n := math.Sqrt(measuredUp[0]*measuredUp[0] + measuredUp[1]*measuredUp[1] + measuredUp[2]*measuredUp[2])
	roll, pitch := levelling([3]float64{measuredUp[0] / n, measuredUp[1] / n, measuredUp[2] / n})
	return Mount{YawDegrees: 90, RollDegrees: roll, PitchDegrees: pitch}
}

// drive feeds a motion at 200 Hz from t0 to t1 seconds as consume would: the
// vehicle turning at yawRate(t) °/s, the lidar turned by truth on it, its gyro
// biased by bias(t) about the vehicle's vertical plus noise, and every sample
// turned back into the vehicle frame by the mount as configured.
type drive struct {
	m          *motion
	truth      rotation // how the lidar really sits
	configured rotation // how the vigie is told it sits
	rng        *rand.Rand
	noise      float64 // rad/s per axis
	start      time.Time
	skip       func(t float64) bool // samples that never arrive
}

func newDrive(m *motion, truth, configured Mount) *drive {
	return &drive{m: m, truth: truth.rotation(), configured: configured.rotation(),
		rng: rand.New(rand.NewSource(7)), noise: 0.05 * deg, start: time.Unix(5000, 0)}
}

func (d *drive) run(t0, t1 float64, yawRate, bias func(t float64) float64) {
	toLidar := d.truth.transpose()
	for k := int(math.Round(t0 * 200)); float64(k) <= t1*200; k++ {
		t := float64(k) / 200
		if d.skip != nil && d.skip(t) {
			continue
		}
		w := toLidar.apply([3]float64{0, 0, (yawRate(t) + bias(t)) * deg})
		up := toLidar.apply([3]float64{0, 0, 1})
		for i := range w {
			w[i] += d.noise * d.rng.NormFloat64()
		}
		at := d.start.Add(time.Duration(k) * 5 * time.Millisecond)
		d.m.add(timedIMU{at: at, stamp: uint64(at.UnixNano()), gyro: d.configured.apply(w), acc: d.configured.apply(up)})
	}
}

func still(float64) float64                    { return 0 }
func constant(v float64) func(float64) float64 { return func(float64) float64 { return v } }

func newMotion(mount Mount) *motion {
	cfg := defaultConfig()
	return &motion{window: 100 * time.Millisecond, zeroFor: 3 * time.Second,
		limits: cfg.stillLimits(), rot: mount.rotation(), mount: mount}
}

// Gravity gives the mount's roll and pitch: the ones it suggests level the
// measured up exactly, and they say the lidar leans about 11°.
func TestTheMountIsFoundFromGravity(t *testing.T) {
	m := vehicleMount()
	if !near(m.RollDegrees, 11.0, 0.05) || !near(m.PitchDegrees, -0.77, 0.05) {
		t.Errorf("roll %.3f°, pitch %.3f°; want about 11.0 and −0.77", m.RollDegrees, m.PitchDegrees)
	}
	n := math.Sqrt(measuredUp[0]*measuredUp[0] + measuredUp[1]*measuredUp[1] + measuredUp[2]*measuredUp[2])
	up := m.rotation().apply([3]float64{measuredUp[0] / n, measuredUp[1] / n, measuredUp[2] / n})
	if !near(up[0], 0, 1e-12) || !near(up[1], 0, 1e-12) || !near(up[2], 1, 1e-12) {
		t.Errorf("the suggested mount turns up to %v, not straight up", up)
	}
	if pitch, roll := tilt(up); !near(pitch, 0, 1e-9) || !near(roll, 0, 1e-9) {
		t.Errorf("and the vehicle then reads pitch %.3f°, roll %.3f°", pitch, roll)
	}
}

// With the mount right, a full turn is a full turn. Taken as level, the same
// lidar reads every turn short by the cosine of its tilt.
func TestATiltedLidarTurnsAFullTurnOnlyWithItsMount(t *testing.T) {
	bias := constant(-1.3)
	for _, c := range []struct {
		name       string
		configured Mount
		lo, hi     float64
	}{
		{"mounted as it sits", vehicleMount(), 359.9, 360.1},
		{"taken as level", Mount{YawDegrees: 90}, 350, 355},
	} {
		m := newMotion(c.configured)
		d := newDrive(m, vehicleMount(), c.configured)
		d.run(0, 7, still, bias)
		d.run(7.005, 19, constant(30), bias)
		_, _, heading, _, _ := m.averages()
		if heading < c.lo || heading > c.hi {
			t.Errorf("%s: a turn of 360° read %.2f°, want %.1f to %.1f", c.name, heading, c.lo, c.hi)
		}
	}
}

// From cold the zero moves by about 0.055°/s a minute. Driving two minutes
// between standstills, it moves 0.11°/s: three times the step a zero may take
// for noise alone. With the allowance for drift the zero follows; without it,
// every later standstill is refused and the zero sticks where it started.
func TestTheZeroFollowsTheWarmUp(t *testing.T) {
	warming := func(t float64) float64 { return -1.3 + 0.055*t/60 }
	weaving := func(t float64) float64 { return 10 * math.Sin(2*math.Pi*t/4) }

	for _, allow := range []bool{true, false} {
		m := newMotion(vehicleMount())
		if !allow {
			m.limits.drift = 0
		}
		d := newDrive(m, vehicleMount(), vehicleMount())
		d.run(0, 7, still, warming)
		t0 := 7.005
		worst := 0.0
		for cycle := 0; cycle < 10; cycle++ {
			d.run(t0, t0+120, weaving, warming)
			d.run(t0+120.005, t0+127, still, warming)
			t0 += 127.005
			worst = math.Max(worst, math.Abs(m.state().zero-warming(t0)))
		}
		s := m.state()
		switch {
		case allow && (s.offZero > 0 || worst > 0.01):
			t.Errorf("with the drift allowed: %d standstills refused, the zero %.3f°/s off at worst", s.offZero, worst)
		case !allow && s.offZero == 0:
			t.Error("without the drift allowed, no standstill was refused: the test does not test the allowance")
		}
	}
}

// A steady slow turn spreads as little as a standstill. It must not be taken
// for the zero, or the turn would vanish from the heading.
func TestASlowTurnIsNotTakenForTheZero(t *testing.T) {
	m := newMotion(vehicleMount())
	d := newDrive(m, vehicleMount(), vehicleMount())
	d.noise = 0.01 * deg
	d.run(0, 7, still, constant(-1.3))
	before := m.state().zero
	d.run(7.005, 37, constant(1), constant(-1.3))
	s := m.state()
	if !near(s.zero, before, 1e-12) || s.offZero == 0 {
		t.Errorf("the zero moved from %.4f to %.4f°/s during a 1°/s turn (%d windows refused)", before, s.zero, s.offZero)
	}
	if _, _, heading, _, _ := m.averages(); !near(heading, 30, 0.2) {
		t.Errorf("30 s at 1°/s read %.2f°", heading)
	}
}

// A hole in the stream loses the turn made during it. It is not integrated
// across, and the heading is withheld long enough after it that a poller sees
// there was one.
func TestAGapIsNotIntegratedAndIsSaid(t *testing.T) {
	m := newMotion(vehicleMount())
	d := newDrive(m, vehicleMount(), vehicleMount())
	d.run(0, 7, still, constant(-1.3))
	d.skip = func(t float64) bool { return t > 8 && t < 8.2 }
	d.run(7.005, 8.25, constant(90), constant(-1.3))
	if !m.state().recovering {
		t.Error("just after a 200 ms gap the heading is not withheld")
	}
	d.run(8.255, 8.5, constant(90), constant(-1.3))
	s := m.state()
	if s.recovering {
		t.Error("still withheld 300 ms after the gap")
	}
	if s.gaps != 1 {
		t.Errorf("%d gaps counted, want 1", s.gaps)
	}
	// 1.5 s at 90°/s is 135°, of which the 0.2 s gap held 18°.
	if _, _, heading, _, _ := m.averages(); !near(heading, 117, 0.5) {
		t.Errorf("heading %.2f°, want about 117: the gap's 18° lost, not interpolated", heading)
	}
}
