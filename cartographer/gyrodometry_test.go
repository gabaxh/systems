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
	"encoding/json"
	"math"
	"math/rand"
	"testing"
	"time"
)

// scripted is a vehicle driven by a script, read the way the sampler reads it:
// every 50 ms, the wheels and the vigie's heading, each stamped when it was
// measured rather than when it was asked for. The truth is integrated at 1 ms.
type scripted struct {
	speed    func(t float64) float64 // m/s, of the axle
	yaw      func(t float64) float64 // °/s, the axle's true rate of turn
	slip     func(t float64) float64 // m/s the left wheel reports beyond what it rolled
	bias     float64                 // °/s, the gyro's error
	silent   func(t float64) bool    // the vigie answers 503
	standing bool                    // the wheels are held, and heard only at the loader's heartbeat
	jitter   bool                    // stamps lag the poll by a varying few milliseconds
}

func stillScript(float64) float64 { return 0 }
func steady(v float64) func(float64) float64 {
	return func(float64) float64 { return v }
}

// run plays the script for seconds through a gyrodometer, and returns the
// axle's motion and every interval compared.
func (s scripted) run(t *testing.T, mode string, hold *int, seconds float64) (pose, []interval) {
	t.Helper()
	cfg := defaultGyro()
	applyGyroDefaults(&cfg)
	cfg.Mode, cfg.HoldMs = mode, hold
	odo := defaultOdometry()
	g := newGyrodometer(cfg, odo)
	var ivs []interval
	g.onInterval = func(iv interval) { ivs = append(ivs, iv) }
	if s.slip == nil {
		s.slip = stillScript
	}

	const dt = 0.001
	n := int(seconds/dt) + 1
	left, right, heading := make([]float64, n), make([]float64, n), make([]float64, n)
	for i := 1; i < n; i++ {
		tt := float64(i) * dt
		w := s.yaw(tt) * math.Pi / 180
		v := s.speed(tt)
		left[i], right[i] = left[i-1], right[i-1]
		if !s.standing {
			left[i] += (v - w*odo.Track/2 + s.slip(tt)) * dt
			right[i] += (v + w*odo.Track/2) * dt
		}
		heading[i] = heading[i-1] + (s.yaw(tt)+s.bias)*dt
	}
	at := func(i int) time.Time { return clock.Add(time.Duration(i) * time.Millisecond) }
	rng := rand.New(rand.NewSource(11))
	lag := func(most int) int {
		if !s.jitter {
			return 0
		}
		return rng.Intn(most + 1)
	}

	moved := pose{}
	for i := 50; i < n; i += 50 {
		w := i - lag(20)
		if s.standing {
			w = i / 1000 * 1000 // the newest heartbeat
		}
		g.addWheels(wheelSample{left[w], at(w)}, wheelSample{right[w], at(w)})
		if s.silent == nil || !s.silent(float64(i)*dt) {
			h := i - lag(5)
			g.addHeading(heading[h], at(h))
		}
		g.step(at(i))
		if i%200 == 0 { // a sweep takes what has gathered
			if m := g.take(); m.ok {
				moved = compose(moved, m.axle)
			}
		}
	}
	if m := g.take(); m.ok {
		moved = compose(moved, m.axle)
	}
	return moved, ivs
}

func deg(rad float64) float64 { return rad * 180 / math.Pi }

func ms(v int) *int { return &v }

// The defining property: while the two agree, a biased gyro changes nothing.
// Half a degree a second of bias, a weaving drive, and the heading is the
// wheels' to the last digit.
func TestWhileTheyAgreeABiasedGyroChangesNothing(t *testing.T) {
	s := scripted{speed: steady(0.5), yaw: func(t float64) float64 { return 20 * math.Sin(2*math.Pi*t/10) },
		bias: 0.5, jitter: true}
	fused, ivs := s.run(t, "gyrodometry", nil, 30)
	wheels, _ := s.run(t, "odometry", nil, 30)
	for _, iv := range ivs {
		if iv.fromGyro {
			t.Fatalf("the gyro was believed at %s, with nothing to disagree about", iv.t0.Format("05.000"))
		}
	}
	if !samePose(fused, wheels) {
		t.Errorf("gyrodometry %+v differs from the wheels %+v", fused, wheels)
	}
}

// A wheel spinning for a second while the vehicle drives straight: the wheels
// see an 18° turn that never happened. Gyrodometry sees the gyro's bias over
// that second, and nothing else.
func TestASlipIsCaughtByTheGyro(t *testing.T) {
	s := scripted{speed: steady(1), yaw: stillScript, bias: 0.5, jitter: true,
		slip: func(t float64) float64 {
			if t >= 4 && t < 5 {
				return 0.2
			}
			return 0
		}}
	fused, _ := s.run(t, "gyrodometry", nil, 10)
	wheels, _ := s.run(t, "odometry", nil, 10)
	t.Logf("heading error after the slip: %.2f° with the gyro, %.2f° from the wheels", deg(fused.theta), deg(wheels.theta))
	if math.Abs(deg(fused.theta)) > 1.5 {
		t.Errorf("with the gyro the slip still turned the heading %.2f°", deg(fused.theta))
	}
	if math.Abs(deg(wheels.theta)) < 15 {
		t.Errorf("the wheels alone turned only %.2f°: the test does not slip", deg(wheels.theta))
	}
}

// A disagreement that wanders above and below the threshold, as a wheel
// spinning in a pile does: without a hold the source flips back and forth;
// with one it stays with the gyro. The false turn is 1°/s, under the 1.25°/s
// threshold, and 2°/s every other 200 ms, so whichever way the intervals fall
// the trips come 200 ms apart, inside the hold.
func TestTheHoldStopsChattering(t *testing.T) {
	wander := func(t float64) float64 {
		if t < 3 || t >= 6 {
			return 0
		}
		rate := 1.0 // °/s of false turn
		if int(t*5)%2 == 1 {
			rate = 2.0
		}
		return rate * math.Pi / 180 * 0.6275
	}
	s := scripted{speed: steady(0.5), yaw: stillScript, slip: wander}
	flips := func(hold *int) int {
		_, ivs := s.run(t, "gyrodometry", hold, 8)
		n := 0
		for i := 1; i < len(ivs); i++ {
			if ivs[i].fromGyro != ivs[i-1].fromGyro {
				n++
			}
		}
		return n
	}
	paper, held := flips(ms(0)), flips(nil)
	t.Logf("source changes: %d with the paper's rule, %d with a 300 ms hold", paper, held)
	if held > 2 {
		t.Errorf("with the hold the source changed %d times", held)
	}
	if paper < 6 {
		t.Errorf("without the hold it changed only %d times: the test does not wander", paper)
	}
}

// No vigie at all: the heading is the wheels', exactly as without
// gyrodometry.
func TestWithoutTheVigieItIsTheWheels(t *testing.T) {
	s := scripted{speed: steady(0.5), yaw: steady(10), silent: func(float64) bool { return true },
		slip: func(t float64) float64 { return 0.1 }}
	fused, ivs := s.run(t, "gyrodometry", nil, 10)
	wheels, _ := s.run(t, "odometry", nil, 10)
	if !samePose(fused, wheels) {
		t.Errorf("with no heading, %+v differs from the wheels' %+v", fused, wheels)
	}
	for _, iv := range ivs {
		if iv.haveGyro {
			t.Fatal("an interval had a gyro turn with no vigie answering")
		}
	}
}

// The vigie withholds its heading just after losing part of a turn. An
// interval across that hole must not take the gyro's word, even mid-slip.
func TestAHoleInTheHeadingKeepsTheGyroOut(t *testing.T) {
	s := scripted{speed: steady(1), yaw: stillScript,
		slip:   func(t float64) float64 { return map[bool]float64{true: 0.2}[t >= 4 && t < 5] },
		silent: func(t float64) bool { return t >= 4.2 && t < 4.5 }}
	_, ivs := s.run(t, "gyrodometry", nil, 8)
	hole, after := 0, 0
	for _, iv := range ivs {
		s := iv.t0.Sub(clock).Seconds()
		switch {
		case s >= 4.2 && s < 4.4:
			hole++
			if iv.haveGyro || iv.fromGyro {
				t.Errorf("the interval at %.1f s, inside the hole, used the gyro", s)
			}
		case s >= 4.6 && s < 4.9 && iv.fromGyro:
			after++
		}
	}
	if hole == 0 || after == 0 {
		t.Errorf("%d intervals in the hole, %d from the gyro after it: the test does not test the hole", hole, after)
	}
}

// The wheels' and the heading's stamps lag their polls by different amounts.
// Compared at the same instants, a clean steady turn agrees to far below the
// threshold; compared at the polls, it would not.
func TestTheyAreComparedAtTheSameInstants(t *testing.T) {
	s := scripted{speed: steady(0.5), yaw: steady(20), jitter: true}
	_, ivs := s.run(t, "gyrodometry", nil, 20)
	worst := 0.0
	for _, iv := range ivs {
		worst = math.Max(worst, math.Abs(deg(iv.turnGyro-iv.turnWheels)))
		if iv.fromGyro {
			t.Fatalf("a clean turn tripped the threshold at %s", iv.t0.Format("05.000"))
		}
	}
	if worst > 0.02 {
		t.Errorf("gyro and wheels differ by up to %.3f° per interval, against a threshold of 0.125°", worst)
	}
}

// The joint swinging at standstill turns the front half while its wheels
// stand. They are heard only at the loader's heartbeat; the gyro sees the
// turn, and gyrodometry takes it.
func TestTheJointSwingingAtStandstillIsTheGyros(t *testing.T) {
	swing := func(t float64) float64 { return map[bool]float64{true: 20}[t >= 2 && t < 3] }
	s := scripted{speed: stillScript, yaw: swing, standing: true}
	fused, _ := s.run(t, "gyrodometry", nil, 5)
	wheels, _ := s.run(t, "odometry", nil, 5)
	t.Logf("the swing of 20°: %.2f° with the gyro, %.2f° from the wheels", deg(fused.theta), deg(wheels.theta))
	if !nearTo(deg(fused.theta), 20, 1) || math.Hypot(fused.x, fused.y) > 0.01 {
		t.Errorf("with the gyro the swing read %.2f°, moved %.3f m", deg(fused.theta), math.Hypot(fused.x, fused.y))
	}
	if deg(wheels.theta) != 0 {
		t.Errorf("the held wheels turned %.2f°: the test does not hold them", deg(wheels.theta))
	}
}

func nearTo(a, b, tol float64) bool { return math.Abs(a-b) <= tol }

// Gyrodometry is opted into: a configuration without it, or written before
// it existed, maps exactly as it always has.
func TestGyrodometryIsOffUnlessAskedFor(t *testing.T) {
	var cfg CartographerConfig
	applyDefaults(&cfg)
	if cfg.Gyro.on() {
		t.Error("a configuration without a gyro block has gyrodometry on")
	}
	if err := json.Unmarshal([]byte(`{"gyro": {"enabled": true}}`), &cfg); err != nil {
		t.Fatal(err)
	}
	applyDefaults(&cfg)
	if !cfg.Gyro.on() || cfg.Gyro.Sensor["Model"][0] != "Livox Mid-360" || cfg.Gyro.hold() != 300*time.Millisecond {
		t.Errorf("enabled: %+v", cfg.Gyro)
	}
	if tr := mappingTraits(nil); tr.gyro != nil {
		t.Error("a cartographer with gyrodometry off has a gyrodometer")
	}
}

// Down a corridor whose walls cannot fix the heading, a wheel that spins for
// half a second bends every wall drawn after it. With gyrodometry the map
// keeps straight.
func TestASlipDoesNotBendTheMap(t *testing.T) {
	headingAfter := func(gyro bool) float64 {
		const step = 0.07 // metres per sweep
		var left, right float64 = 57.3, 57.3
		at := clock
		// The wheels are stamped with the test's clock, not the wall clock.
		// Stamped with the wall clock, two sweeps a few milliseconds apart in
		// real time made the slip a jump of several metres a second, which
		// the odometer takes for the loader restarting and drops — so the
		// wheels-only drive never slipped, whenever the matcher was quick.
		// That made this test fail now and then, and always once the map
		// drew faster. The clock is far from now, so the odometer is told
		// not to mind the readings' age.
		tr := mappingTraits(func() (wheelSample, wheelSample, error) {
			return wheelSample{left, at}, wheelSample{right, at}, nil
		})
		tr.odo.cfg.MaxAgeMs = 1 << 40
		walls := false
		tr.cfg.HeadingFromWalls = &walls
		if gyro {
			tr.cfg.Gyro.Enabled = &on
			tr.gyro = newGyrodometer(tr.cfg.Gyro, tr.cfg.Odometry)
		}
		truth := pose{x: 2}
		for i := 0; i <= 30; i++ {
			// Four polls a sweep: the wheels and the heading, still at 0.
			for k := 0; k < 4; k++ {
				at = at.Add(50 * time.Millisecond)
				slipping := i >= 10 && i < 13
				left += step / 4
				if slipping {
					left += step / 4 // the left wheel spins twice as fast as it rolls
				}
				right += step / 4
				if tr.gyro != nil {
					tr.gyro.addWheels(wheelSample{left, at}, wheelSample{right, at})
					tr.gyro.addHeading(0, at)
					tr.gyro.step(at)
				}
			}
			tr.consume(scanAt(truth, at))
			truth.x += step
		}
		return math.Abs(wrapAngle(tr.at.theta)) * 180 / math.Pi
	}
	withGyro, wheelsOnly := headingAfter(true), headingAfter(false)
	t.Logf("heading error after a slip: %.2f° with gyrodometry, %.2f° from the wheels", withGyro, wheelsOnly)
	if withGyro > 1 {
		t.Errorf("with gyrodometry the slip still turned the map %.2f°", withGyro)
	}
	if wheelsOnly < 5 {
		t.Errorf("the wheels alone turned only %.2f°: the test does not slip", wheelsOnly)
	}
}
