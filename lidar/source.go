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
	"context"
	"fmt"
	"math"
	"math/rand/v2"
	"time"
)

// source is where IMU samples come from: the MID-360 over UDP, or the
// simulator. As for the guetteur, the simulator is not a convenience: the
// estimator's whole point is tilt and bias, and both have to be exercised
// where nobody can tilt the real sensor.
type source interface {
	// samples delivers samples in time order until the context is cancelled
	// or the source fails. A closed channel means it has stopped for good.
	samples(ctx context.Context) (<-chan imuSample, error)
	// status is one line on how the stream is doing, for the status log.
	status() string
	close() error
}

//-------------------------------------The simulator

// motion is one stretch of the simulator's script.
type motion struct {
	seconds float64
	yawRate float64 // deg/s, positive to the left
	rollDeg float64 // amplitude of a side-to-side rock
	rollHz  float64
}

// defaultScript stands still long enough for the first bias, turns left a
// quarter, stands, rocks from side to side without turning, and turns back. It
// repeats, and ends where it began, so an unattended run can be checked by eye:
// yaw should come back to zero every 27 seconds.
var defaultScript = []motion{
	{seconds: 5},
	{seconds: 4.5, yawRate: 20},
	{seconds: 3.5},
	{seconds: 6, rollDeg: 2, rollHz: 0.5},
	{seconds: 4.5, yawRate: -20},
	{seconds: 3.5},
}

// simulatedSource is a gyro and accelerometer tilted forward on a vehicle that
// follows a script, with a bias on every axis and white noise.
type simulatedSource struct {
	script []motion
	period time.Duration
	tilt   float64    // radians, forward
	bias   [3]float64 // rad/s, in the sensor's frame
	noise  float64    // rad/s per sample
	rng    *rand.Rand
	out    chan imuSample
}

// stillScript never moves: a recording of it has a known Allan deviation,
// white noise only, which tests the analysis end to end.
var stillScript = []motion{{seconds: 3600}}

func newSimulatedSource(cfg LidarConfig) *simulatedSource {
	b := radians(cfg.SimBiasDegPerSec)
	script := defaultScript
	if cfg.SimScript == "still" {
		script = stillScript
	}
	return &simulatedSource{
		script: script,
		period: 5 * time.Millisecond, // the MID-360's 200 Hz
		tilt:   radians(cfg.SimTiltDegrees),
		bias:   [3]float64{b, -b, b},
		noise:  radians(cfg.SimNoiseDegPerSec),
		rng:    rand.New(rand.NewPCG(1, 2)),
		out:    make(chan imuSample, 64),
	}
}

func (s *simulatedSource) close() error { return nil }

func (s *simulatedSource) status() string {
	return fmt.Sprintf("simulated, bias %.2f°/s on each axis, tilted %.0f°", degrees(s.bias[0]), degrees(s.tilt))
}

// samples emits the script in real time. Each sample is stamped with when it
// is due rather than when it is sent, as a device stamps its own.
func (s *simulatedSource) samples(ctx context.Context) (<-chan imuSample, error) {
	go func() {
		defer close(s.out)
		tick := time.NewTicker(4 * s.period)
		defer tick.Stop()
		start := time.Now()
		for k := 0; ; {
			select {
			case <-ctx.Done():
				return
			case now := <-tick.C:
				for at := start.Add(time.Duration(k) * s.period); !at.After(now); at = start.Add(time.Duration(k) * s.period) {
					select {
					case s.out <- s.sample(float64(k)*s.period.Seconds(), at):
					case <-ctx.Done():
						return
					}
					k++
				}
			}
		}
	}()
	return s.out, nil
}

// body is the vehicle's yaw rate, roll and roll rate, in radians, t seconds
// into the script.
func (s *simulatedSource) body(t float64) (yawRate, roll, rollRate float64) {
	total := 0.0
	for _, m := range s.script {
		total += m.seconds
	}
	t = math.Mod(t, total)
	for _, m := range s.script {
		if t < m.seconds {
			w := 2 * math.Pi * m.rollHz
			return radians(m.yawRate), radians(m.rollDeg) * math.Sin(w*t), radians(m.rollDeg) * w * math.Cos(w*t)
		}
		t -= m.seconds
	}
	return 0, 0, 0
}

// sample is what the sensor reads t seconds into the script.
//
// The vehicle's frame is x forward, y left, z up, and it yaws and then rolls.
// Its angular velocity in its own frame is then (roll rate, yaw rate·sin roll,
// yaw rate·cos roll), and the vertical is (0, sin roll, cos roll). The sensor
// is that frame pitched forward by the tilt. The centripetal acceleration is
// left out; it is a few hundredths of a g in a tight turn.
func (s *simulatedSource) sample(t float64, at time.Time) imuSample {
	psiDot, phi, phiDot := s.body(t)
	w := pitch([3]float64{phiDot, psiDot * math.Sin(phi), psiDot * math.Cos(phi)}, s.tilt)
	up := pitch([3]float64{0, math.Sin(phi), math.Cos(phi)}, s.tilt)
	var smp imuSample
	for i := range 3 {
		smp.gyro[i] = w[i] + s.bias[i] + s.noise*s.rng.NormFloat64()
		smp.accel[i] = up[i] + 0.002*s.rng.NormFloat64()
	}
	smp.at = at
	return smp
}

// pitch expresses a vector of the vehicle's frame in the frame of a sensor
// pitched forward by a.
func pitch(v [3]float64, a float64) [3]float64 {
	c, s := math.Cos(a), math.Sin(a)
	return [3]float64{c*v[0] - s*v[2], v[1], s*v[0] + c*v[2]}
}

//-------------------------------------Choosing one

func newSource(cfg LidarConfig) (source, error) {
	switch cfg.Source {
	case "simulated", "":
		if cfg.SimScript != "cycle" && cfg.SimScript != "still" {
			return nil, fmt.Errorf("unknown simScript %q: expected \"cycle\" or \"still\"", cfg.SimScript)
		}
		return newSimulatedSource(cfg), nil
	case "livox":
		return newLivoxSource(cfg)
	default:
		return nil, fmt.Errorf("unknown source %q: expected \"livox\" or \"simulated\"", cfg.Source)
	}
}
