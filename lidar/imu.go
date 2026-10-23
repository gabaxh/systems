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

// From gyro samples to the angle turned about the vertical.
//
// The MID-360 is tilted forward to see the floor, so its own z axis is not the
// vertical. At a tilt α, the z gyro reads every turn cos α short (13 % at 30°)
// and picks up sin α of every roll, so a vehicle rocking on rough ground would
// seem to turn. The rate about the vertical is instead the gyro's vector
// projected on the unit up vector u, measured in the sensor's frame: ψ̇ = ω·u.
// The roll and pitch rates are perpendicular to u and drop out, and all of a
// turn is kept, whatever the tilt. Nothing here needs to know how the sensor's
// axes are laid out, only that they are right-handed, so positive is a turn to
// the left seen from above.
//
// The accelerometer is used for two things only: to find u, and to help tell
// a standstill. Never for position: a tilt of half a degree reads as much as a
// typical acceleration.
//
// The bias is the gyro's mean at a standstill. It is measured once before yaw
// answers at all, and again at every later standstill, within a limit, so that
// a slow steady turn that passes the standstill test cannot become the bias.
// The integral starts at zero when the bias is first known.

import (
	"errors"
	"fmt"
	"math"
	"time"
)

// imuSample is one reading of the IMU, in the sensor's own frame, stamped on
// this computer's clock.
type imuSample struct {
	gyro  [3]float64 // rad/s
	accel [3]float64 // g, so 1 at rest
	at    time.Time
}

// reading is what the services publish.
type reading struct {
	yaw  float64 // degrees, positive to the left, never wrapped
	rate float64 // deg/s, the mean over the rate window
	bias float64 // deg/s, about u
	// at is the newest sample's time; biasAt when the bias was last measured.
	at, biasAt time.Time
	// ready is true once u and the bias are known, biased once the bias is,
	// and recovering just after a gap whose turn was lost.
	ready, biased, recovering bool
}

type timedRate struct {
	at   time.Time
	rate float64 // rad/s
}

// stillBlock accumulates one standstill window.
type stillBlock struct {
	start        time.Time
	n            int
	sumG, sumG2  [3]float64
	sumA         [3]float64
	sumAccelNorm float64
}

type estimator struct {
	cfg  LidarConfig
	logf func(format string, args ...any)

	up     [3]float64
	haveUp bool
	bias   [3]float64 // rad/s, in the sensor's frame
	biased bool
	biasAt time.Time

	block stillBlock

	yaw         float64 // radians
	last        time.Time
	lastRate    float64 // rad/s
	integrating bool
	recent      []timedRate
	gapUntil    time.Time
	lastGapLog  time.Time

	// Counters for the status line.
	samples, stills, offBias, gaps int
}

func newEstimator(cfg LidarConfig) (*estimator, error) {
	e := &estimator{cfg: cfg, logf: logf}
	if cfg.UpVector != nil {
		if len(cfg.UpVector) != 3 {
			return nil, fmt.Errorf("upVector has %d components, not 3", len(cfg.UpVector))
		}
		u := [3]float64{cfg.UpVector[0], cfg.UpVector[1], cfg.UpVector[2]}
		if norm3(u) < 0.5 || norm3(u) > 2 {
			return nil, errors.New("upVector is not close to a unit vector; measure it again")
		}
		e.up, e.haveUp = unit3(u), true
	}
	return e, nil
}

// add takes one sample, in time order.
func (e *estimator) add(s imuSample) {
	if !e.last.IsZero() && !s.at.After(e.last) {
		// The clock map never goes back, so this is the same sample twice.
		return
	}
	e.samples++
	gap := !e.last.IsZero() && s.at.Sub(e.last) > time.Duration(e.cfg.MaxGapMs)*time.Millisecond
	if gap {
		e.gaps++
		e.block = stillBlock{} // a window is only still if it was watched throughout
		if e.integrating {
			e.gapUntil = s.at.Add(time.Duration(e.cfg.StaleAfterMs) * time.Millisecond)
			if s.at.Sub(e.lastGapLog) > 10*time.Second {
				e.logf("%d ms with no IMU sample: the turn made during it is lost", s.at.Sub(e.last).Milliseconds())
				e.lastGapLog = s.at
			}
		}
	}

	e.observe(s)

	if e.haveUp && e.biased {
		rate := dot3(sub3(s.gyro, e.bias), e.up)
		if e.integrating && !gap {
			// The trapezoid rule, on the device's own spacing.
			e.yaw += 0.5 * (e.lastRate + rate) * s.at.Sub(e.last).Seconds()
		}
		e.integrating = true
		e.lastRate = rate
		e.recent = append(e.recent, timedRate{s.at, rate})
		window := time.Duration(e.cfg.RateWindowMs) * time.Millisecond
		cut := 0
		for cut < len(e.recent)-1 && s.at.Sub(e.recent[cut].at) > window {
			cut++
		}
		e.recent = append(e.recent[:0], e.recent[cut:]...)
	}
	e.last = s.at
}

// observe adds a sample to the standstill window, and judges the window once
// it is long enough. Windows do not overlap: a window that saw the vehicle
// start to move is thrown away whole.
func (e *estimator) observe(s imuSample) {
	b := &e.block
	if b.n == 0 {
		b.start = s.at
	}
	b.n++
	for i := range 3 {
		b.sumG[i] += s.gyro[i]
		b.sumG2[i] += s.gyro[i] * s.gyro[i]
		b.sumA[i] += s.accel[i]
	}
	b.sumAccelNorm += norm3(s.accel)
	if s.at.Sub(b.start).Seconds() < e.cfg.StandstillSeconds {
		return
	}
	e.judge(*b, s.at)
	e.block = stillBlock{}
}

// judge decides whether a full window was a standstill, and if it was, takes
// the bias from it, and u too if u is not known yet.
func (e *estimator) judge(b stillBlock, at time.Time) {
	n := float64(b.n)
	var meanG, meanA [3]float64
	worstVar := 0.0
	for i := range 3 {
		meanG[i] = b.sumG[i] / n
		meanA[i] = b.sumA[i] / n
		worstVar = math.Max(worstVar, b.sumG2[i]/n-meanG[i]*meanG[i])
	}
	if math.Abs(b.sumAccelNorm/n-1) > e.cfg.StandstillAccelToleranceG {
		return
	}
	if math.Sqrt(math.Max(worstVar, 0)) > radians(e.cfg.StandstillGyroStdDegPerSec) {
		return
	}
	up := e.up
	if !e.haveUp {
		up = unit3(meanA)
	}
	if e.biased {
		// A steady turn has a small variance too. What gives it away is a
		// mean that has moved from the bias by more than a bias drifts.
		if math.Abs(dot3(sub3(meanG, e.bias), up)) > radians(e.cfg.MaxBiasStepDegPerSec) {
			e.offBias++
			return
		}
	} else if math.Abs(dot3(meanG, up)) > radians(e.cfg.StandstillMaxRateDegPerSec) {
		return
	}

	e.stills++
	if !e.haveUp {
		e.up, e.haveUp = up, true
		e.logf("measured up as %s, a tilt of %.1f° from the sensor's z. This assumes the vehicle stood on level ground; "+
			"to keep it, add \"upVector\": %s to the IMU's traits", vectorJSON(up), tiltDegrees(up), vectorJSON(up))
	}
	first := !e.biased
	e.bias, e.biased, e.biasAt = meanG, true, at
	if first {
		e.logf("bias %.3f°/s about the vertical; yaw starts at 0 from here", degrees(dot3(meanG, e.up)))
	}
}

func (e *estimator) reading() reading {
	r := reading{
		yaw:        degrees(e.yaw),
		at:         e.last,
		biasAt:     e.biasAt,
		ready:      e.haveUp && e.biased,
		biased:     e.biased,
		recovering: e.last.Before(e.gapUntil),
	}
	if e.biased {
		r.bias = degrees(dot3(e.bias, e.up))
	}
	if len(e.recent) > 0 {
		sum := 0.0
		for _, tr := range e.recent {
			sum += tr.rate
		}
		r.rate = degrees(sum / float64(len(e.recent)))
	}
	return r
}

//-------------------------------------Three-vectors

func dot3(a, b [3]float64) float64 { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }

func sub3(a, b [3]float64) [3]float64 { return [3]float64{a[0] - b[0], a[1] - b[1], a[2] - b[2]} }

func norm3(a [3]float64) float64 { return math.Sqrt(dot3(a, a)) }

func unit3(a [3]float64) [3]float64 {
	n := norm3(a)
	return [3]float64{a[0] / n, a[1] / n, a[2] / n}
}

func radians(deg float64) float64 { return deg * math.Pi / 180 }

func degrees(rad float64) float64 { return rad * 180 / math.Pi }
