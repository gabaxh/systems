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
	"fmt"
	"math"
	"time"
)

// The speed loop: what keeps a wheel at the speed it was asked for, uphill and
// down.
//
// The drives do not hold a speed. Operating mode 3 is the axis and its output
// with no loop of the drive's own, and 0x77 is a motor command — a share of the
// voltage — not a velocity. The 256 counts per RPM of rpmToCommand is the
// assumption that full voltage turns a wheel at 120 RPM, which holds on the
// flat and nowhere else: uphill the load holds the wheels back, downhill the
// slope pushes them on, and the same stick drove the loader at three speeds.
// The drives cannot close the loop themselves, since the wheel encoders are
// CANopen nodes on the bus and not wired to them.
//
// So the loader closes it. Each wheel's command is its target speed, as before,
// plus a correction from what its encoder says:
//
//	command = reference + kp·error + ki·∫error dt,  error = reference − measured
//
// The reference is the target from the kinematics, ramped; the integral is what
// learns the slope. Only for the vehicle-level velocity: a motor commanded
// directly on the bench gets the open-loop command it asked for, so that
// measurements such as the lowest speed that turns a wheel keep their meaning.
//
// A wheel whose encoder goes quiet falls back to the open-loop command, which
// is the vehicle as it was before this loop. A dead encoder must not read as a
// stalled wheel the loop pushes ever harder on.

// SpeedLoopConfig tunes the wheels' speed loop. Not tuned on the vehicle.
type SpeedLoopConfig struct {
	// OpenLoop turns the loop off and sends the target speed as it is, the
	// vehicle as it drove before the loop.
	OpenLoop bool `json:"openLoop"`
	// ProportionalGain is RPM of correction per RPM of error.
	ProportionalGain float64 `json:"proportionalGain"`
	// IntegralGainPerSecond is RPM of correction per RPM of error per second
	// it lasts. This is the part that holds the speed on a slope.
	IntegralGainPerSecond float64 `json:"integralGainPerSecond"`
	// MaxCorrectionRPM bounds how far the loop may move a wheel's command from
	// its target, either way: a wrong measurement costs at most this much.
	MaxCorrectionRPM float64 `json:"maxCorrectionRPM"`

	// Standing still on a hill; see standstill. HoldRelaxAfterSeconds is how
	// long the loop holds the vehicle actively before letting the drivetrain
	// hold it, 0 for always: the default, until someone has seen that this
	// vehicle stays put on a slope with its motors off. CreepMetres is how far
	// any wheel may roll while resting before the loop catches it, and
	// MaxCatches how many catches in a minute mean the slope needs holding.
	HoldRelaxAfterSeconds float64 `json:"holdRelaxAfterSeconds"`
	CreepMetres           float64 `json:"creepMetres"`
	MaxCatches            int     `json:"maxCatches"`
}

func defaultSpeedLoop() SpeedLoopConfig {
	return SpeedLoopConfig{
		ProportionalGain:      0.3,
		IntegralGainPerSecond: 1.0,
		MaxCorrectionRPM:      60,
		CreepMetres:           0.02,
		MaxCatches:            3,
	}
}

func applySpeedLoopDefaults(s *SpeedLoopConfig) {
	d := defaultSpeedLoop()
	if s.ProportionalGain < 0 {
		s.ProportionalGain = 0
	}
	if s.ProportionalGain == 0 && s.IntegralGainPerSecond == 0 {
		s.ProportionalGain, s.IntegralGainPerSecond = d.ProportionalGain, d.IntegralGainPerSecond
	}
	if s.IntegralGainPerSecond < 0 {
		s.IntegralGainPerSecond = 0
	}
	if s.MaxCorrectionRPM <= 0 {
		s.MaxCorrectionRPM = d.MaxCorrectionRPM
	}
	if s.HoldRelaxAfterSeconds < 0 {
		s.HoldRelaxAfterSeconds = 0
	}
	if s.CreepMetres <= 0 {
		s.CreepMetres = d.CreepMetres
	}
	if s.MaxCatches <= 0 {
		s.MaxCatches = d.MaxCatches
	}
}

// maxFrameGap is the longest gap between two encoder frames that still gives a
// speed. They come every 50 ms; a longer gap is a frame lost, or an encoder
// that has just come back, and the difference across it measures nothing.
const maxFrameGap = 250 * time.Millisecond

// speedLoop is one wheel's loop. It runs every command cycle and learns only
// when its encoder has sent a new frame, at 20 Hz against the loop's 50:
// between frames it holds its correction.
type speedLoop struct {
	active    bool    // closing the loop, as against freshly reset
	open      bool    // fell back to open loop for want of an encoder
	reference float64 // the ramped target, RPM
	integral  float64 // RPM
	measured  float64 // RPM, from the last two frames
	haveSpeed bool

	frameAt  time.Time // the newest frame used
	frameRev float64   // and its revolutions
}

// reset forgets everything the loop has learned. The next command it closes
// starts again from the command the wheel already has.
func (s *speedLoop) reset() {
	*s = speedLoop{}
}

// ramp walks the reference towards the target at the wheels' ramp, in RPM per
// cycle: accelStep and brakeStep are counts of the motor command, 256 to the
// RPM, and the loop is fed the same feel the open-loop command had. It starts
// from from, the wheel's present command, when the loop was not yet running.
func (s *speedLoop) ramp(target, from float64, accelStep, brakeStep int) {
	if !s.active {
		s.reference, s.active = from, true
	}
	last := s.reference
	signsDiffer := (target > 0 && last < 0) || (target < 0 && last > 0)
	towardsZero := (last > 0 && target < last && target >= 0) ||
		(last < 0 && target > last && target <= 0)
	step := float64(accelStep) / rpmToCommand
	if signsDiffer || towardsZero {
		step = float64(brakeStep) / rpmToCommand
	}
	s.reference = math.Max(last-step, math.Min(last+step, target))
}

// step is this cycle's command for the wheel, in RPM, from its reference and
// its encoder's newest reading. fullRPM is the speed full scale stands for,
// where the command saturates. note is non-empty when the loop has just fallen
// back to open loop or come back from it, for the log.
func (s *speedLoop) step(cfg SpeedLoopConfig, r wheelReading, fresh bool, fullRPM float64) (command float64, note string) {
	if !fresh {
		if !s.open {
			note = "no fresh encoder reading, so open loop"
		}
		reference := s.reference
		s.reset()
		s.active, s.open, s.reference = true, true, reference
		return reference, note
	}
	if s.open {
		s.open = false
		note = "the encoder reports again, so closed loop"
	}
	if r.at.After(s.frameAt) {
		gap := r.at.Sub(s.frameAt)
		if !s.frameAt.IsZero() && gap <= maxFrameGap {
			dt := gap.Seconds()
			s.measured = (r.revolutions - s.frameRev) / dt * 60
			s.haveSpeed = true
			s.learn(cfg, dt, fullRPM)
		}
		s.frameAt, s.frameRev = r.at, r.revolutions
	}
	return s.reference + s.correction(cfg), note
}

// track follows the encoder without learning: the loop's measured speed stays
// current while it is not driving the wheel, so that when it takes the wheel
// again it starts from the truth.
func (s *speedLoop) track(r wheelReading, fresh bool) {
	if !fresh || !r.at.After(s.frameAt) {
		return
	}
	if gap := r.at.Sub(s.frameAt); !s.frameAt.IsZero() && gap <= maxFrameGap {
		s.measured = (r.revolutions - s.frameRev) / gap.Seconds() * 60
		s.haveSpeed = true
	}
	s.frameAt, s.frameRev = r.at, r.revolutions
}

// learn moves the integral by one frame's error, and then no further than
// keeps the command inside full scale and the correction inside its bound. An
// integral that grows against a saturated motor is an overshoot waiting for
// the load to go: uphill at full scale it grew while the target was still
// ramping and then stayed, and over the top the wheel ran to 120 RPM for 110.
// The bound never pushes the integral across zero, which would be the loop
// arguing with itself.
func (s *speedLoop) learn(cfg SpeedLoopConfig, dt, fullRPM float64) {
	err := s.reference - s.measured
	p := cfg.ProportionalGain * err
	high := math.Max(0, math.Min(cfg.MaxCorrectionRPM, fullRPM-s.reference)-p)
	low := math.Min(0, math.Max(-cfg.MaxCorrectionRPM, -fullRPM-s.reference)-p)
	next := s.integral + cfg.IntegralGainPerSecond*err*dt
	s.integral = math.Max(low, math.Min(high, next))
}

// correction is what the loop adds to the reference now, held between frames.
func (s *speedLoop) correction(cfg SpeedLoopConfig) float64 {
	if !s.haveSpeed {
		return 0
	}
	c := cfg.ProportionalGain*(s.reference-s.measured) + s.integral
	return math.Max(-cfg.MaxCorrectionRPM, math.Min(cfg.MaxCorrectionRPM, c))
}

//-------------------------------------Standing still on a hill

// standstill is the vehicle standing still while someone has control, with the
// stick centred: the velocity is zero and the wheels have stopped.
//
// The speed loop then holds every wheel at zero RPM, and on a hill that is the
// motors pushing against the slope for as long as the vehicle stands there —
// stall current, which is what heated them on 1 October 2026. If the
// drivetrain holds the vehicle by itself, as gearboxes often do, that is heat
// for nothing. So, when HoldRelaxAfterSeconds is set, the loader holds actively
// for that long, then eases the wheels' effort to zero over half a second and
// watches the encoders. A wheel that rolls CreepMetres is caught: the loop takes
// it again, starting from the effort it had learned, so the vehicle falls back
// a couple of centimetres and no more. MaxCatches catches in a minute mean
// this slope needs holding, and the loader holds actively for as long as the
// vehicle stands there.
//
// With no fresh encoder there is nothing to see a creep with, and the loader
// never rests. A stop, silence or a handover still zero everything at once:
// how the vehicle should stop on a hill is another question.
type standstill struct {
	mode  standMode
	since time.Time // when the mode began

	restAt   map[int]float64 // node ID -> the wheel's distance, metres, when resting began
	restFrom map[int]float64 // node ID -> the command, RPM, it was holding with
	ease     float64         // the share of restFrom still applied, 1 to 0

	catches   []time.Time
	permanent bool

	hardSince time.Time // since when an active hold has pushed hard
	warned    bool
}

type standMode int

const (
	driving standMode = iota
	holding
	resting
)

const (
	// standstillRPM is how slow every wheel must be for the vehicle to be
	// standing still.
	standstillRPM = 0.5
	// easeSeconds is how long resting takes to bring the effort to zero: long
	// enough that a vehicle starting to roll is caught before it is let go.
	easeSeconds = 0.5
	// catchWindow is the time within which MaxCatches catches mean the slope
	// needs holding.
	catchWindow = time.Minute
	// hardHoldPercent and hardHoldFor are when an active hold is worth a
	// warning: it never lets go on its own, but it is the motors' heat.
	hardHoldPercent = 40
	hardHoldFor     = 30 * time.Second
)

// standWheel is one wheel as the standstill sees it this cycle.
type standWheel struct {
	node     int
	still    bool    // target and ramped reference both zero
	fresh    bool    // its encoder reports
	rpm      float64 // measured
	distance float64 // metres rolled, from the encoder
	command  float64 // RPM, what it was given last cycle
}

// update moves the standstill on by one command cycle and says why, when the
// mode changes or something is worth the log.
func (st *standstill) update(cfg SpeedLoopConfig, now time.Time, commandHz int, wheels []standWheel) string {
	still, fresh, stopped := true, true, true
	hardest := 0.0
	for _, w := range wheels {
		still = still && w.still
		fresh = fresh && w.fresh
		if w.fresh && math.Abs(w.rpm) >= standstillRPM {
			stopped = false
		}
		hardest = math.Max(hardest, math.Abs(w.command)*rpmToCommand/fullScale*100)
	}
	if !still {
		st.set(driving, now)
		st.permanent, st.warned, st.hardSince = false, false, time.Time{}
		return ""
	}
	switch st.mode {
	case driving:
		if stopped {
			st.set(holding, now)
		}
	case holding:
		if hardest >= hardHoldPercent {
			if st.hardSince.IsZero() {
				st.hardSince = now
			}
		} else {
			st.hardSince = time.Time{}
		}
		if !st.warned && !st.hardSince.IsZero() && now.Sub(st.hardSince) >= hardHoldFor {
			st.warned = true
			return fmt.Sprintf("holding on the slope at %.0f%% effort for %v — that is the motors' heat; "+
				"if the vehicle stays put with its motors off, holdRelaxAfterSeconds lets it rest", hardest, hardHoldFor)
		}
		if cfg.HoldRelaxAfterSeconds > 0 && !st.permanent && fresh && stopped &&
			now.Sub(st.since).Seconds() >= cfg.HoldRelaxAfterSeconds {
			st.set(resting, now)
			st.restAt, st.restFrom, st.ease = map[int]float64{}, map[int]float64{}, 1
			for _, w := range wheels {
				st.restAt[w.node], st.restFrom[w.node] = w.distance, w.command
			}
			return fmt.Sprintf("standing still for %gs: resting, with the drivetrain holding the vehicle", cfg.HoldRelaxAfterSeconds)
		}
	case resting:
		if !fresh {
			st.set(holding, now)
			return "a wheel encoder went quiet while resting, so a creep could not be seen: holding actively"
		}
		crept := 0.0
		for _, w := range wheels {
			crept = math.Max(crept, math.Abs(w.distance-st.restAt[w.node]))
		}
		if crept > cfg.CreepMetres {
			st.set(holding, now)
			kept := st.catches[:0]
			for _, at := range st.catches {
				if now.Sub(at) < catchWindow {
					kept = append(kept, at)
				}
			}
			st.catches = append(kept, now)
			if len(st.catches) >= cfg.MaxCatches {
				st.permanent = true
				return fmt.Sprintf("the vehicle crept %.0f cm while resting, %d times in a minute: this slope needs "+
					"holding, so holding actively for as long as it stands here", crept*100, len(st.catches))
			}
			return fmt.Sprintf("the vehicle crept %.0f cm while resting: caught, holding actively", crept*100)
		}
		st.ease = math.Max(0, st.ease-1/(easeSeconds*float64(commandHz)))
	}
	return ""
}

func (st *standstill) set(m standMode, now time.Time) {
	if st.mode != m {
		st.mode, st.since = m, now
	}
}

// restingCommand is a wheel's command while resting, in RPM: its holding
// command, eased towards zero.
func (st *standstill) restingCommand(node int) float64 {
	return st.restFrom[node] * st.ease
}
