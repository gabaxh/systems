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
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"math"
	"net/http"
	"sync"
	"time"

	"github.com/sdoque/mbaigo/components"
	"github.com/sdoque/mbaigo/forms"
	"github.com/sdoque/mbaigo/usecases"
)

const (
	unitDegree          = "<http://qudt.org/vocab/unit/DEG>"
	unitDegreePerSecond = "<http://qudt.org/vocab/unit/DEG-PER-SEC>"

	qkAngle           = "<http://qudt.org/vocab/quantitykind/Angle>"
	qkAngularVelocity = "<http://qudt.org/vocab/quantitykind/AngularVelocity>"

	// statusEvery is how often the system says how the stream is doing.
	statusEvery = 30 * time.Second
)

// LidarConfig is what the operator may set. A zero means the default.
type LidarConfig struct {
	// Source is "livox" for the device or "simulated" for a tilted gyro
	// driven through a scripted set of turns. The template ships simulated so
	// the system runs anywhere, including on a laptop with nothing plugged in.
	Source string `json:"source"`

	// DeviceIP is the MID-360's address. Empty means find it by broadcast.
	// HostIP is this computer's address as the device should send to it.
	// Empty means the local address on the route to the device.
	DeviceIP string `json:"deviceIP"`
	HostIP   string `json:"hostIP"`

	// The ports on this computer. The point cloud's is reserved for the
	// point-cloud asset and not opened yet.
	IMUPort        int `json:"imuPort"`
	PointCloudPort int `json:"pointCloudPort"`
	ControlPort    int `json:"controlPort"`

	// ConfigureDevice tells the device at start-up, and again whenever its
	// IMU stream goes quiet, where to send the IMU and to send it. Absent
	// means true; false only listens, for a device set up with Livox Viewer.
	ConfigureDevice *bool `json:"configureDevice"`
	// WorkMode is "sampling" (the laser running) or "idle".
	WorkMode string `json:"workMode"`

	// UpVector is the unit vector pointing up, in the sensor's own frame, for
	// the mount as it is bolted on. The yaw rate is the gyro's rate about it,
	// which keeps all of a turn and none of a roll however the sensor is
	// tilted. Absent, it is measured at the first standstill, which assumes
	// level ground, and logged for pasting here. Re-measure it whenever the
	// mount is touched.
	UpVector []float64 `json:"upVector,omitempty"`

	// The standstill test: the accelerometer's magnitude within
	// StandstillAccelToleranceG of 1 g and the gyro's standard deviation below
	// StandstillGyroStdDegPerSec, both over StandstillSeconds. Before the first
	// bias, the mean rate must also be below StandstillMaxRateDegPerSec; after
	// it, a window may not move the bias by more than MaxBiasStepDegPerSec,
	// and a window that would is taken for slow turning. These are
	// placeholders until the gyro has been characterized on the vehicle.
	StandstillSeconds          float64 `json:"standstillSeconds"`
	StandstillAccelToleranceG  float64 `json:"standstillAccelToleranceG"`
	StandstillGyroStdDegPerSec float64 `json:"standstillGyroStdDegPerSec"`
	StandstillMaxRateDegPerSec float64 `json:"standstillMaxRateDegPerSec"`
	MaxBiasStepDegPerSec       float64 `json:"maxBiasStepDegPerSec"`

	// RateWindowMs is what yawRate averages over. StaleAfterMs is how old the
	// newest sample may be before yaw stops answering. MaxGapMs is the longest
	// hole in the stream that is integrated across; a longer one loses the turn
	// made during it, and yaw then answers 503 for StaleAfterMs more so that a
	// poller cannot miss that it happened.
	RateWindowMs int `json:"rateWindowMs"`
	StaleAfterMs int `json:"staleAfterMs"`
	MaxGapMs     int `json:"maxGapMs"`

	// RecordDir, when set, is where every raw sample is written as CSV, one
	// new file per start, for characterizing the gyro. About 58 MB an hour.
	RecordDir string `json:"recordDir"`

	// The simulator only: the gyro's bias on every axis, its noise per sample,
	// how far the sensor is tilted forward, and what the vehicle does:
	// "cycle" turns, rocks and turns back every 27 s; "still" never moves.
	SimBiasDegPerSec  float64 `json:"simBiasDegPerSec"`
	SimNoiseDegPerSec float64 `json:"simNoiseDegPerSec"`
	SimTiltDegrees    float64 `json:"simTiltDegrees"`
	SimScript         string  `json:"simScript"`
}

// Traits is the sensor, and what a service handler is given.
type Traits struct {
	cfg LidarConfig
	src source
	rec *recorder // nil unless recording; collect's own

	mu  sync.RWMutex
	est *estimator
}

//-------------------------------------Instantiate a unit asset template

func initTemplate() *components.UnitAsset {
	yaw := components.Service{
		Definition: "yaw",
		SubPath:    "yaw",
		Details: map[string][]string{
			"Forms":        {"SignalA_v1a"},
			"Unit":         {unitDegree},
			"QuantityKind": {qkAngle},
			"Methods":      components.HTTPMethods("GET"),
		},
		RegPeriod:     2,
		SubscribeAble: true,
		Description:   "degrees turned about the vertical since the bias was first known, positive to the left and never wrapped; unavailable until then, and when the stream is stale",
	}

	yawRate := components.Service{
		Definition: "yawRate",
		SubPath:    "yawRate",
		Details: map[string][]string{
			"Forms":        {"SignalA_v1a"},
			"Unit":         {unitDegreePerSecond},
			"QuantityKind": {qkAngularVelocity},
			"Methods":      components.HTTPMethods("GET"),
		},
		RegPeriod:     2,
		SubscribeAble: true,
		Description:   "the turn rate about the vertical, averaged over the last few samples; for diagnostics, as yaw is what to integrate",
	}

	bias := components.Service{
		Definition: "bias",
		SubPath:    "bias",
		Details: map[string][]string{
			"Forms":        {"SignalA_v1a"},
			"Unit":         {unitDegreePerSecond},
			"QuantityKind": {qkAngularVelocity},
			"Methods":      components.HTTPMethods("GET"),
		},
		RegPeriod:   5,
		Description: "the gyro's bias about the vertical, as last measured at a standstill",
	}

	configure := true
	return &components.UnitAsset{
		Name:     "IMU",
		Mission:  components.MissionMeasurement,
		Mobility: components.MobilityMovable,
		Details:  map[string][]string{"Model": {"Livox MID-360"}},
		ServicesMap: components.Services{
			yaw.SubPath:     &yaw,
			yawRate.SubPath: &yawRate,
			bias.SubPath:    &bias,
		},
		Traits: &LidarConfig{
			Source:                     "simulated",
			IMUPort:                    56401,
			PointCloudPort:             56301,
			ControlPort:                56101,
			ConfigureDevice:            &configure,
			WorkMode:                   "sampling",
			StandstillSeconds:          2,
			StandstillAccelToleranceG:  0.05,
			StandstillGyroStdDegPerSec: 0.3,
			StandstillMaxRateDegPerSec: 2,
			MaxBiasStepDegPerSec:       0.2,
			RateWindowMs:               50,
			StaleAfterMs:               100,
			MaxGapMs:                   50,
			SimBiasDegPerSec:           0.5,
			SimNoiseDegPerSec:          0.05,
			SimTiltDegrees:             30,
			SimScript:                  "cycle",
		},
	}
}

//-------------------------------------Instantiate the unit asset

func newResource(uac usecases.ConfigurableAsset, sys *components.System) (*components.UnitAsset, func()) {
	cfg := LidarConfig{}
	if len(uac.Traits) > 0 {
		if err := json.Unmarshal(uac.Traits[0], &cfg); err != nil {
			log.Fatalf("%s: cannot parse traits: %v", systemName, err)
		}
	}
	applyDefaults(&cfg)
	if _, ok := workModes[cfg.WorkMode]; !ok {
		log.Fatalf("%s: unknown workMode %q: expected \"sampling\" or \"idle\"", systemName, cfg.WorkMode)
	}

	est, err := newEstimator(cfg)
	if err != nil {
		log.Fatalf("%s: %v", systemName, err)
	}
	src, err := newSource(cfg)
	if err != nil {
		log.Fatalf("%s: %v", systemName, err)
	}

	t := &Traits{cfg: cfg, src: src, est: est}
	if cfg.RecordDir != "" {
		if t.rec, err = newRecorder(cfg.RecordDir, time.Now()); err != nil {
			log.Fatalf("%s: cannot record: %v", systemName, err)
		}
		logf("recording every raw sample to %s", t.rec.path)
	}

	ch, err := src.samples(sys.Ctx)
	if err != nil {
		src.close()
		log.Fatalf("%s: cannot start %s source: %v", systemName, cfg.Source, err)
	}
	go t.collect(sys.Ctx, ch)

	if cfg.Source == "simulated" {
		logf("running against the SIMULATOR — no MID-360 is being read")
	} else {
		logf("listening for the MID-360's IMU on port %d", cfg.IMUPort)
	}
	if est.haveUp {
		logf("up is %s from the configuration, a tilt of %.1f° from the sensor's z", vectorJSON(est.up), tiltDegrees(est.up))
	} else {
		logf("no upVector configured: it will be measured at the first standstill")
	}

	ua := &components.UnitAsset{
		Name:        uac.Name,
		Mission:     uac.Mission,
		Mobility:    uac.Mobility,
		TetheredTo:  uac.TetheredTo,
		Owner:       sys,
		Details:     uac.Details,
		ServicesMap: usecases.MakeServiceMap(uac.Services),
		Traits:      t,
	}
	ua.ServingFunc = func(w http.ResponseWriter, r *http.Request, servicePath string) {
		serving(t, w, r, servicePath)
	}
	return ua, func() {
		src.close()
		logf("sensor closed")
	}
}

func applyDefaults(cfg *LidarConfig) {
	if cfg.Source == "" {
		cfg.Source = "simulated"
	}
	if cfg.IMUPort <= 0 {
		cfg.IMUPort = 56401
	}
	if cfg.PointCloudPort <= 0 {
		cfg.PointCloudPort = 56301
	}
	if cfg.ControlPort <= 0 {
		cfg.ControlPort = 56101
	}
	if cfg.ConfigureDevice == nil {
		configure := true
		cfg.ConfigureDevice = &configure
	}
	if cfg.WorkMode == "" {
		cfg.WorkMode = "sampling"
	}
	if cfg.StandstillSeconds <= 0 {
		cfg.StandstillSeconds = 2
	}
	if cfg.StandstillAccelToleranceG <= 0 {
		cfg.StandstillAccelToleranceG = 0.05
	}
	if cfg.StandstillGyroStdDegPerSec <= 0 {
		cfg.StandstillGyroStdDegPerSec = 0.3
	}
	if cfg.StandstillMaxRateDegPerSec <= 0 {
		cfg.StandstillMaxRateDegPerSec = 2
	}
	if cfg.MaxBiasStepDegPerSec <= 0 {
		cfg.MaxBiasStepDegPerSec = 0.2
	}
	if cfg.RateWindowMs <= 0 {
		cfg.RateWindowMs = 50
	}
	if cfg.StaleAfterMs <= 0 {
		cfg.StaleAfterMs = 100
	}
	if cfg.MaxGapMs <= 0 {
		cfg.MaxGapMs = 50
	}
	if cfg.SimBiasDegPerSec == 0 {
		cfg.SimBiasDegPerSec = 0.5
	}
	if cfg.SimNoiseDegPerSec <= 0 {
		cfg.SimNoiseDegPerSec = 0.05
	}
	if cfg.SimTiltDegrees == 0 {
		cfg.SimTiltDegrees = 30
	}
	if cfg.SimScript == "" {
		cfg.SimScript = "cycle"
	}
}

// collect feeds every sample to the estimator, in order, and to the recorder
// when there is one, and says every statusEvery how the stream is doing.
// Unlike the guetteur's newest sweep, nothing may be skipped here: every
// sample is part of the integral.
func (t *Traits) collect(ctx context.Context, ch <-chan imuSample) {
	report := time.NewTicker(statusEvery)
	defer report.Stop()
	flush := time.NewTicker(time.Second)
	defer flush.Stop()
	defer t.stopRecording(nil)
	for {
		select {
		case <-ctx.Done():
			return
		case s, open := <-ch:
			if !open {
				logf("the IMU source has stopped")
				return
			}
			t.mu.Lock()
			t.est.add(s)
			t.mu.Unlock()
			if t.rec != nil {
				if err := t.rec.write(s); err != nil {
					t.stopRecording(err)
				}
			}
		case <-flush.C:
			if t.rec != nil {
				if err := t.rec.flush(); err != nil {
					t.stopRecording(err)
				}
			}
		case <-report.C:
			t.mu.RLock()
			r, e := t.est.reading(), *t.est
			t.mu.RUnlock()
			state := fmt.Sprintf("bias %.3f°/s, yaw %.1f°", r.bias, r.yaw)
			if !r.biased {
				state = "no standstill yet, so no bias and no yaw"
			}
			logf("%s; %s; %d samples, %d standstills, %d still but off the bias, %d gaps",
				t.src.status(), state, e.samples, e.stills, e.offBias, e.gaps)
		}
	}
}

// stopRecording closes the recording, saying why if it failed. A full disk
// ends the recording, never the system.
func (t *Traits) stopRecording(cause error) {
	if t.rec == nil {
		return
	}
	err := t.rec.close()
	if cause != nil {
		err = cause
	}
	if err != nil {
		logf("recording stopped: %v", err)
	} else {
		logf("recording closed: %s", t.rec.path)
	}
	t.rec = nil
}

// current returns the newest reading, or why it must not be used.
func (t *Traits) current() (reading, error) {
	t.mu.RLock()
	r := t.est.reading()
	t.mu.RUnlock()
	switch {
	case !r.ready:
		return r, errors.New("no bias yet: the IMU needs one standstill first")
	case time.Since(r.at) > time.Duration(t.cfg.StaleAfterMs)*time.Millisecond:
		return r, errors.New("no IMU sample within the freshness window")
	case r.recovering:
		return r, errors.New("just after a gap in the IMU stream: the turn made during it is lost")
	}
	return r, nil
}

//-------------------------------------The services

// yawService answers with the angle turned, never with the rate. Two readings
// of it give the exact turn between their timestamps, at whatever rate they
// are polled; a rate polled at 20 Hz would keep one sample in ten.
//
// It answers 503 rather than a number whenever the number is not to be
// trusted: before the bias is known, when the stream is stale, and just after
// a gap. A consumer differencing two readings across a 503 must not use that
// difference.
func (t *Traits) yawService(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method is not supported.", http.StatusNotFound)
		return
	}
	rd, err := t.current()
	if err != nil {
		http.Error(w, err.Error(), http.StatusServiceUnavailable)
		return
	}
	f := forms.SignalA_v1a{}
	f.NewForm()
	f.Value = rd.yaw
	f.Unit = unitDegree
	f.Timestamp = rd.at
	usecases.HTTPProcessGetRequest(w, r, &f)
}

func (t *Traits) yawRateService(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method is not supported.", http.StatusNotFound)
		return
	}
	rd, err := t.current()
	if err != nil {
		http.Error(w, err.Error(), http.StatusServiceUnavailable)
		return
	}
	f := forms.SignalA_v1a{}
	f.NewForm()
	f.Value = rd.rate
	f.Unit = unitDegreePerSecond
	f.Timestamp = rd.at
	usecases.HTTPProcessGetRequest(w, r, &f)
}

// biasService answers once the bias has been measured, stamped with when it
// was, so a stale stream still shows the last estimate.
func (t *Traits) biasService(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method is not supported.", http.StatusNotFound)
		return
	}
	t.mu.RLock()
	rd := t.est.reading()
	t.mu.RUnlock()
	if !rd.biased {
		http.Error(w, "no standstill yet, so no bias", http.StatusServiceUnavailable)
		return
	}
	f := forms.SignalA_v1a{}
	f.NewForm()
	f.Value = rd.bias
	f.Unit = unitDegreePerSecond
	f.Timestamp = rd.biasAt
	usecases.HTTPProcessGetRequest(w, r, &f)
}

// vectorJSON writes an up vector the way it goes into the configuration.
func vectorJSON(v [3]float64) string {
	return fmt.Sprintf("[%.4f, %.4f, %.4f]", v[0], v[1], v[2])
}

// tiltDegrees is the angle between an up vector and the sensor's own z axis.
func tiltDegrees(up [3]float64) float64 {
	return math.Acos(math.Max(-1, math.Min(1, up[2]))) * 180 / math.Pi
}
