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
	"context"
	"encoding/json"
	"fmt"
	"log"
	"math"
	"net/http"
	"slices"
	"sync"
	"time"

	"github.com/sdoque/mbaigo/components"
	"github.com/sdoque/mbaigo/forms"
	"github.com/sdoque/mbaigo/usecases"
)

// The motors are driven by Magellan motion-control ICs reached over CAN at
// 0x600 + node ID. These opcodes and the initialization order below are taken
// from the artitrax can_dds bridge, which is the tested reference; the delays
// are its delays and are not decoration — a motor that is not given them does
// not come up.
const (
	motorBase             = 0x600
	cmdUpdate             = 0x1A
	cmdReset              = 0x39
	cmdSetCurrentFoldback = 0x41
	cmdSetOperatingMode   = 0x65
	cmdSetVelocity        = 0x77
)

// rpmToCommand converts a wheel speed to the controller's own integer scale:
// 120 RPM is 0x7800, so one RPM is 256 counts. fullScale is the clamp that
// keeps a command inside int16 and inside what the drive will accept.
const (
	rpmToCommand = 0x7800 / 120.0 // 256 counts per RPM
	fullScale    = 0x7800         // 30720
)

// LoaderConfig is what the operator may set. Every field has a working default
// so the generated file needs no editing to drive a motor.
type LoaderConfig struct {
	Interface string `json:"canInterface"`
	// SensorInterface is the second bus, carrying the waist angle sensor alone.
	// The reference brings it up at 250 kbit/s while the motor bus runs at 500.
	// Empty means the sensor is not fitted, and the waist service then says so
	// rather than inventing an angle.
	SensorInterface string `json:"canSensorInterface"`
	// WaistPollHz is how often the articulation sensor is asked; it answers only
	// when polled.
	WaistPollHz int `json:"waistPollHz"`
	// FeedbackStaleMs is how old a measurement may be before it stops counting
	// as one.
	FeedbackStaleMs int `json:"feedbackStaleMs"`
	CommandHz       int `json:"commandHz"`
	// SafetyStopMs is how long the system in control may say nothing before
	// the vehicle stops.
	SafetyStopMs int     `json:"safetyStopMs"`
	MaxWheelRPM  float64 `json:"maxWheelRPM"`
	AccelStep    int     `json:"accelStep"`
	BrakeStep    int     `json:"brakeStep"`
	// WheelStallPercent and WheelStallMs are the wheels' watchdog: a wheel
	// asked to move and given at least WheelStallPercent of full scale for
	// WheelStallMs without turning stops the vehicle. See watchWheelsLocked.
	WheelStallPercent float64 `json:"wheelStallPercent"`
	WheelStallMs      int     `json:"wheelStallMs"`
	// Priority names the systems that may take control from anyone, and the
	// only ones that may take it after a stop. See helm.go.
	Priority []string `json:"priority"`

	// MaxSpeed caps the vehicle-level velocity command, in m/s.
	MaxSpeed float64 `json:"maxSpeedMetresPerSecond"`
	// Geometry is the vehicle's dimensions, and Waist the articulation
	// sensor's calibration and the steering limits. See kinematics.go and
	// waist.go: these are what make the loader drive a vehicle rather than
	// five motors, and what change when it is moved to another machine.
	Geometry Geometry    `json:"geometry"`
	Waist    WaistConfig `json:"waist"`
	// SpeedLoop holds each wheel at its speed against the slope; see
	// speedloop.go.
	SpeedLoop SpeedLoopConfig `json:"speedLoop"`

	Motors []MotorSpec `json:"motors"`
}

// MotorSpec names one motor on the bus. Kind decides how a setpoint is read: a
// wheel is commanded in RPM, the steering in percent of full effort, positive
// to the left. A wheel also says where it is, which is what the kinematics
// need to know which wheel is on the inside of a curve.
type MotorSpec struct {
	Name   string `json:"name"`
	NodeID int    `json:"nodeID"`
	Kind   string `json:"kind"`           // "wheel" or "steering"
	Axle   string `json:"axle,omitempty"` // "front" or "back", for a wheel
	Side   string `json:"side,omitempty"` // "left" or "right", for a wheel
}

// position is the wheel's place in the kinematics' order, or -1.
func (m MotorSpec) position() int {
	switch m.Axle + "/" + m.Side {
	case "front/left":
		return frontLeft
	case "front/right":
		return frontRight
	case "back/left":
		return backLeft
	case "back/right":
		return backRight
	}
	return -1
}

// circumferenceOfNode is the rolling circumference of the wheel driven by motor
// node. The encoders are numbered by node and the kinematics by axle and side,
// and this goes through the motor list rather than assuming the two agree.
func (c LoaderConfig) circumferenceOfNode(node int) float64 {
	for _, m := range c.Motors {
		if m.Kind == "wheel" && m.NodeID == node {
			return c.Geometry.circumferenceAt(m.position())
		}
	}
	return c.Geometry.WheelCircumference
}

// drivetrain is the state the five assets share: one CAN socket, one setpoint
// per motor, and the loop that keeps writing them.
type drivetrain struct {
	cfg LoaderConfig
	fd  int
	sys *components.System

	fb *feedback

	mu       sync.Mutex
	helm     *helm
	setpoint map[int]float64 // node ID -> RPM (wheel) or percent (steering)
	last     map[int]int16   // node ID -> last raw command, for rate limiting

	// The vehicle-level command. While byVelocity is set the wheels follow
	// velocity through the kinematics and their own setpoints are written
	// from it; while byAngle is set the steering follows curvature through
	// the angle loop. A direct setpoint on a motor clears the matching flag.
	velocity   float64 // m/s, of the front axle
	curvature  float64 // 1/m, positive to the left
	byVelocity bool
	byAngle    bool

	waist  *waistState
	loops  map[int]*speedLoop // node ID -> the wheel's speed loop
	shared sharedLoop         // the integral the wheels share; see speedloop.go
	stand  standstill         // standing still on a hill; see speedloop.go

	// The wheels' watchdog: since when each wheel has been driven hard without
	// turning, and the fault it raised, cleared by taking control again.
	stallSince map[int]time.Time
	wheelFault string
}

// Traits is one motor, and what a service handler is given.
type Traits struct {
	Name   string
	NodeID int
	Kind   string
	unit   string
	// encoderIndex is this motor's place in the wheel-encoder frames, or -1
	// when it has none.
	encoderIndex int
	dt           *drivetrain
	ua           *components.UnitAsset
}

// servicesFor keeps only the services a given asset can actually answer. Every
// asset is built from the same configured list, but a steering motor has no
// wheel encoder, a wheel has no articulation sensor, and control belongs to the
// vehicle rather than to any one motor. A service that is registered but cannot
// answer is worse than one that was never offered.
func servicesFor(kind string, configured []components.Service) components.Services {
	svcs := usecases.MakeServiceMap(configured)
	keep := map[string][]string{
		"wheel":    {"setpoint", "speed", "travel", "distance"},
		"steering": {"setpoint", "waist"},
		"vehicle":  {"control", "stop", "velocity", "curvature", "articulation", "speedLimit", "curvatureLimit"},
	}[kind]
	for name := range svcs {
		if !slices.Contains(keep, name) {
			delete(svcs, name)
		}
	}
	return svcs
}

// vehicleServices are the ones the helm answers. A configuration file written
// before they existed does not list them, and a loader started from it would
// register no way to take control — and so could never be driven, with nothing
// in the log to say why.
var vehicleServices = []string{"control", "stop", "velocity", "curvature", "articulation",
	"speedLimit", "curvatureLimit", "distance"}

//-------------------------------------Instantiate a unit asset template

func initTemplate() *components.UnitAsset {
	setpoint := components.Service{
		Definition: "setpoint",
		SubPath:    "setpoint",
		Details: map[string][]string{
			"Forms":   {"SignalA_v1a"},
			"Unit":    {"<http://qudt.org/vocab/unit/REV-PER-MIN>"},
			"Methods": components.HTTPMethods("GET", "PUT"),
		},
		RegPeriod:   30,
		Description: "what this motor is commanded to (GET), or commands it (PUT): RPM for a wheel, percent of effort positive to the left for the steering",
	}

	speed := components.Service{
		Definition: "speed",
		SubPath:    "speed",
		Details: map[string][]string{
			"Forms":        {"SignalA_v1a"},
			"Unit":         {"<http://qudt.org/vocab/unit/REV-PER-MIN>"},
			"QuantityKind": {"<http://qudt.org/vocab/quantitykind/AngularVelocity>"},
			"Methods":      components.HTTPMethods("GET"),
		},
		RegPeriod:     2,
		SubscribeAble: true,
		// A reading that does not change is not sent, so a wheel standing
		// still would be heard from only at the heartbeat. One second, the
		// framework's shortest, lets a follower tell it from a dead encoder:
		// the heartbeat carries the newest reading and its timestamp.
		Heartbeat:   1,
		Description: "the speed this wheel is measured to be turning, from its encoder",
	}

	travel := components.Service{
		Definition: "travel",
		SubPath:    "travel",
		Details: map[string][]string{
			"Forms":   {"SignalA_v1a"},
			"Unit":    {"<http://qudt.org/vocab/unit/REV>"},
			"Methods": components.HTTPMethods("GET"),
		},
		RegPeriod:     2,
		SubscribeAble: true,
		// A reading that does not change is not sent, so a wheel standing
		// still would be heard from only at the heartbeat. One second, the
		// framework's shortest, lets a follower tell it from a dead encoder:
		// the heartbeat carries the newest reading and its timestamp.
		Heartbeat:   1,
		Description: "revolutions this wheel has turned since the loader started",
	}

	waist := components.Service{
		Definition: "waist",
		SubPath:    "waist",
		Details: map[string][]string{
			"Forms":   {"SignalA_v1a"},
			"Methods": components.HTTPMethods("GET"),
		},
		RegPeriod:     2,
		SubscribeAble: true,
		// A reading that does not change is not sent, so a wheel standing
		// still would be heard from only at the heartbeat. One second, the
		// framework's shortest, lets a follower tell it from a dead encoder:
		// the heartbeat carries the newest reading and its timestamp.
		Heartbeat:   1,
		Description: "the articulation sensor's raw 10-bit reading, for calibrating it; articulation gives degrees",
	}

	distance := components.Service{
		Definition: "distance",
		SubPath:    "distance",
		Details: map[string][]string{
			"Forms":        {"SignalA_v1a"},
			"Unit":         {"<http://qudt.org/vocab/unit/M>"},
			"QuantityKind": {"<http://qudt.org/vocab/quantitykind/Length>"},
			"Methods":      components.HTTPMethods("GET"),
		},
		RegPeriod:     2,
		SubscribeAble: true,
		Heartbeat:     1,
		Description:   "meters this wheel has rolled since the loader started, from its encoder and the wheel's circumference",
	}

	velocity := components.Service{
		Definition: "velocity",
		SubPath:    "velocity",
		Details: map[string][]string{
			"Forms":        {"SignalA_v1a"},
			"Unit":         {"<http://qudt.org/vocab/unit/M-PER-SEC>"},
			"QuantityKind": {"<http://qudt.org/vocab/quantitykind/Velocity>"},
			"Methods":      components.HTTPMethods("GET", "PUT"),
		},
		RegPeriod:   30,
		Description: "the speed of the front axle, negative in reverse; PUT commands it, and needs control",
	}

	curvature := components.Service{
		Definition: "curvature",
		SubPath:    "curvature",
		Details: map[string][]string{
			"Forms":   {"SignalA_v1a"},
			"Unit":    {"<http://qudt.org/vocab/unit/PER-M>"},
			"Methods": components.HTTPMethods("GET", "PUT"),
		},
		RegPeriod:   30,
		Description: "the curvature of the front axle's path, positive to the left (ISO 8855); PUT commands it, and needs control and a calibrated waist",
	}

	articulation := components.Service{
		Definition: "articulation",
		SubPath:    "articulation",
		Details: map[string][]string{
			"Forms":        {"SignalA_v1a"},
			"Unit":         {"<http://qudt.org/vocab/unit/DEG>"},
			"QuantityKind": {"<http://qudt.org/vocab/quantitykind/Angle>"},
			"Methods":      components.HTTPMethods("GET"),
		},
		RegPeriod:     2,
		SubscribeAble: true,
		Heartbeat:     1,
		Description:   "the waist's measured angle, positive to the left; unavailable until the sensor is calibrated",
	}

	speedLimit := components.Service{
		Definition: "speedLimit",
		SubPath:    "speedLimit",
		Details: map[string][]string{
			"Forms":   {"SignalA_v1a"},
			"Unit":    {"<http://qudt.org/vocab/unit/M-PER-SEC>"},
			"Methods": components.HTTPMethods("GET"),
		},
		RegPeriod:   30,
		Description: "the largest velocity this vehicle will accept",
	}

	curvatureLimit := components.Service{
		Definition: "curvatureLimit",
		SubPath:    "curvatureLimit",
		Details: map[string][]string{
			"Forms":   {"SignalA_v1a"},
			"Unit":    {"<http://qudt.org/vocab/unit/PER-M>"},
			"Methods": components.HTTPMethods("GET"),
		},
		RegPeriod:   30,
		Description: "the tightest curvature this vehicle can follow, either way: one over its smallest turning radius",
	}

	control := components.Service{
		Definition: "control",
		SubPath:    "control",
		Details: map[string][]string{
			"Forms":   {"SignalA_v1a"},
			"Methods": components.HTTPMethods("GET", "PUT"),
		},
		RegPeriod:   30,
		Description: "PUT 1 to take control of the vehicle, 0 to release it; GET says whether the caller has it",
	}

	stop := components.Service{
		Definition: "stop",
		SubPath:    "stop",
		Details: map[string][]string{
			"Forms":   {"SignalA_v1a"},
			"Methods": components.HTTPMethods("GET", "PUT"),
		},
		RegPeriod:   30,
		Description: "PUT 1 to stop the vehicle, whoever has control; GET says whether it is stopped",
	}

	return &components.UnitAsset{
		Name:    "Drivetrain",
		Mission: components.MissionActuation,
		// Fixed: it needs this host's CAN bus. It said movable, which would
		// have let a balancer propose moving it to a machine with no vehicle.
		Mobility: components.MobilityFixed,
		Details:  map[string][]string{"Model": {"artitrax"}, "FunctionalLocation": {"Loader"}},
		ServicesMap: components.Services{
			setpoint.SubPath: &setpoint,
			speed.SubPath:    &speed,
			travel.SubPath:   &travel,
			waist.SubPath:    &waist,
			control.SubPath:  &control,
			stop.SubPath:     &stop,

			distance.SubPath:       &distance,
			velocity.SubPath:       &velocity,
			curvature.SubPath:      &curvature,
			articulation.SubPath:   &articulation,
			speedLimit.SubPath:     &speedLimit,
			curvatureLimit.SubPath: &curvatureLimit,
		},
		Traits: &LoaderConfig{
			Interface:       "can0",
			SensorInterface: "can1",
			// Once a command cycle: the steering limit and the angle loop are
			// only as good as the newest reading, and the joint moves between
			// readings. The sensor is alone on its bus.
			WaistPollHz:     50,
			FeedbackStaleMs: 500,
			// The reference's own cycle: can_dds writes every 20 ms. The drives
			// treat a command older than 100 ms as stale, so anything above ten
			// works, but the ramp steps below are per cycle and were tuned at
			// this rate.
			CommandHz: 50,
			// The vehicle stops if the system in control says nothing for this
			// long. Every other actuator in this cloud holds its last state when
			// the controller goes quiet, which is right for a heater and wrong
			// for something with wheels. can_dds allows 100 ms; this allows
			// half a second, for a command that crosses the network.
			SafetyStopMs: 500,
			MaxWheelRPM:  120,
			// The values can_dds passes in main.cpp — not the MotorController
			// constructor's defaults of 30 and 100, which is what this system
			// first copied. At 20 Hz those took 51 s to reach full speed and
			// 15 s to stop from it; at these values and 50 Hz it is 4.1 s and
			// 1.2 s, which is what the vehicle has always done under can_dds.
			AccelStep: 150,
			BrakeStep: 500,
			// Two seconds at 40% with a wheel standing still is a stall, not a
			// start: on 1 October 2026 full stick drove all four into a stall
			// until the drives cut out hot.
			WheelStallPercent: 40,
			WheelStallMs:      2000,
			Priority:          []string{"gamer"},
			MaxSpeed:          1.5,
			// Measured on the Artitrax: L1 = L2, together 1235 mm; the wheels
			// 627.5 mm apart; a rolling circumference of 1335 mm, unloaded.
			Geometry: Geometry{
				JointToFront:       0.6175,
				JointToRear:        0.6175,
				Track:              0.6275,
				WheelCircumference: 1.335,
			},
			Waist:     defaultWaist(),
			SpeedLoop: defaultSpeedLoop(),
			Motors: []MotorSpec{
				{Name: "FrontLeft", NodeID: 1, Kind: "wheel", Axle: "front", Side: "left"},
				{Name: "FrontRight", NodeID: 2, Kind: "wheel", Axle: "front", Side: "right"},
				{Name: "BackLeft", NodeID: 3, Kind: "wheel", Axle: "back", Side: "left"},
				{Name: "BackRight", NodeID: 4, Kind: "wheel", Axle: "back", Side: "right"},
				{Name: "Steering", NodeID: 5, Kind: "steering"},
			},
		},
	}
}

//-------------------------------------Instantiate the unit assets

// newResource turns the one configured asset into one unit asset per motor, the
// same expansion the busdriver does for signals: the operator describes the
// vehicle once and gets five addressable motors.
func newResource(configuredAsset usecases.ConfigurableAsset, sys *components.System) ([]*components.UnitAsset, func()) {
	cfg := LoaderConfig{}
	if len(configuredAsset.Traits) > 0 {
		if err := json.Unmarshal(configuredAsset.Traits[0], &cfg); err != nil {
			log.Fatalf("loader: cannot parse traits: %v", err)
		}
	}
	applyDefaults(&cfg)
	if err := cfg.Geometry.checkCircumferences(); err != nil {
		log.Fatalf("loader: %v", err)
	}
	log.Printf("loader: wheel circumferences %.4f / %.4f m front, %.4f / %.4f m back (left / right)",
		cfg.Geometry.circumferenceAt(frontLeft), cfg.Geometry.circumferenceAt(frontRight),
		cfg.Geometry.circumferenceAt(backLeft), cfg.Geometry.circumferenceAt(backRight))
	if len(configuredAsset.Traits) > 0 {
		for _, k := range staleWaistKeys(configuredAsset.Traits[0]) {
			hint := ""
			if k == "minEffortPercent" {
				hint = "; if it was measured, it is the breakaway, and waist.breakawayPercent takes it"
			}
			log.Printf("loader: waist.%s is gone and ignored — see the README's angle loop%s", k, hint)
		}
	}
	if err := cfg.Geometry.checkCircumferences(); err != nil {
		log.Fatalf("loader: %v", err)
	}
	log.Printf("loader: wheel circumferences %.4f / %.4f m front, %.4f / %.4f m back (left / right)",
		cfg.Geometry.circumferenceAt(frontLeft), cfg.Geometry.circumferenceAt(frontRight),
		cfg.Geometry.circumferenceAt(backLeft), cfg.Geometry.circumferenceAt(backRight))

	fd, err := openCAN(cfg.Interface)
	if err != nil {
		log.Fatalf("loader: cannot open %s: %v", cfg.Interface, err)
	}
	log.Printf("loader: opened %s", cfg.Interface)
	// The command socket only writes. Unfiltered, the kernel would queue every
	// frame on the bus for it, and nobody would ever read them.
	if err := filterCAN(fd); err != nil {
		log.Printf("loader: cannot filter the command socket on %s: %v", cfg.Interface, err)
	}

	configured := usecases.MakeServiceMap(configuredAsset.Services)
	for _, name := range vehicleServices {
		if _, ok := configured[name]; !ok {
			closeCAN(fd)
			log.Fatalf("loader: the configuration lists no %q service, so the vehicle could never be driven; "+
				"it was written by an older version — delete systemconfig.json and start again to regenerate it", name)
		}
	}

	for _, m := range cfg.Motors {
		if m.Kind == "wheel" && m.position() < 0 {
			closeCAN(fd)
			log.Fatalf("loader: wheel %s says nothing about where it is (axle and side); the kinematics "+
				"need it — delete systemconfig.json and start again to regenerate it", m.Name)
		}
	}

	dt := &drivetrain{
		cfg:      cfg,
		fd:       fd,
		sys:      sys,
		fb:       newFeedback(time.Duration(cfg.FeedbackStaleMs) * time.Millisecond),
		helm:     newHelm(cfg.Priority),
		setpoint: make(map[int]float64),
		last:     make(map[int]int16),
		waist:    newWaistState(cfg.Waist),
	}
	dt.logCalibration()

	// A second socket on the motor bus, for reading. One socket shared between
	// the command loop and the encoder listener would make them take turns, and
	// the command loop must never wait on a sensor: the drives zero their
	// outputs if they are not spoken to.
	encFd, err := openCAN(cfg.Interface)
	if err != nil {
		closeCAN(fd)
		log.Fatalf("loader: cannot open %s for encoder feedback: %v", cfg.Interface, err)
	}
	// Only the encoders' frames, not the motor commands the other socket sends,
	// which the kernel loops back to this one five hundred times a second.
	encoderIDs := make([]uint32, encoderCount)
	for i := range encoderIDs {
		encoderIDs[i] = uint32(encoderBaseID + i)
	}
	if err := filterCAN(encFd, encoderIDs...); err != nil {
		log.Printf("loader: cannot filter the encoder socket on %s: %v", cfg.Interface, err)
	}
	configureEncoders(encFd, true)
	go dt.fb.listenEncoders(sys.Ctx, encFd)
	go dt.fb.superviseEncoders(sys.Ctx, encFd)

	waistFd := -1
	if cfg.SensorInterface != "" {
		waistFd, err = openCAN(cfg.SensorInterface)
		if err != nil {
			log.Printf("loader: no waist angle sensor on %s (%v) — the waist service will report unavailable", cfg.SensorInterface, err)
			waistFd = -1
		} else {
			// Only the sensor's answers; pollOnce discards anything else that
			// gets through, but a frame that never queues costs nothing.
			if err := filterCAN(waistFd, waistReplyID); err != nil {
				log.Printf("loader: cannot filter the waist sensor socket on %s: %v", cfg.SensorInterface, err)
			}
			go dt.fb.pollWaist(sys.Ctx, waistFd, time.Second/time.Duration(cfg.WaistPollHz))
		}
	}

	if err := dt.initMotors(); err != nil {
		closeCAN(fd)
		log.Fatalf("loader: motor initialization failed: %v", err)
	}

	var assets []*components.UnitAsset
	for _, m := range cfg.Motors {
		t := &Traits{Name: m.Name, NodeID: m.NodeID, Kind: m.Kind, dt: dt, encoderIndex: -1}
		// Wheel encoders answer on 0x18B..0x18E in motor-node order, so node 1
		// is index 0. The steering motor has no wheel encoder; the articulation
		// is measured by its own sensor on the other bus.
		if m.Kind == "wheel" && m.NodeID >= 1 && m.NodeID <= encoderCount {
			t.encoderIndex = m.NodeID - 1
		}
		t.unit = "<http://qudt.org/vocab/unit/REV-PER-MIN>"
		if m.Kind == "steering" {
			t.unit = "<http://qudt.org/vocab/unit/PERCENT>"
		}

		details := copyDetails(configuredAsset.Details)
		details["Unit"] = []string{t.unit}
		details["NodeID"] = []string{fmt.Sprintf("%d", m.NodeID)}

		ua := &components.UnitAsset{
			Name:        m.Name,
			Mission:     configuredAsset.Mission,
			Mobility:    configuredAsset.Mobility,
			TetheredTo:  configuredAsset.TetheredTo,
			Owner:       sys,
			Details:     details,
			ServicesMap: servicesFor(m.Kind, configuredAsset.Services),
			Traits:      t,
		}
		ua.ServingFunc = func(w http.ResponseWriter, r *http.Request, servicePath string) {
			serving(t, w, r, servicePath)
		}
		t.ua = ua
		assets = append(assets, ua)
	}

	// The vehicle itself, which is what control and stop act on.
	vt := &Traits{Name: "Vehicle", Kind: "vehicle", dt: dt, encoderIndex: -1}
	vehicle := &components.UnitAsset{
		Name:        vt.Name,
		Mission:     configuredAsset.Mission,
		Mobility:    configuredAsset.Mobility,
		TetheredTo:  configuredAsset.TetheredTo,
		Owner:       sys,
		Details:     copyDetails(configuredAsset.Details),
		ServicesMap: servicesFor(vt.Kind, configuredAsset.Services),
		Traits:      vt,
	}
	vehicle.ServingFunc = func(w http.ResponseWriter, r *http.Request, servicePath string) {
		serving(vt, w, r, servicePath)
	}
	vt.ua = vehicle
	assets = append(assets, vehicle)

	dt.publishFeedback(assets)
	log.Printf("loader: the vehicle is stopped until %s takes control", dt.helm.priorityNames())

	go dt.run(sys.Ctx)

	return assets, func() {
		dt.stopAll()
		closeCAN(fd)
		closeCAN(encFd)
		if waistFd >= 0 {
			closeCAN(waistFd)
		}
		log.Println("loader: motors stopped and CAN closed")
	}
}

func applyDefaults(cfg *LoaderConfig) {
	if cfg.Interface == "" {
		cfg.Interface = "can0"
	}
	if cfg.CommandHz <= 0 {
		cfg.CommandHz = 50
	}
	if cfg.SafetyStopMs <= 0 {
		cfg.SafetyStopMs = 500
	}
	if cfg.MaxWheelRPM <= 0 {
		cfg.MaxWheelRPM = 120
	}
	if cfg.AccelStep <= 0 {
		cfg.AccelStep = 150
	}
	if cfg.BrakeStep <= 0 {
		cfg.BrakeStep = 500
	}
	if cfg.WaistPollHz <= 0 {
		cfg.WaistPollHz = 50
	}
	if cfg.MaxSpeed <= 0 {
		cfg.MaxSpeed = 1.5
	}
	if cfg.Geometry.WheelCircumference <= 0 || cfg.Geometry.Track <= 0 ||
		cfg.Geometry.JointToFront <= 0 || cfg.Geometry.JointToRear <= 0 {
		perWheel := cfg.Geometry.PerWheel
		cfg.Geometry = Geometry{JointToFront: 0.6175, JointToRear: 0.6175, Track: 0.6275, WheelCircumference: 1.335}
		cfg.Geometry.PerWheel = perWheel
	}
	applyWaistDefaults(&cfg.Waist)
	applySpeedLoopDefaults(&cfg.SpeedLoop)
	if cfg.WheelStallPercent <= 0 {
		cfg.WheelStallPercent = 40
	}
	if cfg.WheelStallMs <= 0 {
		cfg.WheelStallMs = 2000
	}
	if cfg.FeedbackStaleMs <= 0 {
		cfg.FeedbackStaleMs = 500
	}
}

//-------------------------------------The CAN conversation

// initMotors runs the three-step bring-up on every motor. The delays are the
// reference implementation's and the motors need them.
func (d *drivetrain) initMotors() error {
	for _, m := range d.cfg.Motors {
		id := uint32(motorBase + m.NodeID)
		if err := sendCAN(d.fd, id, []byte{0x00, cmdReset}); err != nil {
			return fmt.Errorf("reset node %d: %w", m.NodeID, err)
		}
		time.Sleep(500 * time.Millisecond)
		if err := sendCAN(d.fd, id, []byte{0x00, cmdSetCurrentFoldback, 0x00, 0x00, 0x98, 0x8F}); err != nil {
			return fmt.Errorf("current foldback node %d: %w", m.NodeID, err)
		}
		time.Sleep(500 * time.Millisecond)
		if err := sendCAN(d.fd, id, []byte{0x00, cmdSetOperatingMode, 0x00, 0x03}); err != nil {
			return fmt.Errorf("operating mode node %d: %w", m.NodeID, err)
		}
		time.Sleep(50 * time.Millisecond)
		log.Printf("loader: %s (node %d) initialized", m.Name, m.NodeID)
	}
	return nil
}

// setVelocity writes one raw command and tells the drive to act on it.
func (d *drivetrain) setVelocity(nodeID int, raw int16) error {
	id := uint32(motorBase + nodeID)
	u := uint16(raw)
	if err := sendCAN(d.fd, id, []byte{0x00, cmdSetVelocity, byte(u >> 8), byte(u)}); err != nil {
		return err
	}
	return sendCAN(d.fd, id, []byte{0x00, cmdUpdate})
}

// run keeps the motors fed. Nothing here reads a request: the services move a
// setpoint, and this loop is what puts it on the wire, often enough that the
// drives never see a stale command.
func (d *drivetrain) run(ctx context.Context) {
	tick := time.NewTicker(time.Second / time.Duration(d.cfg.CommandHz))
	defer tick.Stop()
	for {
		select {
		case <-ctx.Done():
			d.stopAll()
			return
		case <-tick.C:
			d.writeCycle()
		}
	}
}

// writeCycle sends every motor its rate-limited command, or zero if no client
// has said anything recently.
//
// In order: the steering watchdog looks at what the last cycle did; the wheels
// are set from the velocity command, if there is one, through the kinematics;
// the steering effort is worked out, from the curvature command or its own
// setpoint, and passed through the guard; and everything is rate-limited and
// sent.
func (d *drivetrain) writeCycle() {
	d.writeCycleAt(time.Now())
}

// writeCycleAt is writeCycle at a given time, which a test can simulate.
func (d *drivetrain) writeCycleAt(now time.Time) {
	d.mu.Lock()
	if d.helm.silent(now, time.Duration(d.cfg.SafetyStopMs)*time.Millisecond) {
		log.Printf("loader: stopped — %s", d.helm.why)
		d.haltLocked()
	}

	raw, at, fresh := d.waistReadingLocked(now)
	d.followJointLocked(raw, at, fresh)
	steer, hasSteering := d.steeringMotor()

	// The watchdog judges the effort the motor has actually been given.
	if hasSteering {
		applied := float64(d.last[steer.NodeID]) / fullScale * 100 * d.waist.rawEffortSign()
		fault, observed := d.waist.watch(applied, raw, fresh, now)
		if observed != "" && d.waist.every("observed", 5*time.Second, now) {
			log.Printf("loader: %s", observed)
		}
		if fault != "" && d.waist.fault == "" {
			d.waist.fault = fault
			log.Printf("loader: STOP — %s; take control again to clear it", fault)
			d.helm.stop(caller{name: "the loader's steering watchdog", known: true}, "stopped")
			d.haltLocked()
		}
	}

	if d.byVelocity && !d.helm.stopped {
		gamma, rate := d.jointForWheelsLocked(raw, fresh)
		rpm := d.cfg.Geometry.wheelRPMs(d.cfg.Geometry.wheelSpeedsMoving(d.velocity, gamma, rate), d.cfg.MaxWheelRPM)
		for _, m := range d.cfg.Motors {
			if pos := m.position(); m.Kind == "wheel" && pos >= 0 {
				d.setpoint[m.NodeID] = rpm[pos]
			}
		}
	}

	// The wheels ramp in RPM steps; the steering builds up by its own, much
	// faster ramp, since the waist does nothing until its effort passes the
	// breakaway, and lets go at once (steerLimit). Driven by velocity, a wheel
	// ramps its target instead and closes its speed loop on it; anything else
	// leaves the loops to start afresh next time.
	steerStep := d.cfg.Waist.rampStep(d.cfg.CommandHz)
	closed := d.byVelocity && !d.helm.stopped && !d.cfg.SpeedLoop.OpenLoop
	mode := driving
	if closed {
		mode = d.standstillLocked(now)
	} else {
		d.stand = standstill{}
	}
	var wheels map[int]int16
	if closed {
		wheels = d.wheelsRawLocked(mode)
	} else {
		d.shared.reset()
	}
	targets := make(map[int]int16, len(d.cfg.Motors))
	for _, m := range d.cfg.Motors {
		if m.Kind == "wheel" {
			if closed {
				d.last[m.NodeID] = wheels[m.NodeID]
				targets[m.NodeID] = wheels[m.NodeID]
				continue
			}
			d.loopFor(m.NodeID).reset()
		}
		desired := int16(0)
		if !d.helm.stopped {
			if m.Kind == "steering" {
				desired = d.steeringRawLocked(m, raw, at, fresh, now)
			} else {
				desired = d.rawFor(m)
			}
		}
		var limited int16
		if m.Kind == "steering" {
			limited = steerLimit(d.last[m.NodeID], desired, steerStep)
		} else {
			limited = rateLimit(d.last[m.NodeID], desired, int16(d.cfg.AccelStep), int16(d.cfg.BrakeStep))
		}
		d.last[m.NodeID] = limited
		targets[m.NodeID] = limited
	}
	d.watchWheelsLocked(now, targets)
	if d.helm.stopped {
		for node := range targets {
			targets[node] = 0
		}
	}
	fd := d.fd
	d.mu.Unlock()

	if fd == 0 {
		return
	}
	for node, raw := range targets {
		if err := d.setVelocity(node, raw); err != nil {
			log.Printf("loader: node %d: %v", node, err)
		}
	}
}

// steeringRawLocked is this cycle's raw command for the steering motor: the
// angle loop's effort while a curvature command stands, the motor's own
// setpoint otherwise, and in either case only what the guard allows. A refused
// effort also resets the ramp, so the motor stops now rather than coasting
// through the limit on the way down, and the angle loop's integral, so that it
// does not wind up against the refusal. at is when the reading raw was taken.
// Caller holds the lock.
//
// Until 7 October 2026 the angle loop was proportional, with a minimum effort
// for the breakaway, a push to cross the chain's slack, a deadband with
// hysteresis, and its effort cut at once whenever it turned; each answered a
// symptom, and together they swung the waist about its target. The loop in
// waist.go replaces all four.
//
// Every time the loop starts or stops moving the joint, the log says so: a
// loop that has settled says nothing, and one that hunts says it every second.
func (d *drivetrain) steeringRawLocked(m MotorSpec, raw int, at time.Time, fresh bool, now time.Time) int16 {
	effort := d.setpoint[m.NodeID]
	if d.byAngle {
		effort = 0
		if deg, ok := d.articulationLocked(raw, fresh); ok {
			limit := d.cfg.Waist.targetLimitDegrees() * math.Pi / 180
			target := d.cfg.Geometry.articulationFor(d.curvature, limit) * 180 / math.Pi
			was := d.waist.moving
			effort = d.waist.anglePI(target, deg, at)
			if d.waist.moving != was {
				if d.waist.moving != 0 {
					log.Printf("loader: waist moving %s: target %.1f°, at %.1f°, effort %.0f%%",
						sideName(d.waist.moving), target, deg, effort)
				} else {
					log.Printf("loader: waist holding: target %.1f°, at %.1f°, %.1f°/s", target, deg, d.waist.joint.rate())
				}
			}
		} else {
			d.waist.resetPI()
		}
	} else {
		d.waist.resetPI()
	}
	allowed, why := d.waist.guard(effort, raw, fresh)
	if allowed != effort {
		d.last[m.NodeID] = 0
		d.waist.resetPI()
		if why != "" && d.waist.every(why, 2*time.Second, now) {
			log.Printf("loader: steering held — %s", why)
		}
	}
	return clampToScale(allowed / 100 * fullScale * d.waist.rawEffortSign())
}

// Thresholds of the wheels' watchdog that are not worth configuring: a wheel
// is asked to move when its target is wheelStallTargetRPM or more, and is not
// turning below wheelStallTurningRPM — a tenth of a turn a minute is noise.
const (
	wheelStallTargetRPM  = 5
	wheelStallTurningRPM = 2
)

// watchWheelsLocked is the wheels' watchdog. A wheel asked to move, given at
// least WheelStallPercent of full scale and reported by a fresh encoder as not
// turning, for WheelStallMs, stops the vehicle. Caller holds the lock.
//
// On a slope on 1 October 2026 full stick drove all four wheels at full scale
// into a stall, the drives cut out hot, and the stick kept asking: nothing in
// the loader noticed a wheel commanded hard that did not turn, while the
// steering had a watchdog for exactly that. Whatever drives the wheel — the
// speed loop, open loop, or a setpoint on the bench — the watchdog judges what
// it was sent. Holding still on a hill is a target of zero and never counts,
// and a stale encoder says nothing either way.
func (d *drivetrain) watchWheelsLocked(now time.Time, sent map[int]int16) {
	if d.helm.stopped {
		d.stallSince = nil
		return
	}
	for _, m := range d.cfg.Motors {
		if m.Kind != "wheel" {
			continue
		}
		r, fresh := d.fb.wheel(m.NodeID - 1)
		effort := math.Abs(float64(sent[m.NodeID])) / fullScale * 100
		stalling := fresh && math.Abs(d.setpoint[m.NodeID]) >= wheelStallTargetRPM &&
			effort >= d.cfg.WheelStallPercent && math.Abs(r.rpm) < wheelStallTurningRPM
		if !stalling {
			delete(d.stallSince, m.NodeID)
			continue
		}
		if d.stallSince == nil {
			d.stallSince = make(map[int]time.Time)
		}
		since, ok := d.stallSince[m.NodeID]
		if !ok {
			d.stallSince[m.NodeID] = now
			continue
		}
		if held := now.Sub(since); held >= time.Duration(d.cfg.WheelStallMs)*time.Millisecond {
			d.wheelFault = fmt.Sprintf("%s was driven at %.0f%% for %d ms and did not turn — a blocked or stalled "+
				"wheel, or a drive that has cut out hot", m.Name, effort, held.Milliseconds())
			log.Printf("loader: STOP — %s; take control again to clear it", d.wheelFault)
			d.helm.stop(caller{name: "the loader's wheel watchdog", known: true}, "stopped")
			d.haltLocked()
			d.stallSince = nil
			return
		}
	}
}

// wheelsRawLocked is this cycle's raw command for every wheel, driven by
// velocity: its kinematic setpoint, ramped, through the speed loop — the
// integral the wheels share, or with perWheelIntegral one each. Wheel encoders
// answer in motor-node order, so node 1 is encoder 0. Caller holds the lock.
//
// Resting on a hill, the loops only follow the encoders, keeping what they have
// learned, and each wheel gets its holding effort eased towards zero.
func (d *drivetrain) wheelsRawLocked(mode standMode) map[int]int16 {
	cfg := d.cfg.SpeedLoop
	fullRPM := fullScale / rpmToCommand
	frozen := cfg.FreezeIntegralWhileSteering && d.waist.joint.rate() != 0
	d.shared.frozen = frozen
	out := make(map[int]int16, 4)
	var nodes []int
	var loops []*speedLoop
	for _, m := range d.cfg.Motors {
		if m.Kind != "wheel" {
			continue
		}
		s := d.loopFor(m.NodeID)
		s.frozen = frozen
		s.ramp(d.setpoint[m.NodeID], float64(d.last[m.NodeID])/rpmToCommand, d.cfg.AccelStep, d.cfg.BrakeStep)
		r, fresh := d.fb.wheel(m.NodeID - 1)
		if mode == resting {
			s.track(r, fresh)
			out[m.NodeID] = clampToScale(d.stand.restingCommand(m.NodeID) * rpmToCommand)
			continue
		}
		var note string
		if cfg.PerWheelIntegral {
			var rpm float64
			rpm, note = s.step(cfg, r, fresh, fullRPM)
			out[m.NodeID] = clampToScale(rpm * rpmToCommand)
		} else {
			note = s.measure(r, fresh)
			nodes, loops = append(nodes, m.NodeID), append(loops, s)
		}
		if note != "" {
			log.Printf("loader: %s: %s", m.Name, note)
		}
	}
	if len(loops) > 0 {
		for i, rpm := range d.shared.step(cfg, loops, fullRPM) {
			out[nodes[i]] = clampToScale(rpm * rpmToCommand)
		}
	}
	return out
}

// standstillLocked moves the standstill on by a cycle and says which mode the
// wheels are in. Caller holds the lock.
func (d *drivetrain) standstillLocked(now time.Time) standMode {
	var wheels []standWheel
	for _, m := range d.cfg.Motors {
		if m.Kind != "wheel" {
			continue
		}
		r, fresh := d.fb.wheel(m.NodeID - 1)
		wheels = append(wheels, standWheel{
			node:     m.NodeID,
			still:    d.setpoint[m.NodeID] == 0 && d.loopFor(m.NodeID).reference == 0,
			fresh:    fresh,
			rpm:      r.rpm,
			distance: r.revolutions * d.cfg.circumferenceOfNode(m.NodeID),
			command:  float64(d.last[m.NodeID]) / rpmToCommand,
		})
	}
	if note := d.stand.update(d.cfg.SpeedLoop, now, d.cfg.CommandHz, wheels); note != "" {
		log.Printf("loader: %s", note)
	}
	return d.stand.mode
}

// loopFor is the wheel's speed loop, made on first use.
func (d *drivetrain) loopFor(node int) *speedLoop {
	if d.loops == nil {
		d.loops = make(map[int]*speedLoop)
	}
	s := d.loops[node]
	if s == nil {
		s = &speedLoop{}
		d.loops[node] = s
	}
	return s
}

// steeringMotor is the configured steering motor, if there is one.
func (d *drivetrain) steeringMotor() (MotorSpec, bool) {
	for _, m := range d.cfg.Motors {
		if m.Kind == "steering" {
			return m, true
		}
	}
	return MotorSpec{}, false
}

// waistNowLocked is the newest waist reading and whether it is fresh enough to
// steer by. Caller holds the lock.
func (d *drivetrain) waistNowLocked(now time.Time) (int, bool) {
	raw, _, fresh := d.waistReadingLocked(now)
	return raw, fresh
}

// waistReadingLocked is waistNowLocked with the time the reading was taken.
// Caller holds the lock.
func (d *drivetrain) waistReadingLocked(now time.Time) (raw int, at time.Time, fresh bool) {
	raw, at, have := d.fb.waistLatest()
	fresh = have && now.Sub(at) <= time.Duration(d.cfg.Waist.StaleMs)*time.Millisecond
	return raw, at, fresh
}

// followJointLocked moves the joint estimate on by the newest reading, and
// forgets it when there is no fresh, calibrated one. Caller holds the lock.
func (d *drivetrain) followJointLocked(raw int, at time.Time, fresh bool) {
	deg, ok := d.articulationLocked(raw, fresh)
	if !ok {
		d.waist.joint = jointEstimate{}
		return
	}
	w := d.cfg.Waist
	d.waist.joint.update(deg, at, w.JointFilterSeconds, time.Duration(w.StaleMs)*time.Millisecond)
}

// jointForWheelsLocked is the articulation and its rate, in radians and
// radians a second, that the wheels are set from: the joint estimate's, or,
// with ignoreJointRate, the raw reading and no rate. Straight and still when
// there is no reading to go by. Caller holds the lock.
func (d *drivetrain) jointForWheelsLocked(raw int, fresh bool) (gamma, rate float64) {
	if d.cfg.Waist.IgnoreJointRate {
		deg, _ := d.articulationLocked(raw, fresh)
		return deg * math.Pi / 180, 0
	}
	if j := d.waist.joint; j.ok {
		return j.deg * math.Pi / 180, j.rate() * math.Pi / 180
	}
	return 0, 0
}

// articulationLocked is the measured articulation in degrees, positive to the
// left, when there is a fresh reading and a calibration to read it with.
func (d *drivetrain) articulationLocked(raw int, fresh bool) (float64, bool) {
	if !fresh || !d.cfg.Waist.calibrated() {
		return 0, false
	}
	return d.cfg.Waist.degrees(raw), true
}

// curvatureLimit is one over the smallest turning radius the angle loop steers
// to: the waist's software limit, less its margin, since the loop never aims
// at the limit itself.
func (d *drivetrain) curvatureLimit() float64 {
	return d.cfg.Geometry.frontCurvature(d.cfg.Waist.targetLimitDegrees() * math.Pi / 180)
}

// logCalibration says at startup what the loader believes about its waist, so
// a mistyped calibration is visible before the vehicle moves.
func (d *drivetrain) logCalibration() {
	w := d.cfg.Waist
	if !w.calibrated() {
		log.Printf("loader: the waist is NOT calibrated — steering is limited to %d counts either side of %d, "+
			"and cannot be commanded by curvature", w.UncalibratedWindowCounts, w.StraightCount)
	} else {
		cpd := w.countsPerDegree()
		log.Printf("loader: waist calibrated at %.2f counts per degree (%s is left); limit ±%.0f° is counts %.0f to %.0f; "+
			"tightest turn %.2f m radius", math.Abs(cpd), map[bool]string{true: "down", false: "up"}[cpd < 0],
			w.LimitDegrees, float64(w.StraightCount)-w.LimitDegrees*math.Abs(cpd), float64(w.StraightCount)+w.LimitDegrees*math.Abs(cpd),
			1/d.curvatureLimit())
		if math.Abs(cpd) > 11.25 {
			log.Printf("loader: WARNING — %.2f counts per degree would put %.0f° beyond the sensor's range from %d; "+
				"check the calibration", math.Abs(cpd), w.LimitDegrees, w.StraightCount)
		}
	}
	if !w.IgnoreJointRate && d.cfg.WaistPollHz < 25 {
		log.Printf("loader: waistPollHz is %d — the wheels follow the joint's rate, and at fewer than 25 readings "+
			"a second it lags; 50 is the default", d.cfg.WaistPollHz)
	}
	switch w.EffortTurnsLeft {
	case 0:
		log.Println("loader: effortTurnsLeft is not set — steer a little to the left with the pad and watch which way " +
			"the vehicle turns; until it is set the waist cannot be brought back from beyond its limit")
	case 1, -1:
	default:
		log.Printf("loader: effortTurnsLeft is %d; it must be 1, -1 or 0", w.EffortTurnsLeft)
	}
}

// rawFor converts a motor's setpoint into the controller's integer scale.
// Caller holds the lock.
func (d *drivetrain) rawFor(m MotorSpec) int16 {
	sp := d.setpoint[m.NodeID]
	var raw float64
	if m.Kind == "steering" {
		raw = sp / 100.0 * fullScale // percent of full effort
	} else {
		raw = sp * rpmToCommand
	}
	return clampToScale(raw)
}

func clampToScale(v float64) int16 {
	if math.IsNaN(v) {
		return 0
	}
	if v > fullScale {
		v = fullScale
	}
	if v < -fullScale {
		v = -fullScale
	}
	return int16(v)
}

// rateLimit walks the command towards what was asked for, no faster than the
// configured step. Braking gets its own, larger step: coming to a stop should
// not be slower than setting off.
// steerLimit is the steering motor's rate limit: effort builds up by step a
// cycle and is let go at once, and a reversal passes through zero for a cycle.
// Let go down a ramp, the joint coasted on through it, past where the angle
// loop let go: at 100% a second, 22% took a fifth of a second to reach zero.
func steerLimit(last, desired, step int16) int16 {
	if (desired > 0 && last < 0) || (desired < 0 && last > 0) {
		return 0
	}
	return rateLimit(last, desired, step, fullScale)
}

func rateLimit(last, desired, accel, brake int16) int16 {
	signsDiffer := (desired > 0 && last < 0) || (desired < 0 && last > 0)
	towardsZero := (last > 0 && desired < last && desired >= 0) ||
		(last < 0 && desired > last && desired <= 0)
	step := int32(accel)
	if signsDiffer || towardsZero {
		step = int32(brake)
	}
	l, dv := int32(last), int32(desired)
	if dv > l+step {
		return int16(l + step)
	}
	if dv < l-step {
		return int16(l - step)
	}
	return desired
}

// haltLocked clears every setpoint and the rate limiter's memory, so the next
// cycle commands zero outright rather than ramping down to it. This is what
// can_dds does on an emergency, by resetting its motor controllers: a stop that
// took the braking ramp would take 1.2 s from full speed. Caller holds the lock.
func (d *drivetrain) haltLocked() {
	d.clearSetpointsLocked()
	for k := range d.last {
		d.last[k] = 0
	}
}

// clearSetpointsLocked zeroes what is asked for but keeps the ramp, for a
// handover: the vehicle comes to rest at the normal braking rate. Caller holds
// the lock.
//
// The vehicle-level command goes with them. In particular the steering does not
// return to straight on its own: with nobody in control, nothing moves.
func (d *drivetrain) clearSetpointsLocked() {
	for k := range d.setpoint {
		d.setpoint[k] = 0
	}
	d.velocity, d.curvature = 0, 0
	d.byVelocity, d.byAngle = false, false
	d.stand = standstill{}
	d.waist.resetPI()
}

// callerOf names who sent a request, and brings the helm up to date with
// whether this loader can tell callers apart at all.
func (d *drivetrain) callerOf(r *http.Request) caller {
	certified := false
	select {
	case <-usecases.EnsureCertReady(d.sys):
		certified = true
	default:
	}
	d.mu.Lock()
	if d.helm.anonymousPilot == certified { // it changed
		d.helm.anonymousPilot = !certified
		if !certified {
			log.Println("loader: this loader holds no certificate, so callers cannot be told apart; " +
				"anyone may take control and control protects nothing")
		}
	}
	d.mu.Unlock()
	if cn, ok := usecases.PeerCN(r); ok {
		return caller{name: cn, known: true}
	}
	return caller{}
}

// stopAll zeroes every motor immediately, bypassing the rate limiter. Used on
// shutdown, where the point is that the wheels stop.
func (d *drivetrain) stopAll() {
	d.mu.Lock()
	for k := range d.setpoint {
		d.setpoint[k] = 0
	}
	for k := range d.last {
		d.last[k] = 0
	}
	motors := d.cfg.Motors
	fd := d.fd
	d.mu.Unlock()
	if fd == 0 {
		return
	}
	for _, m := range motors {
		if err := d.setVelocity(m.NodeID, 0); err != nil {
			log.Printf("loader: stopping node %d: %v", m.NodeID, err)
		}
	}
}

//-------------------------------------Service handlers

func (t *Traits) setpointService(w http.ResponseWriter, r *http.Request) {
	switch r.Method {
	case "GET":
		f := t.get()
		usecases.HTTPProcessGetRequest(w, r, &f)
	case "PUT":
		sig, err := usecases.HTTPProcessSetRequest(w, r)
		if err != nil {
			log.Printf("loader: %s: bad set request: %v", t.Name, err)
			http.Error(w, "malformed request", http.StatusBadRequest)
			return
		}
		confirmation, err := t.set(t.dt.callerOf(r), sig)
		if err != nil {
			refuse(w, err)
			return
		}
		respond(w, &confirmation)
	default:
		http.Error(w, "Method is not supported.", http.StatusNotFound)
	}
}

func (t *Traits) get() (f forms.SignalA_v1a) {
	f.NewForm()
	t.dt.mu.Lock()
	f.Value = t.dt.setpoint[t.NodeID]
	t.dt.mu.Unlock()
	f.Unit = t.unit
	f.Timestamp = time.Now()
	return f
}

// set records what was asked for and clamps it, if the caller has control. The
// value reaches a motor on the next cycle of the loop, not here, so a request
// never blocks on the bus.
func (t *Traits) set(c caller, sig forms.SignalA_v1a) (forms.SignalA_v1a, error) {
	v := sig.Value
	limit := t.dt.cfg.MaxWheelRPM
	if t.Kind == "steering" {
		limit = 100
	}
	if v > limit {
		v = limit
	}
	if v < -limit {
		v = -limit
	}

	t.dt.mu.Lock()
	if err := t.dt.helm.command(c, time.Now()); err != nil {
		t.dt.mu.Unlock()
		return forms.SignalA_v1a{}, err
	}
	t.dt.setpoint[t.NodeID] = v
	// A motor commanded directly is no longer following the vehicle-level
	// command.
	if t.Kind == "steering" {
		t.dt.byAngle = false
	} else {
		t.dt.byVelocity = false
	}
	t.dt.mu.Unlock()

	if v != sig.Value {
		log.Printf("loader: %s asked for %.2f, clamped to %.2f", t.Name, sig.Value, v)
	}
	return t.get(), nil
}

//-------------------------------------The measured services

// speedService reports what the wheel is doing, as against the setpoint
// service, which reports what it was asked to do.
//
// It answers 503 when the encoder has gone quiet, and that is the point of it.
// The obvious alternative — fall back to the setpoint — would produce a system
// that reports full speed for a wheel that is not turning, which is exactly the
// number an odometry consumer would integrate into a map of a journey that
// never happened.
func (t *Traits) speedService(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method is not supported.", http.StatusNotFound)
		return
	}
	reading, fresh := t.dt.fb.wheel(t.encoderIndex)
	if !fresh {
		http.Error(w, "no recent encoder frame for this wheel", http.StatusServiceUnavailable)
		return
	}
	usecases.HTTPProcessGetRequest(w, r, speedForm(reading))
}

// travelService reports the wheel's cumulative revolutions since this system
// started. The encoder's own counter wraps every 204.8 revolutions; this one
// does not, because the loader unwraps it as the frames arrive.
func (t *Traits) travelService(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method is not supported.", http.StatusNotFound)
		return
	}
	reading, fresh := t.dt.fb.wheel(t.encoderIndex)
	if !fresh {
		http.Error(w, "no recent encoder frame for this wheel", http.StatusServiceUnavailable)
		return
	}
	usecases.HTTPProcessGetRequest(w, r, travelForm(reading))
}

// waistService publishes the articulation sensor's raw ten-bit reading. It is
// what a calibration is read from; articulation, on the Vehicle, gives the
// angle in degrees once the calibration is in the configuration.
func (t *Traits) waistService(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method is not supported.", http.StatusNotFound)
		return
	}
	raw, at, fresh := t.dt.fb.waist()
	if !fresh {
		http.Error(w, "the articulation sensor is not answering", http.StatusServiceUnavailable)
		return
	}
	usecases.HTTPProcessGetRequest(w, r, waistForm(raw, at))
}

func (t *Traits) vehicleTravel(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method is not supported.", http.StatusNotFound)
		return
	}
	var readingsum wheelReading
	for i := 0; i <= 3; i++ {
		reading, fresh := t.dt.fb.wheel(i)
		readingsum.rpm = readingsum.rpm + reading.rpm
		readingsum.revolutions = readingsum.revolutions + reading.revolutions
		readingsum.at = reading.at

		if !fresh {
			http.Error(w, "no recent encoder frame for one of the wheels", http.StatusServiceUnavailable)
			return
		}
	}
	readingsum.rpm = readingsum.rpm / 4
	readingsum.revolutions = readingsum.revolutions / 4
	usecases.HTTPProcessGetRequest(w, r, travelForm(readingsum))
}
