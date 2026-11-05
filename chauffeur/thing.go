/*******************************************************************************
 * Copyright (c) 2026 Synecdoque
 *
 * The software is licensed under the MIT License. See the LICENSE file in this
 * repository for details.
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
	unitMetre  = "<http://qudt.org/vocab/unit/M>"
	unitDegree = "<http://qudt.org/vocab/unit/DEG>"
	unitMPerS  = "<http://qudt.org/vocab/unit/M-PER-SEC>"
	unitPerM   = "<http://qudt.org/vocab/unit/PER-M>"
)

// ChauffeurConfig is what the operator may set.
type ChauffeurConfig struct {
	// Cruise is how fast the route is driven. The wheels need about 0.45 m/s to
	// turn reliably on the old bench figures; measure the lowest speed before
	// lowering this.
	Cruise float64 `json:"cruiseMetresPerSecond"`

	// Lookahead is how far along the route the vehicle aims. Longer is steadier and
	// cuts corners more; shorter follows closer and, with a slow waist or an old
	// pose, weaves. 1.2 m is from simulation, with a waist speed that is a guess.
	Lookahead float64 `json:"lookaheadMetres"`

	CommandHz int `json:"commandHz"`

	// HoldClearance is the clearance ahead under which the vehicle holds, and
	// StopClearance the one under which it stops outright, which only the pad can
	// undo. The sector the clearance looks at is narrow and straight ahead, so a
	// wall in a curve can trigger them.
	HoldClearance float64 `json:"holdClearanceMetres"`
	StopClearance float64 `json:"stopClearanceMetres"`

	// Arrive is how near the route's end counts as there.
	Arrive float64 `json:"arriveMetres"`

	// OffRoute is how far from the route the vehicle may be before it holds. Half
	// the vehicle's width and this must be less than the navigator's clearance.
	OffRoute float64 `json:"offRouteMetres"`

	MaxPoseAgeMs      int `json:"maxPoseAgeMs"`
	MaxClearanceAgeMs int `json:"maxClearanceAgeMs"`
	MaxRouteAgeS      int `json:"maxRouteAgeSeconds"`

	// RouteEndMoved and RouteLonger are how much a replanned route may differ
	// before it counts as one nobody has looked at, and the vehicle holds.
	RouteEndMoved float64 `json:"routeEndMovedMetres"`
	RouteLonger   float64 `json:"routeLongerMetres"`

	// MaxCurvature is used when the loader does not say.
	MaxCurvature float64 `json:"maxCurvature"`

	// Vehicle picks the loader out of the cloud; ClearanceSensor picks which
	// clearance to read, when there is more than one: the guetteur's or the vigie's.
	Vehicle         map[string][]string `json:"vehicle"`
	ClearanceSensor map[string][]string `json:"clearanceSensor"`
}

func applyDefaults(c *ChauffeurConfig) {
	def := func(v *float64, d float64) {
		if *v <= 0 {
			*v = d
		}
	}
	defI := func(v *int, d int) {
		if *v <= 0 {
			*v = d
		}
	}
	def(&c.Cruise, 0.5)
	def(&c.Lookahead, 1.2)
	defI(&c.CommandHz, 20)
	def(&c.HoldClearance, 0.8)
	def(&c.StopClearance, 0.4)
	def(&c.Arrive, 0.25)
	def(&c.OffRoute, 0.2)
	defI(&c.MaxPoseAgeMs, 1000)
	defI(&c.MaxClearanceAgeMs, 1000)
	defI(&c.MaxRouteAgeS, 15)
	def(&c.RouteEndMoved, 1)
	def(&c.RouteLonger, 3)
	def(&c.MaxCurvature, 0.4)
	if c.Vehicle == nil {
		c.Vehicle = map[string][]string{"Model": {"artitrax"}}
	}
}

func (c ChauffeurConfig) tuning() tuning {
	return tuning{
		cruise: c.Cruise, lookahead: c.Lookahead,
		holdClearance: c.HoldClearance, stopClear: c.StopClearance,
		arrive: c.Arrive, offRoute: c.OffRoute,
		maxPoseAge:      time.Duration(c.MaxPoseAgeMs) * time.Millisecond,
		maxClearanceAge: time.Duration(c.MaxClearanceAgeMs) * time.Millisecond,
		maxRouteAge:     time.Duration(c.MaxRouteAgeS) * time.Second,
		brakeFor:        time.Second,
		routeEndMoved:   c.RouteEndMoved, routeLonger: c.RouteLonger,
	}
}

// Traits is the chauffeur's state.
type Traits struct {
	cfg   ChauffeurConfig
	owner *components.System

	// The four readings, each from a goroutine of its own, so that a slow answer
	// to one cannot hold up the commands: the loader stops a vehicle whose pilot
	// is silent for half a second. Fields, so that tests can stand in for them.
	readPose      func() (pose, error)
	readRoute     func() (*forms.PathA_v1a, error)
	readClearance func() (float64, time.Time, error)
	readLimit     func() (float64, error)

	velocity, curvature, control, stop commandSink
	replies                            chan reply

	mu   sync.Mutex
	m    *mission
	seen latest
}

// pose is the vehicle's place, from the cartographer.
type pose struct {
	x, y, th float64
	frame    string
	at       time.Time
}

// latest is the newest of each reading.
type latest struct {
	pose      pose
	havePose  bool
	route     *route
	clearance float64
	clearAt   time.Time
	haveClear bool
	limit     float64
}

// commandSink is where a command goes: a sender, or a test.
type commandSink interface {
	offer(value float64, epoch int)
	put(value float64) error
}

//-------------------------------------Instantiate a unit asset template

func initTemplate() *components.UnitAsset {
	engage := components.Service{
		Definition: "engage",
		SubPath:    "engage",
		Details: map[string][]string{
			"Forms":   {"SignalA_v1a"},
			"Methods": components.HTTPMethods("GET", "PUT"),
		},
		RegPeriod:   10,
		Description: "PUT 1 to drive the navigator's route, or to go on from a hold; PUT 0 to come to rest and release control; GET 1 while it has or wants control",
	}
	state := components.Service{
		Definition: "drivingState",
		SubPath:    "drivingState",
		Details: map[string][]string{
			"Forms":   {"SignalA_v1a"},
			"Methods": components.HTTPMethods("GET"),
		},
		RegPeriod:   10,
		Description: "what the chauffeur is doing, and why, as a code: see its README",
	}
	return &components.UnitAsset{
		Name:     "Driver",
		Mission:  components.MissionControl,
		Mobility: components.MobilityMovable,
		ServicesMap: components.Services{
			engage.SubPath: &engage,
			state.SubPath:  &state,
		},
		Traits: &ChauffeurConfig{
			Cruise: 0.5, Lookahead: 1.2, CommandHz: 20,
			HoldClearance: 0.8, StopClearance: 0.4, Arrive: 0.25, OffRoute: 0.2,
			MaxPoseAgeMs: 1000, MaxClearanceAgeMs: 1000, MaxRouteAgeS: 15,
			RouteEndMoved: 1, RouteLonger: 3, MaxCurvature: 0.4,
			Vehicle:         map[string][]string{"Model": {"artitrax"}},
			ClearanceSensor: map[string][]string{},
		},
	}
}

//-------------------------------------Instantiate the unit asset

func newResource(uac usecases.ConfigurableAsset, sys *components.System) (*components.UnitAsset, func()) {
	cfg := ChauffeurConfig{}
	if len(uac.Traits) > 0 {
		if err := json.Unmarshal(uac.Traits[0], &cfg); err != nil {
			log.Fatalf("chauffeur: cannot parse traits: %v", err)
		}
	}
	applyDefaults(&cfg)

	protos := components.SProtocols(sys.Husk.ProtoPort)
	get := func(def string, details map[string][]string) *components.Cervice {
		return &components.Cervice{
			Definition: def, Protos: protos, Mode: "get",
			Nodes: make(map[string][]components.NodeInfo), Details: details,
		}
	}
	set := func(def string) *components.Cervice {
		return &components.Cervice{
			Definition: def, Protos: protos, Mode: "set",
			Nodes: make(map[string][]components.NodeInfo), Details: cfg.Vehicle,
		}
	}
	poseCer, pathCer := get("pose", nil), get("path", nil)
	clearCer, limitCer := get("clearance", cfg.ClearanceSensor), get("curvatureLimit", cfg.Vehicle)
	velCer, curCer, ctlCer, stopCer := set("velocity"), set("curvature"), set("control"), set("stop")

	t := &Traits{
		cfg:     cfg,
		owner:   sys,
		replies: make(chan reply, 16),
		m:       &mission{cfg: cfg.tuning()},
	}
	t.seen.limit = cfg.MaxCurvature
	t.readPose = func() (pose, error) { return fetchPose(poseCer, sys) }
	t.readRoute = func() (*forms.PathA_v1a, error) { return fetchPath(pathCer, sys) }
	t.readClearance = func() (float64, time.Time, error) { return fetchSignal(clearCer, sys) }
	t.readLimit = func() (float64, error) {
		v, _, err := fetchSignal(limitCer, sys)
		return v, err
	}
	vel := newSender("velocity", velCer, sys, unitMPerS, t.replies, velocityReply)
	cur := newSender("curvature", curCer, sys, unitPerM, t.replies, curvatureReply)
	ctl := newSender("control", ctlCer, sys, "", t.replies, controlReply)
	stp := newSender("stop", stopCer, sys, "", t.replies, stopReply)
	t.velocity, t.curvature, t.control, t.stop = vel, cur, ctl, stp

	ua := &components.UnitAsset{
		Name:        uac.Name,
		Mission:     uac.Mission,
		Mobility:    uac.Mobility,
		Owner:       sys,
		Details:     uac.Details,
		ServicesMap: usecases.MakeServiceMap(uac.Services),
		CervicesMap: components.Cervices{
			"pose": poseCer, "path": pathCer, "clearance": clearCer, "curvatureLimit": limitCer,
			"velocity": velCer, "curvature": curCer, "control": ctlCer, "stop": stopCer,
		},
		Traits: t,
	}
	ua.ServingFunc = func(w http.ResponseWriter, r *http.Request, servicePath string) {
		serving(t, w, r, servicePath)
	}

	for _, s := range []*sender{vel, cur, ctl, stp} {
		go s.run(sys.Ctx)
	}
	t.startReading(sys.Ctx)
	go t.run(sys.Ctx)
	return ua, func() { log.Println("chauffeur: shut down") }
}

//-------------------------------------Reading

func fetch(cer *components.Cervice, sys *components.System) (forms.Form, error) {
	if len(cer.Nodes) == 0 {
		if err := usecases.Search4Services(cer, sys); err != nil {
			return nil, err
		}
	}
	f, err := usecases.GetState(cer, sys)
	if err != nil {
		var refused *usecases.ProviderRefusal
		if !errors.As(err, &refused) {
			cer.Nodes = make(map[string][]components.NodeInfo)
		}
		return nil, err
	}
	return f, nil
}

func fetchPose(cer *components.Cervice, sys *components.System) (pose, error) {
	f, err := fetch(cer, sys)
	if err != nil {
		return pose{}, err
	}
	p, ok := f.(*forms.PoseA_v1a)
	if !ok {
		return pose{}, fmt.Errorf("the pose came as %s", f.FormVersion())
	}
	if p.DistanceUnit != "" && p.DistanceUnit != unitMetre {
		return pose{}, fmt.Errorf("a pose in %s; it must be in meters", p.DistanceUnit)
	}
	th := p.Heading
	if p.AngleUnit == "" || p.AngleUnit == unitDegree {
		th *= math.Pi / 180
	}
	return pose{x: p.X, y: p.Y, th: th, frame: p.Frame, at: p.Timestamp}, nil
}

func fetchPath(cer *components.Cervice, sys *components.System) (*forms.PathA_v1a, error) {
	f, err := fetch(cer, sys)
	if err != nil {
		return nil, err
	}
	p, ok := f.(*forms.PathA_v1a)
	if !ok {
		return nil, fmt.Errorf("the route came as %s", f.FormVersion())
	}
	return p, nil
}

func fetchSignal(cer *components.Cervice, sys *components.System) (float64, time.Time, error) {
	f, err := fetch(cer, sys)
	if err != nil {
		return 0, time.Time{}, err
	}
	s, ok := f.(*forms.SignalA_v1a)
	if !ok {
		return 0, time.Time{}, fmt.Errorf("%s came as %s", cer.Definition, f.FormVersion())
	}
	return s.Value, s.Timestamp, nil
}

// startReading starts a goroutine per reading.
func (t *Traits) startReading(ctx context.Context) {
	every := func(period time.Duration, read func()) {
		go func() {
			tick := time.NewTicker(period)
			defer tick.Stop()
			for {
				select {
				case <-ctx.Done():
					return
				case <-tick.C:
					read()
				}
			}
		}()
	}
	every(100*time.Millisecond, t.updatePose)
	every(100*time.Millisecond, t.updateClearance)
	every(time.Second, t.updateRoute)
	every(5*time.Second, t.updateLimit)
}

func (t *Traits) updatePose() {
	p, err := t.readPose()
	t.mu.Lock()
	defer t.mu.Unlock()
	t.seen.havePose = err == nil
	if err == nil {
		t.seen.pose = p
	}
}

func (t *Traits) updateClearance() {
	v, at, err := t.readClearance()
	t.mu.Lock()
	defer t.mu.Unlock()
	// A 503 means blind, and blind is a reason to hold: there is no float that
	// safely stands for it.
	t.seen.haveClear = err == nil
	if err == nil {
		t.seen.clearance, t.seen.clearAt = v, at
	}
}

// updateRoute keeps one route object per plan, since the mission tells a new route
// from the old by its identity.
func (t *Traits) updateRoute() {
	p, err := t.readRoute()
	var refused *usecases.ProviderRefusal
	t.mu.Lock()
	defer t.mu.Unlock()
	switch {
	case errors.As(err, &refused):
		t.seen.route = nil // the navigator answered: there is no route
	case err != nil:
		// The navigator cannot be reached. The route keeps its stamp, so it goes
		// stale on its own and the vehicle holds.
	case t.seen.route == nil || !t.seen.route.stamp.Equal(p.Timestamp):
		r, err := newRoute(p)
		if err != nil {
			log.Printf("chauffeur: cannot use the route: %v", err)
			t.seen.route = nil
			return
		}
		t.seen.route = r
	}
}

func (t *Traits) updateLimit() {
	if v, err := t.readLimit(); err == nil && v > 0 {
		t.mu.Lock()
		t.seen.limit = v
		t.mu.Unlock()
	}
}

//-------------------------------------The control loop

func (t *Traits) run(ctx context.Context) {
	tick := time.NewTicker(time.Second / time.Duration(t.cfg.CommandHz))
	defer tick.Stop()
	for {
		select {
		case <-ctx.Done():
			t.shutdown()
			return
		case r := <-t.replies:
			t.applyReply(r)
		case now := <-tick.C:
			t.cycle(now)
		}
	}
}

// cycle is one pass of the control loop. It only reads what the goroutines have
// left, so it never waits on the network.
func (t *Traits) cycle(now time.Time) {
	t.mu.Lock()
	s := t.seen
	before := t.m.code
	a := t.m.step(observation{
		now: now, havePose: s.havePose, x: s.pose.x, y: s.pose.y, th: s.pose.th,
		poseFrame: s.pose.frame, poseAt: s.pose.at, route: s.route,
		haveClear: s.haveClear, clearance: s.clearance, clearAt: s.clearAt, curveLimit: s.limit,
	})
	after := t.m.code
	t.mu.Unlock()
	if after != before {
		log.Printf("chauffeur: %s", after)
	}

	if a.stop {
		t.stop.offer(1, a.epoch)
	}
	if a.take {
		t.control.offer(1, a.epoch)
	}
	if a.release {
		t.control.offer(0, a.epoch)
	}
	if a.drive {
		t.velocity.offer(a.velocity, a.epoch)
		t.curvature.offer(a.curvature, a.epoch)
	}
}

// shutdown stops the vehicle if this system was driving it: closing the program
// that has control is a pilot leaving the controls.
func (t *Traits) shutdown() {
	t.mu.Lock()
	active := t.m.code.active() && t.m.code != codeTaking
	t.mu.Unlock()
	if !active {
		return
	}
	done := make(chan error, 1)
	go func() { done <- t.stop.put(1) }()
	select {
	case err := <-done:
		if err != nil {
			log.Printf("chauffeur: had control on the way out, and the vehicle did not confirm the stop: %v", err)
			return
		}
		log.Println("chauffeur: had control on the way out — the vehicle is stopped")
	case <-time.After(time.Second):
		log.Println("chauffeur: had control on the way out, and the vehicle did not confirm the stop within a second")
	}
}

//-------------------------------------Replies

type replyKind int

const (
	velocityReply replyKind = iota
	curvatureReply
	controlReply
	stopReply
)

type reply struct {
	kind  replyKind
	value float64
	epoch int
	err   error
}

func (t *Traits) applyReply(r reply) {
	var refused *usecases.ProviderRefusal
	isRefusal := errors.As(r.err, &refused)
	conflict := isRefusal && refused.StatusCode == http.StatusConflict

	t.mu.Lock()
	before := t.m.code
	switch {
	case conflict && r.kind != stopReply:
		// Someone stopped the vehicle or has control. Never retried.
		t.m.refused(r.epoch, refused.Detail)
	case r.kind == controlReply && r.err == nil && r.value != 0:
		t.m.granted(r.epoch, time.Now())
	case r.kind == controlReply && r.err == nil:
		t.m.released(r.epoch)
	case r.kind == curvatureReply && isRefusal && refused.StatusCode == http.StatusServiceUnavailable:
		t.m.steeringRefused(r.epoch, refused.Detail)
	}
	// Anything else, an unreachable loader above all, the sender has already
	// logged. The belief does not change on a failure to talk, and the loader's own
	// silence limit stops a vehicle nobody can reach.
	after := t.m.code
	t.mu.Unlock()
	if after != before {
		log.Printf("chauffeur: %s", after)
	}
}

//-------------------------------------One sender per service

// sender delivers one service's commands, as the gamer's does: it holds only the
// newest value, so a slow network gives a vehicle that follows late, and never one
// that replays what it was told a while ago.
type sender struct {
	name    string
	cer     *components.Cervice
	sys     *components.System
	unit    string
	kind    replyKind
	replies chan<- reply
	next    chan job
	lastErr time.Time
}

type job struct {
	value float64
	epoch int
}

func newSender(name string, cer *components.Cervice, sys *components.System, unit string, replies chan<- reply, kind replyKind) *sender {
	return &sender{name: name, cer: cer, sys: sys, unit: unit, kind: kind, replies: replies, next: make(chan job, 1)}
}

func (s *sender) offer(v float64, epoch int) {
	j := job{value: v, epoch: epoch}
	for {
		select {
		case s.next <- j:
			return
		default:
		}
		select {
		case <-s.next:
		default:
		}
	}
}

func (s *sender) run(ctx context.Context) {
	for {
		select {
		case <-ctx.Done():
			return
		case j := <-s.next:
			err := s.put(j.value)
			if err != nil && time.Since(s.lastErr) > 5*time.Second {
				log.Printf("chauffeur: %s: %v", s.name, err)
				s.lastErr = time.Now()
			}
			select {
			case s.replies <- reply{kind: s.kind, value: j.value, epoch: j.epoch, err: err}:
			case <-ctx.Done():
				return
			}
		}
	}
}

func (s *sender) put(v float64) error {
	f := forms.SignalA_v1a{}
	f.NewForm()
	f.Value = v
	f.Unit = s.unit
	f.Timestamp = time.Now()
	body, err := usecases.Pack(&f, "application/json")
	if err != nil {
		return err
	}
	if len(s.cer.Nodes) == 0 {
		if err := usecases.Search4Services(s.cer, s.sys); err != nil {
			return err
		}
	}
	_, err = usecases.SetState(s.cer, s.sys, body)
	return err
}

//-------------------------------------Service handlers

// mayEngage is whether this caller may set the vehicle moving. The loader refuses
// a pilot that cannot be identified once certificates are in use, and engaging is
// the same act at one remove: it would be no protection if anyone on the network
// could reach the chauffeur's plain HTTP port and ask it to do the driving. On a
// bench with no certificates nobody can be told apart, as at the loader.
func (t *Traits) mayEngage(r *http.Request) bool {
	select {
	case <-usecases.EnsureCertReady(t.owner):
		_, known := usecases.PeerCN(r)
		return known
	default:
		return true
	}
}

func (t *Traits) engageService(w http.ResponseWriter, r *http.Request) {
	switch r.Method {
	case http.MethodGet:
		t.mu.Lock()
		active := t.m.code.active()
		t.mu.Unlock()
		v := 0.0
		if active {
			v = 1
		}
		usecases.HTTPProcessGetRequest(w, r, signal(v))
	case http.MethodPut:
		sig, err := usecases.HTTPProcessSetRequest(w, r)
		if err != nil {
			http.Error(w, "malformed request", http.StatusBadRequest)
			return
		}
		if sig.Value != 0 && sig.Value != 1 {
			http.Error(w, "1 to drive or go on, 0 to come to rest", http.StatusUnprocessableEntity)
			return
		}
		if !t.mayEngage(r) {
			http.Error(w, "an unidentified caller cannot set the vehicle moving", http.StatusConflict)
			return
		}
		t.mu.Lock()
		before := t.m.code
		if sig.Value == 1 {
			t.m.start(time.Now())
		} else {
			t.m.cancel(time.Now())
		}
		after := t.m.code
		t.mu.Unlock()
		if after != before {
			log.Printf("chauffeur: %s", after)
		}
		usecases.HTTPProcessGetRequest(w, r, signal(sig.Value))
	default:
		http.Error(w, "Method is not supported.", http.StatusNotFound)
	}
}

func (t *Traits) stateService(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method is not supported.", http.StatusNotFound)
		return
	}
	t.mu.Lock()
	c := t.m.code
	t.mu.Unlock()
	usecases.HTTPProcessGetRequest(w, r, signal(float64(c)))
}

func signal(v float64) *forms.SignalA_v1a {
	f := &forms.SignalA_v1a{Value: v, Timestamp: time.Now()}
	f.NewForm()
	return f
}
