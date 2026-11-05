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
	"hash/fnv"
	"io"
	"log"
	"math"
	"net/http"
	"sync"
	"time"

	"github.com/sdoque/mbaigo/components"
	"github.com/sdoque/mbaigo/forms"
	"github.com/sdoque/mbaigo/usecases"
)

const unitDegree = "<http://qudt.org/vocab/unit/DEG>"

// The chauffeur's drivingState codes. They are its contract: see its README.
const (
	stateIdle    = 0
	stateTaking  = 1
	stateDriving = 2
	stateArrived = 3
	stateBraking = 4
)

var stateTexts = map[int]string{
	stateIdle:    "Ready",
	stateTaking:  "Asking the loader for control",
	stateDriving: "Driving",
	stateArrived: "Arrived",
	stateBraking: "Coming to rest",
	10:           "Holding: something is too close ahead. Press Start when it is clear",
	11:           "Holding: no clearance reading from the sensor. Press Start when it is back",
	12:           "Holding: no recent position from the cartographer. Press Start when it is back",
	13:           "Holding: the route and the position are in different maps. Cancel and set a new destination",
	14:           "Holding: no current route, the way may be blocked. Press Start when a route is shown",
	15:           "Holding: too far from the route. Press Start once a new route is shown",
	16:           "Holding: the loader refused to steer; is the waist calibrated? See the chauffeur's log",
	17:           "Holding: the route has changed. Check the new route, then press Start",
	20:           "Stopped: something came too close. Take and release control with the pad, then press Start",
	21:           "No control: the vehicle is stopped or someone else is driving. Take and release control with the pad, then press Start",
}

// CommanderConfig is what the operator may set.
type CommanderConfig struct {
	// Vehicle picks the loader out of the cloud, for its stop.
	Vehicle map[string][]string `json:"vehicle"`
}

// Traits is the commander's state.
type Traits struct {
	cfg CommanderConfig

	// What the commander asks of the other systems. Fields, so that tests can stand
	// in for them.
	readMap   func() (*forms.MapA_v1a, error)
	readPose  func() (*forms.PoseA_v1a, error)
	readRoute func() (*forms.PathA_v1a, error)
	readState func() (int, error)
	setGoal   func(body []byte) (forms.Form, error)
	dropGoal  func() error
	engage    func(on bool) error
	stop      func() error

	mu       sync.Mutex
	picture  *mapPicture
	mapHash  uint64
	mapError string
	pose     *poseView
	route    *routeView
	state    stateView
	goal     *goalView
}

// The overview, as the page reads it.
type poseView struct {
	X, Y, Heading float64 // heading in degrees
	Frame         string
}

type routeView struct {
	X, Y  []float64
	Frame string
}

type goalView struct {
	X, Y           float64 // where the route ends
	AskedX, AskedY float64 // where it was asked for
	MovedBy        float64 // how far apart those are
	Frame          string
}

type stateView struct {
	Code      int
	Text      string
	Active    bool // the chauffeur has, or wants, control
	Reachable bool
}

type overview struct {
	Pose       *poseView
	Route      *routeView
	Goal       *goalView
	State      stateView
	Generation int
	MapError   string
}

//-------------------------------------Instantiate a unit asset template

func initTemplate() *components.UnitAsset {
	svc := func(def, methods, description string) *components.Service {
		return &components.Service{
			Definition:  def,
			SubPath:     def,
			Details:     map[string][]string{"Methods": components.HTTPMethods(methods)},
			RegPeriod:   30,
			Description: description,
		}
	}
	// Not "goal": a commander offering a service by that name could find itself when
	// it looks for the navigator's.
	view := svc("view", "GET", "the page: the map, the vehicle and the route; click to set a destination")
	over := svc("overview", "GET", "what the page shows, as JSON")
	image := svc("mapImage", "GET", "the seen part of the map as a PNG, and where it lies")
	dest := svc("destination", "PUT", "a destination for the navigator: a PoseA in the map, or a PolarA from the vehicle")
	cmd := svc("command", "PUT", "?do=start, cancel or stop")
	return &components.UnitAsset{
		Name:     "Console",
		Mission:  components.MissionControl,
		Mobility: components.MobilityMovable,
		ServicesMap: components.Services{
			view.SubPath: view, over.SubPath: over, image.SubPath: image, dest.SubPath: dest, cmd.SubPath: cmd,
		},
		Traits: &CommanderConfig{Vehicle: map[string][]string{"Model": {"artitrax"}}},
	}
}

//-------------------------------------Instantiate the unit asset

func newResource(uac usecases.ConfigurableAsset, sys *components.System) (*components.UnitAsset, func()) {
	cfg := CommanderConfig{}
	if len(uac.Traits) > 0 {
		if err := json.Unmarshal(uac.Traits[0], &cfg); err != nil {
			log.Fatalf("commander: cannot parse traits: %v", err)
		}
	}
	if cfg.Vehicle == nil {
		cfg.Vehicle = map[string][]string{"Model": {"artitrax"}}
	}

	protos := components.SProtocols(sys.Husk.ProtoPort)
	cer := func(def, mode string, details map[string][]string) *components.Cervice {
		return &components.Cervice{
			Definition: def, Protos: protos, Mode: mode,
			Nodes: make(map[string][]components.NodeInfo), Details: details,
		}
	}
	mapCer, poseCer, pathCer := cer("map", "get", nil), cer("pose", "get", nil), cer("path", "get", nil)
	stateCer := cer("drivingState", "get", nil)
	goalCer, cancelCer := cer("goal", "set", nil), cer("cancelGoal", "set", nil)
	engageCer, stopCer := cer("engage", "set", nil), cer("stop", "set", cfg.Vehicle)

	t := &Traits{cfg: cfg, state: stateView{Text: "Waiting for the chauffeur"}}
	t.readMap = func() (*forms.MapA_v1a, error) {
		f, err := fetch(mapCer, sys)
		if err != nil {
			return nil, err
		}
		m, ok := f.(*forms.MapA_v1a)
		if !ok {
			return nil, fmt.Errorf("the map came as %s", f.FormVersion())
		}
		return m, nil
	}
	t.readPose = func() (*forms.PoseA_v1a, error) {
		f, err := fetch(poseCer, sys)
		if err != nil {
			return nil, err
		}
		p, ok := f.(*forms.PoseA_v1a)
		if !ok {
			return nil, fmt.Errorf("the pose came as %s", f.FormVersion())
		}
		return p, nil
	}
	t.readRoute = func() (*forms.PathA_v1a, error) {
		f, err := fetch(pathCer, sys)
		if err != nil {
			return nil, err
		}
		p, ok := f.(*forms.PathA_v1a)
		if !ok {
			return nil, fmt.Errorf("the route came as %s", f.FormVersion())
		}
		return p, nil
	}
	t.readState = func() (int, error) {
		f, err := fetch(stateCer, sys)
		if err != nil {
			return 0, err
		}
		s, ok := f.(*forms.SignalA_v1a)
		if !ok {
			return 0, fmt.Errorf("the state came as %s", f.FormVersion())
		}
		return int(s.Value), nil
	}
	t.setGoal = func(body []byte) (forms.Form, error) { return put(goalCer, sys, body) }
	t.dropGoal = func() error { return putSignal(cancelCer, sys, 1) }
	t.engage = func(on bool) error {
		v := 0.0
		if on {
			v = 1
		}
		return putSignal(engageCer, sys, v)
	}
	t.stop = func() error { return putSignal(stopCer, sys, 1) }

	ua := &components.UnitAsset{
		Name:        uac.Name,
		Mission:     uac.Mission,
		Mobility:    uac.Mobility,
		Owner:       sys,
		Details:     uac.Details,
		ServicesMap: usecases.MakeServiceMap(uac.Services),
		CervicesMap: components.Cervices{
			"map": mapCer, "pose": poseCer, "path": pathCer, "drivingState": stateCer,
			"goal": goalCer, "cancelGoal": cancelCer, "engage": engageCer, "stop": stopCer,
		},
		Traits: t,
	}
	ua.ServingFunc = func(w http.ResponseWriter, r *http.Request, servicePath string) {
		serving(t, w, r, servicePath)
	}
	t.startReading(sys.Ctx)
	return ua, func() { log.Println("commander: shut down") }
}

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

func put(cer *components.Cervice, sys *components.System, body []byte) (forms.Form, error) {
	if len(cer.Nodes) == 0 {
		if err := usecases.Search4Services(cer, sys); err != nil {
			return nil, err
		}
	}
	return usecases.SetState(cer, sys, body)
}

func putSignal(cer *components.Cervice, sys *components.System, v float64) error {
	f := forms.SignalA_v1a{Value: v, Timestamp: time.Now()}
	f.NewForm()
	body, err := usecases.Pack(&f, "application/json")
	if err != nil {
		return err
	}
	_, err = put(cer, sys, body)
	return err
}

//-------------------------------------Reading

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
	every(2*time.Second, t.updateMap)
	every(250*time.Millisecond, t.updatePose)
	every(250*time.Millisecond, t.updateState)
	every(time.Second, t.updateRoute)
}

// updateMap draws the map again when it has changed.
func (t *Traits) updateMap() {
	m, err := t.readMap()
	if err != nil {
		t.mu.Lock()
		t.mapError = err.Error()
		t.mu.Unlock()
		return
	}
	h := fnv.New64a()
	h.Write(m.Cells)
	fmt.Fprintf(h, "%s %d %d %v %v", m.Frame, m.Width, m.Height, m.OriginX, m.OriginY)
	sum := h.Sum64()

	t.mu.Lock()
	same := t.picture != nil && t.mapHash == sum
	t.mu.Unlock()
	if same {
		t.mu.Lock()
		t.mapError = ""
		t.mu.Unlock()
		return
	}
	p, err := render(m)
	t.mu.Lock()
	defer t.mu.Unlock()
	if err != nil {
		t.mapError = err.Error()
		return
	}
	if t.picture != nil {
		p.Generation = t.picture.Generation
	}
	p.Generation++
	t.picture, t.mapHash, t.mapError = p, sum, ""
}

func (t *Traits) updatePose() {
	p, err := t.readPose()
	t.mu.Lock()
	defer t.mu.Unlock()
	if err != nil {
		t.pose = nil
		return
	}
	th := p.Heading
	if p.AngleUnit != "" && p.AngleUnit != unitDegree {
		th *= 180 / math.Pi
	}
	t.pose = &poseView{X: p.X, Y: p.Y, Heading: th, Frame: p.Frame}
}

func (t *Traits) updateRoute() {
	p, err := t.readRoute()
	t.mu.Lock()
	defer t.mu.Unlock()
	if err != nil {
		t.route = nil
		return
	}
	t.route = &routeView{X: p.X, Y: p.Y, Frame: p.Frame}
}

// updateState follows the chauffeur. Arriving ends the goal as well: the navigator
// checks for arrival only when it replans, and a goal left standing would go on
// being planned from where the vehicle now stands.
func (t *Traits) updateState() {
	code, err := t.readState()
	t.mu.Lock()
	if err != nil {
		t.state = stateView{Text: "The chauffeur cannot be reached"}
		t.mu.Unlock()
		return
	}
	arrivedNow := code == stateArrived && t.state.Code != stateArrived
	text, known := stateTexts[code]
	if !known {
		text = fmt.Sprintf("Chauffeur state %d", code)
	}
	t.state = stateView{Code: code, Text: text, Reachable: true, Active: isActive(code)}
	if arrivedNow {
		t.goal, t.route = nil, nil
	}
	t.mu.Unlock()
	if arrivedNow {
		if err := t.dropGoal(); err != nil {
			log.Printf("commander: arrived, and could not drop the goal: %v", err)
		}
	}
}

// isActive is whether the chauffeur has, or wants, control.
func isActive(code int) bool {
	switch code {
	case stateTaking, stateDriving, stateBraking:
		return true
	}
	return code >= 10 && code < 20
}

//-------------------------------------Service handlers

func (t *Traits) viewService(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method is not supported.", http.StatusNotFound)
		return
	}
	w.Header().Set("Content-Type", "text/html; charset=utf-8")
	if _, err := io.WriteString(w, pageHTML); err != nil {
		log.Printf("commander: writing the page: %v", err)
	}
}

func (t *Traits) overviewService(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method is not supported.", http.StatusNotFound)
		return
	}
	t.mu.Lock()
	o := overview{Pose: t.pose, Route: t.route, Goal: t.goal, State: t.state, MapError: t.mapError}
	if t.picture != nil {
		o.Generation = t.picture.Generation
	}
	t.mu.Unlock()
	// Only what belongs to the map on show: a route or goal from a previous run of
	// the cartographer names somewhere else now.
	if o.Pose != nil {
		if o.Route != nil && o.Route.Frame != o.Pose.Frame {
			o.Route = nil
		}
		if o.Goal != nil && o.Goal.Frame != o.Pose.Frame {
			o.Goal = nil
		}
	}
	writeJSON(w, o)
}

func (t *Traits) mapImageService(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method is not supported.", http.StatusNotFound)
		return
	}
	t.mu.Lock()
	p := t.picture
	t.mu.Unlock()
	if p == nil {
		http.Error(w, "no map yet", http.StatusServiceUnavailable)
		return
	}
	writeJSON(w, p)
}

// destinationService passes a destination on to the navigator. It is refused while
// the chauffeur is working: a new route under a moving vehicle is one nobody has
// looked at. The chauffeur would hold for it anyway, and this says why at once.
func (t *Traits) destinationService(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPut {
		http.Error(w, "Method is not supported.", http.StatusNotFound)
		return
	}
	t.mu.Lock()
	active := t.state.Active
	t.mu.Unlock()
	if active {
		http.Error(w, "the vehicle is on a route: cancel it before setting another destination", http.StatusConflict)
		return
	}
	body, err := io.ReadAll(io.LimitReader(r.Body, 1<<16))
	if err != nil {
		http.Error(w, "cannot read the request", http.StatusBadRequest)
		return
	}
	asked, err := usecases.Unpack(body, "application/json")
	if err != nil {
		http.Error(w, "malformed destination: "+err.Error(), http.StatusBadRequest)
		return
	}
	f, err := t.setGoal(body)
	if err != nil {
		writeRefusal(w, err)
		return
	}
	end, ok := f.(*forms.PoseA_v1a)
	if !ok {
		http.Error(w, "the navigator answered with "+f.FormVersion(), http.StatusBadGateway)
		return
	}
	g := &goalView{X: end.X, Y: end.Y, AskedX: end.X, AskedY: end.Y, Frame: end.Frame}
	// The navigator says in a header why a route ends elsewhere, and the framework's
	// consumers do not see headers. The distance can be worked out, and that is what
	// a person deciding whether to press Start needs. A distance and bearing from the
	// vehicle cannot be compared: it is not a point until the navigator makes it one.
	if p, ok := asked.(*forms.PoseA_v1a); ok {
		g.AskedX, g.AskedY = p.X, p.Y
		g.MovedBy = math.Hypot(end.X-p.X, end.Y-p.Y)
	}
	t.mu.Lock()
	t.goal = g
	t.mu.Unlock()
	writeJSON(w, g)
}

// commandService is the page's three buttons. A PUT, which a browser will not send
// to another site's address without asking first, so a page open in another tab
// cannot press them.
func (t *Traits) commandService(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPut {
		http.Error(w, "Method is not supported.", http.StatusNotFound)
		return
	}
	switch r.URL.Query().Get("do") {
	case "start":
		t.mu.Lock()
		hasRoute := t.route != nil
		t.mu.Unlock()
		if !hasRoute {
			http.Error(w, "there is no route to start on", http.StatusConflict)
			return
		}
		if err := t.engage(true); err != nil {
			writeRefusal(w, err)
			return
		}
	case "cancel":
		// Both, even if one fails: a chauffeur that cannot be reached is no reason to
		// leave the goal standing.
		engageErr := t.engage(false)
		dropErr := t.dropGoal()
		t.mu.Lock()
		t.goal, t.route = nil, nil
		t.mu.Unlock()
		if err := errors.Join(engageErr, dropErr); err != nil {
			writeRefusal(w, err)
			return
		}
	case "stop":
		if err := t.stop(); err != nil {
			writeRefusal(w, err)
			return
		}
		log.Println("commander: STOP sent to the loader")
	default:
		http.Error(w, "do must be start, cancel or stop", http.StatusBadRequest)
		return
	}
	w.WriteHeader(http.StatusNoContent)
}

// writeRefusal passes on what another system said, so that the reason reaches the
// person who clicked.
func writeRefusal(w http.ResponseWriter, err error) {
	var refused *usecases.ProviderRefusal
	if errors.As(err, &refused) && refused.Detail != "" {
		http.Error(w, refused.Detail, refused.StatusCode)
		return
	}
	http.Error(w, err.Error(), http.StatusBadGateway)
}

func writeJSON(w http.ResponseWriter, v any) {
	body, err := json.Marshal(v)
	if err != nil {
		http.Error(w, "cannot encode the response", http.StatusInternalServerError)
		return
	}
	w.Header().Set("Content-Type", "application/json")
	w.Header().Set("Cache-Control", "no-store")
	if _, err := w.Write(body); err != nil {
		log.Printf("commander: writing response: %v", err)
	}
}
