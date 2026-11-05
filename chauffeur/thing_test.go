package main

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"sync"
	"testing"
	"time"

	"github.com/sdoque/mbaigo/components"
	"github.com/sdoque/mbaigo/forms"
	"github.com/sdoque/mbaigo/usecases"
)

// fakeSink stands in for the loader's services.
type fakeSink struct {
	mu     sync.Mutex
	values []float64
	times  []time.Time
	answer func(float64) error
	kind   replyKind
	reply  chan<- reply
}

func (f *fakeSink) offer(v float64, epoch int) {
	f.mu.Lock()
	f.values, f.times = append(f.values, v), append(f.times, time.Now())
	answer := f.answer
	f.mu.Unlock()
	var err error
	if answer != nil {
		err = answer(v)
	}
	if f.reply != nil {
		f.reply <- reply{kind: f.kind, value: v, epoch: epoch, err: err}
	}
}

func (f *fakeSink) put(float64) error { return nil }

func (f *fakeSink) sent() []float64 {
	f.mu.Lock()
	defer f.mu.Unlock()
	return append([]float64(nil), f.values...)
}

// rig is a chauffeur with its readings and the loader stood in for.
func rig() (*Traits, map[string]*fakeSink) {
	cfg := ChauffeurConfig{}
	applyDefaults(&cfg)
	t := &Traits{cfg: cfg, replies: make(chan reply, 256), m: &mission{cfg: cfg.tuning()}}
	t.seen.limit = 0.48
	sinks := map[string]*fakeSink{
		"velocity":  {kind: velocityReply, reply: t.replies},
		"curvature": {kind: curvatureReply, reply: t.replies},
		"control":   {kind: controlReply, reply: t.replies},
		"stop":      {kind: stopReply, reply: t.replies},
	}
	t.velocity, t.curvature, t.control, t.stop = sinks["velocity"], sinks["curvature"], sinks["control"], sinks["stop"]
	t.readPose = func() (pose, error) {
		return pose{x: 5, frame: "map@sim", at: time.Now()}, nil
	}
	planned := time.Now()
	t.readRoute = func() (*forms.PathA_v1a, error) {
		return &forms.PathA_v1a{X: []float64{0, 5, 15}, Y: []float64{0, 0, 0}, Frame: "map@sim", Timestamp: planned}, nil
	}
	t.readClearance = func() (float64, time.Time, error) { return 10, time.Now(), nil }
	t.readLimit = func() (float64, error) { return 0.48, nil }
	return t, sinks
}

// settle runs n control cycles with the readings up to date.
func settle(t *Traits, n int) {
	for i := 0; i < n; i++ {
		t.updatePose()
		t.updateClearance()
		t.updateRoute()
		t.cycle(time.Now())
		for len(t.replies) > 0 {
			t.applyReply(<-t.replies)
		}
		time.Sleep(5 * time.Millisecond)
	}
}

func refusedWith(code int) error {
	return &usecases.ProviderRefusal{StatusCode: code, Status: http.StatusText(code), Detail: "the loader says no"}
}

func TestItTakesControlAndDrives(t *testing.T) {
	tr, sinks := rig()
	tr.startMission()
	settle(tr, 3)
	if got := sinks["control"].sent(); len(got) != 1 || got[0] != 1 {
		t.Errorf("control %v, want one take", got)
	}
	v := sinks["velocity"].sent()
	if len(v) == 0 || v[len(v)-1] != 0.5 {
		t.Errorf("velocity %v", v)
	}
}

func (t *Traits) startMission() {
	t.mu.Lock()
	t.m.start(time.Now())
	t.mu.Unlock()
}

// The loader's 409: no more commands of any kind, and no asking again.
func TestA409EndsItAtTheLoader(t *testing.T) {
	tr, sinks := rig()
	sinks["velocity"].answer = func(float64) error { return refusedWith(http.StatusConflict) }
	tr.startMission()
	settle(tr, 3)
	if tr.m.code != codeLostControl {
		t.Fatalf("%q", tr.m.code)
	}
	nV, nC := len(sinks["velocity"].sent()), len(sinks["control"].sent())
	settle(tr, 5)
	if len(sinks["velocity"].sent()) != nV || len(sinks["control"].sent()) != nC {
		t.Error("went on commanding after a 409")
	}
}

// The loader's 503 on a curvature keeps control and holds.
func TestA503OnCurvatureHolds(t *testing.T) {
	tr, sinks := rig()
	sinks["curvature"].answer = func(float64) error { return refusedWith(http.StatusServiceUnavailable) }
	tr.startMission()
	settle(tr, 4)
	if tr.m.code != codeHoldSteering {
		t.Fatalf("%q", tr.m.code)
	}
	v := sinks["velocity"].sent()
	if v[len(v)-1] != 0 {
		t.Errorf("velocity %v, want the last at zero", v)
	}
}

// A 503 from the guetteur means blind, and blind holds.
func TestABlindSensorHolds(t *testing.T) {
	tr, _ := rig()
	tr.startMission()
	settle(tr, 3)
	tr.readClearance = func() (float64, time.Time, error) { return 0, time.Time{}, refusedWith(http.StatusServiceUnavailable) }
	settle(tr, 2)
	if tr.m.code != codeHoldBlind {
		t.Errorf("%q", tr.m.code)
	}
}

// The navigator answering that there is no route takes the route away. Failing to
// reach it does not: the route goes stale by its stamp.
func TestNoRouteAndNoNavigatorAreDifferent(t *testing.T) {
	tr, _ := rig()
	tr.updateRoute()
	first := tr.seen.route
	if first == nil {
		t.Fatal("no route")
	}
	tr.updateRoute()
	if tr.seen.route != first {
		t.Error("the same plan made a new route")
	}
	tr.readRoute = func() (*forms.PathA_v1a, error) { return nil, errors.New("connection refused") }
	tr.updateRoute()
	if tr.seen.route != first {
		t.Error("an unreachable navigator took the route away")
	}
	tr.readRoute = func() (*forms.PathA_v1a, error) { return nil, refusedWith(http.StatusServiceUnavailable) }
	tr.updateRoute()
	if tr.seen.route != nil {
		t.Error("the navigator said no route, and the route stayed")
	}
}

// The reason each reading has a goroutine: a fetch that hangs must not silence the
// commands, because the loader stops a pilot quiet for half a second.
func TestAHungFetchDoesNotSilenceTheCommands(t *testing.T) {
	tr, sinks := rig()
	release := make(chan struct{})
	defer close(release)
	tr.startMission()
	settle(tr, 2)
	hang := func() (*forms.PathA_v1a, error) {
		<-release
		return nil, errors.New("timed out")
	}
	tr.readRoute = hang
	tr.readPose = func() (pose, error) {
		<-release
		return pose{}, errors.New("timed out")
	}
	tr.readClearance = func() (float64, time.Time, error) {
		<-release
		return 0, time.Time{}, errors.New("timed out")
	}

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	tr.startReading(ctx)
	go tr.run(ctx)
	time.Sleep(1600 * time.Millisecond) // long enough for the pose and clearance to go stale

	sinks["velocity"].mu.Lock()
	times := append([]time.Time(nil), sinks["velocity"].times...)
	sinks["velocity"].mu.Unlock()
	for i := 1; i < len(times); i++ {
		if gap := times[i].Sub(times[i-1]); gap > 300*time.Millisecond {
			t.Errorf("%v of silence with fetches hanging", gap)
		}
	}
	tr.mu.Lock()
	code := tr.m.code
	tr.mu.Unlock()
	if !code.holding() {
		t.Errorf("%q, want a hold for readings gone stale", code)
	}
}

// An engage over plain HTTP, with nothing to say who is asking.
func engage(t *testing.T, tr *Traits, v float64) *httptest.ResponseRecorder {
	t.Helper()
	f := signal(v)
	body, err := json.Marshal(f)
	if err != nil {
		t.Fatal(err)
	}
	r := httptest.NewRequest(http.MethodPut, "/chauffeur/Driver/engage", bytes.NewReader(body))
	r.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	tr.engageService(w, r)
	return w
}

func benchSystem() *components.System {
	sys := components.NewSystem("chauffeur", context.Background())
	sys.Husk = &components.Husk{}
	return &sys
}

func TestEngageStartsAndCancels(t *testing.T) {
	tr, _ := rig()
	tr.owner = benchSystem()
	if w := engage(t, tr, 1); w.Code != http.StatusOK {
		t.Fatalf("%d %s", w.Code, w.Body)
	}
	if tr.m.code != codeTaking {
		t.Errorf("%q", tr.m.code)
	}
	if w := engage(t, tr, 0); w.Code != http.StatusOK || tr.m.code != codeBraking {
		t.Errorf("%d, %q", w.Code, tr.m.code)
	}
	if w := engage(t, tr, 2); w.Code != http.StatusUnprocessableEntity {
		t.Errorf("a value of 2: %d", w.Code)
	}
}

func TestStateIsTheCode(t *testing.T) {
	tr, _ := rig()
	tr.m.code = codeHoldObstacle
	w := httptest.NewRecorder()
	tr.stateService(w, httptest.NewRequest(http.MethodGet, "/chauffeur/Driver/drivingState", nil))
	var f forms.SignalA_v1a
	if err := json.Unmarshal(w.Body.Bytes(), &f); err != nil {
		t.Fatal(err)
	}
	if Code(f.Value) != codeHoldObstacle {
		t.Errorf("%v", f.Value)
	}
}
