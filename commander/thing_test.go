package main

import (
	"bytes"
	"encoding/json"
	"errors"
	"image/png"
	"math"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/sdoque/mbaigo/forms"
	"github.com/sdoque/mbaigo/usecases"
)

const frame = "map@2026-11-05T12:00:00Z"

// rig is a commander with the other systems stood in for, and what it asked of them.
func rig() (*Traits, *[]string) {
	calls := &[]string{}
	note := func(s string) { *calls = append(*calls, s) }
	t := &Traits{}
	t.setGoal = func(body []byte) (forms.Form, error) {
		note("goal")
		f, err := usecases.Unpack(body, "application/json")
		if err != nil {
			return nil, err
		}
		p := f.(*forms.PoseA_v1a)
		end := &forms.PoseA_v1a{X: p.X, Y: p.Y, Frame: frame}
		end.NewForm()
		return end, nil
	}
	t.dropGoal = func() error { note("drop"); return nil }
	t.engage = func(on bool) error {
		if on {
			note("engage 1")
		} else {
			note("engage 0")
		}
		return nil
	}
	t.stop = func() error { note("stop"); return nil }
	return t, calls
}

func call(t *Traits, method, path, body string) *httptest.ResponseRecorder {
	r := httptest.NewRequest(method, "http://localhost:20207/commander/Console/"+path, strings.NewReader(body))
	r.RemoteAddr = "127.0.0.1:50000"
	w := httptest.NewRecorder()
	service := strings.SplitN(path, "?", 2)[0]
	serving(t, w, r, service)
	return w
}

func TestOnlyThisMachineMayAsk(t *testing.T) {
	for _, c := range []struct {
		remote, host string
		want         bool
	}{
		{"127.0.0.1:5000", "localhost:20207", true},
		{"127.0.0.1:5000", "127.0.0.1:20207", true},
		{"[::1]:5000", "[::1]:20207", true},
		{"127.0.0.1:5000", "localhost", true},
		{"192.168.1.20:5000", "localhost:20207", false},
		{"192.168.1.20:5000", "192.168.1.5:20207", false},
		// A web page's own name pointed at this machine: DNS rebinding.
		{"127.0.0.1:5000", "evil.example:20207", false},
		{"127.0.0.1:5000", "192.168.1.5:20207", false},
		{"nonsense", "localhost", false},
	} {
		r := httptest.NewRequest(http.MethodGet, "/", nil)
		r.RemoteAddr, r.Host = c.remote, c.host
		if got := fromThisMachine(r); got != c.want {
			t.Errorf("from %s to %s: %v, want %v", c.remote, c.host, got, c.want)
		}
	}
	tr, calls := rig()
	r := httptest.NewRequest(http.MethodPut, "http://192.168.1.5:20207/commander/Console/command?do=start", nil)
	r.RemoteAddr = "192.168.1.20:5000"
	w := httptest.NewRecorder()
	serving(tr, w, r, "command")
	if w.Code != http.StatusForbidden || len(*calls) != 0 {
		t.Errorf("%d, and asked %v", w.Code, *calls)
	}
}

func TestADestinationGoesToTheNavigator(t *testing.T) {
	tr, calls := rig()
	w := call(tr, http.MethodPut, "destination", `{"x": 4, "y": 2, "frame": "`+frame+`", "version": "PoseA_v1.0"}`)
	if w.Code != http.StatusOK || len(*calls) != 1 || (*calls)[0] != "goal" {
		t.Fatalf("%d %s, asked %v", w.Code, w.Body, *calls)
	}
	if tr.goal == nil || tr.goal.X != 4 || tr.goal.MovedBy != 0 {
		t.Errorf("%+v", tr.goal)
	}
}

// The route ending elsewhere is what a person needs to know before pressing Start.
func TestARouteThatEndsElsewhereIsReported(t *testing.T) {
	tr, _ := rig()
	tr.setGoal = func([]byte) (forms.Form, error) {
		end := &forms.PoseA_v1a{X: 1, Y: 2, Frame: frame}
		end.NewForm()
		return end, nil
	}
	w := call(tr, http.MethodPut, "destination", `{"x": 4, "y": 6, "version": "PoseA_v1.0"}`)
	var g goalView
	if err := json.Unmarshal(w.Body.Bytes(), &g); err != nil {
		t.Fatal(err)
	}
	if math.Abs(g.MovedBy-5) > 1e-9 || g.AskedX != 4 || g.X != 1 {
		t.Errorf("%+v", g)
	}
}

// What the navigator says goes back to the person who clicked.
func TestTheNavigatorsReasonReachesThePage(t *testing.T) {
	tr, _ := rig()
	tr.setGoal = func([]byte) (forms.Form, error) {
		return nil, &usecases.ProviderRefusal{StatusCode: 409, Status: "409 Conflict", Detail: "no route: no forward route"}
	}
	w := call(tr, http.MethodPut, "destination", `{"x": 4, "y": 6, "version": "PoseA_v1.0"}`)
	if w.Code != 409 || !strings.Contains(w.Body.String(), "no forward route") {
		t.Errorf("%d %s", w.Code, w.Body)
	}
	if tr.goal != nil {
		t.Error("a refused destination was kept")
	}
}

func TestNoNewDestinationWhileOnARoute(t *testing.T) {
	tr, calls := rig()
	tr.state.Active = true
	w := call(tr, http.MethodPut, "destination", `{"x": 4, "y": 2, "version": "PoseA_v1.0"}`)
	if w.Code != http.StatusConflict || len(*calls) != 0 {
		t.Errorf("%d, asked %v", w.Code, *calls)
	}
}

func TestABadDestinationIsRefusedHere(t *testing.T) {
	tr, calls := rig()
	if w := call(tr, http.MethodPut, "destination", `not json`); w.Code != http.StatusBadRequest || len(*calls) != 0 {
		t.Errorf("%d, asked %v", w.Code, *calls)
	}
}

func TestStartNeedsARoute(t *testing.T) {
	tr, calls := rig()
	if w := call(tr, http.MethodPut, "command?do=start", ""); w.Code != http.StatusConflict || len(*calls) != 0 {
		t.Errorf("%d, asked %v", w.Code, *calls)
	}
	tr.route = &routeView{X: []float64{0, 1}, Y: []float64{0, 0}, Frame: frame}
	if w := call(tr, http.MethodPut, "command?do=start", ""); w.Code != http.StatusNoContent || (*calls)[0] != "engage 1" {
		t.Errorf("%d, asked %v", w.Code, *calls)
	}
}

func TestCancelEndsBothEvenIfOneFails(t *testing.T) {
	tr, calls := rig()
	tr.goal, tr.route = &goalView{}, &routeView{}
	tr.engage = func(bool) error { *calls = append(*calls, "engage"); return errors.New("no chauffeur") }
	w := call(tr, http.MethodPut, "command?do=cancel", "")
	if w.Code != http.StatusBadGateway {
		t.Errorf("%d", w.Code)
	}
	if len(*calls) != 2 || (*calls)[1] != "drop" {
		t.Errorf("asked %v", *calls)
	}
	if tr.goal != nil || tr.route != nil {
		t.Error("the goal and route are still shown")
	}
}

func TestStopGoesToTheLoader(t *testing.T) {
	tr, calls := rig()
	if w := call(tr, http.MethodPut, "command?do=stop", ""); w.Code != http.StatusNoContent || (*calls)[0] != "stop" {
		t.Errorf("%d, asked %v", w.Code, *calls)
	}
}

// A GET must not be able to press a button: a link or an image could send one.
func TestCommandsAreNotGets(t *testing.T) {
	tr, calls := rig()
	tr.route = &routeView{}
	for _, path := range []string{"command?do=start", "command?do=stop", "destination"} {
		if w := call(tr, http.MethodGet, path, ""); w.Code == http.StatusNoContent || w.Code == http.StatusOK {
			t.Errorf("GET %s: %d", path, w.Code)
		}
	}
	if len(*calls) != 0 {
		t.Errorf("asked %v", *calls)
	}
	if w := call(tr, http.MethodPut, "command?do=fly", ""); w.Code != http.StatusBadRequest {
		t.Errorf("an unknown command: %d", w.Code)
	}
}

func TestArrivingEndsTheGoalOnce(t *testing.T) {
	tr, calls := rig()
	tr.goal, tr.route = &goalView{}, &routeView{}
	tr.readState = func() (int, error) { return stateArrived, nil }
	tr.updateState()
	tr.updateState()
	if len(*calls) != 1 || (*calls)[0] != "drop" {
		t.Errorf("asked %v", *calls)
	}
	if tr.goal != nil || tr.route != nil {
		t.Error("the goal and route are still shown")
	}
}

func TestEveryChauffeurStateIsDescribed(t *testing.T) {
	for _, code := range []int{0, 1, 2, 3, 4, 10, 11, 12, 13, 14, 15, 16, 17, 20, 21} {
		if stateTexts[code] == "" {
			t.Errorf("state %d has no text", code)
		}
	}
	tr, _ := rig()
	tr.readState = func() (int, error) { return 10, nil }
	tr.updateState()
	if !tr.state.Active || !strings.Contains(tr.state.Text, "too close") {
		t.Errorf("%+v", tr.state)
	}
	tr.readState = func() (int, error) { return 21, nil }
	tr.updateState()
	if tr.state.Active {
		t.Error("having lost control counted as active")
	}
	tr.readState = func() (int, error) { return 0, errors.New("gone") }
	tr.updateState()
	if tr.state.Reachable {
		t.Error("a chauffeur that cannot be reached counted as reachable")
	}
}

// A route from a run of the cartographer that has ended names somewhere else.
func TestARouteFromAnotherMapIsNotShown(t *testing.T) {
	tr, _ := rig()
	tr.pose = &poseView{Frame: frame}
	tr.route = &routeView{Frame: "map@earlier"}
	tr.goal = &goalView{Frame: "map@earlier"}
	var o overview
	if err := json.Unmarshal(call(tr, http.MethodGet, "overview", "").Body.Bytes(), &o); err != nil {
		t.Fatal(err)
	}
	if o.Route != nil || o.Goal != nil {
		t.Errorf("%+v", o)
	}
}

// testMap is 10 by 8 m at 10 cm, with its origin somewhere other than zero, unseen
// but for a free patch and a wall at known places.
func testMap() *forms.MapA_v1a {
	m := &forms.MapA_v1a{Width: 100, Height: 80, Resolution: 0.1, OriginX: -3, OriginY: 2, Frame: frame}
	m.NewForm()
	m.Cells = make([]byte, 8000)
	for i := range m.Cells {
		m.Cells[i] = forms.MapUnknown
	}
	for j := 30; j < 50; j++ {
		for i := 40; i < 70; i++ {
			m.Cells[j*100+i] = forms.MapFree
		}
	}
	m.Cells[45*100+60] = forms.MapOccupied
	return m
}

// The picture lies where its numbers say: the wall's pixel, turned back into map
// coordinates the way the page does it, is the wall.
func TestThePictureIsPlacedCorrectly(t *testing.T) {
	m := testMap()
	p, err := render(m)
	if err != nil {
		t.Fatal(err)
	}
	img, err := png.Decode(bytes.NewReader(p.PNG))
	if err != nil {
		t.Fatal(err)
	}
	if b := img.Bounds(); b.Dx() != p.Width || b.Dy() != p.Height {
		t.Fatalf("%v, says %d by %d", b, p.Width, p.Height)
	}
	// Cropped to the seen 30 by 20 cells and ten cells of margin all round.
	if p.Width != 50 || p.Height != 40 {
		t.Errorf("%d by %d", p.Width, p.Height)
	}
	dark := 0
	for y := 0; y < p.Height; y++ {
		for x := 0; x < p.Width; x++ {
			if r, _, _, _ := img.At(x, y).RGBA(); r>>8 < 100 {
				dark++
				wx := p.MinX + (float64(x)+0.5)*p.Resolution
				wy := p.MaxY - (float64(y)+0.5)*p.Resolution
				// The wall's cell runs x 3..3.1 and y 6.5..6.6.
				if math.Abs(wx-3.05) > 1e-9 || math.Abs(wy-6.55) > 1e-9 {
					t.Errorf("the wall came out at (%.3f, %.3f)", wx, wy)
				}
			}
		}
	}
	if dark != 1 {
		t.Errorf("%d wall pixels", dark)
	}
}

func TestAMapWithNothingSeenIsNotDrawn(t *testing.T) {
	m := testMap()
	for i := range m.Cells {
		m.Cells[i] = forms.MapUnknown
	}
	if _, err := render(m); err == nil {
		t.Error("drew an empty map")
	}
}

func TestTheMapIsDrawnAgainOnlyWhenItChanges(t *testing.T) {
	tr, _ := rig()
	m := testMap()
	tr.readMap = func() (*forms.MapA_v1a, error) { return m, nil }
	tr.updateMap()
	first := tr.picture
	if first == nil || first.Generation != 1 {
		t.Fatalf("%+v", first)
	}
	tr.updateMap()
	if tr.picture != first {
		t.Error("drawn again with nothing changed")
	}
	m.Cells[0] = forms.MapFree
	tr.updateMap()
	if tr.picture == first || tr.picture.Generation != 2 {
		t.Errorf("not drawn again: generation %d", tr.picture.Generation)
	}
}

func TestThePageIsServedAndSelfContained(t *testing.T) {
	tr, _ := rig()
	w := call(tr, http.MethodGet, "view", "")
	if w.Code != http.StatusOK || !strings.HasPrefix(w.Header().Get("Content-Type"), "text/html") {
		t.Fatalf("%d %s", w.Code, w.Header().Get("Content-Type"))
	}
	body := w.Body.String()
	for _, outside := range []string{"http://", "https://", "src=\"", "<link"} {
		if strings.Contains(body, outside) {
			t.Errorf("the page reaches outside itself: %q", outside)
		}
	}
}
