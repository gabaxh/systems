package main

import (
	"context"
	"encoding/json"
	"math"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"

	"github.com/sdoque/mbaigo/forms"
)

func testTraits(sw sweep, got bool) *Traits {
	cfg := GuetteurConfig{}
	applyDefaults(&cfg)
	return &Traits{cfg: cfg, latest: sw, got: got, sector: cfg.SectorDegrees}
}

func freshSweep(angles, distances []float64, valid []bool) sweep {
	return sweep{angles: angles, distances: distances, valid: valid, taken: time.Now()}
}

// The argument this whole system exists to make: a sector with no returns must
// not answer with a distance. There is no float that safely means "blind".
func TestClearanceRefusesToAnswerWhenBlind(t *testing.T) {
	tr := testTraits(freshSweep(
		[]float64{-10, 0, 10},
		[]float64{0, 0, 0},
		[]bool{false, false, false},
	), true)

	w := httptest.NewRecorder()
	tr.clearanceService(w, httptest.NewRequest(http.MethodGet, "/clearance", nil))

	if w.Code != http.StatusServiceUnavailable {
		t.Fatalf("a blind sensor answered %d with %q; it must not answer with a distance",
			w.Code, w.Body.String())
	}
}

// A maximum-range reading and a silence are the same bytes on the wire and
// opposite facts on a moving vehicle.
func TestClearanceIgnoresInvalidPointsButUsesValidOnes(t *testing.T) {
	tr := testTraits(freshSweep(
		[]float64{-10, 0, 10},
		[]float64{50, 4.0, 50}, // the 50s are what a sensor reports when it saw nothing
		[]bool{false, true, false},
	), true)

	w := httptest.NewRecorder()
	tr.clearanceService(w, httptest.NewRequest(http.MethodGet, "/clearance", nil))
	if w.Code != http.StatusOK {
		t.Fatalf("clearance returned %d, want 200", w.Code)
	}
	var f forms.SignalA_v1a
	if err := json.Unmarshal(w.Body.Bytes(), &f); err != nil {
		t.Fatalf("decode: %v", err)
	}
	if f.Value != 4.0 {
		t.Errorf("clearance = %v, want 4.0 — the invalid 50 m readings must not count", f.Value)
	}
}

// A sweep older than the freshness window is not an answer either. A file
// cannot say whether it is still true; a service can.
func TestClearanceRefusesAStaleSweep(t *testing.T) {
	sw := freshSweep([]float64{0}, []float64{3}, []bool{true})
	sw.taken = time.Now().Add(-5 * time.Second)
	tr := testTraits(sw, true)

	w := httptest.NewRecorder()
	tr.clearanceService(w, httptest.NewRequest(http.MethodGet, "/clearance", nil))
	if w.Code != http.StatusServiceUnavailable {
		t.Errorf("a five-second-old sweep was served as current (%d)", w.Code)
	}
}

func TestNearestInSectorRespectsTheSector(t *testing.T) {
	sw := freshSweep(
		[]float64{-80, 0, 80},
		[]float64{1.0, 6.0, 1.0}, // the close returns are off to the sides
		[]bool{true, true, true},
	)
	d, ok := nearestInSector(sw, 40)
	if !ok {
		t.Fatal("nothing found in the forward sector")
	}
	if d != 6.0 {
		t.Errorf("clearance = %v, want 6.0: the 1 m returns are outside the ±20° sector", d)
	}
}

// quality is what tells a consumer why clearance has gone quiet, so it must
// still answer when the sweep is all silence.
func TestQualityReportsZeroWhenNothingCameBack(t *testing.T) {
	tr := testTraits(freshSweep(
		[]float64{-10, 0, 10},
		[]float64{0, 0, 0},
		[]bool{false, false, false},
	), true)

	w := httptest.NewRecorder()
	tr.qualityService(w, httptest.NewRequest(http.MethodGet, "/quality", nil))
	if w.Code != http.StatusOK {
		t.Fatalf("quality returned %d, want 200 — it must answer precisely when clearance cannot", w.Code)
	}
	var f forms.SignalA_v1a
	if err := json.Unmarshal(w.Body.Bytes(), &f); err != nil {
		t.Fatalf("decode: %v", err)
	}
	if f.Value != 0 {
		t.Errorf("quality = %v, want 0", f.Value)
	}
}

func TestScanServiceProducesACheckableForm(t *testing.T) {
	tr := testTraits(freshSweep(
		[]float64{-1, 0, 1},
		[]float64{2, 3, 4},
		[]bool{true, true, false},
	), true)

	w := httptest.NewRecorder()
	tr.scanService(w, httptest.NewRequest(http.MethodGet, "/scan", nil))
	if w.Code != http.StatusOK {
		t.Fatalf("scan returned %d", w.Code)
	}
	var f forms.ScanA_v1a
	if err := json.Unmarshal(w.Body.Bytes(), &f); err != nil {
		t.Fatalf("decode: %v", err)
	}
	if err := f.Check(); err != nil {
		t.Errorf("the served scan does not check out: %v", err)
	}
	if f.ValidCount() != 2 {
		t.Errorf("ValidCount = %d, want 2", f.ValidCount())
	}
}

// A bounced beam reports a point behind the wall its neighbours agree on. It
// must leave the sweep as a non-return, and nothing that looks like a real
// thing may go with it.
func TestRejectSpikes(t *testing.T) {
	cases := []struct {
		name      string
		distances []float64
		valid     []bool
		jump      float64
		want      []bool
	}{
		{"a ghost behind a wall is dropped",
			[]float64{2, 2, 5, 2, 2}, []bool{true, true, true, true, true}, 0.5,
			[]bool{true, true, false, true, true}},
		{"a post in front of a wall is kept",
			[]float64{4, 4, 1, 4, 4}, []bool{true, true, true, true, true}, 0.5,
			[]bool{true, true, true, true, true}},
		{"the ends of the sweep are kept",
			[]float64{6, 2, 2, 2, 6}, []bool{true, true, true, true, true}, 0.5,
			[]bool{true, true, true, true, true}},
		{"a point beside a non-return is kept",
			[]float64{2, 0, 5, 2, 2}, []bool{true, false, true, true, true}, 0.5,
			[]bool{true, false, true, true, true}},
		{"a two-point ghost gets through, a known limit",
			[]float64{2, 5, 5, 2}, []bool{true, true, true, true}, 0.5,
			[]bool{true, true, true, true}},
		{"a small step is a surface, not a ghost",
			[]float64{2, 2, 2.3, 2, 2}, []bool{true, true, true, true, true}, 0.5,
			[]bool{true, true, true, true, true}},
		{"a negative jump turns the filter off",
			[]float64{2, 2, 5, 2, 2}, []bool{true, true, true, true, true}, -1,
			[]bool{true, true, true, true, true}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			angles := make([]float64, len(c.distances))
			for i := range angles {
				angles[i] = float64(i)
			}
			in := freshSweep(angles, c.distances, c.valid)
			before := append([]bool(nil), in.valid...)

			out := rejectSpikes(in, c.jump)
			for i := range c.want {
				if out.valid[i] != c.want[i] {
					t.Errorf("valid = %v, want %v", out.valid, c.want)
					break
				}
			}
			for i := range before {
				if in.valid[i] != before[i] {
					t.Fatalf("the sweep handed in was modified: %v, was %v", in.valid, before)
				}
			}
		})
	}
}

// The simulator has to produce a corridor a person would recognize, or it is
// not a stand-in for the sensor.
func TestSimulatorDrawsTheCorridor(t *testing.T) {
	cfg := GuetteurConfig{}
	applyDefaults(&cfg)
	src := newSimulatedSource(cfg)

	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	ch, err := src.sweeps(ctx)
	if err != nil {
		t.Fatalf("sweeps: %v", err)
	}

	select {
	case sw := <-ch:
		if len(sw.angles) != cfg.PointsPerSweep {
			t.Fatalf("got %d points, want %d", len(sw.angles), cfg.PointsPerSweep)
		}
		if len(sw.angles) != len(sw.distances) || len(sw.angles) != len(sw.valid) {
			t.Fatal("the simulator produced ragged arrays")
		}
		// The corridor is 3 m wide and the vehicle drives down the middle, so
		// straight out to the side must be about 1.5 m.
		for i, a := range sw.angles {
			if math.Abs(a-80) < 1.0 && sw.valid[i] {
				if d := sw.distances[i]; d < 1.0 || d > 2.5 {
					t.Errorf("at %.0f° the wall is %.2f m away; the corridor is 3 m wide", a, d)
				}
			}
		}
	case <-ctx.Done():
		t.Fatal("the simulator produced no sweep within two seconds")
	}
}
