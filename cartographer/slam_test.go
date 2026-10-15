package main

import (
	"math"
	"os"
	"path/filepath"
	"reflect"
	"testing"
	"time"

	"github.com/sdoque/mbaigo/forms"
)

// A corridor with a doorway, matching the one the guetteur's simulator draws.
// A plain corridor is deliberately not used: nothing in a straight, featureless
// hallway says how far along it you are, so a matcher tested only against one
// would look far better than it is.
var testWalls = []struct{ x1, y1, x2, y2 float64 }{
	{0, -1.5, 20, -1.5},
	{0, 1.5, 8, 1.5},
	{9.2, 1.5, 20, 1.5},
	{8, 1.5, 8, 2.7},
	{9.2, 1.5, 9.2, 2.7},
	{8, 2.7, 9.2, 2.7},
	{20, -1.5, 20, 1.5},
	{0, -1.5, 0, 1.5},
}

// sweepFrom ray-casts the corridor from a known pose, which is what makes the
// matcher testable: the answer is known before it is asked for.
func sweepFrom(p pose, points int, sector float64) ([]float64, []float64, []bool) {
	angles := make([]float64, 0, points)
	dists := make([]float64, 0, points)
	valid := make([]bool, 0, points)
	half := sector / 2
	for i := 0; i < points; i++ {
		a := -half + float64(i)*sector/float64(points-1)
		theta := p.theta + a*math.Pi/180
		dx, dy := math.Cos(theta), math.Sin(theta)
		best := math.Inf(1)
		for _, w := range testWalls {
			sx, sy := w.x2-w.x1, w.y2-w.y1
			den := dx*sy - dy*sx
			if math.Abs(den) < 1e-12 {
				continue
			}
			t := ((w.x1-p.x)*sy - (w.y1-p.y)*sx) / den
			u := ((w.x1-p.x)*dy - (w.y1-p.y)*dx) / den
			if t >= 0 && u >= 0 && u <= 1 && t < best {
				best = t
			}
		}
		angles = append(angles, a)
		if math.IsInf(best, 1) {
			dists = append(dists, 0)
			valid = append(valid, false)
		} else {
			dists = append(dists, best)
			valid = append(valid, true)
		}
	}
	return angles, dists, valid
}

// A straight corridor cannot say how far along it you are. Two parallel walls
// look identical from everywhere between them, and no amount of processing
// recovers what the geometry never encoded.
//
// So this test does not ask the matcher to succeed. It asks it to KNOW that it
// has not — because the failure that matters is not a wrong pose, it is a wrong
// pose reported confidently, which produces a corridor of the wrong length that
// looks entirely convincing.
func TestMatcherAdmitsWhenTheCorridorCannotFixIt(t *testing.T) {
	g := newGrid(40, 40, 0.05)
	g.matchRange = 10
	cfg := defaultSearch()

	truth := pose{x: 2, y: 0, theta: 0}
	a, d, v := sweepFrom(truth, 200, 200)
	g.integrate(truth, a, d, v)

	truth.x += 0.07
	a, d, v = sweepFrom(truth, 200, 200)
	m := g.match(pose{2.07, 0, 0}, a, d, v, cfg)

	if m.sharp {
		t.Error("the matcher called a featureless corridor a sharp fix; " +
			"it cannot know where along the corridor it is, and must say so")
	}
}

// Given odometry — which is what the wheel encoders and the waist encoder
// provide — the along-corridor motion is supplied and the matcher only has to
// keep the vehicle straight and centred. That is the regime this system is
// built for, and it must track.
func TestMatcherTracksWithOdometry(t *testing.T) {
	g := newGrid(40, 40, 0.05)
	g.matchRange = 10
	cfg := defaultSearch()

	truth := pose{x: 2, y: 0, theta: 0}
	a, d, v := sweepFrom(truth, 200, 200)
	g.integrate(truth, a, d, v)
	est := truth

	const step = 0.07
	worst := 0.0
	for i := 0; i < 30; i++ {
		truth = pose{truth.x + step, truth.y, truth.theta}
		a, d, v = sweepFrom(truth, 200, 200)

		// Odometry with a realistic 4% scale error and a slow heading bias,
		// which is what wheel encoders on a slipping machine actually give.
		prior := pose{est.x + step*1.04, est.y, est.theta + 0.05*math.Pi/180}
		m := g.match(prior, a, d, v, cfg)
		if m.sharp {
			est = m.at
		} else {
			est = prior // trust odometry where the view says nothing
		}
		g.integrate(est, a, d, v)

		if e := math.Hypot(est.x-truth.x, est.y-truth.y); e > worst {
			worst = e
		}
	}
	t.Logf("30 steps, %.2f m driven: worst error %.3f m, final (%.2f,%.2f) vs truth (%.2f,%.2f)",
		30*step, worst, est.x, est.y, truth.x, truth.y)

	// 4% of 2.1 m is 8 cm of odometry scale error, and with the corridor unable
	// to correct it that is the error to expect. What must not happen is the
	// estimate sitting still while the vehicle drives away.
	if worst > 0.20 {
		t.Errorf("worst tracking error %.3f m over %.2f m driven, want <= 0.20", worst, 30*step)
	}
	if math.Abs(est.x-truth.x) > 0.20 {
		t.Errorf("final along-corridor error %.3f m; the estimate did not follow the vehicle",
			math.Abs(est.x-truth.x))
	}
}

// A turn is constrained by the walls even in a corridor, so the matcher must
// get it right on its own.
func TestScanMatcherFollowsATurn(t *testing.T) {
	g := newGrid(40, 40, 0.05)
	g.matchRange = 10
	cfg := defaultSearch()

	truth := pose{x: 4, y: 0, theta: 0}
	a, d, v := sweepFrom(truth, 200, 200)
	g.integrate(truth, a, d, v)
	est := truth

	for i := 0; i < 10; i++ {
		truth = pose{truth.x, truth.y, truth.theta + 2*math.Pi/180}
		a, d, v = sweepFrom(truth, 200, 200)
		est = g.match(est, a, d, v, cfg).at
		g.integrate(est, a, d, v)
	}
	if e := math.Abs(wrapAngle(est.theta-truth.theta)) * 180 / math.Pi; e > 3 {
		t.Errorf("after turning 20°, heading error %.2f°, want <= 3", e)
	}
}

func TestIntegrateMarksFreeBetweenSensorAndWall(t *testing.T) {
	g := newGrid(20, 20, 0.1)
	at := pose{x: 0, y: 0, theta: 0}
	// One beam straight ahead, hitting something 3 m away.
	g.integrate(at, []float64{0}, []float64{3}, []bool{true})

	cx, cy, _ := g.cellOf(3, 0)
	if g.at(cx, cy) <= 0 {
		t.Errorf("the cell at the end of the beam is not occupied: %v", g.at(cx, cy))
	}
	mx, my, _ := g.cellOf(1.5, 0)
	if g.at(mx, my) >= 0 {
		t.Errorf("the cell halfway along the beam is not free: %v", g.at(mx, my))
	}
	// Beyond the obstacle nothing was observed, and must stay unknown.
	bx, by, _ := g.cellOf(4.5, 0)
	if g.at(bx, by) != 0 {
		t.Errorf("the cell behind the obstacle was written to: %v", g.at(bx, by))
	}
}

// Baseline reproduction of the fixed-grid limitation. Once automatic growth is
// implemented, replace the small-grid expectation with retained observations.
func TestFixedGridDropsBeamWithEndpointOutsideMap(t *testing.T) {
	for _, tc := range []struct {
		name         string
		width        float64
		wantRecorded bool
	}{
		{"endpoint outside small map", 4, false},
		{"same endpoint inside larger map", 8, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			g := newGrid(tc.width, tc.width, 0.1)
			// The scanner is at the origin; the obstacle is 3 m ahead.
			g.integrate(pose{}, []float64{0}, []float64{3}, []bool{true})

			cx, cy, inside := g.cellOf(1, 0)
			if !inside {
				t.Fatal("the point along the beam must be inside both maps")
			}
			if gotFree := g.at(cx, cy) < 0; gotFree != tc.wantRecorded {
				t.Errorf("free space recorded = %v, want %v", gotFree, tc.wantRecorded)
			}
			ex, ey, endpointInside := g.cellOf(3, 0)
			if endpointInside != tc.wantRecorded {
				t.Errorf("endpoint inside map = %v, want %v", endpointInside, tc.wantRecorded)
			}
			if tc.wantRecorded && g.at(ex, ey) <= 0 {
				t.Error("the obstacle should be recorded in the larger map")
			}
			if !tc.wantRecorded {
				for i, v := range g.cells {
					if v != 0 {
						t.Fatalf("cell %d changed although the whole beam is currently discarded", i)
					}
				}
			}
		})
	}
}

// An invalid point must contribute nothing at all. Treating it as free space to
// maximum range is the failure this whole design is built to avoid.
func TestIntegrateIgnoresInvalidReturns(t *testing.T) {
	g := newGrid(20, 20, 0.1)
	g.integrate(pose{}, []float64{0}, []float64{0}, []bool{false})
	for i, v := range g.cells {
		if v != 0 {
			t.Fatalf("cell %d was written from an invalid return: %v", i, v)
		}
	}
}

func TestFormKeepsUnknownDistinctFromFree(t *testing.T) {
	g := newGrid(2, 2, 1)
	g.add(0, 0, logFree)
	g.add(1, 1, logOcc)

	f := g.form()
	if err := f.Check(); err != nil {
		t.Fatalf("form does not check out: %v", err)
	}
	if f.Cells[0] != forms.MapFree {
		t.Errorf("observed-free cell is %d, want %d", f.Cells[0], forms.MapFree)
	}
	if f.Cells[3] != forms.MapOccupied {
		t.Errorf("occupied cell is %d, want %d", f.Cells[3], forms.MapOccupied)
	}
	if f.Cells[1] != forms.MapUnknown {
		t.Errorf("untouched cell is %d, want %d (unknown)", f.Cells[1], forms.MapUnknown)
	}
}

func TestWritePGM(t *testing.T) {
	g := newGrid(1, 1, 0.5) // 2x2
	g.add(0, 0, logOcc)
	path := filepath.Join(t.TempDir(), "map.pgm")
	if err := g.writePGM(path); err != nil {
		t.Fatalf("writePGM: %v", err)
	}
	b, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("read back: %v", err)
	}
	if want := "P5\n2 2\n255\n"; string(b[:len(want)]) != want {
		t.Errorf("header is %q, want %q", string(b[:len(want)]), want)
	}
	if len(b) != len("P5\n2 2\n255\n")+4 {
		t.Errorf("file holds %d bytes, want header + 4 pixels", len(b))
	}
}

func TestLineExcludesTheEndpoint(t *testing.T) {
	cells := line(0, 0, 3, 0)
	for _, c := range cells {
		if c[0] == 3 && c[1] == 0 {
			t.Fatal("the endpoint is in the free-space list; it is the obstacle")
		}
	}
	if len(cells) != 3 {
		t.Errorf("got %d intermediate cells, want 3", len(cells))
	}
}

func TestGridGrowthPreservesWorldCoordinates(t *testing.T) {
	for _, tc := range []struct {
		name                     string
		left, right, bottom, top int
	}{
		{"left", 2, 0, 0, 0},
		{"right", 0, 2, 0, 0},
		{"bottom", 0, 0, 2, 0},
		{"top", 0, 0, 0, 2},
		{"bottom left corner", 2, 0, 3, 0},
		{"top right corner", 0, 2, 0, 3},
		{"all sides", 1, 2, 3, 4},
	} {
		t.Run(tc.name, func(t *testing.T) {
			g := newGrid(1, 1, 0.25)
			for i := range g.cells {
				g.cells[i] = float32(i+1) / 10
			}
			g.cells[0] = logFree
			original := append([]float32(nil), g.cells...)
			originX, originY := g.originX, g.originY
			g.matchRange = 10

			// Repeat growth to catch row-copy errors after the first resize.
			for pass := 1; pass <= 2; pass++ {
				if err := g.grow(tc.left, tc.right, tc.bottom, tc.top, 10000); err != nil {
					t.Fatal(err)
				}
				if g.width != 4+pass*(tc.left+tc.right) || g.height != 4+pass*(tc.bottom+tc.top) {
					t.Fatalf("unexpected dimensions: %d x %d", g.width, g.height)
				}
				if g.res != 0.25 || g.matchRange != 10 || !g.dirty || len(g.field) != len(g.cells) {
					t.Fatal("resolution/range changed or matching field not invalidated")
				}
				oldCells := make(map[int]bool)
				for i, want := range original {
					// Sample cell centres to avoid floating-point boundary ambiguity.
					x := originX + (float64(i%4)+0.5)*g.res
					y := originY + (float64(i/4)+0.5)*g.res
					cx, cy, inside := g.cellOf(x, y)
					if !inside || g.at(cx, cy) != want {
						t.Fatalf("evidence moved at world coordinate (%v, %v)", x, y)
					}
					oldCells[cy*g.width+cx] = true
				}
				for i, v := range g.cells {
					if !oldCells[i] && v != 0 {
						t.Fatalf("new cell %d is not unknown", i)
					}
				}
				g.rebuildField()
				if g.dirty {
					t.Fatal("matching field did not rebuild")
				}
			}
		})
	}
}

func TestGridGrowthLimitsLeaveMapUnchanged(t *testing.T) {
	maxInt := int(^uint(0) >> 1)
	for _, tc := range []struct {
		name                            string
		left, right, bottom, top, limit int
	}{
		{"negative addition", -1, 0, 0, 0, 100},
		{"no cell budget", 1, 0, 0, 0, 0},
		{"area exceeds budget", 1, 0, 0, 0, 4},
		{"width overflow", maxInt, 1, 0, 0, maxInt},
		{"height overflow", 0, 0, maxInt, 1, maxInt},
		{"area overflow", maxInt / 2, 0, 0, 0, maxInt},
	} {
		t.Run(tc.name, func(t *testing.T) {
			g := newGrid(2, 2, 1)
			g.cells[0] = logOcc
			before := *g
			before.cells = append([]float32(nil), g.cells...)
			before.field = append([]float32(nil), g.field...)
			if err := g.grow(tc.left, tc.right, tc.bottom, tc.top, tc.limit); err == nil {
				t.Fatal("expected growth to be refused")
			}
			if !reflect.DeepEqual(before, *g) {
				t.Fatal("refused growth changed the map")
			}
		})
	}
}

func TestGridGrowthAtLimitAndNoOp(t *testing.T) {
	g := newGrid(2, 2, 1)
	if err := g.grow(1, 0, 0, 0, 6); err != nil {
		t.Fatalf("growth exactly at limit was refused: %v", err)
	}
	g.rebuildField()
	before := *g
	if err := g.grow(0, 0, 0, 0, 6); err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(before, *g) {
		t.Fatal("zero growth changed the map")
	}
}

func TestScanExpansionEdgesAndCorners(t *testing.T) {
	for _, xy := range [][2]float64{{3, 0}, {-3, 0}, {0, 3}, {0, -3}, {3, 3}, {-3, 3}, {3, -3}, {-3, -3}, {2, 0}} {
		g := newGrid(4, 4, 1)
		g.cells[0] = logOcc
		cfg := ExpansionConfig{Enabled: true, BlockMetres: 2, MaxCells: 100, MaxRangeMetres: 10}
		a := []float64{math.Atan2(xy[1], xy[0]) * 180 / math.Pi}
		d := []float64{math.Hypot(xy[0], xy[1])}
		if err := g.expandForScan(pose{}, a, d, []bool{true}, cfg); err != nil {
			t.Fatal(err)
		}
		cx, cy, ok := g.cellOf(-1.5, -1.5)
		if !ok || g.at(cx, cy) != logOcc {
			t.Fatal("old evidence moved")
		}
		g.integrate(pose{}, a, d, []bool{true})
		// Use the same projection as the mapper, including its rounding.
		x, y := project(pose{}, a[0], d[0])
		cx, cy, ok = g.cellOf(x, y)
		if !ok || g.at(cx, cy) <= 0 {
			t.Fatalf("endpoint (%v,%v) lost", x, y)
		}
		w, h := g.width, g.height
		if err := g.expandForScan(pose{}, a, d, []bool{true}, cfg); err != nil {
			t.Fatal(err)
		}
		if g.width != w || g.height != h {
			t.Fatal("identical scan grew map again")
		}
	}
}

func TestScanExpansionMarginAndFiltering(t *testing.T) {
	cfg := ExpansionConfig{Enabled: true, MarginMetres: 1, BlockMetres: 2, MaxCells: 100, MaxRangeMetres: 10}
	g := newGrid(4, 4, 1)
	if err := g.expandForScan(pose{}, []float64{0}, []float64{1}, []bool{true}, cfg); err != nil {
		t.Fatal(err)
	}
	if g.width != 6 || g.height != 4 {
		t.Fatalf("margin did not trigger right-side growth: %dx%d", g.width, g.height)
	}
	for _, tc := range []struct {
		angle, distance float64
		valid           bool
	}{
		{0, 3, false}, {0, 0, true}, {0, -3, true}, {0, 11, true},
		{0, math.NaN(), true}, {0, math.Inf(1), true}, {math.NaN(), 3, true}, {math.Inf(1), 3, true},
	} {
		g = newGrid(4, 4, 1)
		if err := g.expandForScan(pose{}, []float64{tc.angle}, []float64{tc.distance}, []bool{tc.valid}, cfg); err != nil {
			t.Fatal(err)
		}
		if g.width != 4 || g.height != 4 {
			t.Fatal("invalid or out-of-range return triggered growth")
		}
	}
}

func TestScanExpansionDisabledOrRefused(t *testing.T) {
	for _, cfg := range []ExpansionConfig{
		{},
		{Enabled: true, BlockMetres: 2, MaxCells: 16, MaxRangeMetres: 10},
		{Enabled: true, BlockMetres: 2, MaxCells: 100, MaxRangeMetres: 10, MarginMetres: math.NaN()},
	} {
		g := newGrid(4, 4, 1)
		err := g.expandForScan(pose{}, []float64{0}, []float64{3}, []bool{true}, cfg)
		if cfg.Enabled && err == nil {
			t.Fatal("expected refusal")
		}
		if !cfg.Enabled && err != nil {
			t.Fatal(err)
		}
		if g.width != 4 || g.height != 4 || g.originX != -2 || g.originY != -2 {
			t.Fatal("map changed on refusal/disabled expansion")
		}
	}
}

func TestConsumeExpandsBeforeIntegrating(t *testing.T) {
	for _, enabled := range []bool{false, true} {
		off := false
		cfg := CartographerConfig{WidthMetres: 4, HeightMetres: 4, Resolution: 1,
			Odometry:  OdometryConfig{Enabled: &off},
			Expansion: ExpansionConfig{Enabled: enabled, BlockMetres: 2, MaxCells: 100, MaxRangeMetres: 10}}
		applyDefaults(&cfg)
		tr := &Traits{cfg: cfg, g: newGrid(4, 4, 1), frame: "test-frame"}
		tr.consume(&forms.ScanA_v1a{Angles: []float64{0}, Distances: []float64{3}, Valid: []bool{true}, Timestamp: time.Now()})
		cx, cy, inside := tr.g.cellOf(3, 0)
		if inside != enabled {
			t.Fatalf("endpoint inside = %v, expansion enabled = %v", inside, enabled)
		}
		if enabled && tr.g.at(cx, cy) <= 0 {
			t.Fatal("new observation not integrated")
		}
		if tr.frame != "test-frame" || tr.at != (pose{}) {
			t.Fatal("growth changed frame or vehicle pose")
		}
	}
}

// Known poses isolate expansion from localisation: this checks the corridor
// geometry used by the scan simulator, not live Guetteur/Arrowhead integration.
func TestExpansionAlongSimulatedCorridor(t *testing.T) {
	g := newGrid(4, 4, 0.1)
	cfg := ExpansionConfig{Enabled: true, MarginMetres: 0.5, BlockMetres: 4,
		MaxCells: 100000, MaxRangeMetres: 12}
	growths, sweeps := 0, 0
	for _, returning := range []bool{false, true} {
		for i := 0; i <= 32; i++ {
			x, heading := 2+float64(i)*0.5, 0.0
			if returning {
				x, heading = 18-float64(i)*0.5, math.Pi
			}
			angles, distances, valid := sweepFrom(pose{x: x, theta: heading}, 200, 200)
			// Map origin is the first scan position (corridor x=2).
			at := pose{x: x - 2, theta: heading}
			w, h := g.width, g.height
			if err := g.expandForScan(at, angles, distances, valid, cfg); err != nil {
				t.Fatal(err)
			}
			if g.width != w || g.height != h {
				growths++
			}
			if len(g.cells) > cfg.MaxCells {
				t.Fatal("cell budget exceeded")
			}
			for j, d := range distances {
				if !valid[j] || d <= 0 || d > cfg.MaxRangeMetres {
					continue
				}
				px, py := project(at, angles[j], d)
				if _, _, ok := g.cellOf(px, py); !ok {
					t.Fatalf("accepted return outside expanded map: (%v, %v)", px, py)
				}
			}
			g.integrate(at, angles, distances, valid)
			sweeps++
		}
	}
	if growths == 0 {
		t.Fatal("corridor route never expanded the map")
	}
	// The corridor ends at x=20, which is x=18 in the map frame.
	cx, cy, inside := g.cellOf(18, 0)
	if !inside || g.at(cx, cy) <= 0 {
		t.Fatal("far wall was not retained beyond the original map")
	}
	g.rebuildField()
	f := g.form()
	if err := f.Check(); err != nil {
		t.Fatal(err)
	}
	if f.Width != g.width || f.Height != g.height || f.OriginX != g.originX || f.OriginY != g.originY {
		t.Fatal("map output does not reflect expanded bounds")
	}
	t.Logf("%d simulated sweeps; %d growth events; 4x4 m initial map -> %.1fx%.1f m; %d cells",
		sweeps, growths, float64(g.width)*g.res, float64(g.height)*g.res, len(g.cells))
}
