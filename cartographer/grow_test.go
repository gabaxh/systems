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
	"math"
	"testing"
)

// Growing a grid moves nothing in the world: every cell drawn before is the
// same cell of the world after, and the extent moves with the indices.
func TestGrowingKeepsTheWorldWhereItWas(t *testing.T) {
	g := newGrid(30, 30, 0.05)
	// Off the cell boundaries, where rounding may tip a point either way.
	marks := [][2]float64{{0.01, 0.01}, {-14.89, -14.89}, {14.89, 14.89}, {3.21, -7.71}}
	for k, p := range marks {
		cx, cy, ok := g.cellOf(p[0], p[1])
		if !ok {
			t.Fatalf("mark %v is not in the grid", p)
		}
		g.add(cx, cy, float32(k+1))
	}
	ex0, ey0, ex1, ey1 := g.ex0, g.ey0, g.ex1, g.ey1

	g.growBy(200, 0, 40, 100)

	if g.width != 600+200 || g.height != 600+40+100 {
		t.Fatalf("grown to %d × %d cells", g.width, g.height)
	}
	if x0, y0, x1, y1 := g.bounds(); math.Abs(x0+25) > 1e-9 || math.Abs(x1-15) > 1e-9 || math.Abs(y0+17) > 1e-9 || math.Abs(y1-20) > 1e-9 {
		t.Errorf("grown bounds x %g..%g y %g..%g, want x -25..15 y -17..20", x0, x1, y0, y1)
	}
	for k, p := range marks {
		cx, cy, ok := g.cellOf(p[0], p[1])
		if !ok || g.at(cx, cy) != float32(k+1) {
			t.Errorf("mark %v reads %g after growing, want %d", p, g.at(cx, cy), k+1)
		}
	}
	if g.ex0 != ex0+200 || g.ex1 != ex1+200 || g.ey0 != ey0+40 || g.ey1 != ey1+40 {
		t.Errorf("extent %d..%d, %d..%d after growing; want %d..%d, %d..%d",
			g.ex0, g.ex1, g.ey0, g.ey1, ex0+200, ex1+200, ey0+40, ey1+40)
	}
}

// The likelihood field moves with the cells, and rebuilding it over the
// extent after growing and drawing more is still the field over the whole
// grid.
func TestTheFieldSurvivesGrowing(t *testing.T) {
	g := bigRing(t, 40)
	g.rebuildField()
	g.growBy(60, 20, 0, 80)
	g.dirty = true
	g.rebuildField()
	want := fullField(g)
	for i := range want {
		if g.field[i] != want[i] {
			t.Fatalf("cell %d: %g over the extent, %g over the grid", i, g.field[i], want[i])
		}
	}
}

// An empty grid grown is still empty.
func TestGrowingAnEmptyGridLeavesItEmpty(t *testing.T) {
	g := newGrid(10, 10, 0.05)
	g.growBy(10, 10, 10, 10)
	if g.ex1 >= g.ex0 || g.ey1 >= g.ey0 {
		t.Errorf("an empty grid grown has extent %d..%d, %d..%d", g.ex0, g.ex1, g.ey0, g.ey1)
	}
}

func growingTraits() *Traits {
	cfg := CartographerConfig{}
	applyDefaults(&cfg)
	return &Traits{cfg: cfg, g: newGrid(cfg.WidthMetres, cfg.HeightMetres, cfg.Resolution)}
}

func sameMetres(a, b float64) bool { return math.Abs(a-b) < 1e-9 }

// The map starts 30 m square and grows 10 m toward an edge the vehicle comes
// within 5 m of, and only that edge.
func TestTheMapGrowsTheWayTheVehicleDrives(t *testing.T) {
	cases := []struct {
		name           string
		from, to       pose
		x0, y0, x1, y1 float64
	}{
		{"in the middle", pose{0, 0, 0}, pose{0.2, 0, 0}, -15, -15, 15, 15},
		{"east, still 5 m from the edge", pose{9.8, 0, 0}, pose{10, 0, 0}, -15, -15, 15, 15},
		{"east, inside 5 m", pose{9.9, 0, 0}, pose{10.1, 0, 0}, -15, -15, 25, 15},
		{"west", pose{-10, 3, 0}, pose{-10.2, 3, 0}, -25, -15, 15, 15},
		{"north", pose{2, 11, 0}, pose{2, 11.2, 0}, -15, -15, 15, 25},
		{"south", pose{2, -11, 0}, pose{2, -11.2, 0}, -15, -25, 15, 15},
		{"diagonally into the north-east corner", pose{11, 11, 0}, pose{11.1, 11.1, 0}, -15, -15, 25, 25},
		{"near the east edge but driving away", pose{12.2, 0, 0}, pose{12, 0, 0}, -15, -15, 15, 15},
		{"along the east edge", pose{12, -1, 0}, pose{12, 1, 0}, -15, -15, 15, 15},
		{"standing at the east edge", pose{12, 0, 0}, pose{12, 0, 0}, -15, -15, 15, 15},
		{"outside the grid, driving away from it", pose{19, 0, 0}, pose{18, 0, 0}, -15, -15, 25, 15},
		{"well outside", pose{30, 0, 0}, pose{30.2, 0, 0}, -15, -15, 45, 15},
	}
	for _, c := range cases {
		tr := growingTraits()
		tr.growToward(tr.g, c.from, c.to)
		x0, y0, x1, y1 := tr.g.bounds()
		if !sameMetres(x0, c.x0) || !sameMetres(y0, c.y0) || !sameMetres(x1, c.x1) || !sameMetres(y1, c.y1) {
			t.Errorf("%s: x %g..%g y %g..%g, want x %g..%g y %g..%g", c.name, x0, x1, y0, y1, c.x0, c.x1, c.y0, c.y1)
		}
	}
}

// Driving on past each new edge grows the map again, 10 m at a time.
func TestTheMapKeepsGrowingAheadOfTheVehicle(t *testing.T) {
	tr := growingTraits()
	from := pose{}
	for x := 0.1; x <= 35; x += 0.1 {
		to := pose{x, 0, 0}
		tr.growToward(tr.g, from, to)
		if _, _, x1, _ := tr.g.bounds(); x1-to.x < tr.cfg.GrowMarginMetres-1e-6 {
			t.Fatalf("at x=%.1f the east edge is at %g", x, x1)
		}
		from = to
	}
	x0, y0, x1, y1 := tr.g.bounds()
	if !sameMetres(x0, -15) || !sameMetres(x1, 45) || !sameMetres(y0, -15) || !sameMetres(y1, 15) {
		t.Errorf("after 35 m east: x %g..%g y %g..%g, want x -15..45 y -15..15", x0, x1, y0, y1)
	}
}

// A negative step keeps the map at its starting size.
func TestANegativeStepKeepsTheMapItsSize(t *testing.T) {
	cfg := CartographerConfig{GrowStepMetres: -1}
	applyDefaults(&cfg)
	tr := &Traits{cfg: cfg, g: newGrid(cfg.WidthMetres, cfg.HeightMetres, cfg.Resolution)}
	tr.growToward(tr.g, pose{12, 0, 0}, pose{14, 0, 0})
	if tr.g.width != 600 || tr.g.height != 600 {
		t.Errorf("grew to %d × %d cells with growth off", tr.g.width, tr.g.height)
	}
}

// A redraw's grid reaches every keyframe with the margin to spare.
func TestARedrawCoversItsKeyframes(t *testing.T) {
	tr := growingTraits()
	g := emptyLike(tr.g)
	tr.coverKeyframes(g, []keyframe{{pose: pose{0, 0, 0}}, {pose: pose{12, 3, 0}}, {pose: pose{-3, -27, 0}}})
	x0, y0, x1, y1 := g.bounds()
	if !sameMetres(x0, -15) || !sameMetres(x1, 25) || !sameMetres(y0, -35) || !sameMetres(y1, 15) {
		t.Errorf("x %g..%g y %g..%g, want x -15..25 y -35..15", x0, x1, y0, y1)
	}
}

// A beam whose return lies past the edge of the grid still clears the cells
// it crosses, out to the edge, and marks none occupied. It used to be dropped
// whole, and a vehicle driving toward an edge left a grey wedge ahead of it.
func TestABeamPastTheEdgeClearsUpToIt(t *testing.T) {
	for _, angle := range []float64{0, 30, -135, 90} {
		g := newGrid(10, 10, 0.1) // −5 … 5 m
		p := pose{0.05, 0.05, 0}
		g.integrate(p, []float64{angle}, []float64{20}, []bool{true})
		occupied := 0
		for _, v := range g.cells {
			if v > 0 {
				occupied++
			}
		}
		if occupied > 0 {
			t.Errorf("%.0f°: %d cells marked occupied by a return off the grid", angle, occupied)
		}
		// Every cell of the line out to the last cell inside the grid, that
		// one included, is clear, and the last one is at the edge.
		sx, sy, _ := g.cellOf(p.x, p.y)
		last := [2]int{sx, sy}
		for d := 0.0; ; d += 0.01 {
			cx, cy, ok := g.cellOf(project(p, angle, d))
			if !ok {
				break
			}
			last = [2]int{cx, cy}
		}
		if last[0] != 0 && last[1] != 0 && last[0] != g.width-1 && last[1] != g.height-1 {
			t.Fatalf("%.0f°: the beam's last cell %v is not at the edge", angle, last)
		}
		for _, c := range append(line(sx, sy, last[0], last[1]), last) {
			if v := g.at(c[0], c[1]); v >= 0 {
				t.Errorf("%.0f°: cell %v on the beam is %g, not clear", angle, c, v)
				break
			}
		}
	}
	// A submap drops such a beam whole (see grid.dropPastEdge).
	g := newGrid(10, 10, 0.1)
	g.dropPastEdge = true
	g.integrate(pose{0.05, 0.05, 0}, []float64{0}, []float64{20}, []bool{true})
	for _, v := range g.cells {
		if v != 0 {
			t.Fatal("a submap drew a beam whose return is off it")
		}
	}
}

// Past freeRange the cleared part counts farWeight, as it does for a beam
// whose return is on the grid.
func TestABeamPastTheEdgeIsWeighedLikeAnyOther(t *testing.T) {
	g := newGrid(10, 10, 0.1)
	g.freeRange = 2
	p := pose{0.05, 0.05, 0}
	g.integrate(p, []float64{0}, []float64{20}, []bool{true})
	near, _, _ := g.cellOf(1, 0.05)
	far, row, _ := g.cellOf(4, 0.05)
	if v := g.at(near, row); math.Abs(float64(v-logFree)) > 1e-6 {
		t.Errorf("1 m along: %g, want %g", v, logFree)
	}
	if v := g.at(far, row); math.Abs(float64(v-farWeight*logFree)) > 1e-6 {
		t.Errorf("4 m along: %g, want %g", v, farWeight*logFree)
	}
}

// The map grows to fit the returns within GrowFitMetres, whichever way the
// vehicle is going; a return further out does not grow it. The room of 20
// November was seen from 7.5 m inside the edge, beyond the margin.
func TestTheMapGrowsToFitWhatTheLidarSees(t *testing.T) {
	tr := growingTraits()
	p := pose{-7.5, 0, math.Pi / 2} // facing north, 7.5 m from the west edge
	tr.growToFit(tr.g, p,
		[]float64{90, 0, -90, 45}, // west, north, east, north-west, in the sensor's frame
		[]float64{11, 3, 30, 2},   // the east one is past GrowFitMetres
		[]bool{true, true, true, true})
	x0, y0, x1, y1 := tr.g.bounds()
	if !sameMetres(x0, -25) || !sameMetres(x1, 15) || !sameMetres(y0, -15) || !sameMetres(y1, 15) {
		t.Errorf("x %g..%g y %g..%g, want x -25..15 y -15..15", x0, x1, y0, y1)
	}
	// Off, it does nothing.
	tr = growingTraits()
	tr.cfg.GrowFitMetres = -1
	tr.growToFit(tr.g, p, []float64{90}, []float64{11}, []bool{true})
	if x0, _, _, _ := tr.g.bounds(); !sameMetres(x0, -15) {
		t.Errorf("with growFitMetres off the map grew to x0 = %g", x0)
	}
}

// A redraw's grid fits the keyframes' returns too, where the closure has put
// the keyframes.
func TestARedrawFitsItsKeyframesReturns(t *testing.T) {
	tr := growingTraits()
	g := emptyLike(tr.g)
	west := storedSweep{angles: []float32{180}, distances: []float32{11}, valid: []bool{true}}
	tr.coverKeyframes(g, []keyframe{{pose: pose{-7.5, 0, 0}, sweep: west}})
	if x0, _, _, _ := g.bounds(); !sameMetres(x0, -25) {
		t.Errorf("the redraw's grid starts at x0 = %g, want -25", x0)
	}
}
