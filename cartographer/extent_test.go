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

// bigRing is the ring corridor of loops_test.go drawn into a grid w metres
// across at 5 cm. At 250 m, the size of the grid on the vehicle on
// 5 November, that is 25 million cells, of which the drive covers a few
// hundred thousand.
func bigRing(t testing.TB, w float64) *grid {
	ringWorld(t)
	g := newGrid(w, w, 0.05)
	g.matchRange = 10
	for _, p := range route(pose{1.5, 1.5, 0}, leg{0, 17}, leg{90, 7}, leg{90, 17}, leg{90, 7}) {
		a, d, v := sweepFrom(p, 360, 359)
		g.integrate(p, a, d, v)
	}
	return g
}

// How long the likelihood field takes to rebuild on the vehicle's grid. The
// front end rebuilds it after every sweep it draws, and after every redraw.
func BenchmarkRebuildFieldOn250Metres(b *testing.B) {
	g := bigRing(b, 250)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		g.dirty = true
		g.rebuildField()
	}
}

// fullField is the likelihood field as rebuildField computed it before it
// learned about the extent: over every cell of the grid. Kept as the
// reference the faster one must agree with exactly.
func fullField(g *grid) []float32 {
	const big = float32(1e9)
	const diag = float32(1.41421356)
	d := make([]float32, len(g.cells))
	for i, v := range g.cells {
		if v > 0 {
			d[i] = 0
		} else {
			d[i] = big
		}
	}
	at := func(x, y int) float32 {
		if x < 0 || y < 0 || x >= g.width || y >= g.height {
			return big
		}
		return d[y*g.width+x]
	}
	relax := func(x, y int, candidates ...float32) {
		i := y*g.width + x
		for _, c := range candidates {
			if c < d[i] {
				d[i] = c
			}
		}
	}
	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			relax(x, y, at(x-1, y)+1, at(x, y-1)+1, at(x-1, y-1)+diag, at(x+1, y-1)+diag)
		}
	}
	for y := g.height - 1; y >= 0; y-- {
		for x := g.width - 1; x >= 0; x-- {
			relax(x, y, at(x+1, y)+1, at(x, y+1)+1, at(x+1, y+1)+diag, at(x-1, y+1)+diag)
		}
	}
	out := make([]float32, len(d))
	cutoffCells := float32(fieldCutoff * fieldSigma / g.res)
	for i, dist := range d {
		if dist > cutoffCells {
			continue
		}
		m := float64(dist) * g.res
		out[i] = float32(math.Exp(-m * m / (2 * fieldSigma * fieldSigma)))
	}
	return out
}

// The field rebuilt over the extent is the field rebuilt over the whole grid,
// cell for cell: nothing outside the extent was ever drawn, so nothing there
// is near a wall.
func TestTheFieldOverTheExtentIsTheWholeField(t *testing.T) {
	g := bigRing(t, 40)
	g.dirty = true
	g.rebuildField()
	want := fullField(g)
	for i := range want {
		if g.field[i] != want[i] {
			t.Fatalf("cell %d: %g over the extent, %g over the grid", i, g.field[i], want[i])
		}
	}
	// And once more after drawing somewhere new, which grows the extent: the
	// old field must not linger where the extent used to end.
	p := pose{-12, -12, 0.3}
	a, d, v := sweepFrom(p, 360, 359)
	g.integrate(p, a, d, v)
	g.rebuildField()
	want = fullField(g)
	for i := range want {
		if g.field[i] != want[i] {
			t.Fatalf("after growing, cell %d: %g over the extent, %g over the grid", i, g.field[i], want[i])
		}
	}
}

// An empty map has no extent and no field.
func TestAnEmptyGridHasNoExtent(t *testing.T) {
	g := newGrid(10, 10, 0.05)
	if g.ex1 >= g.ex0 {
		t.Errorf("an empty grid has extent %d..%d", g.ex0, g.ex1)
	}
	g.dirty = true
	g.rebuildField()
	for i, v := range g.field {
		if v != 0 {
			t.Fatalf("field %g at cell %d of an empty map", v, i)
		}
	}
}

// How long a redraw takes on the vehicle's grid: a fresh grid, every kept
// sweep of the ring drawn into it, and the field the front end then needs.
func BenchmarkRedrawOn250Metres(b *testing.B) {
	ringWorld(b)
	m := testMemory()
	remember(m, route(pose{1.5, 1.5, 0}, leg{0, 17}, leg{90, 7}, leg{90, 17}, leg{90, 7}, leg{90, 4}),
		func(p pose) pose { return p })
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		g := newGrid(250, 250, 0.05)
		m.drawAll(g)
		g.rebuildField()
	}
}

// ray draws exactly the cells line lists, in every direction.
func TestARayIsTheLine(t *testing.T) {
	ends := [][4]int{{5, 5, 40, 12}, {40, 12, 5, 5}, {20, 20, 20, 3}, {3, 30, 33, 0}, {10, 10, 10, 10}, {0, 0, 49, 49}, {49, 0, 0, 7}}
	for _, e := range ends {
		g := newGrid(2.5, 2.5, 0.05) // 50 × 50 cells
		g.ray(e[0], e[1], e[2], e[3], 1)
		want := map[[2]int]bool{}
		for _, c := range line(e[0], e[1], e[2], e[3]) {
			want[c] = true
		}
		for cy := 0; cy < g.height; cy++ {
			for cx := 0; cx < g.width; cx++ {
				if got := g.at(cx, cy) != 0; got != want[[2]int{cx, cy}] {
					t.Fatalf("ray %v: cell (%d, %d) drawn %v, line says %v", e, cx, cy, got, want[[2]int{cx, cy}])
				}
			}
		}
	}
}
