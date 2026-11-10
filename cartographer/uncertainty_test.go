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

// plainCorridor replaces the test walls, for the length of one test, with a
// straight corridor 3 m wide and 100 m long, running at the given angle through
// the origin: nothing in it says how far along it the vehicle is.
func plainCorridor(t *testing.T, angleDeg float64) {
	t.Helper()
	saved := testWalls
	t.Cleanup(func() { testWalls = saved })
	a := angleDeg * math.Pi / 180
	ux, uy := math.Cos(a), math.Sin(a)
	nx, ny := -uy, ux
	testWalls = testWalls[:0:0]
	for _, side := range []float64{1.5, -1.5} {
		testWalls = append(testWalls, struct{ x1, y1, x2, y2 float64 }{
			-50*ux + side*nx, -50*uy + side*ny, 50*ux + side*nx, 50*uy + side*ny})
	}
}

// quadratic is vᵀ·m·v: how much information m carries in direction v.
func quadratic(m mat3, v [3]float64) float64 {
	w := m.mulVec(v)
	return v[0]*w[0] + v[1]*w[1] + v[2]*w[2]
}

// A loop found in a corridor is worth having for its heading and its position
// across the corridor, and worth nothing along it. The information must say
// exactly that, whichever way the corridor runs in the map — otherwise a loop
// closure would drag the map along the corridor to wherever the sampling
// pattern happened to score best.
func TestACorridorSaysNothingAlongIt(t *testing.T) {
	plainCorridor(t, 30)
	g := newGrid(40, 40, 0.05)
	g.matchRange = 10
	a := 30 * math.Pi / 180
	ux, uy := math.Cos(a), math.Sin(a)
	nx, ny := -uy, ux

	// The map holds the corridor well beyond the sweep's reach either way, as
	// it does once the vehicle has driven down it: no wall ends in view.
	for s := -12.0; s <= 12; s += 0.5 {
		p := pose{s * ux, s * uy, a}
		an, d, v := sweepFrom(p, 200, 200)
		g.integrate(p, an, d, v)
	}
	truth := pose{0, 0, a}
	an, d, v := sweepFrom(truth, 200, 200)
	prior := pose{truth.x + 0.2*ux + 0.1*nx, truth.y + 0.2*uy + 0.1*ny, truth.theta + 2*math.Pi/180}

	_, info, seen := g.matchSampled(prior, an, d, v, defaultSearch())
	if seen != 2 {
		t.Fatalf("a plain corridor fixed %d directions, want 2 (across it, and the heading)", seen)
	}
	along := quadratic(info, [3]float64{ux, uy, 0})
	across := quadratic(info, [3]float64{nx, ny, 0})
	heading := quadratic(info, [3]float64{0, 0, 1})
	t.Logf("information along %.3g, across %.3g /m², heading %.3g /rad²", along, across, heading)
	if along > 1e-3*across {
		t.Errorf("information %.3g along the corridor against %.3g across it", along, across)
	}
	if across <= 0 || heading <= 0 {
		t.Errorf("the walls fix the position across the corridor and the heading; got %.3g and %.3g", across, heading)
	}
}

// Where the walls do pin the vehicle down — near the end of the test corridor,
// with the end wall filling a third of the view — all three directions are
// seen.
//
// Further back they are not, and that is the coarse pass read loosely doing
// its job: four metres from the end the end wall is a few percent of the
// returns, sliding the sweep along the corridor costs only those few percent,
// and that is not enough to count as seeing the position along it.
//
// The grid is 50 m and not the 40 m of the other tests: the end wall is at
// x = 20, which in a 40 m grid centred on the origin is just off its edge and
// never drawn.
func TestTheEndOfTheCorridorFixesAllThree(t *testing.T) {
	g := newGrid(50, 50, 0.05)
	g.matchRange = 10
	for x := 15.0; x <= 19.0; x += 0.5 {
		p := pose{x, 0, 0}
		an, d, v := sweepFrom(p, 360, 359)
		g.integrate(p, an, d, v)
	}
	truth := pose{18.8, 0.2, 0}
	an, d, v := sweepFrom(truth, 360, 359)
	_, info, seen := g.matchSampled(pose{18.7, 0.15, 1 * math.Pi / 180}, an, d, v, defaultSearch())
	if seen != 3 {
		t.Fatalf("the end of the corridor fixed %d directions, want 3", seen)
	}
	for _, dir := range [][3]float64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} {
		if q := quadratic(info, dir); q <= 0 {
			t.Errorf("no information in direction %v (%.3g)", dir, q)
		}
	}
}

// matchSampled is match with its scores kept. The front end's answer must not
// change by asking for them.
func TestKeepingTheScoresChangesNoMatch(t *testing.T) {
	g := newGrid(40, 40, 0.05)
	g.matchRange = 10
	p := pose{5, 0, 0}
	an, d, v := sweepFrom(p, 200, 200)
	g.integrate(p, an, d, v)

	an, d, v = sweepFrom(pose{5.1, 0.05, 0.02}, 200, 200)
	prior := pose{5.05, 0, 0}
	plain := g.match(prior, an, d, v, defaultSearch())
	kept, _, _ := g.matchSampled(prior, an, d, v, defaultSearch())
	if plain != kept {
		t.Errorf("match gave %+v, matchSampled %+v", plain, kept)
	}
}

func TestAnEmptyMapSaysNothing(t *testing.T) {
	g := newGrid(20, 20, 0.05)
	an, d, v := sweepFrom(pose{2, 0, 0}, 200, 200)
	_, info, seen := g.matchSampled(pose{2, 0, 0}, an, d, v, defaultSearch())
	if seen != 0 || !info.isZero() {
		t.Errorf("an empty map fixed %d directions with information %v", seen, info)
	}
}

func TestInverseAndEigen(t *testing.T) {
	a := mat3{{4, 1, 0.5}, {1, 3, 0.2}, {0.5, 0.2, 2}}
	inv, ok := a.inverse()
	if !ok {
		t.Fatal("a regular matrix reported singular")
	}
	id := a.mul(inv)
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			want := 0.0
			if i == j {
				want = 1
			}
			if math.Abs(id[i][j]-want) > 1e-12 {
				t.Fatalf("a·a⁻¹ = %v", id)
			}
		}
	}
	if _, ok := (mat3{{1, 2, 0}, {2, 4, 0}, {0, 0, 1}}).inverse(); ok {
		t.Error("a singular matrix was inverted")
	}

	// A = V·diag(λ)·Vᵀ, from the eigen decomposition.
	values, vectors := a.eigen()
	back := vectors.mul(diag3(values[0], values[1], values[2])).mul(vectors.transpose())
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if math.Abs(back[i][j]-a[i][j]) > 1e-9 {
				t.Fatalf("V·Λ·Vᵀ = %v, want %v", back, a)
			}
		}
	}
}

// One metre driven straight is uncertain by the along and cross fractions,
// in the direction it was driven, and by the heading drift per metre.
func TestAStepIsUncertainAlongTheWayItWent(t *testing.T) {
	n := defaultOdometryNoise()
	drift := n.HeadingDegreesPerMeter * math.Pi / 180

	c := n.stepCovariance(pose{1, 0, 0}, 0)
	if !nearTo(c[0][0], 0.03*0.03, 1e-12) || !nearTo(c[1][1], 0.02*0.02, 1e-12) || !nearTo(c[2][2], drift*drift, 1e-12) {
		t.Errorf("east from heading 0: %v", c)
	}

	// The same step from a vehicle facing north: along is now y.
	c = n.stepCovariance(pose{1, 0, 0}, math.Pi/2)
	if !nearTo(c[1][1], 0.03*0.03, 1e-12) || !nearTo(c[0][0], 0.02*0.02, 1e-12) {
		t.Errorf("the same step facing north: %v", c)
	}

	// Standing still is not certain: the encoders resolve only so much.
	c = n.stepCovariance(pose{}, 0)
	if c[0][0] < minStepMetres*minStepMetres*0.999 || c[2][2] < minStepRadians*minStepRadians*0.999 {
		t.Errorf("standing still: %v", c)
	}
}
