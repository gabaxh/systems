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
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

const degree = math.Pi / 180

var testTime = time.Unix(1700000000, 0)

// The Jacobians are the closed form of the residual's derivative; central
// differences of the residual itself must agree with them, everywhere.
func TestTheJacobiansAreTheDerivatives(t *testing.T) {
	cases := []struct{ z, xi, xj pose }{
		{pose{1, 0, 0}, pose{0, 0, 0}, pose{1.1, 0.05, 0.02}},
		{pose{0.8, -0.3, 0.4}, pose{2, -1, 2.5}, pose{1.4, -0.2, 3.0}},
		{pose{-0.5, 0.2, -1.2}, pose{-3, 4, -2.9}, pose{-3.2, 4.6, 2.8}},
	}
	const h = 1e-6
	for n, c := range cases {
		a, b := jacobians(c.z, c.xi, c.xj)
		e := edge{z: c.z}
		for col := 0; col < 3; col++ {
			bump := func(p pose, d float64) pose {
				switch col {
				case 0:
					p.x += d
				case 1:
					p.y += d
				default:
					p.theta += d
				}
				return p
			}
			plusI, minusI := residual(e, bump(c.xi, h), c.xj), residual(e, bump(c.xi, -h), c.xj)
			plusJ, minusJ := residual(e, c.xi, bump(c.xj, h)), residual(e, c.xi, bump(c.xj, -h))
			for row := 0; row < 3; row++ {
				di := (plusI[row] - minusI[row]) / (2 * h)
				dj := (plusJ[row] - minusJ[row]) / (2 * h)
				if math.Abs(di-a[row][col]) > 1e-6 {
					t.Errorf("case %d: ∂e%d/∂x_i%d is %.7f, the closed form says %.7f", n, row, col, di, a[row][col])
				}
				if math.Abs(dj-b[row][col]) > 1e-6 {
					t.Errorf("case %d: ∂e%d/∂x_j%d is %.7f, the closed form says %.7f", n, row, col, dj, b[row][col])
				}
			}
		}
	}
}

// squareLoop is a 10 m square driven once round in 1 m steps, turning left at
// each corner: 40 keyframes, the last one step short of the first.
func squareLoop() (truth []pose, steps []pose) {
	truth = []pose{{}}
	for k := 0; k < 39; k++ {
		step := pose{x: 1}
		if k%10 == 9 {
			step.theta = 90 * degree
		}
		steps = append(steps, step)
		truth = append(truth, compose(truth[k], step))
	}
	return truth, steps
}

func worstError(nodes, truth []pose) (metres, radians float64) {
	for i := range nodes {
		metres = math.Max(metres, math.Hypot(nodes[i].x-truth[i].x, nodes[i].y-truth[i].y))
		radians = math.Max(radians, math.Abs(wrapAngle(nodes[i].theta-truth[i].theta)))
	}
	return metres, radians
}

// The case loop closure exists for. Wheels that report half a degree of turn
// every metre that never happened bend a 10 m square into a spiral: after 39 m
// the vehicle believes it is metres from where it started, when it is one step
// from it. One loop edge saying so, and the optimiser has to take the bend
// back out of every stretch — not just move the last keyframe.
func TestASquareLoopClosesShut(t *testing.T) {
	truth, steps := squareLoop()
	seqInfo := diag3(1/(0.02*0.02), 1/(0.02*0.02), 1/math.Pow(0.5*degree, 2))
	g := &poseGraph{nodes: []pose{{}}}
	for k, s := range steps {
		z := s
		z.theta += 0.5 * degree // the wheels' phantom turn
		g.edges = append(g.edges, edge{i: k, j: k + 1, z: z, info: seqInfo})
		g.nodes = append(g.nodes, compose(g.nodes[k], z))
	}
	loop := edge{i: 39, j: 0, z: compose(inverse(truth[39]), truth[0]), loop: true,
		info: diag3(1/(0.01*0.01), 1/(0.01*0.01), 1/math.Pow(0.1*degree, 2))}
	g.edges = append(g.edges, loop)

	before, _ := worstError(g.nodes, truth)
	if before < 1 {
		t.Fatalf("the bent square is only %.2f m out; the test would prove nothing", before)
	}
	res := g.optimise(graphOptions{maxIterations: 20})
	m, r := worstError(g.nodes, truth)
	t.Logf("worst %.2f m out before, %.4f m and %.3f° after %d iterations; cost %.3g → %.3g",
		before, m, r/degree, res.iterations, res.before, res.after)
	if m > 0.02 || r > 0.2*degree {
		t.Errorf("after closing the loop a keyframe is still %.3f m and %.2f° from the truth", m, r/degree)
	}
	if c := g.chi2(len(g.edges) - 1); c > 7.8 {
		t.Errorf("the loop edge's χ² is %.2f after optimising; the graph did not accept it", c)
	}
}

// truthGraph is the square with every edge exact and a loop edge that closes
// it: a graph that has already been optimised.
func truthGraph() *poseGraph {
	truth, steps := squareLoop()
	seqInfo := diag3(1/(0.02*0.02), 1/(0.02*0.02), 1/math.Pow(0.5*degree, 2))
	g := &poseGraph{nodes: append([]pose(nil), truth...)}
	for k, s := range steps {
		g.edges = append(g.edges, edge{i: k, j: k + 1, z: s, info: seqInfo})
	}
	g.edges = append(g.edges, edge{i: 39, j: 0, z: compose(inverse(truth[39]), truth[0]), loop: true,
		info: diag3(1/(0.01*0.01), 1/(0.01*0.01), 1/math.Pow(0.1*degree, 2))})
	return g
}

// A loop that got past every gate and is wrong — keyframe 20, on the far side
// of the square, matched against the start as if the two were 3 m apart and
// not 10 — must not fold the map. With the kernel it pulls with a force that
// flattens off; without it, it would win.
func TestOneWrongLoopCannotFoldTheMap(t *testing.T) {
	truth, _ := squareLoop()
	wrong := func() *poseGraph {
		g := truthGraph()
		z := compose(inverse(truth[20]), truth[0])
		z.x -= 7 // 3 m apart instead of 10
		g.edges = append(g.edges, edge{i: 20, j: 0, z: z, loop: true,
			info: diag3(1/(0.05*0.05), 1/(0.05*0.05), 1/math.Pow(1*degree, 2))})
		return g
	}

	robust := wrong()
	res := robust.optimise(graphOptions{maxIterations: 20, robust: 3})
	if res.moved > 0.05 {
		t.Errorf("one wrong loop moved the map %.3f m with the kernel", res.moved)
	}
	plain := wrong()
	pres := plain.optimise(graphOptions{maxIterations: 20})
	t.Logf("moved %.3f m with the kernel, %.3f m without", res.moved, pres.moved)
	if pres.moved < 0.5 {
		t.Errorf("without the kernel the wrong loop moved the map only %.3f m; the test would prove nothing", pres.moved)
	}
}

// A graph that agrees with itself is already where it should be, kernel or
// not: optimising it moves nothing.
func TestAGraphThatAgreesIsLeftAlone(t *testing.T) {
	g := truthGraph()
	if res := g.optimise(graphOptions{maxIterations: 20, robust: 3}); res.moved > 1e-6 {
		t.Errorf("a closed, consistent square moved %.2g m", res.moved)
	}
}

// Before any loop the graph is the front end's own account, and optimising it
// changes nothing: the back end must not move a map that has nothing to
// correct.
func TestTheFrontEndsOwnGraphDoesNotMove(t *testing.T) {
	m := testMemory()
	start := pose{}
	m.keep(start, scanAt(start, testTime))
	at := start
	for i := 1; i <= 60; i++ {
		next := compose(at, pose{0.07, 0, 2 * degree})
		m.step(at, next, true)
		at = next
		m.keep(at, scanAt(at, testTime))
	}
	g := m.graph(nil)
	if len(g.edges) != len(m.frames)-1 || len(g.nodes) != len(m.frames) {
		t.Fatalf("%d nodes and %d edges from %d keyframes", len(g.nodes), len(g.edges), len(m.frames))
	}
	res := g.optimise(graphOptions{maxIterations: 20})
	if res.before > 1e-12 || res.moved > 1e-9 {
		t.Errorf("the front end's own graph had cost %.3g and moved %.3g m", res.before, res.moved)
	}
	for k := range g.edges {
		e := g.edges[k]
		if e.info[0][0] <= 0 || e.info[2][2] <= 0 {
			t.Errorf("edge %d→%d has no weight: %v", e.i, e.j, e.info)
		}
	}
}

func TestTheGraphIsWrittenForG2O(t *testing.T) {
	g := truthGraph()
	path := filepath.Join(t.TempDir(), "graph.g2o")
	if err := g.writeG2O(path); err != nil {
		t.Fatal(err)
	}
	b, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	lines := strings.Split(strings.TrimSpace(string(b)), "\n")
	if want := len(g.nodes) + 1 + len(g.edges); len(lines) != want {
		t.Fatalf("%d lines, want %d", len(lines), want)
	}
	if lines[0] != "VERTEX_SE2 0 0.000000 0.000000 0.000000" || lines[len(g.nodes)] != "FIX 0" {
		t.Errorf("first vertex %q, then %q", lines[0], lines[len(g.nodes)])
	}
	if f := strings.Fields(lines[len(lines)-1]); f[0] != "EDGE_SE2" || f[1] != "39" || f[2] != "0" || len(f) != 12 {
		t.Errorf("the loop edge is written %q", lines[len(lines)-1])
	}
}

// circleLoop is a long run: n keyframes half a metre apart round a circle, with
// the wheels reporting drift degrees of turn per keyframe that never happened,
// and one loop edge at the end saying where the start really is.
func circleLoop(n int, drift float64) (*poseGraph, []pose) {
	step := pose{x: 0.5, theta: 2 * math.Pi / float64(n)}
	seqInfo := diag3(1/(0.02*0.02), 1/(0.02*0.02), 1/math.Pow(0.5*degree, 2))
	g := &poseGraph{nodes: []pose{{}}}
	truth := []pose{{}}
	for k := 0; k < n-1; k++ {
		z := step
		z.theta += drift * degree
		g.edges = append(g.edges, edge{i: k, j: k + 1, z: z, info: seqInfo})
		g.nodes = append(g.nodes, compose(g.nodes[k], z))
		truth = append(truth, compose(truth[k], step))
	}
	g.edges = append(g.edges, edge{i: n - 1, j: 0, z: compose(inverse(truth[n-1]), truth[0]), loop: true,
		info: diag3(1/(0.01*0.01), 1/(0.01*0.01), 1/math.Pow(0.1*degree, 2))})
	return g, truth
}

// A kilometre loop, 2000 keyframes, with 10° and then 100° of drift in it.
// Gauss–Newton from the front end's chain follows the bend only part of the
// way round with each step; starting from the headings, then the positions,
// solved alone, it has almost nothing left to do.
func TestALongBendIsTakenOutFirst(t *testing.T) {
	for _, drift := range []float64{0.005, 0.05} {
		g, truth := circleLoop(2000, drift)
		res := g.optimise(graphOptions{maxIterations: 20, initialise: true})
		m, r := worstError(g.nodes, truth)
		t.Logf("%.0f° of drift: %d iterations, cost %.3g → %.3g, worst %.4f m and %.4f° from the truth",
			drift*2000, res.iterations, res.before, res.after, m, r/degree)
		if m > 0.01 || r > 0.01*degree {
			t.Errorf("%.0f° of drift: a keyframe is still %.3f m and %.3f° out after %d iterations",
				drift*2000, m, r/degree, res.iterations)
		}
		if res.iterations > 5 {
			t.Errorf("%.0f° of drift took %d iterations; initialising should leave a few", drift*2000, res.iterations)
		}
	}
}

// How long the back end takes to close a kilometre loop is what decides
// whether it can run beside the front end.
func BenchmarkALongLoop(b *testing.B) {
	for i := 0; i < b.N; i++ {
		b.StopTimer()
		g, _ := circleLoop(2000, 0.05)
		b.StartTimer()
		res := g.optimise(graphOptions{maxIterations: 20, initialise: true})
		if i == 0 {
			b.Logf("%d iterations, cost %.3g → %.3g, moved %.2f m", res.iterations, res.before, res.after, res.moved)
		}
	}
}
