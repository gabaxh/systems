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

// curvedChain is a pose graph of n keyframes along a gentle curve, each edge
// exactly the motion between its two keyframes, with an uncertainty that
// differs from edge to edge.
func curvedChain(n int) *poseGraph {
	g := &poseGraph{nodes: []pose{{}}}
	for k := 1; k < n; k++ {
		step := pose{x: 0.5, y: 0.02 * float64(k%3), theta: 3 * degree * math.Sin(float64(k))}
		g.nodes = append(g.nodes, compose(g.nodes[k-1], step))
		cov := diag3(1e-3*float64(1+k%4), 4e-4, 1e-5*float64(1+k%2))
		cov[0][2], cov[2][0] = 2e-5, 2e-5
		info, _ := cov.inverse()
		g.edges = append(g.edges, edge{i: k - 1, j: k, z: step, info: info})
	}
	return g
}

// compound is the covariance of a ∘ b from theirs, a and b independent, to
// first order: J_a Σ_a J_aᵀ + J_b Σ_b J_bᵀ.
func compound(a pose, sa mat3, b pose, sb mat3) mat3 {
	c, s := math.Cos(a.theta), math.Sin(a.theta)
	ja := mat3{{1, 0, -s*b.x - c*b.y}, {0, 1, c*b.x - s*b.y}, {0, 0, 1}}
	jb := mat3{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}
	return ja.mul(sa).mul(ja.transpose()).add(jb.mul(sb).mul(jb.transpose()))
}

func closeMat(a, b mat3, rel float64) bool {
	var scale float64
	for p := range a {
		for q := range a[p] {
			scale = math.Max(scale, math.Abs(b[p][q]))
		}
	}
	for p := range a {
		for q := range a[p] {
			if math.Abs(a[p][q]-b[p][q]) > rel*scale {
				return false
			}
		}
	}
	return true
}

// On a chain alone, how uncertain one keyframe is from another is the edges
// between them compounded one after the other: the marginal of the graph and
// the dead reckoning agree.
func TestTheRelativeCovarianceOfAChainIsItsCompounding(t *testing.T) {
	g := curvedChain(30)
	mg, ok := g.marginals()
	if !ok {
		t.Fatal("no marginals for a plain chain")
	}
	for _, pair := range [][2]int{{0, 1}, {0, 29}, {4, 17}, {12, 13}} {
		i, j := pair[0], pair[1]
		at, want := pose{}, mat3{}
		for k := i + 1; k <= j; k++ {
			e := g.edges[k-1]
			// The edge's information is in its residual's frame, turned by
			// θ_z from the frame of its first keyframe.
			q, _ := e.info.inverse()
			cz, sz := math.Cos(e.z.theta), math.Sin(e.z.theta)
			rz := mat3{{cz, -sz, 0}, {sz, cz, 0}, {0, 0, 1}}
			q = rz.mul(q).mul(rz.transpose())
			want = compound(at, want, e.z, q)
			at = compose(at, e.z)
		}
		if got := mg.relative(i, j); !closeMat(got, want, 1e-6) {
			t.Errorf("%d → %d: %v, compounded %v", i, j, got, want)
		}
	}
}

// A loop edge between two keyframes is a second, independent account of
// where one is from the other, and the graph fuses the two: the information
// adds, (Σ_chain⁻¹ + Ω_loop)⁻¹, exactly so in the linearised problem.
func TestALoopIsFusedWithTheChain(t *testing.T) {
	g := curvedChain(40)
	i, j := 5, 35
	before, _ := g.marginals()
	chain := before.relative(i, j)

	z := compose(inverse(g.nodes[i]), g.nodes[j])
	// Isotropic in position, so the same in the edge's frame as in i's.
	loop := diag3(400, 400, 2e4)
	g.edges = append(g.edges, edge{i: i, j: j, z: z, info: loop, loop: true})
	after, _ := g.marginals()
	got := after.relative(i, j)

	ci, _ := chain.inverse()
	want, _ := ci.add(loop).inverse()
	if !closeMat(got, want, 1e-6) {
		t.Errorf("with the loop: %v, want %v", got, want)
	}
	if got[0][0] > 0.5*chain[0][0] {
		t.Errorf("the loop left the variance along x at %.4g of %.4g", got[0][0], chain[0][0])
	}
	// Keyframes the loop ties together only through the chain are pinned
	// too, if less: one three keyframes past j.
	if unpinned, pinned := before.relative(i, j), after.relative(i, j+3); pinned[0][0] >= unpinned[0][0] {
		t.Errorf("three keyframes past the loop: %.4g, against %.4g at the loop without it", pinned[0][0], unpinned[0][0])
	}
}

// The window is SearchSigmas standard deviations the way the position is
// least sure, within the search's range and MaxSearchMetres.
func TestTheWindowFollowsTheUncertainty(t *testing.T) {
	c := testCloser()
	for _, tc := range []struct {
		cov  mat3
		want float64
	}{
		{diag3(0.01, 0.01, 1e-4), c.cfg.Search.SearchRange},           // 0.3 m, under the floor
		{diag3(0.25, 1, 1e-4), 3},                                     // σ 1 m across
		{mat3{{0.625, 0.375, 0}, {0.375, 0.625, 0}, {0, 0, 1e-4}}, 3}, // the same, turned 45°
		{diag3(4, 0.01, 1e-4), c.cfg.MaxSearchMetres},                 // 6 m, over the cap
	} {
		if got := c.window(tc.cov); math.Abs(got-tc.want) > 1e-9 {
			t.Errorf("%v: window %.3f m, want %.3f", tc.cov, got, tc.want)
		}
	}
}

// Down a long stretch with no loop the window grows with the uncertainty the
// memory collected; once a loop pins the stretch, a keyframe just past the
// loop is searched for over a small window again, and one far past it over a
// larger one. The path rule had the same window for both.
func TestTheWindowShrinksWhereALoopPins(t *testing.T) {
	ringWorld(t)
	m := testMemory()
	truths := route(pose{1.5, 1.5, 0}, leg{0, 17}, leg{90, 7}, leg{90, 17}, leg{90, 7}, leg{90, 17})
	remember(m, truths, func(step pose) pose {
		step.theta += 0.05 * degree * math.Hypot(step.x, step.y)
		return step
	})
	c := testCloser()
	last := len(m.frames) - 1
	far := c.candidates(m, last)
	if len(far) == 0 {
		t.Fatal("no candidates for the last keyframe, one lap and a side round")
	}
	unpinned := far[0].reach
	if _, n := considerAll(c, m); n == 0 {
		t.Fatal("no loop closed")
	}
	pinned := c.candidates(m, last)
	if len(pinned) == 0 {
		t.Fatal("no candidates after closing")
	}
	t.Logf("the last keyframe's window: %.2f m before the loops, %.2f m after (%d loops)", unpinned, pinned[0].reach, len(c.loops))
	if pinned[0].reach >= unpinned {
		t.Errorf("the loops left the window at %.2f m, against %.2f m before", pinned[0].reach, unpinned)
	}
}
