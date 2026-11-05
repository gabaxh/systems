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

// The pose graph: the keyframes, the edges between them, and the poses that
// disagree least with all of the edges together.
//
// Each edge says where one keyframe was seen from another, and how sure it is.
// The sequential edges are the motion sensors' account of each stretch; the
// loop edges are matches against a part of the map seen long before. Before
// any loop the graph agrees with the front end exactly, and optimising it
// changes nothing. A loop edge is the first thing that disagrees: it says the
// vehicle is back where it started, the chain of sequential edges says it is
// some way off, and optimising shares the difference out along the chain —
// most where the chain is least sure of itself.
//
// This is the standard formulation — Lu & Milios (1997), in the notation of
// Grisetti, Kümmerle, Stachniss and Burgard, A Tutorial on Graph-Based SLAM
// (IEEE ITS Magazine, 2010) — solved by Levenberg–Marquardt, with each linear
// step solved by conjugate gradients. See loop_closure_proposal_2026-11-05.md,
// sections 3.2 and 3.5.

import (
	"bufio"
	"fmt"
	"math"
	"os"
)

// edge is one constraint: x_j as seen from x_i is z, with information info.
type edge struct {
	i, j int
	z    pose
	info mat3
	// loop edges come from a match against an older part of the map, and are
	// the ones the robust kernel is for.
	loop bool
}

// poseGraph is the keyframe poses and the edges between them. Node 0 is fixed:
// it is the origin of the map frame, and the map must not move as a whole.
type poseGraph struct {
	nodes []pose
	edges []edge
}

// graphOptions are how one optimisation is run.
type graphOptions struct {
	// maxIterations bounds the Levenberg–Marquardt iterations.
	maxIterations int

	// robust is the Cauchy kernel's scale on loop edges, in standard
	// deviations; 0 means no kernel.
	//
	// The kernel is for loops that have already been accepted, in a graph
	// that has already closed: there their residuals are small, and one that
	// later turns out to be wrong pulls with a force that flattens off rather
	// than growing with the error, so it cannot fold the map in half. It is
	// not for judging a new loop. A new loop that is right starts with the
	// whole of the drift as its residual — tens or hundreds of its standard
	// deviations — and the kernel would weigh it down to nothing and the loop
	// would never close. A new loop is judged instead by its χ² once it has
	// been optimised without one.
	robust float64

	// initialise starts from the headings, and then the positions, that the
	// edges imply when each is solved for alone (see initialise). It is for a
	// graph that has just been given a new loop, not one already closed; it
	// ignores the kernel, so it is not used together with robust.
	initialise bool
}

// graphResult is what an optimisation did.
type graphResult struct {
	iterations    int
	before, after float64 // the cost, Σ ρ(eᵀ Ω e)
	moved         float64 // the furthest any node moved, in metres
}

// residual is e_ij = t2v(z⁻¹ ∘ (x_i⁻¹ ∘ x_j)): how far x_j is from where the
// edge says it should be, in the frame the edge says it should be in.
func residual(e edge, xi, xj pose) [3]float64 {
	r := compose(inverse(e.z), compose(inverse(xi), xj))
	return [3]float64{r.x, r.y, r.theta}
}

// jacobians are ∂e/∂x_i and ∂e/∂x_j, from the closed form of the residual,
//
//	e = [ R_zᵀ (R_iᵀ (t_j − t_i) − t_z) ;  θ_j − θ_i − θ_z ],
//
// as in Grisetti et al., section IV.
func jacobians(z, xi, xj pose) (a, b mat3) {
	phi := xi.theta + z.theta
	c, s := math.Cos(phi), math.Sin(phi) // R_zᵀ R_iᵀ = R(−φ) = [c s; −s c]
	ci, si := math.Cos(xi.theta), math.Sin(xi.theta)
	cz, sz := math.Cos(z.theta), math.Sin(z.theta)
	dx, dy := xj.x-xi.x, xj.y-xi.y
	// dR_iᵀ/dθ_i · (t_j − t_i)
	ux := -si*dx + ci*dy
	uy := -ci*dx - si*dy
	a = mat3{
		{-c, -s, cz*ux + sz*uy},
		{s, -c, -sz*ux + cz*uy},
		{0, 0, -1},
	}
	b = mat3{
		{c, s, 0},
		{-s, c, 0},
		{0, 0, 1},
	}
	return a, b
}

func quad(v [3]float64, m mat3) float64 {
	w := m.mulVec(v)
	return v[0]*w[0] + v[1]*w[1] + v[2]*w[2]
}

// rho is the cost of an edge whose χ² is s, and its weight, ρ′(s). Without
// a kernel these are s and 1; with the Cauchy kernel of scale c they are
// c²·ln(1 + s/c²) and 1/(1 + s/c²).
func rho(s float64, loop bool, robust float64) (cost, weight float64) {
	if !loop || robust <= 0 {
		return s, 1
	}
	c2 := robust * robust
	return c2 * math.Log1p(s/c2), 1 / (1 + s/c2)
}

// chi2 is edge k's χ², eᵀ Ω e, at the graph's current poses.
func (g *poseGraph) chi2(k int) float64 {
	e := g.edges[k]
	return quad(residual(e, g.nodes[e.i], g.nodes[e.j]), e.info)
}

func (g *poseGraph) cost(nodes []pose, robust float64) float64 {
	var total float64
	for _, e := range g.edges {
		c, _ := rho(quad(residual(e, nodes[e.i], nodes[e.j]), e.info), e.loop, robust)
		total += c
	}
	return total
}

// system is H Δx = −b, linearised at the current poses, in blocks: H's
// diagonal blocks per node, its off-diagonal block per edge, and b per node.
type system struct {
	diag []mat3
	off  []mat3
	b    [][3]float64
}

func (g *poseGraph) linearise(robust float64) system {
	return g.assemble(func(e edge, xi, xj pose) ([3]float64, mat3, mat3, mat3) {
		r := residual(e, xi, xj)
		_, w := rho(quad(r, e.info), e.loop, robust)
		omega := e.info
		if w != 1 {
			for p := range omega {
				for q := range omega[p] {
					omega[p][q] *= w
				}
			}
		}
		a, b := jacobians(e.z, xi, xj)
		return r, a, b, omega
	})
}

// linearModel is one edge's part of a linear least-squares problem in the
// node increments: its residual r, its Jacobians a and b with respect to x_i
// and x_j, and its weight omega.
type linearModel func(e edge, xi, xj pose) (r [3]float64, a, b, omega mat3)

// assemble adds every edge's part into H and b.
func (g *poseGraph) assemble(model linearModel) system {
	n := len(g.nodes)
	sys := system{diag: make([]mat3, n), off: make([]mat3, len(g.edges)), b: make([][3]float64, n)}
	for k, e := range g.edges {
		r, a, b, omega := model(e, g.nodes[e.i], g.nodes[e.j])
		at, bt := a.transpose(), b.transpose()
		sys.diag[e.i] = sys.diag[e.i].add(at.mul(omega).mul(a))
		sys.diag[e.j] = sys.diag[e.j].add(bt.mul(omega).mul(b))
		sys.off[k] = at.mul(omega).mul(b)
		gi := at.mul(omega).mulVec(r)
		gj := bt.mul(omega).mulVec(r)
		for c := 0; c < 3; c++ {
			sys.b[e.i][c] += gi[c]
			sys.b[e.j][c] += gj[c]
		}
	}
	return sys
}

// solve is the damped step: (H + λ·diag(H)) Δx = −b, with node 0 held fixed,
// by conjugate gradients preconditioned with the chain of keyframes.
//
// Conjugate gradients rather than a factorisation, because H is as sparse as
// the graph and needs only multiplying by; a factorisation in the order the
// keyframes were taken fills in every block between the two ends of each
// loop.
//
// The preconditioner is what makes it fast, and the first one tried was not.
// Preconditioned only by its diagonal blocks, conjugate gradients on a chain
// of n keyframes needs of the order of n iterations — a chain is as badly
// conditioned as a matrix gets, its stiffness falling with the square of its
// length — and closing a 2000-keyframe loop took fifteen seconds on a laptop.
// So the preconditioner is the chain itself: H with the loops' off-diagonal
// blocks left out, which is block-tridiagonal and solved exactly in one sweep
// down the chain and one back. What is left over is a few blocks per loop, and
// conjugate gradients converges in a few iterations for each (subgraph
// preconditioning: Dellaert, Carlson, Ila, Ni and Thorpe, IROS 2010).
func (g *poseGraph) solve(sys system, lambda float64) ([][3]float64, bool) {
	n := len(g.nodes)
	damped := make([]mat3, n)
	for i := 1; i < n; i++ {
		d := sys.diag[i]
		for c := 0; c < 3; c++ {
			d[c][c] += lambda*d[c][c] + 1e-9
		}
		damped[i] = d
	}
	precondition, ok := chainPreconditioner(g.edges, sys.off, damped)
	if !ok {
		return nil, false
	}
	mulH := func(x [][3]float64, y [][3]float64) {
		for i := range y {
			y[i] = [3]float64{}
		}
		for i := 1; i < n; i++ {
			y[i] = damped[i].mulVec(x[i])
		}
		for k, e := range g.edges {
			o := sys.off[k]
			yi := o.mulVec(x[e.j])
			yj := o.transpose().mulVec(x[e.i])
			for c := 0; c < 3; c++ {
				y[e.i][c] += yi[c]
				y[e.j][c] += yj[c]
			}
		}
		y[0] = [3]float64{} // node 0 is not a variable
	}
	dot := func(a, b [][3]float64) float64 {
		var s float64
		for i := 1; i < n; i++ {
			s += a[i][0]*b[i][0] + a[i][1]*b[i][1] + a[i][2]*b[i][2]
		}
		return s
	}

	x := make([][3]float64, n)
	r := make([][3]float64, n)
	z := make([][3]float64, n)
	p := make([][3]float64, n)
	hp := make([][3]float64, n)
	for i := 1; i < n; i++ {
		for c := 0; c < 3; c++ {
			r[i][c] = -sys.b[i][c]
		}
	}
	precondition(r, z)
	copy(p, z)
	rz := dot(r, z)
	norm0 := math.Sqrt(dot(r, r))
	if norm0 == 0 {
		return x, true
	}
	// With the chain as preconditioner a few iterations per loop are enough;
	// the limit is for a graph that is not a chain at all.
	limit := 3 * n
	for it := 0; it < limit; it++ {
		mulH(p, hp)
		php := dot(p, hp)
		if php <= 0 {
			break
		}
		alpha := rz / php
		for i := 1; i < n; i++ {
			for c := 0; c < 3; c++ {
				x[i][c] += alpha * p[i][c]
				r[i][c] -= alpha * hp[i][c]
			}
		}
		if math.Sqrt(dot(r, r)) <= 1e-10*norm0 {
			break
		}
		precondition(r, z)
		next := dot(r, z)
		beta := next / rz
		rz = next
		for i := 1; i < n; i++ {
			for c := 0; c < 3; c++ {
				p[i][c] = z[i][c] + beta*p[i][c]
			}
		}
	}
	return x, true
}

// chainPreconditioner factors the block-tridiagonal part of the damped H — its
// diagonal blocks, and the off-diagonal blocks of the edges between
// consecutive keyframes — and returns what applies its inverse: z = M⁻¹ r.
//
// M is block-tridiagonal, so it is factored by block Gaussian elimination
// down the chain and solved by substitution back up it (the block Thomas
// algorithm), in time proportional to its length. It is positive definite
// wherever H is: leaving out a loop edge's off-diagonal blocks leaves its two
// diagonal blocks, each positive semidefinite on its own.
func chainPreconditioner(edges []edge, off []mat3, damped []mat3) (func(r, z [][3]float64), bool) {
	n := len(damped)
	// upper[k] is the block coupling node k to node k+1.
	upper := make([]mat3, n)
	for k, e := range edges {
		switch {
		case e.j == e.i+1:
			upper[e.i] = upper[e.i].add(off[k])
		case e.i == e.j+1:
			upper[e.j] = upper[e.j].add(off[k].transpose())
		}
	}
	// Node 0 is fixed, so the chain of variables starts at node 1.
	pivotInv := make([]mat3, n) // (D′_k)⁻¹
	for k := 1; k < n; k++ {
		d := damped[k]
		if k > 1 {
			// D′_k = D_k − U_{k−1}ᵀ D′_{k−1}⁻¹ U_{k−1}
			u := upper[k-1]
			s := u.transpose().mul(pivotInv[k-1]).mul(u)
			for p := range d {
				for q := range d[p] {
					d[p][q] -= s[p][q]
				}
			}
		}
		inv, ok := d.inverse()
		if !ok {
			return nil, false
		}
		pivotInv[k] = inv
	}
	return func(r, z [][3]float64) {
		// Forward: y_k = r_k − U_{k−1}ᵀ D′_{k−1}⁻¹ y_{k−1}, kept in z.
		z[0] = [3]float64{}
		for k := 1; k < n; k++ {
			z[k] = r[k]
			if k > 1 {
				v := upper[k-1].transpose().mulVec(pivotInv[k-1].mulVec(z[k-1]))
				for c := 0; c < 3; c++ {
					z[k][c] -= v[c]
				}
			}
		}
		// Back: x_k = D′_k⁻¹ (y_k − U_k x_{k+1}).
		for k := n - 1; k >= 1; k-- {
			y := z[k]
			if k < n-1 {
				v := upper[k].mulVec(z[k+1])
				for c := 0; c < 3; c++ {
					y[c] -= v[c]
				}
			}
			z[k] = pivotInv[k].mulVec(y)
		}
	}, true
}

// optimise moves the nodes to where the edges, together, put them.
//
// Levenberg–Marquardt: a Gauss–Newton step, damped by λ, tried; kept if the
// cost falls, with λ then lowered towards Gauss–Newton; otherwise λ raised and
// the step tried again, shorter and closer to steepest descent. It stops when
// no node moves by more than a tenth of a millimetre or a ten-thousandth of a
// radian, or after maxIterations.
func (g *poseGraph) optimise(opt graphOptions) graphResult {
	res := graphResult{before: g.cost(g.nodes, opt.robust)}
	res.after = res.before
	if len(g.nodes) < 2 || len(g.edges) == 0 {
		return res
	}
	start := append([]pose(nil), g.nodes...)
	if opt.initialise && opt.robust <= 0 {
		g.initialise(res.after)
		res.after = g.cost(g.nodes, 0)
	}
	lambda := 1e-4
	trial := make([]pose, len(g.nodes))
	for res.iterations < opt.maxIterations {
		res.iterations++
		sys := g.linearise(opt.robust)
		accepted := false
		var step float64
		for tries := 0; tries < 10; tries++ {
			dx, ok := g.solve(sys, lambda)
			if !ok {
				lambda *= 10
				continue
			}
			step = 0
			for i, p := range g.nodes {
				trial[i] = pose{p.x + dx[i][0], p.y + dx[i][1], wrapAngle(p.theta + dx[i][2])}
				step = math.Max(step, math.Max(math.Hypot(dx[i][0], dx[i][1]), math.Abs(dx[i][2])))
			}
			if c := g.cost(trial, opt.robust); c < res.after {
				copy(g.nodes, trial)
				res.after = c
				lambda = math.Max(lambda/10, 1e-9)
				accepted = true
				break
			}
			lambda *= 10
		}
		if !accepted || step < 1e-4 {
			break
		}
	}
	for i, p := range g.nodes {
		res.moved = math.Max(res.moved, math.Hypot(p.x-start[i].x, p.y-start[i].y))
	}
	return res
}

// initialise moves the nodes to a starting point close to the optimum: the
// headings first, from the edges' turns alone, and then the positions, from
// the edges' displacements with those headings held. It keeps the result only
// if it costs less than cost, the cost before.
//
// Gauss–Newton from the chain as the front end left it is slow when a loop
// has a large heading error in it. The error is a bend spread along the whole
// chain, and a long chain bends very easily; each linearised step follows the
// bend only part of the way round, and closing a 1 km loop with 10° of drift in
// it took forty iterations to settle to within a centimetre. But each half of
// the problem alone is linear: with the positions left out, the headings obey
// θ_j − θ_i = z_θ; with the headings fixed, the positions obey
// R_iᵀ (t_j − t_i) = t_z. Solving those two in turn lands next to the optimum,
// and Gauss–Newton finishes from there in a few iterations (Carlone, Aragues,
// Castellanos and Bona, A linear approximation for graph-based simultaneous
// localization and mapping, RSS 2011).
//
// Turns are taken as the current estimate wraps them; a loop with more than
// half a turn of drift in it would be closed the wrong way round, and the
// cost check then keeps the poses as they were.
func (g *poseGraph) initialise(cost float64) {
	saved := append([]pose(nil), g.nodes...)
	ident := diag3(1, 1, 1)
	minus := diag3(-1, -1, -1)

	// The headings. The positions are carried along with weight one and no
	// residual, so their increments come out zero.
	headings := g.assemble(func(e edge, xi, xj pose) ([3]float64, mat3, mat3, mat3) {
		r := [3]float64{0, 0, wrapAngle(xj.theta - xi.theta - e.z.theta)}
		return r, minus, ident, diag3(1, 1, math.Max(e.info[2][2], 0))
	})
	dx, ok := g.solve(headings, 0)
	if !ok {
		return
	}
	for i := range g.nodes {
		g.nodes[i].theta = wrapAngle(g.nodes[i].theta + dx[i][2])
	}

	// The positions, with the headings now fixed. The residual's position
	// part is linear in them, so one solve is exact.
	positions := g.assemble(func(e edge, xi, xj pose) ([3]float64, mat3, mat3, mat3) {
		r := residual(e, xi, xj)
		r[2] = 0
		a, b := jacobians(e.z, xi, xj)
		a[0][2], a[1][2] = 0, 0
		w := e.info
		w[0][2], w[1][2], w[2][0], w[2][1], w[2][2] = 0, 0, 0, 0, 1
		return r, a, b, w
	})
	dx, ok = g.solve(positions, 0)
	if !ok {
		copy(g.nodes, saved)
		return
	}
	for i := range g.nodes {
		g.nodes[i].x += dx[i][0]
		g.nodes[i].y += dx[i][1]
	}
	if g.cost(g.nodes, 0) >= cost {
		copy(g.nodes, saved)
	}
}

// writeG2O writes the graph in the g2o text format — VERTEX_SE2, FIX and
// EDGE_SE2 lines — which g2o_viewer and a few lines of MATLAB can draw.
func (g *poseGraph) writeG2O(path string) error {
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	w := bufio.NewWriter(f)
	for i, p := range g.nodes {
		fmt.Fprintf(w, "VERTEX_SE2 %d %.6f %.6f %.6f\n", i, p.x, p.y, p.theta)
	}
	fmt.Fprintln(w, "FIX 0")
	for _, e := range g.edges {
		m := e.info
		fmt.Fprintf(w, "EDGE_SE2 %d %d %.6f %.6f %.6f %.6g %.6g %.6g %.6g %.6g %.6g\n",
			e.i, e.j, e.z.x, e.z.y, e.z.theta, m[0][0], m[0][1], m[0][2], m[1][1], m[1][2], m[2][2])
	}
	if err := w.Flush(); err != nil {
		f.Close()
		return err
	}
	return f.Close()
}

//-------------------------------------From the memory

// sequentialEdges are the motion sensors' account of each stretch between two
// keyframes. The measurement is the front end's own relative pose, as it was
// when the keyframe was made (keyframe.rel), so a graph of these alone agrees
// with the front end exactly, and goes on saying what the front end saw after
// loops have moved the poses; the weight is the covariance the memory grew
// over the stretch.
func (m *memory) sequentialEdges() []edge {
	if len(m.frames) < 2 {
		return nil
	}
	edges := make([]edge, 0, len(m.frames)-1)
	for k := 1; k < len(m.frames); k++ {
		prev, f := m.frames[k-1], m.frames[k]
		info, ok := f.cov.inverse()
		if !ok {
			// Cannot happen with the floors in stepCovariance, but a weak edge
			// is better than a chain broken in two.
			info = diag3(1, 1, 1)
		}
		edges = append(edges, edge{i: prev.id, j: f.id, z: f.rel, info: info})
	}
	return edges
}

// graph is the memory's keyframes as a pose graph, with its sequential edges
// and the given loop edges.
func (m *memory) graph(loops []edge) *poseGraph {
	g := &poseGraph{nodes: make([]pose, len(m.frames))}
	for i, f := range m.frames {
		g.nodes[i] = f.pose
	}
	g.edges = append(m.sequentialEdges(), loops...)
	return g
}
