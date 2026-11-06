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

// Finding a loop, and believing it.
//
// For each new keyframe, the older keyframes that could be the same place are
// tried: far enough back along the path that the front end has long since
// stopped matching against them, and near enough by the current estimate that
// the drift could explain the difference. The keyframe's sweep is matched
// against a submap drawn from the candidate and its neighbours, in the
// candidate's own frame; a match against the whole grid would find the
// doubled wall the loop is meant to remove.
//
// Then the match has to be believed, and most of this file is about that. A
// wrong loop is the most dangerous thing in the design: corridors and the
// doors along them repeat, and a convincing match in the wrong place folds
// the map. So a match must score well, must fix at least two directions, must
// be found again from the next keyframe with the same correction, and must
// agree with the rest of the graph once it is in it. See
// loop_closure_proposal_2026-11-05.md, section 3.4 and section 4.

import (
	"bufio"
	"context"
	"fmt"
	"log"
	"math"
	"os"
	"sort"
	"sync"
	"time"
)

// loopMatch is one match of keyframe j against the submap around keyframe i.
type loopMatch struct {
	i, j int

	// prior is where the estimate puts j in i's frame; z is where the match
	// does.
	prior, z pose

	// info is the match's information in the frame of the edge's residual;
	// infoGrid the same in i's frame, as the search measured it.
	info, infoGrid mat3
	seen           int

	// score is the likelihood under each valid return within matchRange, on
	// average: 1 for every return exactly on a wall.
	score float64

	accepted bool
}

// correction is where the match says keyframe j is, in the map frame, as a
// change to where the estimate has it: applied to any pose near j, it moves it
// the way the match would.
func (lm loopMatch) correction(frames []keyframe) pose {
	at := compose(frames[lm.i].pose, lm.z)
	return compose(at, inverse(frames[lm.j].pose))
}

// closer is the loop closure's state: the loops it has accepted, and the
// matches it is waiting to see confirmed. It is the loop goroutine's alone.
type closer struct {
	cfg        LoopConfig
	matchRange float64
	resolution float64

	loops   []edge
	streak  []loopMatch
	done    int // keyframes considered so far
	skipped int // keyframes skipped because the closer fell behind
	log     *loopLog

	// now stamps the log; a field so that tests need no clock.
	now func() time.Time
}

func newCloser(cfg LoopConfig, matchRange, resolution float64) *closer {
	return &closer{cfg: cfg, matchRange: matchRange, resolution: resolution, now: time.Now}
}

// closure is what accepting a loop decided: the keyframe poses after
// optimising, and the loop edges that were new.
type closure struct {
	nodes  []pose
	added  []loopMatch
	moved  float64 // the furthest any keyframe moved, in metres
	turned float64 // the most any keyframe turned, in radians
}

// candidates are the keyframes j could be a return to, nearest first.
func (c *closer) candidates(m *memory, j int) []int {
	fj := m.frames[j]
	type near struct {
		i int
		d float64
	}
	var found []near
	for i := 0; i < j; i++ {
		fi := m.frames[i]
		gap := fj.path - fi.path
		if gap < c.cfg.MinGapMetres {
			break // the keyframes after it are nearer along the path still
		}
		d := math.Hypot(fj.pose.x-fi.pose.x, fj.pose.y-fi.pose.y)
		if d <= c.cfg.SearchRadiusMetres+c.cfg.DriftFraction*gap {
			found = append(found, near{i, d})
		}
	}
	sort.Slice(found, func(a, b int) bool { return found[a].d < found[b].d })
	if len(found) > c.cfg.MaxCandidates {
		found = found[:c.cfg.MaxCandidates]
	}
	out := make([]int, len(found))
	for k, f := range found {
		out[k] = f.i
	}
	return out
}

// submap draws keyframe i and its neighbours into a grid in i's frame. Only
// neighbours far enough back along the path from j are drawn: one close to j
// would be the front end's own recent past, and the match would be against
// itself.
func (c *closer) submap(m *memory, i, j int) *grid {
	g := newGrid(c.cfg.SubmapMetres, c.cfg.SubmapMetres, c.resolution)
	g.matchRange = c.matchRange
	origin := inverse(m.frames[i].pose)
	from, to := i-c.cfg.SubmapKeyframes, i+c.cfg.SubmapKeyframes
	if from < 0 {
		from = 0
	}
	if to >= len(m.frames) {
		to = len(m.frames) - 1
	}
	limit := m.frames[j].path - c.cfg.MinGapMetres/2
	for k := from; k <= to; k++ {
		if m.frames[k].path > limit {
			to = k - 1
			break
		}
		f := m.frames[k]
		f.sweep.draw(g, compose(origin, f.pose))
	}
	for _, s := range m.sweepsOf(from, to) {
		s.draw(g, compose(origin, m.sweepPose(s)))
	}
	return g
}

// float64s is the sweep in the form the grid works in.
func (s storedSweep) float64s() ([]float64, []float64, []bool) {
	a := make([]float64, len(s.angles))
	d := make([]float64, len(s.distances))
	for k := range s.angles {
		a[k] = float64(s.angles[k])
	}
	for k := range s.distances {
		d[k] = float64(s.distances[k])
	}
	return a, d, s.valid
}

// match tries keyframe j against the submap around keyframe i.
func (c *closer) match(m *memory, i, j int) loopMatch {
	g := c.submap(m, i, j)
	prior := compose(inverse(m.frames[i].pose), m.frames[j].pose)
	a, d, v := m.frames[j].sweep.float64s()
	res, info, seen := g.matchSampled(prior, a, d, v, c.cfg.Search)

	returns := 0
	for k := range v {
		if v[k] && (c.matchRange <= 0 || d[k] <= c.matchRange) {
			returns++
		}
	}
	z := res.at
	if seen > 0 && seen < 3 {
		raw := info
		info = headingOnly(raw)
		if seen == 2 && c.cfg.corridorCrossTrack() {
			if axis, ok := g.corridorAxis(res.at, res.score, a, d, v); ok {
				z, info = acrossCorridor(prior, res.at, axis, raw)
			}
		}
	}
	lm := loopMatch{i: i, j: j, prior: prior, z: z, infoGrid: info, seen: seen}
	if returns > 0 {
		lm.score = res.score / float64(returns)
	}
	lm.info = edgeInformation(info, z.theta)
	return lm
}

// acrossCorridor is a corridor match as the graph should have it: the
// heading and the position across the corridor from the match, the position
// along it from the estimate, prior. axis is the corridor's direction in i's
// frame, as corridorAxis finds it on the submap; info the match's
// information.
//
// The position across a corridor is what a second pass down it needs. On the
// drive of 6 November, driven three times down the same corridor, the matches
// kept saying the vehicle was 0.2 to 0.5 m to one side, consistently against
// every old keyframe, while headingOnly threw it away; the pass came out
// beside the first, and each wall twice.
//
// headingOnly exists because the first version leaked the meaningless
// position along the corridor into the position across it, through a free
// direction found a few degrees off the axis. Two things stop that here. The
// position along the axis is the estimate's, so there is nothing along it to
// leak. And the axis is the submap's own — the direction in which the score
// does not fall — to within a degree, not read off the coarse search's
// samples.
func acrossCorridor(prior, at pose, axis float64, info mat3) (pose, mat3) {
	ux, uy := math.Cos(axis), math.Sin(axis)
	along := (prior.x-at.x)*ux + (prior.y-at.y)*uy
	z := pose{at.x + along*ux, at.y + along*uy, at.theta}
	// The information across the corridor and in heading, and none along it:
	// Q Ω Q, with Q the projection onto the across direction and the heading.
	px, py := -uy, ux
	q := mat3{{px * px, px * py, 0}, {py * px, py * py, 0}, {0, 0, 1}}
	return z, q.mul(info).mul(q)
}

// headingOnly keeps, of a match that does not fix all three directions, only
// what it says about the heading.
//
// The first version kept everything but the direction the view does not fix,
// which is right in theory and went wrong on the first real drive replayed
// through it. In a corridor that direction is found from the coarse pass read
// loosely, and comes out a few degrees off the corridor's axis — (0.99, 0.11,
// −0.07) where it should be (1, 0, 0). The match's position along the corridor
// is meaningless, anywhere in the window; through that tilt, 1.26 m of it
// became a 0.22 m and 1° correction across the corridor that nothing had
// measured, and the optimiser bent the loop to meet it and moved the vehicle
// 1.3 m along the corridor, from 0.14 m out to 1.2 m.
//
// The heading is what a corridor fixes beyond doubt — turn a sweep a few
// degrees and its long walls leave the map — and on its own it cannot carry
// the along-corridor error anywhere. So that is what is kept: the heading's
// information with the position left free, 1/(Ω⁺)θθ from the
// pseudo-inverse over the directions the view fixes.
func headingOnly(info mat3) mat3 {
	values, vectors := info.eigen()
	top := math.Max(values[0], math.Max(values[1], values[2]))
	var varTheta float64
	for e, lambda := range values {
		if lambda > 1e-9*top {
			varTheta += vectors[2][e] * vectors[2][e] / lambda
		}
	}
	if varTheta <= 0 {
		return mat3{}
	}
	return diag3(0, 0, 1/varTheta)
}

// edgeInformation turns a match's information from the frame it was searched
// in — offsets of j's pose in i's frame — to the frame of the edge's residual,
// e = t2v(z⁻¹ ∘ (x_i⁻¹ ∘ x_j)), whose position part is the offset turned by
// −θ_z. With e = Q·d for Q = diag(R(−θ_z), 1), dᵀ Ω d = eᵀ (Q Ω Qᵀ) e.
func edgeInformation(grid mat3, thetaZ float64) mat3 {
	c, s := math.Cos(thetaZ), math.Sin(thetaZ)
	q := mat3{{c, s, 0}, {-s, c, 0}, {0, 0, 1}}
	return q.mul(grid).mul(q.transpose())
}

// believable is the gate a single match must pass before it counts at all.
func (c *closer) believable(lm loopMatch) bool {
	return lm.score >= c.cfg.MinScore && lm.seen >= 2
}

// consistencyTolerance is how closely two matches must agree on the
// correction, in the directions both fix.
const (
	consistentMetres  = 0.20
	consistentRadians = 3 * math.Pi / 180
)

// consistent says whether two matches, from consecutive keyframes, say the
// same thing: applied to b's keyframe, the two corrections must put it in the
// same place, in every direction b's match fixes. Along a corridor b's match
// fixes nothing, so it cannot disagree there either.
func consistent(frames []keyframe, a, b loopMatch) bool {
	target := frames[b.j].pose
	pa := compose(a.correction(frames), target)
	pb := compose(b.correction(frames), target)
	d := [3]float64{pa.x - pb.x, pa.y - pb.y, wrapAngle(pa.theta - pb.theta)}

	// The directions b's match fixes, in the map frame: its information in
	// i's frame, turned by i's heading.
	ci, si := math.Cos(frames[b.i].pose.theta), math.Sin(frames[b.i].pose.theta)
	r := mat3{{ci, -si, 0}, {si, ci, 0}, {0, 0, 1}}
	info := r.mul(b.infoGrid).mul(r.transpose())
	values, vectors := info.eigen()
	top := math.Max(values[0], math.Max(values[1], values[2]))
	var kept [3]float64
	for e, lambda := range values {
		if lambda <= 1e-9*top {
			continue // a direction the view does not fix
		}
		var along float64
		for k := 0; k < 3; k++ {
			along += vectors[k][e] * d[k]
		}
		for k := 0; k < 3; k++ {
			kept[k] += along * vectors[k][e]
		}
	}
	return math.Hypot(kept[0], kept[1]) <= consistentMetres && math.Abs(kept[2]) <= consistentRadians
}

// consider is the loop closure's work for keyframe j: try it against its
// candidates, and if it confirms a loop that the graph accepts, the poses the
// graph puts the keyframes at. m is a snapshot; nothing here changes it.
func (c *closer) consider(m *memory, j int) (closure, bool) {
	var best loopMatch
	found := false
	for _, i := range c.candidates(m, j) {
		lm := c.match(m, i, j)
		ok := c.believable(lm)
		c.record(m, lm, ok, false, 0)
		if ok && (!found || lm.score > best.score) {
			best, found = lm, true
		}
	}
	if !found {
		c.streak = nil
		return closure{}, false
	}
	if n := len(c.streak); n > 0 {
		last := c.streak[n-1]
		if last.j != j-1 || !consistent(m.frames, last, best) {
			c.streak = nil
		}
	}
	c.streak = append(c.streak, best)
	if len(c.streak) < c.cfg.ConsistentMatches {
		return closure{}, false
	}

	var fresh []loopMatch
	var freshIdx []int
	for k, lm := range c.streak {
		if !lm.accepted {
			fresh = append(fresh, lm)
			freshIdx = append(freshIdx, k)
		}
	}
	if len(fresh) == 0 {
		return closure{}, false
	}
	loops := append([]edge(nil), c.loops...)
	for _, lm := range fresh {
		loops = append(loops, edge{i: lm.i, j: lm.j, z: lm.z, info: lm.info, loop: true})
	}
	g := m.graph(loops)
	first := len(g.edges) - len(fresh)

	// Phase one: the new loops without the kernel, from the headings and
	// positions solved alone, and then judged by their own χ².
	g.optimise(graphOptions{maxIterations: c.cfg.MaxIterations, initialise: true})
	// The χ² each new loop is judged by, and the one the log records: before
	// the kernel, in a graph that had to make room for it. After the kernel a
	// loop the rest disagrees with is weighed down rather than met, and its
	// χ² can come out far above maxChi2 although it passed — which is what
	// made the log of 5 November show accepted loops at 10 and 21.
	judged := make([]float64, len(fresh))
	for k := range fresh {
		chi2 := g.chi2(first + k)
		judged[k] = chi2
		if chi2 > c.cfg.MaxChi2 {
			c.record(m, fresh[k], true, false, chi2)
			log.Printf("cartographer: loop refused, keyframe %d ↔ %d: χ² %.1f with the rest of the map (at most %.1f)",
				fresh[k].j, fresh[k].i, chi2, c.cfg.MaxChi2)
			c.streak = nil
			return closure{}, false
		}
	}
	// Phase two: everything, with the kernel on the loops, from where phase
	// one left it.
	g.optimise(graphOptions{maxIterations: c.cfg.MaxIterations, robust: c.cfg.RobustScale})

	out := closure{nodes: g.nodes, added: fresh}
	for i, p := range g.nodes {
		out.moved = math.Max(out.moved, math.Hypot(p.x-m.frames[i].pose.x, p.y-m.frames[i].pose.y))
		out.turned = math.Max(out.turned, math.Abs(wrapAngle(p.theta-m.frames[i].pose.theta)))
	}
	for k, lm := range fresh {
		c.streak[freshIdx[k]].accepted = true
		c.record(m, lm, true, true, judged[k])
		c.loops = append(c.loops, edge{i: lm.i, j: lm.j, z: lm.z, info: lm.info, loop: true})
	}
	if c.cfg.GraphPath != "" {
		if err := g.writeG2O(c.cfg.GraphPath); err != nil {
			log.Printf("cartographer: writing the pose graph: %v", err)
		}
	}
	return out, true
}

// loopBacklog is how many keyframes the loop closer may fall behind the front
// end before it skips ahead to the newest. A loop is still found from the
// keyframes after it; one found from keyframes a minute old would be applied
// to a map that has moved on.
const loopBacklog = 20

// closeLoops is the loop closure's goroutine. It takes a snapshot of the
// memory whenever there are keyframes it has not considered, and considers
// them in order, outside the lock, while the front end goes on mapping.
func (t *Traits) closeLoops(ctx context.Context) {
	tick := time.NewTicker(500 * time.Millisecond)
	defer tick.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-tick.C:
		}
		for t.closeBatch() {
			// A closure ended the batch early: go straight on with a fresh
			// snapshot rather than wait for the next tick.
		}
		if c := t.closer; c.log != nil {
			if err := c.log.flush(); err != nil {
				log.Printf("cartographer: writing the loop log: %v", err)
			}
		}
	}
}

// closeBatch takes a snapshot of the memory, if there are keyframes the loop
// closer has not considered, and considers them in order. It returns true when
// a closure ended the batch with keyframes left to consider.
func (t *Traits) closeBatch() bool {
	c := t.closer
	t.mu.RLock()
	var snap *memory
	if len(t.mem.frames) > c.done {
		snap = t.mem.snapshot()
	}
	t.mu.RUnlock()
	if snap == nil {
		return false
	}
	if behind := len(snap.frames) - c.done; behind > loopBacklog {
		c.skipped += behind - 1
		c.done = len(snap.frames) - 1
		c.streak = nil
		log.Printf("cartographer: the loop closer fell %d keyframes behind and skipped them (%d so far)", behind-1, c.skipped)
	}
	start := c.done
	c.done = len(snap.frames)
	for j := start; j < len(snap.frames); j++ {
		if cl, ok := c.consider(snap, j); ok {
			t.loopClosed(snap, cl)
			// The snapshot is stale now: its keyframes are where they were
			// before this closure moved them. Matching the rest of it would
			// find the same offset again and close the same loop again — the
			// 5 November run did, six times over. So stop, and go on from a
			// fresh snapshot.
			c.done = j + 1
			c.streak = nil
			return c.done < len(snap.frames)
		}
	}
	return false
}

// loopClosed puts a closure into the map: the keyframes where the graph put
// them, the grid drawn again from them, and the current pose moved with them.
//
// The grid is drawn outside the lock, from the snapshot the closure was found
// in; for a long run that is seconds of work, and the front end goes on
// mapping into the old grid meanwhile. Then, under the lock, whatever the
// front end added in those seconds is drawn into the new grid too — moved by
// the correction of the newest keyframe the graph knew of, which is where it
// was drawn from — and the new grid takes the old one's place.
func (t *Traits) loopClosed(snap *memory, cl closure) {
	for _, lm := range cl.added {
		dz := compose(inverse(lm.prior), lm.z)
		log.Printf("cartographer: loop closed, keyframe %d ↔ %d (score %.2f, %d directions): %.2f m and %.1f° from where the map had it",
			lm.j, lm.i, lm.score, lm.seen, math.Hypot(dz.x, dz.y), math.Abs(dz.theta)*180/math.Pi)
	}
	if cl.moved < t.cfg.Loop.RedrawMetres && cl.turned < t.cfg.Loop.RedrawDegrees*math.Pi/180 {
		// In the graph, and too small to redraw the map for. Said, so that a
		// log shows how much of what the graph found reached the map.
		log.Printf("cartographer: kept in the graph, not redrawn: keyframes moved up to %.3f m and turned up to %.2f°",
			cl.moved, cl.turned*180/math.Pi)
		return
	}

	redrawn := snap.snapshot()
	copyPoses(redrawn.frames, cl.nodes)
	g := newGrid(t.cfg.WidthMetres, t.cfg.HeightMetres, t.cfg.Resolution)
	g.matchRange = t.cfg.MatchRangeMetres
	g.freeRange = t.cfg.freeRange()
	redrawn.drawAll(g)

	t.mu.Lock()
	defer t.mu.Unlock()
	// The vehicle's own displacement, not the correction's translation: the
	// correction is a transform of the map frame, and a turn of 2° about an
	// origin 40 m away has a translation of 1.4 m however little the vehicle
	// itself moved (the log of 6 November said 1.33 m for a few centimetres).
	before := t.at
	correction := t.applyClosure(g, snap, cl.nodes)
	log.Printf("cartographer: the map is redrawn from %d keyframes, moved by up to %.2f m (%d loops in all); the vehicle moved %.2f m and turned %.1f°",
		len(cl.nodes), cl.moved, len(t.closer.loops), math.Hypot(t.at.x-before.x, t.at.y-before.y), math.Abs(correction.theta)*180/math.Pi)
}

func copyPoses(frames []keyframe, nodes []pose) {
	for k := range frames {
		frames[k].pose = nodes[k]
	}
}

// applyClosure is the part of loopClosed done under the lock: g is the grid
// drawn from snap's sweeps at nodes. It returns the correction it moved the
// vehicle and the newer keyframes by.
//
// The correction for what came after the snapshot is taken from where the
// snapshot's newest keyframe is now, not from where the snapshot had it. The
// two differ when another closure has moved it since; the graph's poses say
// where things are, and moving by the difference from the stale pose would
// apply that other closure a second time. That is what turned the vehicle
// 25° on the 5 November run (TestTwoClosuresFromOneSnapshotDoNotStack).
func (t *Traits) applyClosure(g *grid, snap *memory, nodes []pose) pose {
	m := t.mem
	last := len(snap.frames) - 1
	correction := compose(nodes[last], inverse(m.frames[last].pose))
	frames, sweeps := m.since(snap)
	copyPoses(m.frames[:len(snap.frames)], nodes)
	for _, k := range frames {
		m.frames[k].pose = compose(correction, m.frames[k].pose)
		m.frames[k].sweep.draw(g, m.frames[k].pose)
	}
	for _, s := range sweeps {
		s.draw(g, m.sweepPose(s))
	}
	t.g = g

	// The vehicle is near the newest keyframe, and moves with it. So does
	// its velocity, which is in the map frame.
	t.at = compose(correction, t.at)
	c, s := math.Cos(correction.theta), math.Sin(correction.theta)
	t.vx, t.vy = c*t.vx-s*t.vy, s*t.vx+c*t.vy
	// A sweep being matched against the old grid right now finds out from
	// this that it was, and moves too (see consume).
	t.corrections = compose(correction, t.corrections)
	t.redraws++
	return correction
}

// record writes one match to the log, if there is one.
func (c *closer) record(m *memory, lm loopMatch, believed, accepted bool, chi2 float64) {
	if c.log == nil {
		return
	}
	dz := compose(inverse(lm.prior), lm.z)
	c.log.write(loopRow{
		t: c.now(), i: lm.i, j: lm.j, score: lm.score, seen: lm.seen,
		dx: dz.x, dy: dz.y, dtheta: dz.theta, chi2: chi2, believed: believed, accepted: accepted,
	})
}

//-------------------------------------The loop log

type loopRow struct {
	t                  time.Time
	i, j               int
	score              float64
	seen               int
	dx, dy, dtheta     float64 // where the match moved j from the estimate, in i's frame
	chi2               float64 // after optimising without the kernel, what maxChi2 judges; 0 if not tried
	believed, accepted bool
}

// loopLog writes one CSV line per match tried, and another for each one
// put to the graph: what the gates are tuned from, and what MATLAB reads.
type loopLog struct {
	mu sync.Mutex
	f  *os.File
	w  *bufio.Writer
}

func newLoopLog(path string) (*loopLog, error) {
	f, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_APPEND, 0o644)
	if err != nil {
		return nil, err
	}
	w := bufio.NewWriter(f)
	if st, err := f.Stat(); err == nil && st.Size() == 0 {
		fmt.Fprintln(w, "t,i,j,score,seen,dx,dy,dthetaDeg,chi2,believed,accepted")
	}
	return &loopLog{f: f, w: w}, nil
}

func (l *loopLog) write(r loopRow) {
	l.mu.Lock()
	defer l.mu.Unlock()
	fmt.Fprintf(l.w, "%.3f,%d,%d,%.4f,%d,%.4f,%.4f,%.3f,%.3f,%t,%t\n", float64(r.t.UnixMilli())/1000,
		r.i, r.j, r.score, r.seen, r.dx, r.dy, r.dtheta*180/math.Pi, r.chi2, r.believed, r.accepted)
}

func (l *loopLog) flush() error {
	l.mu.Lock()
	defer l.mu.Unlock()
	return l.w.Flush()
}

func (l *loopLog) close() error {
	l.mu.Lock()
	defer l.mu.Unlock()
	err := l.w.Flush()
	if cerr := l.f.Close(); err == nil {
		err = cerr
	}
	return err
}
