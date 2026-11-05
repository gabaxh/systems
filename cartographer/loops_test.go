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

	"github.com/sdoque/mbaigo/forms"
)

type wall = struct{ x1, y1, x2, y2 float64 }

// ringWorld replaces the test walls, for the length of one test, with a
// corridor 3 m wide running round a 14 × 4 m block, inside a 20 × 10 m
// building: a loop to close. Two recesses in the outer wall, and the corners,
// are what tell one stretch of it from another.
func ringWorld(t testing.TB) {
	t.Helper()
	saved := testWalls
	t.Cleanup(func() { testWalls = saved })
	testWalls = []wall{
		// the outer wall, with a recess in the bottom and one in the top
		{0, 0, 6, 0}, {6, 0, 6, -1}, {6, -1, 7.2, -1}, {7.2, -1, 7.2, 0}, {7.2, 0, 20, 0},
		{20, 0, 20, 10},
		{20, 10, 14.2, 10}, {14.2, 10, 14.2, 11}, {14.2, 11, 13, 11}, {13, 11, 13, 10}, {13, 10, 0, 10},
		{0, 10, 0, 0},
		// the block in the middle
		{3, 3, 17, 3}, {17, 3, 17, 7}, {17, 7, 3, 7}, {3, 7, 3, 3},
	}
}

// leg is a turn on the spot, then a straight run.
type leg struct{ turnDegrees, metres float64 }

// route is the poses a vehicle passes through from start: turns in 10° steps,
// straights in 0.1 m steps — a sweep drawn at each.
func route(start pose, legs ...leg) []pose {
	out := []pose{start}
	at := start
	for _, l := range legs {
		n := int(math.Round(math.Abs(l.turnDegrees) / 10))
		for k := 0; k < n; k++ {
			at = compose(at, pose{theta: math.Copysign(10, l.turnDegrees) * degree})
			out = append(out, at)
		}
		n = int(math.Round(l.metres / 0.1))
		for k := 0; k < n; k++ {
			at = compose(at, pose{x: 0.1})
			out = append(out, at)
		}
	}
	return out
}

// sweep360 is what the vigie gives: a full turn, a return a degree.
func sweep360(p pose, at time.Time) *forms.ScanA_v1a {
	a, d, v := sweepFrom(p, 360, 359)
	return &forms.ScanA_v1a{Angles: a, Distances: d, Valid: v, Timestamp: at}
}

// remember drives the route into a memory the way consume would, with the
// front end's estimate made by wrong(step) from each true step: the sweeps
// are what the truth sees, and they are kept where the estimate put them. It
// returns where each keyframe truly was.
func remember(m *memory, truths []pose, wrong func(pose) pose) []pose {
	var kfTruth []pose
	est := truths[0]
	for k, truth := range truths {
		prev := est
		if k > 0 {
			est = compose(est, wrong(compose(inverse(truths[k-1]), truth)))
			m.step(prev, est, true)
		}
		n := len(m.frames)
		m.keep(est, sweep360(truth, testTime.Add(time.Duration(k)*200*time.Millisecond)))
		if len(m.frames) > n {
			kfTruth = append(kfTruth, truth)
		}
	}
	return kfTruth
}

func testCloser() *closer {
	cfg := LoopConfig{}
	applyLoopDefaults(&cfg)
	c := newCloser(cfg, 10, 0.05)
	c.now = func() time.Time { return testTime }
	return c
}

// considerAll runs the closer over every keyframe, as the loop goroutine
// would, and returns the last closure.
func considerAll(c *closer, m *memory) (closure, int) {
	var last closure
	n := 0
	for j := range m.frames {
		if cl, ok := c.consider(m, j); ok {
			last = cl
			n++
		}
	}
	return last, n
}

// The case all of this is for. Round the ring with wheels that report a
// twentieth of a degree of turn every metre that never happened, the vehicle
// comes back to the start and the estimate has it some way off. The loop must
// be found there, believed, and the keyframes put back where they were.
func TestTheRingClosesShut(t *testing.T) {
	ringWorld(t)
	m := testMemory()
	truths := route(pose{1.5, 1.5, 0},
		leg{0, 17}, leg{90, 7}, leg{90, 17}, leg{90, 7}, leg{90, 4})
	kfTruth := remember(m, truths, func(step pose) pose {
		step.theta += 0.05 * degree * math.Hypot(step.x, step.y)
		return step
	})

	before, _ := worstError(framePoses(m), kfTruth)
	end := len(kfTruth) - 1
	endBefore := math.Hypot(m.frames[end].pose.x-kfTruth[end].x, m.frames[end].pose.y-kfTruth[end].y)

	c := testCloser()
	cl, n := considerAll(c, m)
	if n == 0 {
		t.Fatalf("no loop closed round the ring; the estimate ended %.2f m from the truth", endBefore)
	}
	after, r := worstError(cl.nodes, kfTruth)
	endAfter := math.Hypot(cl.nodes[end].x-kfTruth[end].x, cl.nodes[end].y-kfTruth[end].y)
	t.Logf("%d keyframes, %d closures, %d loops; worst %.3f m out before, %.3f m and %.2f° after; the end %.3f → %.3f m",
		len(m.frames), n, len(c.loops), before, after, r/degree, endBefore, endAfter)
	if before < 0.3 {
		t.Fatalf("the estimate was only %.2f m out; the test would prove nothing", before)
	}
	if endAfter > 0.2*endBefore {
		t.Errorf("the end keyframe went from %.3f to %.3f m out; closing the loop should take out most of it", endBefore, endAfter)
	}
	if after > 0.15 {
		t.Errorf("after closing the loop a keyframe is still %.3f m from the truth", after)
	}
	for _, e := range c.loops {
		if m.frames[e.j].path-m.frames[e.i].path < c.cfg.MinGapMetres {
			t.Errorf("a loop between keyframes %d and %d, only %.1f m apart along the path", e.i, e.j,
				m.frames[e.j].path-m.frames[e.i].path)
		}
	}
}

func framePoses(m *memory) []pose {
	out := make([]pose, len(m.frames))
	for i, f := range m.frames {
		out[i] = f.pose
	}
	return out
}

// The failure this project has met before, in a new place. A corridor with
// nothing in it, driven up and back, with wheels that read 3 % long: coming
// back, every keyframe is a convincing match against the one taken on the way
// out at the same place — and at the wrong place, as far as the wheels are
// concerned. A match there fixes the heading and the position across the
// corridor and must say nothing along it, so closing those loops must not
// shorten or stretch the corridor by a millimetre.
func TestACorridorDrivenUpAndBackIsNotShortened(t *testing.T) {
	plainCorridor(t, 0)
	m := testMemory()
	truths := route(pose{0, 0, 0}, leg{0, 15}, leg{180, 15})
	remember(m, truths, func(step pose) pose {
		step.x *= 1.03
		return step
	})

	c := testCloser()
	cl, n := considerAll(c, m)
	t.Logf("%d keyframes, %d closures, %d loops", len(m.frames), n, len(c.loops))
	if n == 0 {
		// Refusing every loop in a plain corridor is safe too; but then the
		// test has not tested what it is for.
		t.Fatal("no loop was closed coming back down the corridor")
	}
	for _, e := range c.loops {
		// Along the corridor is x in keyframe i's frame, which faced along it
		// on the way out; in the frame of the edge's residual that is x turned
		// by −θ_z.
		dir := [3]float64{math.Cos(e.z.theta), -math.Sin(e.z.theta), 0}
		across := [3]float64{math.Sin(e.z.theta), math.Cos(e.z.theta), 0}
		if a, c := quadratic(e.info, dir), quadratic(e.info, across); a > 1e-3*c {
			t.Errorf("loop %d↔%d carries information %.3g along the corridor, %.3g across it", e.i, e.j, a, c)
		}
	}
	for i, p := range cl.nodes {
		if d := p.x - m.frames[i].pose.x; math.Abs(d) > 0.002 {
			t.Errorf("keyframe %d moved %.4f m along the corridor", i, d)
		}
	}
}

// Driving away and never coming back closes nothing: there are no candidates.
func TestNoReturnNoLoop(t *testing.T) {
	ringWorld(t)
	m := testMemory()
	remember(m, route(pose{1.5, 1.5, 0}, leg{0, 17}, leg{90, 7}), func(p pose) pose { return p })
	c := testCloser()
	if _, n := considerAll(c, m); n != 0 || len(c.loops) != 0 {
		t.Errorf("%d closures and %d loops on a route that never returns", n, len(c.loops))
	}
}

// The candidates are the keyframes far enough back along the path, near
// enough across the map, nearest first and no more than asked for.
func TestCandidatesAreFarAlongAndNearAcross(t *testing.T) {
	c := testCloser()
	c.cfg.MaxCandidates = 2
	m := testMemory()
	// A path that comes back past its start: keyframes at path 0, 1, … 20 m.
	for k := 0; k <= 20; k++ {
		var p pose
		switch {
		case k <= 10:
			p = pose{x: float64(k)}
		default:
			p = pose{x: float64(20 - k), y: 0.3}
		}
		m.frames = append(m.frames, keyframe{id: k, pose: p, path: float64(k)})
	}
	// Keyframe 20 is at x=0, 20 m along: keyframes 0…10 are 10 m or more
	// back; by the map, 0 is 0.3 m away, 1 is 1.04 m, 2 is 2.02 m, 3 is 3.0 m.
	got := c.candidates(m, 20)
	if len(got) != 2 || got[0] != 0 || got[1] != 1 {
		t.Errorf("candidates %v, want [0 1]", got)
	}
	// Keyframe 15 is at x=5, 15 m along: only 0…5 are 10 m back, and the
	// radius is 2 + 0.05 × (15 − path).
	got = c.candidates(m, 15)
	if len(got) != 2 || got[0] != 5 || got[1] != 4 {
		t.Errorf("candidates %v, want [5 4]", got)
	}
}

// The edge's information is the match's, said in the edge's own frame: the
// same displacement weighs the same either way.
func TestTheMatchInformationIsTurnedIntoTheEdge(t *testing.T) {
	grid := mat3{{400, 30, 5}, {30, 90, -2}, {5, -2, 2000}}
	theta := 0.7
	ce := edgeInformation(grid, theta)
	d := [3]float64{0.03, -0.02, 0.004} // an offset of j, in i's frame
	c, s := math.Cos(theta), math.Sin(theta)
	e := [3]float64{c*d[0] + s*d[1], -s*d[0] + c*d[1], d[2]} // R(−θ_z)·d
	if a, b := quadratic(grid, d), quadratic(ce, e); math.Abs(a-b) > 1e-9*a {
		t.Errorf("dᵀΩd = %.6g in i's frame, %.6g in the edge's", a, b)
	}
}

// Two matches from consecutive keyframes agree when their corrections put the
// keyframe in the same place in the directions the view fixes. Along a
// corridor it fixes nothing, so they cannot disagree there.
func TestMatchesAgreeWhereTheViewCanTell(t *testing.T) {
	frames := []keyframe{{id: 0, pose: pose{}}, {id: 1, pose: pose{x: 0.5}},
		{id: 2, pose: pose{x: 12, y: 0.4}}, {id: 3, pose: pose{x: 12.5, y: 0.4}}}
	// Both say the vehicle is 0.4 m to the right of the estimate.
	a := loopMatch{i: 0, j: 2, z: pose{x: 12}, infoGrid: diag3(1e4, 1e4, 1e5)}
	b := loopMatch{i: 1, j: 3, z: pose{x: 12}, infoGrid: diag3(1e4, 1e4, 1e5)}
	if !consistent(frames, a, b) {
		t.Error("two matches with the same correction disagree")
	}
	// b says so too, but 0.5 m further along — which, in a corridor along x,
	// its view cannot tell.
	b.z.x += 0.5
	if consistent(frames, a, b) {
		t.Error("two matches 0.5 m apart along x agree, though the view fixes x")
	}
	b.infoGrid = diag3(0, 1e4, 1e5)
	if !consistent(frames, a, b) {
		t.Error("two matches that differ only along the corridor disagree")
	}
	// Across the corridor it can tell.
	b.z.y += 0.5
	if consistent(frames, a, b) {
		t.Error("two matches 0.5 m apart across the corridor agree")
	}
}

func TestTheLoopLogIsWritten(t *testing.T) {
	path := filepath.Join(t.TempDir(), "loops.csv")
	l, err := newLoopLog(path)
	if err != nil {
		t.Fatal(err)
	}
	l.write(loopRow{t: testTime, i: 3, j: 120, score: 0.71, seen: 3, dx: 0.4, dy: -0.1, dtheta: 2 * degree, chi2: 1.2, believed: true, accepted: true})
	if err := l.close(); err != nil {
		t.Fatal(err)
	}
	b, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	lines := strings.Split(strings.TrimSpace(string(b)), "\n")
	if len(lines) != 2 || !strings.HasPrefix(lines[0], "t,i,j,score") ||
		lines[1] != "1700000000.000,3,120,0.7100,3,0.4000,-0.1000,2.000,1.200,true,true" {
		t.Errorf("the log reads %q", lines)
	}
}

//-------------------------------------Putting a closure into the map

// segmentDistance is how far (x, y) is from the wall w.
func segmentDistance(x, y float64, w wall) float64 {
	dx, dy := w.x2-w.x1, w.y2-w.y1
	u := ((x-w.x1)*dx + (y-w.y1)*dy) / (dx*dx + dy*dy)
	u = math.Max(0, math.Min(1, u))
	return math.Hypot(x-(w.x1+u*dx), y-(w.y1+u*dy))
}

// onTheWalls is the share of the grid's occupied cells within tol of a real
// wall: how much of what the map says is there, is.
func onTheWalls(g *grid, tol float64) float64 {
	on, all := 0, 0
	for cy := 0; cy < g.height; cy++ {
		for cx := 0; cx < g.width; cx++ {
			if g.cells[cy*g.width+cx] <= 0 {
				continue
			}
			x := g.originX + (float64(cx)+0.5)*g.res
			y := g.originY + (float64(cy)+0.5)*g.res
			best := math.Inf(1)
			for _, w := range testWalls {
				best = math.Min(best, segmentDistance(x, y, w))
			}
			all++
			if best <= tol {
				on++
			}
		}
	}
	if all == 0 {
		return 0
	}
	return float64(on) / float64(all)
}

// ringTraits is a cartographer that has driven the ring with drifting wheels:
// its memory, and the grid the front end drew from the drifting estimate.
func ringTraits(t *testing.T) (*Traits, []pose) {
	ringWorld(t)
	tr := mappingTraits(nil)
	tr.cfg.WidthMetres, tr.cfg.HeightMetres = 50, 50
	tr.g = newGrid(50, 50, tr.cfg.Resolution)
	tr.g.matchRange = tr.cfg.MatchRangeMetres
	tr.mem = testMemory()
	tr.closer = testCloser()
	truths := route(pose{1.5, 1.5, 0},
		leg{0, 17}, leg{90, 7}, leg{90, 17}, leg{90, 7}, leg{90, 4})
	kfTruth := remember(tr.mem, truths, func(step pose) pose {
		step.theta += 0.05 * degree * math.Hypot(step.x, step.y)
		return step
	})
	tr.mem.drawAll(tr.g)
	last := tr.mem.frames[len(tr.mem.frames)-1]
	tr.at = last.pose
	for _, s := range tr.mem.sweepsOf(last.id, last.id) {
		tr.at = tr.mem.sweepPose(s)
	}
	return tr, append(kfTruth, truths[len(truths)-1])
}

// Round the ring the front end draws the start twice, a quarter of a metre
// apart. Once the loop is closed the map is drawn again from where the graph
// put the keyframes: the doubled walls are one wall, where the walls are, and
// the vehicle is where it is.
func TestClosingTheRingRedrawsTheMap(t *testing.T) {
	tr, truth := ringTraits(t)
	before := onTheWalls(tr.g, 0.10)
	old := tr.g

	closures := 0
	for j := range tr.mem.frames {
		snap := tr.mem.snapshot()
		if cl, ok := tr.closer.consider(snap, j); ok {
			tr.loopClosed(snap, cl)
			closures++
		}
	}
	if closures == 0 || tr.redraws == 0 {
		t.Fatalf("%d closures, %d redraws", closures, tr.redraws)
	}
	if tr.g == old {
		t.Fatal("the map was not replaced")
	}
	after := onTheWalls(tr.g, 0.10)
	end := truth[len(truth)-1]
	t.Logf("%d closures, %d redraws; %.1f %% of the walls drawn within 10 cm of a real one before, %.1f %% after; the vehicle %.3f m from the truth",
		closures, tr.redraws, 100*before, 100*after, math.Hypot(tr.at.x-end.x, tr.at.y-end.y))
	if after < 0.97 || after <= before {
		t.Errorf("%.1f %% of the redrawn map is on a real wall (%.1f %% before)", 100*after, 100*before)
	}
	if d := math.Hypot(tr.at.x-end.x, tr.at.y-end.y); d > 0.05 {
		t.Errorf("the vehicle is %.3f m from where it is", d)
	}
	// The memory's keyframes are where the graph put them, so the next loop
	// starts from the corrected map.
	worst, _ := worstError(framePoses(tr.mem), truth[:len(tr.mem.frames)])
	if worst > 0.15 {
		t.Errorf("a keyframe is still %.3f m from the truth in the memory", worst)
	}
}

// A closure that moves nothing much is kept in the graph and not drawn: the
// map is not redrawn every time a keyframe confirms a loop already closed.
func TestASmallCorrectionIsNotRedrawn(t *testing.T) {
	tr := mappingTraits(nil)
	tr.mem = testMemory()
	tr.closer = testCloser()
	remember(tr.mem, route(pose{}, leg{0, 3}), func(p pose) pose { return p })
	old, at := tr.g, tr.at
	snap := tr.mem.snapshot()
	nodes := framePoses(snap)
	nodes[len(nodes)-1].x += 0.01
	tr.loopClosed(snap, closure{nodes: nodes, moved: 0.01})
	if tr.g != old || tr.at != at || tr.redraws != 0 {
		t.Error("a 1 cm closure redrew the map")
	}
}

// What the front end keeps while a redraw is under way is told apart from what
// the redraw already has: the keyframes after the snapshot's last, and the
// sweeps after the snapshot's last sweep.
func TestWhatCameAfterTheSnapshotIsFound(t *testing.T) {
	m := testMemory()
	truths := route(pose{}, leg{0, 6})
	remember(m, truths[:33], func(p pose) pose { return p }) // 3.2 m
	snap := m.snapshot()
	inSnap := len(snap.frames) + len(snap.sweeps)
	start := truths[32]
	at := start
	for k := 1; k <= 20; k++ {
		next := compose(at, pose{x: 0.1})
		m.step(at, next, true)
		at = next
		m.keep(at, sweep360(at, testTime))
	}
	frames, sweeps := m.since(snap)
	if got := len(frames) + len(sweeps); got != 20 {
		t.Errorf("%d kept since the snapshot, want 20", got)
	}
	if got := len(m.frames) + len(m.sweeps); got != inSnap+20 {
		t.Errorf("the memory holds %d, the snapshot %d", got, inSnap)
	}
	for _, s := range sweeps {
		if p := m.sweepPose(s); p.x <= start.x {
			t.Errorf("a sweep at x=%.2f is counted as new; the snapshot ended at %.2f", p.x, start.x)
		}
	}
}

// The chain's measurements are what the front end saw, and stay so once the
// graph has moved the keyframes: a loop closed must not become, next time, a
// thing the wheels measured.
func TestTheChainKeepsWhatTheFrontEndSaw(t *testing.T) {
	m := testMemory()
	remember(m, route(pose{}, leg{0, 3}, leg{90, 2}), func(p pose) pose { return p })
	want := m.sequentialEdges()
	for k := range m.frames {
		m.frames[k].pose = compose(pose{0.3, -0.2, 0.1}, compose(m.frames[k].pose, pose{0, 0.01 * float64(k), 0}))
	}
	for k, e := range m.sequentialEdges() {
		if !samePose(e.z, want[k].z) {
			t.Errorf("edge %d→%d measures %+v after the keyframes moved, %+v before", e.i, e.j, e.z, want[k].z)
		}
	}
}

// A loop that fixes only two directions keeps only its heading: the heading's
// information with the position left free, which for a match fixing the
// position across the corridor (y) and the heading, with some coupling
// between them, is (yy·θθ − yθ²)/yy.
func TestACorridorLoopKeepsOnlyItsHeading(t *testing.T) {
	a, b, c := 8000.0, 3000.0, 22000.0
	got := headingOnly(mat3{{0, 0, 0}, {0, a, b}, {0, b, c}})
	want := (a*c - b*b) / a
	if got[0][0] != 0 || got[1][1] != 0 || got[0][1] != 0 || got[1][2] != 0 || math.Abs(got[2][2]-want) > 1e-6*want {
		t.Errorf("heading only: %v, want θθ %.6g and nothing else", got, want)
	}
	if !headingOnly(mat3{}).isZero() {
		t.Error("a match that fixes nothing kept something")
	}
}

// The bug the 5 November run found. On the vehicle the loop closer falls
// behind, and works through several keyframes from one snapshot; each closure
// it finds there was computed from the same stale poses. Applying the second
// must not move the vehicle again by what the first already did: the graph's
// poses are where things are, not how far to move them. Before the fix, each
// closure stacked its whole correction on the last, and six of them turned the
// vehicle 25° and drew a corridor across the map.
func TestTwoClosuresFromOneSnapshotDoNotStack(t *testing.T) {
	tr, _ := ringTraits(t)
	var snap *memory
	var cl closure
	found := false
	for j := range tr.mem.frames {
		snap = tr.mem.snapshot()
		if cl, found = tr.closer.consider(snap, j); found {
			break
		}
	}
	if !found {
		t.Fatal("no loop closed round the ring")
	}
	tr.loopClosed(snap, cl)
	once := tr.at
	keyframe := tr.mem.frames[len(tr.mem.frames)-1].pose

	tr.loopClosed(snap, cl) // the same solution again, from the same snapshot
	if d := math.Hypot(tr.at.x-once.x, tr.at.y-once.y); d > 0.001 || math.Abs(wrapAngle(tr.at.theta-once.theta)) > 0.001 {
		t.Errorf("a second closure from the same snapshot moved the vehicle %.3f m and %.2f° more", d, wrapAngle(tr.at.theta-once.theta)*180/math.Pi)
	}
	if k := tr.mem.frames[len(tr.mem.frames)-1].pose; !samePose(k, keyframe) {
		t.Errorf("the newest keyframe moved again, from %+v to %+v", keyframe, k)
	}
}

// The loop closer as it runs on the vehicle: behind the front end, with a
// backlog of keyframes in one snapshot, the closing ones among them. It must
// close the loop as well as when it keeps up — and before the fix it closed
// it once per keyframe, each closure stacked on the last.
func TestALaggingCloserClosesTheLoopOnce(t *testing.T) {
	tr, truth := ringTraits(t)
	// Behind by a backlog just short of what it would skip: the keyframes
	// round the last corner and back past the start.
	tr.closer.done = len(tr.mem.frames) - loopBacklog
	passes := 0
	for tr.closer.done < len(tr.mem.frames) && passes < 50 {
		tr.closeBatch()
		passes++
	}
	if tr.redraws == 0 {
		t.Fatalf("no closure in %d passes", passes)
	}
	end := truth[len(truth)-1]
	d := math.Hypot(tr.at.x-end.x, tr.at.y-end.y)
	turn := math.Abs(wrapAngle(tr.at.theta-end.theta)) * 180 / math.Pi
	t.Logf("%d passes, %d loops, %d redraws; the vehicle %.3f m and %.2f° from the truth", passes, len(tr.closer.loops), tr.redraws, d, turn)
	if d > 0.05 || turn > 1 {
		t.Errorf("a lagging closer left the vehicle %.3f m and %.2f° from the truth", d, turn)
	}
}
