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
	"time"

	"github.com/sdoque/mbaigo/forms"
)

func testMemory() *memory {
	cfg := LoopConfig{}
	applyLoopDefaults(&cfg)
	return newMemory(cfg)
}

// driveMemory feeds the memory n sweeps taken step metres apart along the
// x axis, each one drawn into the map, as consume does.
func driveMemory(m *memory, n int, step float64, measured bool) pose {
	at := pose{}
	start := time.Now()
	for i := 0; i < n; i++ {
		next := pose{x: at.x + step}
		if i == 0 {
			next = at
		} else {
			how := byConstantVelocity
			if measured {
				how = byWheels
			}
			m.step(at, next, how)
		}
		at = next
		m.keep(at, scanAt(at, start.Add(time.Duration(i)*200*time.Millisecond)))
	}
	return at
}

// Down the corridor the keyframes come every half metre: the first at the first
// sweep, then whenever the vehicle is half a metre on from the last.
func TestAKeyframeEveryHalfMetre(t *testing.T) {
	m := testMemory()
	const step = 0.07
	driveMemory(m, 41, step, true) // 2.8 m

	// 0.07 m steps reach 0.5 m on the 8th, so the keyframes are 0.56 m apart.
	if got, want := len(m.frames), 6; got != want {
		t.Fatalf("%d keyframes over 2.8 m, want %d", got, want)
	}
	for i := 1; i < len(m.frames); i++ {
		gap := m.frames[i].pose.x - m.frames[i-1].pose.x
		if gap < m.cfg.KeyframeTravelMetres || gap > m.cfg.KeyframeTravelMetres+step+1e-9 {
			t.Errorf("keyframes %d and %d are %.3f m apart", i-1, i, gap)
		}
		if p := m.frames[i].path; math.Abs(p-m.frames[i].pose.x) > 1e-9 {
			t.Errorf("keyframe %d at x=%.3f has path %.3f", i, m.frames[i].pose.x, p)
		}
	}
	if got := len(m.frames) + len(m.sweeps); got != 41 {
		t.Errorf("%d sweeps kept of 41 drawn", got)
	}
}

// A turn on the spot makes keyframes too: the view from one place changes
// completely in a quarter turn, and a loop may come back facing any way.
func TestATurnMakesAKeyframe(t *testing.T) {
	m := testMemory()
	start := time.Now()
	at := pose{}
	m.keep(at, scanAt(at, start))
	for i := 1; i <= 9; i++ {
		next := pose{theta: float64(i) * 10 * math.Pi / 180}
		m.step(at, next, byWheels)
		at = next
		m.keep(at, scanAt(at, start))
	}
	// 90° in 10° steps: a keyframe at 0°, then at each first sweep at least 15°
	// on from the last keyframe: 20°, 40°, 60° and 80°.
	if got := len(m.frames); got != 5 {
		t.Errorf("%d keyframes in a quarter turn, want 5", got)
	}
}

// Straight down a corridor, the motion between two keyframes is uncertain along
// the way by the steps' own along-track errors, and in heading by the drift per
// metre — summed, because each step's error is its own. Across the corridor it
// is uncertain by more than the steps' cross-track errors: an error in heading
// early on swings every step after it sideways. That is what makes a wrong
// heading the expensive error, and the weight has to say so.
func TestTheUncertaintyGrowsAsTheWheelsDrive(t *testing.T) {
	m := testMemory()
	const step = 0.07
	driveMemory(m, 9, step, true) // keyframe 0 at 0, keyframe 1 at 0.56 m
	if len(m.frames) != 2 {
		t.Fatalf("%d keyframes, want 2", len(m.frames))
	}
	cov := m.frames[1].cov
	n := m.cfg.Noise
	steps := 8.0
	along := math.Max(minStepMetres, n.AlongFraction*step)
	cross := math.Max(minStepMetres, n.CrossFraction*step)
	turn := math.Max(minStepRadians, n.HeadingDegreesPerMeter*math.Pi/180*step)

	if want := steps * along * along; !nearTo(cov[0][0], want, 1e-12) {
		t.Errorf("along: variance %.3g, want %.3g", cov[0][0], want)
	}
	if want := steps * turn * turn; !nearTo(cov[2][2], want, 1e-12) {
		t.Errorf("heading: variance %.3g, want %.3g", cov[2][2], want)
	}
	if plain := steps * cross * cross; cov[1][1] <= plain {
		t.Errorf("across: variance %.3g, no more than the steps' own %.3g — the heading's share is missing", cov[1][1], plain)
	}
	if cov[1][2] <= 0 {
		t.Errorf("an error in heading moves the position across the corridor, but they are uncorrelated: %v", cov)
	}
	// The next keyframe starts afresh.
	if !m.cov.isZero() {
		t.Errorf("after a keyframe the running covariance is %v, want zero", m.cov)
	}
	if !m.frames[0].cov.isZero() {
		t.Errorf("the first keyframe is the origin and has no edge, but has %v", m.frames[0].cov)
	}
}

// A stretch that had no odometry came from constant velocity, and is the least
// trusted part of the path.
func TestAStepWithoutOdometryIsTrustedLess(t *testing.T) {
	measured, blind := testMemory(), testMemory()
	driveMemory(measured, 9, 0.07, true)
	driveMemory(blind, 9, 0.07, false)
	a, b := measured.frames[1], blind.frames[1]
	for i := 0; i < 3; i++ {
		if want := blindFactor * blindFactor * a.cov[i][i]; !nearTo(b.cov[i][i], want, 1e-9*want) {
			t.Errorf("diagonal %d: %.3g without odometry, %.3g with", i, b.cov[i][i], a.cov[i][i])
		}
	}
	if a.blind != 0 || b.blind != 8 {
		t.Errorf("blind steps counted %d and %d, want 0 and 8", a.blind, b.blind)
	}
}

// A kept sweep hangs off its keyframe: its pose is wherever the keyframe is,
// moved by the offset. Moving the keyframe moves the sweep with it.
func TestASweepMovesWithItsKeyframe(t *testing.T) {
	m := testMemory()
	start := time.Now()
	k := pose{1, 2, 0.3}
	m.keep(k, scanAt(k, start))
	p := compose(k, pose{0.2, 0.05, 0.04})
	m.keep(p, scanAt(p, start))
	if len(m.sweeps) != 1 || m.sweeps[0].kf != 0 {
		t.Fatalf("sweeps %+v", m.sweeps)
	}
	if got := m.sweepPose(m.sweeps[0]); !samePose(got, p) {
		t.Errorf("the sweep is at %+v, was taken at %+v", got, p)
	}
	m.frames[0].pose = pose{1.1, 2, 0.3 + 0.01}
	want := compose(m.frames[0].pose, pose{0.2, 0.05, 0.04})
	if got := m.sweepPose(m.sweeps[0]); !samePose(got, want) {
		t.Errorf("after moving its keyframe the sweep is at %+v, want %+v", got, want)
	}
}

// The sweep is kept as it came, to float32.
func TestASweepIsKeptAsItCame(t *testing.T) {
	m := testMemory()
	sw := &forms.ScanA_v1a{
		Angles:    []float64{-1, 0, 1},
		Distances: []float64{2.5, 0, 3.25},
		Valid:     []bool{true, false, true},
		Timestamp: time.Unix(1700000000, 0),
	}
	m.keep(pose{}, sw)
	s := m.frames[0].sweep
	for i := range sw.Angles {
		if float64(s.angles[i]) != sw.Angles[i] || float64(s.distances[i]) != sw.Distances[i] || s.valid[i] != sw.Valid[i] {
			t.Errorf("bin %d kept as %v %v %v", i, s.angles[i], s.distances[i], s.valid[i])
		}
	}
	sw.Valid[0] = false
	if !s.valid[0] {
		t.Error("the kept sweep shares its valid flags with the form it came from")
	}
	if !s.at.Equal(sw.Timestamp) {
		t.Errorf("kept at %v, taken at %v", s.at, sw.Timestamp)
	}
}

// Past maxSweeps the oldest are thinned: the newest are all still there, and
// no keyframe is lost.
func TestTheOldestSweepsAreThinned(t *testing.T) {
	m := testMemory()
	m.cfg.MaxSweeps = 100
	driveMemory(m, 400, 0.01, true) // 4 m: 8 keyframes, 392 sweeps
	if len(m.sweeps) > m.cfg.MaxSweeps {
		t.Errorf("%d sweeps kept, at most %d asked for", len(m.sweeps), m.cfg.MaxSweeps)
	}
	if got := len(m.frames); got != 8 {
		t.Errorf("%d keyframes, want 8", got)
	}
	last := m.sweeps[len(m.sweeps)-1]
	if x := m.sweepPose(last).x; !nearTo(x, 3.99, 1e-9) {
		t.Errorf("the newest sweep kept is at x=%.3f, want 3.99", x)
	}
	for i := 1; i < len(m.sweeps); i++ {
		if m.sweepPose(m.sweeps[i]).x <= m.sweepPose(m.sweeps[i-1]).x {
			t.Fatalf("thinning put sweeps %d and %d out of order", i-1, i)
		}
	}
}

// Through consume: with loop closure on, the corridor of
// TestTheWheelsCarryTheMapDownACorridor leaves keyframes behind it; off, it
// leaves nothing, and the map is drawn exactly as before.
func TestConsumeRemembersOnlyWhenAskedTo(t *testing.T) {
	drive := func(loop bool) *Traits {
		const step = 0.07
		var left, right float64 = 57.3, 57.3
		tr := mappingTraits(func() (wheelSample, wheelSample, error) {
			now := time.Now()
			return wheelSample{left, now}, wheelSample{right, now}, nil
		})
		if loop {
			tr.mem = newMemory(tr.cfg.Loop)
		}
		truth := pose{x: 2}
		start := time.Now()
		for i := 0; i <= 30; i++ {
			tr.consume(scanAt(truth, start.Add(time.Duration(i)*200*time.Millisecond)))
			truth.x += step
			left += step
			right += step
		}
		return tr
	}
	plain, remembering := drive(false), drive(true)

	if plain.mem != nil {
		t.Error("loop closure is off by default, but a memory was made")
	}
	if !samePose(plain.at, remembering.at) || plain.integrated != remembering.integrated {
		t.Errorf("remembering changed the front end: at %+v and %+v, %d and %d sweeps drawn",
			plain.at, remembering.at, plain.integrated, remembering.integrated)
	}
	m := remembering.mem
	if got := len(m.frames) + len(m.sweeps); got != remembering.integrated {
		t.Errorf("%d sweeps kept, %d drawn", got, remembering.integrated)
	}
	if len(m.frames) < 4 {
		t.Errorf("%d keyframes over 2.1 m", len(m.frames))
	}
	for _, f := range m.frames[1:] {
		if f.cov[0][0] <= 0 || f.cov[2][2] <= 0 {
			t.Errorf("keyframe %d has no uncertainty from the wheels: %v", f.id, f.cov)
		}
	}
}

func TestLoopClosureIsOffUnlessAskedFor(t *testing.T) {
	cfg := CartographerConfig{}
	applyDefaults(&cfg)
	if cfg.Loop.on() {
		t.Error("loop closure is on with no configuration")
	}
	if d := initTemplate().Traits.(*CartographerConfig).Loop; d.on() || d.KeyframeTravelMetres <= 0 || d.MaxSweeps <= 0 {
		t.Errorf("the template's loop block %+v", d)
	}
}

// rectangle is how rectangleDrive drives a 20 × 10 m rectangle once round.
type rectangle struct {
	// old makes every step the wheels' alone, as before the walls were
	// listened to.
	old bool
	// leg2Scale is how long the wheels read on the second leg; 1 is right.
	leg2Scale float64
	// cornerDegrees is a heading error picked up turning the second corner,
	// which the walls then hold for the rest of the drive.
	cornerDegrees float64
}

// rectangleDrive drives the rectangle, the walls holding the heading on the
// straights and the map fixing the corners. It returns the graph with one loop
// edge, from the last keyframe back to the first, where each keyframe truly
// was, and which leg each keyframe is on (-1 for one taken turning).
func rectangleDrive(r rectangle) (*poseGraph, []pose, []int) {
	m := testMemory()
	truths := route(pose{}, leg{0, 20}, leg{90, 10}, leg{90, 20}, leg{90, 10.2})
	est := truths[0]
	var kfTruth []pose
	var kfLeg []int
	corner, legNo := 0, 0
	for k, truth := range truths {
		turning := false
		if k > 0 {
			step := compose(inverse(truths[k-1]), truth)
			turning = step.theta != 0
			if turning && k >= 2 && compose(inverse(truths[k-2]), truths[k-1]).theta == 0 {
				corner++
			}
			wheels := step
			if !turning && corner == 1 && r.leg2Scale != 0 {
				wheels.x *= r.leg2Scale
			}
			prev := est
			est = compose(est, wheels)
			// The walls hold the heading, wrong by whatever was picked up
			// at the second corner.
			est.theta = truth.theta
			if corner >= 2 {
				est.theta = wrapAngle(truth.theta + r.cornerDegrees*degree)
			}
			how := byWallsHeading
			if turning {
				how = byMatch
			}
			if r.old {
				how = byWheels
			}
			m.step(prev, est, how)
			if !turning {
				legNo = corner
			}
		}
		n := len(m.frames)
		m.keep(est, scanAt(est, testTime))
		if len(m.frames) > n {
			kfTruth = append(kfTruth, truth)
			if turning {
				kfLeg = append(kfLeg, -1)
			} else {
				kfLeg = append(kfLeg, legNo)
			}
		}
	}
	last := len(m.frames) - 1
	loop := edge{i: 0, j: last, z: compose(inverse(kfTruth[0]), kfTruth[last]), loop: true,
		info: diag3(1e4, 1e4, 1e5)}
	return m.graph([]edge{loop}), kfTruth, kfLeg
}

// The failure of the run of 5 November, in a rectangle. Along a corridor the
// walls hold the heading and the wheels alone say how far; a wheel that reads
// long makes one leg too long, and closing the loop must shorten it. Weighted
// as the wheels' random walk, the heading looked the loose part, and the
// optimiser bent the straight legs instead.
func TestACorridorStretchesInsteadOfBending(t *testing.T) {
	worst := func(g *poseGraph, truth []pose, _ []int) (metres, degrees float64) {
		g.optimise(graphOptions{maxIterations: 20, initialise: true})
		for i, p := range g.nodes {
			metres = math.Max(metres, math.Hypot(p.x-truth[i].x, p.y-truth[i].y))
			degrees = math.Max(degrees, math.Abs(wrapAngle(p.theta-truth[i].theta))*180/math.Pi)
		}
		return metres, degrees
	}
	newM, newD := worst(rectangleDrive(rectangle{leg2Scale: 1.05}))
	oldM, oldD := worst(rectangleDrive(rectangle{leg2Scale: 1.05, old: true}))
	t.Logf("weighted by the walls: worst %.3f m and %.2f°; as the wheels' random walk: %.3f m and %.2f°", newM, newD, oldM, oldD)
	if newD > 0.3 {
		t.Errorf("a straight leg was bent by %.2f° to close the loop", newD)
	}
	if oldD < 1 {
		t.Errorf("weighted the old way the legs bent only %.2f°; the test would prove nothing", oldD)
	}
	// The leg read 0.5 m long. The loop cannot tell which of the two parallel
	// legs that was, so the most it can do is take half off each: 0.25 m. A
	// little more is the price of letting the corners turn, which is where a
	// real heading error goes (TestAStraightCorridorStaysStraight).
	if newM > 0.30 || newM >= oldM {
		t.Errorf("a keyframe is %.3f m out, against %.3f m the old way and the 0.25 m a loop cannot tell apart", newM, oldM)
	}
}

// The failure of the run of 6 November, in a rectangle. A heading error picked
// up at a corner is held by the walls down every straight after it, so it
// belongs at the corners, where the straights meet. Spread along the
// straights instead, each one comes out curved, and one wall drawn from both
// ends of a straight is drawn twice.
func TestAStraightCorridorStaysStraight(t *testing.T) {
	spread := func(g *poseGraph, truth []pose, legs []int) (worst float64) {
		g.optimise(graphOptions{maxIterations: 20, initialise: true})
		for l := 0; l < 4; l++ {
			lo, hi := math.Inf(1), math.Inf(-1)
			for i, p := range g.nodes {
				if legs[i] == l {
					e := wrapAngle(p.theta-truth[i].theta) * 180 / math.Pi
					lo, hi = math.Min(lo, e), math.Max(hi, e)
				}
			}
			worst = math.Max(worst, hi-lo)
		}
		return worst
	}
	g, truth, legs := rectangleDrive(rectangle{cornerDegrees: 1.5})
	newS := spread(g, truth, legs)
	oldS := spread(rectangleDrive(rectangle{cornerDegrees: 1.5, old: true}))
	t.Logf("heading spread along the worst straight: %.3f° weighted by the walls, %.3f° as the wheels' random walk", newS, oldS)
	if newS > 0.2 {
		t.Errorf("a straight leg curves by %.3f° after closing the loop", newS)
	}
	if oldS < 5*newS || oldS < 0.2 {
		t.Errorf("the old weights curved the legs by only %.3f°; the test would prove nothing", oldS)
	}
	if e := g.chi2(len(g.edges) - 1); e > testMemory().cfg.MaxChi2 {
		t.Errorf("the loop's χ² is %.2f: the chain is too stiff to meet a right loop", e)
	}
}

// Where the walls placed the pose, the heading's uncertainty grows far slower
// than the wheels' and the along-track position's faster in a corridor, and
// neither is reset: a chain reset to the match's accuracy at every sweep was
// too stiff to meet any loop (1b8fd2a).
func TestWhatTheWallsFixedIsWeighedAsTheWalls(t *testing.T) {
	grow := func(how placed) mat3 {
		m := testMemory()
		m.keep(pose{}, scanAt(pose{}, testTime))
		at := pose{}
		for i := 0; i < 4; i++ {
			next := compose(at, pose{x: 0.1})
			m.step(at, next, how)
			at = next
		}
		return m.cov
	}
	wheels, walls, matched := grow(byWheels), grow(byWallsHeading), grow(byMatch)
	if walls[2][2] >= wheels[2][2]/10 || matched[2][2] >= wheels[2][2]/10 {
		t.Errorf("heading variance: wheels %.3g, walls %.3g, matched %.3g", wheels[2][2], walls[2][2], matched[2][2])
	}
	if walls[0][0] <= wheels[0][0] {
		t.Errorf("along a corridor the position should be looser than the wheels' own: %.3g against %.3g", walls[0][0], wheels[0][0])
	}
	two := func() mat3 {
		m := testMemory()
		m.keep(pose{}, scanAt(pose{}, testTime))
		at := pose{}
		for i := 0; i < 8; i++ {
			next := compose(at, pose{x: 0.05})
			m.step(at, next, byMatch)
			at = next
		}
		return m.cov
	}()
	if two[2][2] <= matched[2][2]/2 || two[0][0] <= 0 {
		t.Errorf("matched steps do not accumulate: %v after 8, %v after 4", two, matched)
	}
}
