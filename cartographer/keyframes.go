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

// The memory loop closure needs: where each part of the map was seen from.
//
// The front end draws a sweep into the grid and forgets it. That is all a
// front end needs, and it is why the error collected round a loop can never be
// taken back: the grid holds only the sum of the sweeps, and log-odds cannot
// be undrawn. So with loop closure on, every sweep that is drawn is also kept,
// and every half metre or so one of them becomes a keyframe — a node of the
// pose graph. The other sweeps hang off their keyframe by a fixed offset and
// move with it when the graph moves it, which keeps the graph to about two
// nodes a metre without making a rebuilt map any sparser than the one the
// front end drew.
//
// Between keyframes the memory also keeps how far the motion sensors' account
// of the stretch may be off: the covariance of the motion from one keyframe to
// the next, grown step by step with OdometryNoise. That is the weight of the
// edge between them. See loop_closure_proposal_2026-11-05.md, sections 3.1 and
// 3.3.

import (
	"math"
	"sort"
	"time"

	"github.com/sdoque/mbaigo/forms"
)

// LoopConfig is loop closure's part of the cartographer's configuration.
type LoopConfig struct {
	// Enabled switches loop closure on. Absent means off: until it has been
	// tried on the vehicle the cartographer maps exactly as it always has.
	Enabled *bool `json:"enabled,omitempty"`

	// KeyframeTravelMetres and KeyframeTurnDegrees are how far the vehicle
	// moves, or turns, between one keyframe and the next.
	KeyframeTravelMetres float64 `json:"keyframeTravelMetres"`
	KeyframeTurnDegrees  float64 `json:"keyframeTurnDegrees"`

	// Noise is how far the motion between two sweeps may be off. With
	// gyrodometry on, headingDegreesPerMeter can be smaller: a slip no longer
	// turns the heading.
	Noise OdometryNoise `json:"noise"`

	// Where the walls placed a sweep, they, not the wheels, say how far its
	// heading may be off (see memory.step). MatchedHeadingDegreesPerMeter
	// and MatchedTurnFraction take the place of the noise model's heading
	// terms there. CorridorAlongFraction is the along-track error per step
	// where the walls held only the heading and the wheels alone said how far.
	MatchedHeadingDegreesPerMeter float64 `json:"matchedHeadingDegreesPerMeter"`
	MatchedTurnFraction           float64 `json:"matchedTurnFraction"`
	CorridorAlongFraction         float64 `json:"corridorAlongFraction"`

	// MaxSweeps is how many sweeps are kept. Past it the oldest are thinned;
	// keyframes are never dropped.
	MaxSweeps int `json:"maxSweeps"`

	// RobustScale is the Cauchy kernel's scale on loop edges that have been
	// accepted, in standard deviations; MaxIterations bounds each
	// optimisation. See posegraph.go.
	RobustScale   float64 `json:"robustScale"`
	MaxIterations int     `json:"maxIterations"`

	// Finding a loop (see loops.go). A keyframe is compared with the older
	// keyframes at least MinGapMetres of path back, and within
	// SearchRadiusMetres + DriftFraction × the path between them; the nearest
	// MaxCandidates of them are tried.
	MinGapMetres       float64 `json:"minGapMetres"`
	SearchRadiusMetres float64 `json:"searchRadiusMetres"`
	DriftFraction      float64 `json:"driftFraction"`
	MaxCandidates      int     `json:"maxCandidates"`

	// Each is tried by matching against a submap SubmapMetres across, drawn
	// from the candidate's sweeps and those of SubmapKeyframes keyframes
	// either side of it, with the window Search.
	SubmapMetres    float64      `json:"submapMetres"`
	SubmapKeyframes int          `json:"submapKeyframes"`
	Search          SearchConfig `json:"search"`

	// A match is believed when its score per return is at least MinScore,
	// it fixes at least two directions, and ConsistentMatches keyframes in a
	// row agree on it; and kept when its χ² after optimising is at most
	// MaxChi2.
	MinScore          float64 `json:"minScore"`
	ConsistentMatches int     `json:"consistentMatches"`
	MaxChi2           float64 `json:"maxChi2"`

	// RedrawMetres is how far a closure must move some keyframe for the map
	// to be drawn again from the keyframes. Smaller corrections are kept in
	// the graph and left out of the map: driving through a place already
	// closed, every keyframe confirms the loop and moves the map a few
	// millimetres, and redrawing it each time would be all the work there is.
	RedrawMetres float64 `json:"redrawMetres"`

	// RedrawDegrees is the same for a turn: a closure that turns some keyframe
	// by at least this much redraws the map even if nothing moved
	// RedrawMetres. A heading correction hardly moves a keyframe, and judged
	// by RedrawMetres alone it was kept in the graph but never reached the
	// vehicle, which drove on with the old heading: on 6 November the matches
	// went on finding the same +0.5° to +1° after the loops that said so had
	// been accepted.
	RedrawDegrees float64 `json:"redrawDegrees"`

	// CorridorCrossTrack keeps, of a corridor match, the position across the
	// corridor as well as the heading (see acrossCorridor). Absent means on;
	// off keeps the heading only, as before 7 November.
	CorridorCrossTrack *bool `json:"corridorCrossTrack,omitempty"`

	// LogPath, when set, gets one CSV line per loop tried; GraphPath the pose
	// graph in g2o format after each closure.
	LogPath   string `json:"logPath"`
	GraphPath string `json:"graphPath"`
}

// defaultLoopSearch is the window a loop is searched over: wider than the
// front end's, because the drift a loop removes is what puts the vehicle off
// where the map says it is, and coarser, because it is searched for every
// keyframe that has a candidate.
func defaultLoopSearch() SearchConfig {
	return SearchConfig{
		SearchRange:  1.2,
		CoarseStep:   0.10,
		FineStep:     0.0125,
		AngleRange:   20 * math.Pi / 180,
		CoarseAngle:  2 * math.Pi / 180,
		FineAngle:    0.5 * math.Pi / 180,
		CoarseStride: 4,
	}
}

func (c LoopConfig) on() bool { return c.Enabled != nil && *c.Enabled }

func (c LoopConfig) corridorCrossTrack() bool {
	return c.CorridorCrossTrack == nil || *c.CorridorCrossTrack
}

func defaultLoop() LoopConfig {
	return LoopConfig{
		Enabled:              &off,
		KeyframeTravelMetres: 0.5,
		KeyframeTurnDegrees:  15,
		Noise:                defaultOdometryNoise(),
		// From the run of 6 November: the loop's 1.5° of heading belongs at
		// the corners, where a new corridor is drawn from a turn, and not
		// along a straight one, where the walls hold the heading sweep after
		// sweep. 0.1°/m is 0.05° a keyframe; 2 % of a 90° corner is 1.8°.
		MatchedHeadingDegreesPerMeter: 0.1,
		MatchedTurnFraction:           0.02,
		CorridorAlongFraction:         0.25,
		MaxSweeps:                     20000,
		RobustScale:                   3,
		MaxIterations:                 20,
		MinGapMetres:                  10,
		SearchRadiusMetres:            2,
		DriftFraction:                 0.05,
		MaxCandidates:                 3,
		SubmapMetres:                  16,
		SubmapKeyframes:               5,
		Search:                        defaultLoopSearch(),
		MinScore:                      0.55,
		ConsistentMatches:             2,
		MaxChi2:                       7.8,
		RedrawMetres:                  0.05,
		RedrawDegrees:                 0.25,
		CorridorCrossTrack:            &on,
	}
}

func applyLoopDefaults(c *LoopConfig) {
	d := defaultLoop()
	if c.KeyframeTravelMetres <= 0 {
		c.KeyframeTravelMetres = d.KeyframeTravelMetres
	}
	if c.KeyframeTurnDegrees <= 0 {
		c.KeyframeTurnDegrees = d.KeyframeTurnDegrees
	}
	if c.Noise == (OdometryNoise{}) {
		c.Noise = d.Noise
	}
	if c.MatchedHeadingDegreesPerMeter <= 0 {
		c.MatchedHeadingDegreesPerMeter = d.MatchedHeadingDegreesPerMeter
	}
	if c.MatchedTurnFraction <= 0 {
		c.MatchedTurnFraction = d.MatchedTurnFraction
	}
	if c.CorridorAlongFraction <= 0 {
		c.CorridorAlongFraction = d.CorridorAlongFraction
	}
	if c.MaxSweeps <= 0 {
		c.MaxSweeps = d.MaxSweeps
	}
	if c.RobustScale <= 0 {
		c.RobustScale = d.RobustScale
	}
	if c.MaxIterations <= 0 {
		c.MaxIterations = d.MaxIterations
	}
	if c.MinGapMetres <= 0 {
		c.MinGapMetres = d.MinGapMetres
	}
	if c.SearchRadiusMetres <= 0 {
		c.SearchRadiusMetres = d.SearchRadiusMetres
	}
	if c.DriftFraction <= 0 {
		c.DriftFraction = d.DriftFraction
	}
	if c.MaxCandidates <= 0 {
		c.MaxCandidates = d.MaxCandidates
	}
	if c.SubmapMetres <= 0 {
		c.SubmapMetres = d.SubmapMetres
	}
	if c.SubmapKeyframes <= 0 {
		c.SubmapKeyframes = d.SubmapKeyframes
	}
	if c.Search.CoarseStep <= 0 {
		c.Search = d.Search
	}
	if c.MinScore <= 0 {
		c.MinScore = d.MinScore
	}
	if c.ConsistentMatches <= 0 {
		c.ConsistentMatches = d.ConsistentMatches
	}
	if c.MaxChi2 <= 0 {
		c.MaxChi2 = d.MaxChi2
	}
	if c.RedrawMetres <= 0 {
		c.RedrawMetres = d.RedrawMetres
	}
	if c.RedrawDegrees <= 0 {
		c.RedrawDegrees = d.RedrawDegrees
	}
}

// blindFactor is how much less a step is trusted when it had no odometry and
// came from constant velocity. The graph should put a correction there first.
const blindFactor = 10

// storedSweep is one sweep that was drawn into the map, kept so that it can be
// drawn again from wherever its keyframe turns out to have been.
type storedSweep struct {
	kf     int  // the keyframe it hangs off
	offset pose // where it was taken, in that keyframe's frame
	at     time.Time

	// float32 because a sweep is kept for the length of the run, and the
	// sensor is not good to a micrometre.
	angles, distances []float32
	valid             []bool
}

// keyframe is a node of the pose graph.
type keyframe struct {
	id int

	// pose is where the keyframe's sweep was taken, in the map frame: the
	// front end's estimate until the graph is optimised, then the graph's.
	pose pose

	// path is how far the vehicle had driven, from the first sweep, when it
	// was taken. The distance between two keyframes along the path, not
	// across the map, says how much drift there can be between them.
	path float64

	// cov is how far the motion sensors' account of the motion from the
	// previous keyframe to this one may be off, in the previous keyframe's
	// frame. The first keyframe has none: it is the origin of the map.
	cov mat3

	// rel is where the front end put this keyframe as seen from the previous
	// one: the measurement of the sequential edge between them. It is kept
	// apart from pose because pose is what optimising changes, and the
	// measurement must not change with it. Read back from two optimised poses
	// it would be the graph's own answer, and each loop closed would be
	// written into the chain as if the wheels had measured it.
	rel pose

	// blind counts the steps since the previous keyframe that had no
	// odometry.
	blind int

	sweep storedSweep // its own; its offset is zero
}

// memory is the keyframes and the sweeps between them. It is the Traits' and
// is used under its lock.
type memory struct {
	cfg    LoopConfig
	frames []keyframe
	sweeps []storedSweep

	// Since the last keyframe: the covariance of the motion from it to the
	// latest sweep, in its frame, and the steps that had no odometry.
	cov   mat3
	blind int
	path  float64
}

func newMemory(cfg LoopConfig) *memory { return &memory{cfg: cfg} }

// placed is how the front end placed a sweep, which is what says how far the
// step to it may be off.
type placed int

const (
	// byConstantVelocity: no odometry, and a view that did not fix the pose
	// either — the least trusted.
	byConstantVelocity placed = iota
	// byWheels: the wheels, or gyrodometry, and nothing from the walls.
	byWheels
	// byWallsHeading: a corridor. The walls set the heading and the wheels
	// the position along it.
	byWallsHeading
	// byMatch: the map fixed the whole pose.
	byMatch
)

// minWallStepRadians is the least heading uncertainty a step the walls placed
// has. It is far below the matcher's fine step of 0.5°: each sweep is set
// against the same walls as the one before, so its error is not a new one
// added to the last but mostly the same one again.
const minWallStepRadians = 0.01 * math.Pi / 180

// step accounts for the motion from one sweep to the next, from and to in the
// map frame, the second placed as how says.
//
// The covariance of the motion since the last keyframe, r, grows as the
// covariance of a composition does:
//
//	r ∘ δ:   Σ ← J Σ Jᵀ + R(r.θ) Σ_δ R(r.θ)ᵀ
//
// J is the derivative of r ∘ δ with respect to r: a heading error at r swings
// every step after it sideways, which is how an uncertain heading becomes an
// uncertain position. Σ_δ is the step's own covariance from OdometryNoise,
// turned into the keyframe's frame — which stepCovariance does when it is
// given the heading.
//
// Where the walls placed the sweep, the step's heading is theirs, not the
// wheels'. Down a straight corridor the walls hold it sweep after sweep, so
// it hardly wanders; what drift there is comes in at the corners, where a new
// corridor is drawn from a turn. Weighing those steps as the wheels' 1.5°/m
// made every keyframe of a straight corridor look free to turn half a degree,
// and on 6 November the optimiser spread the loop's 1.5° along the top
// corridor as a curve: one straight wall, drawn from keyframes 0.7° apart,
// came out twice. So there the heading's noise is MatchedHeadingDegreesPerMeter
// and MatchedTurnFraction of the turn. Down a corridor, where the wheels alone
// say how far, the along-track noise is CorridorAlongFraction: their error is
// mostly a scale error, the same every metre, which an independent error per
// step understates.
//
// Σ keeps growing either way. 1b8fd2a reset it to the match's accuracy
// instead, which made the chain so stiff that a right loop could not be met
// and every one was refused.
func (m *memory) step(from, to pose, how placed) {
	if len(m.frames) == 0 {
		return
	}
	last := m.frames[len(m.frames)-1].pose
	r := compose(inverse(last), from)
	delta := compose(inverse(from), to)

	c, s := math.Cos(r.theta), math.Sin(r.theta)
	j := mat3{
		{1, 0, -s*delta.x - c*delta.y},
		{0, 1, c*delta.x - s*delta.y},
		{0, 0, 1},
	}
	noise := m.cfg.Noise
	if how == byWallsHeading {
		noise.AlongFraction = math.Max(noise.AlongFraction, m.cfg.CorridorAlongFraction)
	}
	stepCov := noise.stepCovariance(delta, r.theta)
	if how == byWallsHeading || how == byMatch {
		ds := math.Hypot(delta.x, delta.y)
		turn := math.Max(minWallStepRadians,
			m.cfg.MatchedHeadingDegreesPerMeter*math.Pi/180*ds+m.cfg.MatchedTurnFraction*math.Abs(delta.theta))
		stepCov[2][2] = turn * turn
	}
	if how == byConstantVelocity {
		for i := range stepCov {
			for k := range stepCov[i] {
				stepCov[i][k] *= blindFactor * blindFactor
			}
		}
		m.blind++
	}
	m.cov = j.mul(m.cov).mul(j.transpose()).add(stepCov)
	m.path += math.Hypot(delta.x, delta.y)
}

// keep is the sweep that was just drawn into the map from at. It starts a new
// keyframe if the vehicle has come far enough from the last one, and is
// otherwise kept hanging off that one.
func (m *memory) keep(at pose, sw *forms.ScanA_v1a) {
	s := storedSweep{
		at:        sw.Timestamp,
		angles:    make([]float32, len(sw.Angles)),
		distances: make([]float32, len(sw.Distances)),
		valid:     append([]bool(nil), sw.Valid...),
	}
	for i, v := range sw.Angles {
		s.angles[i] = float32(v)
	}
	for i, v := range sw.Distances {
		s.distances[i] = float32(v)
	}

	var rel pose
	if n := len(m.frames); n > 0 {
		last := m.frames[n-1]
		rel = compose(inverse(last.pose), at)
		far := math.Hypot(rel.x, rel.y) >= m.cfg.KeyframeTravelMetres
		turned := math.Abs(rel.theta)*180/math.Pi >= m.cfg.KeyframeTurnDegrees
		if !far && !turned {
			s.kf = last.id
			s.offset = rel
			m.sweeps = append(m.sweeps, s)
			m.thin()
			return
		}
	}

	id := len(m.frames)
	s.kf = id
	m.frames = append(m.frames, keyframe{id: id, pose: at, path: m.path, cov: m.cov, rel: rel, blind: m.blind, sweep: s})
	m.cov, m.blind = mat3{}, 0
}

// thin halves the older half of the sweeps once there are more than
// MaxSweeps: every other one of them goes. The map they draw is then a little
// sparser where the vehicle was longest ago, and nowhere else; the keyframes'
// own sweeps are not among them and stay.
func (m *memory) thin() {
	if len(m.sweeps) <= m.cfg.MaxSweeps {
		return
	}
	// Into a new slice, not in place: a snapshot taken for the loop closer
	// may still be reading the old one.
	half := len(m.sweeps) / 2
	kept := make([]storedSweep, 0, len(m.sweeps))
	for i, s := range m.sweeps {
		if i < half && i%2 == 1 {
			continue
		}
		kept = append(kept, s)
	}
	m.sweeps = kept
}

// sweepPose is where a kept sweep was taken, given where its keyframe is now.
func (m *memory) sweepPose(s storedSweep) pose {
	return compose(m.frames[s.kf].pose, s.offset)
}

// snapshot is a copy of the memory that the loop closer can read without the
// lock while the front end goes on adding to the original.
//
// The keyframes are copied, because their poses are what an optimisation
// changes. The sweeps are not: a kept sweep is never changed once kept, the
// front end only appends past the end of the snapshot's slice, and thin makes
// a new slice rather than moving sweeps about in the old one.
func (m *memory) snapshot() *memory {
	return &memory{
		cfg:    m.cfg,
		frames: append([]keyframe(nil), m.frames...),
		sweeps: m.sweeps[:len(m.sweeps):len(m.sweeps)],
		path:   m.path,
	}
}

// draw is one kept sweep, drawn into g from at.
func (s storedSweep) draw(g *grid, at pose) {
	a, d, v := s.float64s()
	g.integrate(at, a, d, v)
}

// drawAll draws every kept sweep into g from where its keyframe now is: the
// map as the keyframes' poses say it should look.
func (m *memory) drawAll(g *grid) {
	for _, f := range m.frames {
		f.sweep.draw(g, f.pose)
	}
	for _, s := range m.sweeps {
		s.draw(g, m.sweepPose(s))
	}
}

// since is what the memory has kept that snap had not: the keyframes after
// snap's last, and the sweeps hanging off snap's last keyframe or later that
// came after snap's. The sweeps of the last keyframe are not thinned between
// the two — thinning takes only from the older half — so counting them is
// enough to tell the new from the old.
func (m *memory) since(snap *memory) (frames []int, sweeps []storedSweep) {
	for k := len(snap.frames); k < len(m.frames); k++ {
		frames = append(frames, k)
	}
	if len(snap.frames) == 0 {
		return frames, m.sweeps
	}
	last := len(snap.frames) - 1
	old := len(snap.sweepsOf(last, last))
	tail := m.sweepsOf(last, len(m.frames))
	if old > len(tail) {
		old = len(tail)
	}
	return frames, tail[old:]
}

// sweepsOf is the kept sweeps hanging off keyframes from to to, inclusive.
// The sweeps are in the order they were taken, so in the order of their
// keyframes, and the range is found by bisection.
func (m *memory) sweepsOf(from, to int) []storedSweep {
	lo := sort.Search(len(m.sweeps), func(k int) bool { return m.sweeps[k].kf >= from })
	hi := sort.Search(len(m.sweeps), func(k int) bool { return m.sweeps[k].kf > to })
	return m.sweeps[lo:hi]
}
