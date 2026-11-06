/*******************************************************************************
 * Copyright (c) 2026 Synecdoque
 *
 * The software is licensed under the MIT License. See the LICENSE file in this
 * repository for details.
 *
 * Contributors:
 *   Jan A. van Deventer, Luleå - initial implementation
 ***************************************************************************SDG*/

package main

// The mapping and localization themselves: an occupancy grid in log-odds, and
// a correlative scan matcher that finds where a sweep must have been taken from
// for it to line up with the map so far.
//
// This is a SLAM *front end* and it is honest about being one. On its own it
// has no pose graph and no loop closure, so a long loop through a building
// will not snap shut when it returns to where it started; the drift
// accumulated on the way round simply stays. For a hallway driven up and back
// — which is the experiment the students are being given — that is the right
// amount of machinery.
//
// Loop closure was "a second system's worth of work, to be added when there
// is a loop to close". With the MID-360 there is one, and it is the back end
// in keyframes.go, posegraph.go and loops.go, off unless loop.enabled is set.
// It uses this file's grid and matcher as they are: matchSampled is match
// with its scores kept, and a submap is a grid like any other.

import (
	"fmt"
	"math"
	"os"

	"github.com/sdoque/mbaigo/forms"
)

// pose is where the sensor was, in the map frame. Angles in radians here;
// degrees only ever appear at the edges, in forms and configuration.
type pose struct {
	x, y, theta float64
}

// The log-odds a single observation is worth, and the clamp that stops any one
// cell from becoming so certain that later evidence cannot move it. A corridor
// that is remapped after a door opens has to be able to change its mind.
const (
	logFree  float32 = -0.4
	logOcc   float32 = 0.85
	logClamp float32 = 5.0
)

type grid struct {
	width, height int
	res           float64 // metres per cell
	// world coordinates of the centre of cell (0,0)
	originX, originY float64
	cells            []float32

	// field is the likelihood field the scan matcher scores against: the
	// occupancy evidence spread over its neighbours.
	//
	// Scoring against cells directly does not work, and the way it fails is
	// instructive. One sweep marks only the cells its own beams happened to
	// land on, so a wall enters the map as a dotted line. A later sweep then
	// scores highest where its endpoints fall back onto those same dots —
	// which is the pose the vehicle has already left. The matcher confidently
	// reports no motion, and the map stays beautiful and wrong.
	//
	// Spreading each occupied cell over a small neighbourhood makes a wall
	// continuous, so the along-wall direction stops carrying false evidence and
	// the discrimination comes from where it should: the ends of the corridor
	// and features like doorways.
	field []float32
	dirty bool

	// The cells anything has been drawn in, inclusive: the map's extent. It
	// is empty while ex1 < ex0. Everything outside it is unknown, and so is
	// the likelihood field there, which is why rebuildField looks no further.
	// On a grid sized for a building it is all that matters: on 5 November
	// the grid was 250 m across, 25 million cells, the drive covered a few
	// hundred thousand of them, and rebuilding the whole field after every
	// sweep took half a second on a laptop.
	ex0, ey0, ex1, ey1 int

	// matchRange is how far out the scan matcher trusts a return, in metres;
	// 0 means all of them.
	//
	// Far returns are worth less than they look. A sweep samples a wall at
	// intervals that grow with range, so distant wall is recorded as scattered
	// dots rather than a surface, and the matcher can score highest by landing
	// its endpoints back on those dots — fitting the sampling pattern instead
	// of the geometry. Distant points are also where a small heading error
	// throws an endpoint furthest. Ignoring them for matching costs nothing:
	// they are still integrated into the map.
	matchRange float64

	// freeRange is how far out a beam counts in full, in metres; 0 means all
	// of it. Beyond it, what the beam says — the cells it passes are clear,
	// the one it ends in is not — counts farWeight as much.
	//
	// A far beam is where a small heading error does the most damage to the
	// map. A beam looking 20 m down a corridor from a keyframe half a degree
	// out passes 17 cm to the side of where it should, and at a grazing angle
	// it clears every cell of the wall it runs along: −0.4 a beam against
	// +0.85 a hit, so three beams undo a wall that was seen once. On 6
	// November those beams cut white wedges through the inner walls into the
	// block they surround.
	//
	// Two harder limits were tried first. Drawing nothing beyond 12 m left the
	// front end too little map to match against, and in a replay of that
	// drive it ended 4.8 m out instead of 2.6 m. Drawing the far returns but
	// clearing nothing beyond 12 m left every stray far return in the map for
	// good, and on the next drive the end of a long corridor came out as a
	// cloud of black dots. At a quarter weight a far return is still drawn,
	// for the front end to match against, a few far beams through it clear
	// it, and a grazing beam needs four times as many to undo a wall.
	freeRange float64
}

func newGrid(widthM, heightM, res float64) *grid {
	w := int(math.Ceil(widthM / res))
	h := int(math.Ceil(heightM / res))
	return &grid{
		width: w, height: h, res: res,
		// The vehicle starts at the middle of the grid, because it may drive
		// in any direction and a map that only grows one way wastes three
		// quarters of itself.
		originX: -widthM / 2,
		originY: -heightM / 2,
		cells:   make([]float32, w*h),
		field:   make([]float32, w*h),
		ex0:     w, ey0: h, ex1: -1, ey1: -1,
	}
}

func (g *grid) cellOf(x, y float64) (int, int, bool) {
	cx := int(math.Floor((x - g.originX) / g.res))
	cy := int(math.Floor((y - g.originY) / g.res))
	if cx < 0 || cy < 0 || cx >= g.width || cy >= g.height {
		return 0, 0, false
	}
	return cx, cy, true
}

func (g *grid) at(cx, cy int) float32 {
	if cx < 0 || cy < 0 || cx >= g.width || cy >= g.height {
		return 0
	}
	return g.cells[cy*g.width+cx]
}

func (g *grid) add(cx, cy int, v float32) {
	if cx < 0 || cy < 0 || cx >= g.width || cy >= g.height {
		return
	}
	if cx < g.ex0 {
		g.ex0 = cx
	}
	if cx > g.ex1 {
		g.ex1 = cx
	}
	if cy < g.ey0 {
		g.ey0 = cy
	}
	if cy > g.ey1 {
		g.ey1 = cy
	}
	i := cy*g.width + cx
	n := g.cells[i] + v
	if n > logClamp {
		n = logClamp
	}
	if n < -logClamp {
		n = -logClamp
	}
	g.cells[i] = n
}

// integrate folds one sweep into the map from a known pose.
//
// Invalid points are skipped entirely rather than treated as free space out to
// maximum range. That is the conservative choice and it is deliberate: this
// system cannot tell "nothing within range" from "a surface that returned
// nothing", and a black door or a puddle marked as open floor is precisely the
// error that would drive a vehicle into it. An unknown cell is an honest cell.
// farWeight is how much a beam counts beyond freeRange. A variable only so
// that the drive replay (simulate_test.go) can try others.
var farWeight float32 = 0.25

func (g *grid) integrate(p pose, angles, distances []float64, valid []bool) {
	sx, sy, inside := g.cellOf(p.x, p.y)
	if !inside {
		return
	}
	prevX, prevY, prevOK := 0, 0, false
	for i := range angles {
		if i >= len(valid) || i >= len(distances) || !valid[i] {
			prevOK = false
			continue
		}
		wx, wy := project(p, angles[i], distances[i])
		ex, ey, ok := g.cellOf(wx, wy)
		if !ok {
			prevOK = false
			continue
		}
		w := float32(1)
		if g.freeRange > 0 && distances[i] > g.freeRange {
			// In full out to freeRange, at farWeight beyond it.
			w = farWeight
			if cx, cy, ok := g.cellOf(project(p, angles[i], g.freeRange)); ok {
				g.ray(sx, sy, cx, cy, logFree)
				g.ray(cx, cy, ex, ey, w*logFree)
			}
		} else {
			g.ray(sx, sy, ex, ey, logFree)
		}
		g.add(ex, ey, w*logOcc)

		// Join this return to the one before it when the two are close enough
		// to be the same surface. A sweep samples a wall at intervals that grow
		// with range, so recording only the endpoints leaves a wall as a row of
		// dots with gaps between them — and a later sweep then scores best by
		// dropping its endpoints back into those same dots, which is the pose
		// the vehicle has already left. Drawing the surface between two
		// adjacent returns is what makes a wall a wall.
		//
		// The distance test is what stops it inventing one: at a doorway or a
		// corner the range jumps, the two returns are far apart, and nothing is
		// drawn across the gap.
		if prevOK {
			if dx, dy := float64(ex-prevX)*g.res, float64(ey-prevY)*g.res; math.Hypot(dx, dy) <= surfaceGap {
				g.ray(prevX, prevY, ex, ey, w*logOcc)
			}
		}
		prevX, prevY, prevOK = ex, ey, true
	}
	g.dirty = true
}

// surfaceGap is how far apart two adjacent returns may be and still be treated
// as the same surface, in metres.
const surfaceGap = 0.30

// project turns one (bearing, range) pair into a world point, given the pose
// the sweep was taken from.
func project(p pose, angleDeg, distance float64) (float64, float64) {
	a := p.theta + angleDeg*math.Pi/180
	return p.x + distance*math.Cos(a), p.y + distance*math.Sin(a)
}

// fieldSigma is how far, in metres, a wall's influence reaches. A tenth of a
// metre is about the accuracy worth chasing from a moving vehicle.
const fieldSigma = 0.10

// fieldCutoff is where the Gaussian is small enough to call zero, in sigmas.
const fieldCutoff = 3

// rebuildField computes the likelihood field: for every cell, how close the
// nearest occupied cell is, passed through a Gaussian.
//
// It is a Euclidean distance transform and not a blur, and the difference is
// the whole reason the matcher works. Spreading each occupied cell over a fixed
// kernel leaves a wall as a row of bumps, because a sweep samples a wall at
// intervals that grow with range. The matcher then finds its best score where
// the new sweep's endpoints land back on the old sweep's bumps — which is the
// pose the vehicle has already left. It fits the sampling pattern rather than
// the geometry, and reports that the vehicle has not moved.
//
// A distance transform makes a wall uniformly attractive along its length. The
// along-wall direction then carries no false evidence, and the pose is
// constrained by the things that genuinely constrain it: the ends of a
// corridor, a doorway, a corner.
//
// Only the map's extent is worked on, with a margin of the cutoff round it.
// Nothing outside the extent was ever drawn, so no cell out there is within
// the cutoff of a wall and its field is zero, as it already is. Nor does
// clipping the distance transform to the region change any distance inside
// it: the chamfer path from a cell to its nearest wall stays within the box
// the two span, and both lie in the region. The extent only grows, so the
// field from an earlier rebuild never lies outside the region of a later one.
func (g *grid) rebuildField() {
	g.dirty = false
	if g.ex1 < g.ex0 || g.ey1 < g.ey0 {
		return // nothing drawn, and the field is zero everywhere
	}
	cutoffCells := float32(fieldCutoff * fieldSigma / g.res)
	margin := int(math.Ceil(float64(cutoffCells))) + 1
	x0, y0 := max(0, g.ex0-margin), max(0, g.ey0-margin)
	x1, y1 := min(g.width-1, g.ex1+margin), min(g.height-1, g.ey1+margin)
	rw, rh := x1-x0+1, y1-y0+1

	const big = float32(1e9)
	d := make([]float32, rw*rh)
	for y := 0; y < rh; y++ {
		row := g.cells[(y+y0)*g.width+x0 : (y+y0)*g.width+x0+rw]
		for x, v := range row {
			if v > 0 {
				d[y*rw+x] = 0
			} else {
				d[y*rw+x] = big
			}
		}
	}

	// Two-pass chamfer. The diagonal step is sqrt(2), which is what makes this
	// approximate Euclidean distance rather than city-block distance.
	const diag = float32(1.41421356)
	at := func(x, y int) float32 {
		if x < 0 || y < 0 || x >= rw || y >= rh {
			return big
		}
		return d[y*rw+x]
	}
	relax := func(x, y int, candidates ...float32) {
		i := y*rw + x
		for _, c := range candidates {
			if c < d[i] {
				d[i] = c
			}
		}
	}
	for y := 0; y < rh; y++ {
		for x := 0; x < rw; x++ {
			relax(x, y, at(x-1, y)+1, at(x, y-1)+1, at(x-1, y-1)+diag, at(x+1, y-1)+diag)
		}
	}
	for y := rh - 1; y >= 0; y-- {
		for x := rw - 1; x >= 0; x-- {
			relax(x, y, at(x+1, y)+1, at(x, y+1)+1, at(x+1, y+1)+diag, at(x-1, y+1)+diag)
		}
	}

	// Only near an obstacle is the Gaussian worth computing; beyond the cutoff
	// it is indistinguishable from zero.
	for y := 0; y < rh; y++ {
		for x := 0; x < rw; x++ {
			i := (y+y0)*g.width + x + x0
			dist := d[y*rw+x]
			if dist > cutoffCells {
				g.field[i] = 0
				continue
			}
			m := float64(dist) * g.res
			g.field[i] = float32(math.Exp(-m * m / (2 * fieldSigma * fieldSigma)))
		}
	}
}

// score is how well a sweep taken from this pose agrees with the map: the
// likelihood-field evidence under its endpoints. A sweep in the right place
// puts its endpoints on or beside cells already believed to be walls.
func (g *grid) score(p pose, angles, distances []float64, valid []bool, stride int) float64 {
	var total float64
	for i := 0; i < len(angles); i += stride {
		if i >= len(valid) || i >= len(distances) || !valid[i] {
			continue
		}
		if g.matchRange > 0 && distances[i] > g.matchRange {
			continue
		}
		wx, wy := project(p, angles[i], distances[i])
		cx, cy, ok := g.cellOf(wx, wy)
		if !ok {
			continue
		}
		total += float64(g.field[cy*g.width+cx])
	}
	return total
}

// matchResult is where a sweep was taken from, and whether the map was able to
// say so.
//
// sharp is the part that matters. A scan matcher always returns a best pose,
// and in a place that cannot constrain it — a straight corridor, an empty
// hall — that best pose is whichever one the sampling pattern happened to
// favour. Returning it without comment would be this project's recurring
// failure in a new costume: a thing that cannot see, reporting that all is
// well. So the peak is probed, and a flat one is declared flat.
type matchResult struct {
	at    pose
	score float64
	sharp bool

	// headingSharp is the heading alone. In a corridor the walls cannot say
	// how far along it the sweep was taken, but they say exactly which way the
	// sensor was facing: turn it a few degrees and the long walls no longer
	// line up. Sliding along parallel walls does not change their angle, so
	// the heading of a match that slid is still the heading.
	headingSharp bool
}

// probeDistance and probeAngle are how far the peak is nudged to see whether it
// is a peak at all; sharpRatio is how much the score must fall for it to count
// as one.
const (
	probeDistance = 0.10
	probeAngle    = 3 * math.Pi / 180
	sharpRatio    = 0.95
)

// axisSteps is how many directions, over half a turn, corridorAxis tries: 5°
// apart. The position along the axis it finds is the wheels', and a match may
// have slid along the corridor by up to the search range, 0.3 m; a direction
// 2.5° out turns that into a centimetre across it.
const axisSteps = 36

// corridorAxis is, for a match whose position is not sharp, the one direction
// the view leaves free: the axis of the corridor. ok is false when the score
// falls in no direction either, as in an open hall.
//
// sharpness probes only along the map's x and y, which says whether the
// position is fixed but not which way it is free. Down a corridor it is free
// along the corridor and fixed across it: the walls say exactly how far the
// sensor is from each of them. Without the axis the front end could only
// take the whole position from the wheels, and on the drive of 6 November,
// driven three times down the same corridor, each pass came out up to 0.4 m
// to the side of the one before, and every wall twice.
func (g *grid) corridorAxis(best pose, bestScore float64, angles, distances []float64, valid []bool) (axis float64, ok bool) {
	if bestScore <= 0 {
		return 0, false
	}
	// The lower of the two sides: a direction is free only if the score
	// holds both ways along it.
	probe := func(phi float64) float64 {
		c, s := probeDistance*math.Cos(phi), probeDistance*math.Sin(phi)
		a := g.score(pose{best.x + c, best.y + s, best.theta}, angles, distances, valid, 1)
		b := g.score(pose{best.x - c, best.y - s, best.theta}, angles, distances, valid, 1)
		return math.Min(a, b)
	}
	var scores [axisSteps]float64
	free := 0
	for k := range scores {
		scores[k] = probe(float64(k) * math.Pi / axisSteps)
		if scores[k] > scores[free] {
			free = k
		}
	}
	if scores[(free+axisSteps/2)%axisSteps] >= sharpRatio*bestScore {
		return 0, false // free across it too: nothing to hold the position
	}
	// A parabola through the best direction and its two neighbours puts the
	// axis between the steps. The directions wrap round at half a turn.
	lo, hi := scores[(free+axisSteps-1)%axisSteps], scores[(free+1)%axisSteps]
	offset := 0.0
	if den := lo - 2*scores[free] + hi; den < 0 {
		offset = math.Max(-0.5, math.Min(0.5, 0.5*(lo-hi)/den))
	}
	return (float64(free) + offset) * math.Pi / axisSteps, true
}

// match searches for the pose that best explains this sweep, starting from a
// prior — the previous pose advanced by whatever motion is believed to have
// happened since.
//
// Two passes: a coarse one over a wide window, then a fine one around whatever
// the coarse pass liked. Exhaustive search at fine resolution over the same
// window would be some hundreds of times the work for the same answer.
func (g *grid) match(prior pose, angles, distances []float64, valid []bool, cfg SearchConfig) matchResult {
	m, _, _ := g.matchKeeping(prior, angles, distances, valid, cfg, false)
	return m
}

// matchSampled is match, and also what the search's scores say about the pose:
// its information matrix, in the frame of the grid, and in how many
// independent directions the view fixes it at all (see matchInformation).
//
// The front end does not use it; the loop closure does, to decide whether a
// match against an older part of the map is an edge, and what it is worth.
// The pose and the sharpness are exactly those match returns.
func (g *grid) matchSampled(prior pose, angles, distances []float64, valid []bool, cfg SearchConfig) (matchResult, mat3, int) {
	m, wide, near := g.matchKeeping(prior, angles, distances, valid, cfg, true)
	if m.score <= 0 {
		return m, mat3{}, 0
	}
	info, seen := matchInformation(wide, near, cfg)
	return m, info, seen
}

// matchKeeping is the two-pass search behind match and matchSampled. With keep
// set, every pose each pass scored is kept, as its offset from where that pass
// was centred: the coarse pass's over the whole window, and the fine pass's
// around the peak.
func (g *grid) matchKeeping(prior pose, angles, distances []float64, valid []bool, cfg SearchConfig, keep bool) (matchResult, []matchSample, []matchSample) {
	if g.dirty {
		g.rebuildField()
	}

	search := func(centre pose, linStep, linRange, angStep, angRange float64, stride int, kept *[]matchSample) (pose, float64) {
		local, localScore := centre, math.Inf(-1)
		for dx := -linRange; dx <= linRange+1e-9; dx += linStep {
			for dy := -linRange; dy <= linRange+1e-9; dy += linStep {
				for dt := -angRange; dt <= angRange+1e-9; dt += angStep {
					c := pose{centre.x + dx, centre.y + dy, centre.theta + dt}
					s := g.score(c, angles, distances, valid, stride)
					if kept != nil {
						// At full stride, so that the two passes' scores are
						// on the same scale.
						*kept = append(*kept, matchSample{[3]float64{dx, dy, dt}, s * float64(stride)})
					}
					if s > localScore {
						local, localScore = c, s
					}
				}
			}
		}
		return local, localScore
	}

	var wide, near *[]matchSample
	if keep {
		wide, near = new([]matchSample), new([]matchSample)
	}
	coarse, _ := search(prior, cfg.CoarseStep, cfg.SearchRange, cfg.CoarseAngle, cfg.AngleRange, cfg.CoarseStride, wide)
	best, bestScore := search(coarse, cfg.FineStep, cfg.CoarseStep, cfg.FineAngle, cfg.CoarseAngle, 1, near)

	position, heading := g.sharpness(best, bestScore, angles, distances, valid)
	m := matchResult{at: best, score: bestScore, sharp: position && heading, headingSharp: heading}
	if !keep {
		return m, nil, nil
	}
	return m, *wide, *near
}

// sharpness nudges the winning pose in six directions and asks whether the
// score actually falls: four nudges in position and two in heading, answered
// apart. If the map is happy to put the sweep a hand's breadth away in some
// direction, then it does not know where the sweep was taken from in that
// direction, whatever the best score says.
//
// This is the aperture problem, and it is not a defect to be tuned away: two
// parallel walls genuinely say nothing about how far along them you are. The
// answer is odometry, which is why this system consumes a pose service when one
// exists. What it must never do is invent the missing degree of freedom. Nor,
// though, should it throw away the one the walls do fix: in that same corridor
// a turn of three degrees takes every wall off the map, and the heading is
// known even where the position is not.
func (g *grid) sharpness(best pose, bestScore float64, angles, distances []float64, valid []bool) (position, heading bool) {
	if bestScore <= 0 {
		return false, false
	}
	drops := func(p pose) bool {
		return g.score(p, angles, distances, valid, 1) < sharpRatio*bestScore
	}
	position = drops(pose{best.x + probeDistance, best.y, best.theta}) &&
		drops(pose{best.x - probeDistance, best.y, best.theta}) &&
		drops(pose{best.x, best.y + probeDistance, best.theta}) &&
		drops(pose{best.x, best.y - probeDistance, best.theta})
	heading = drops(pose{best.x, best.y, best.theta + probeAngle}) &&
		drops(pose{best.x, best.y, best.theta - probeAngle})
	return position, heading
}

// SearchConfig is the scan matcher's window, exposed because the right values
// depend on how fast the vehicle moves between sweeps.
type SearchConfig struct {
	SearchRange  float64 `json:"searchRangeMetres"`
	CoarseStep   float64 `json:"coarseStepMetres"`
	FineStep     float64 `json:"fineStepMetres"`
	AngleRange   float64 `json:"angleRangeRadians"`
	CoarseAngle  float64 `json:"coarseAngleRadians"`
	FineAngle    float64 `json:"fineAngleRadians"`
	CoarseStride int     `json:"coarseStride"`
}

func defaultSearch() SearchConfig {
	return SearchConfig{
		SearchRange:  0.30,
		CoarseStep:   0.05,
		FineStep:     0.0125,
		AngleRange:   12 * math.Pi / 180,
		CoarseAngle:  2 * math.Pi / 180,
		FineAngle:    0.5 * math.Pi / 180,
		CoarseStride: 3,
	}
}

// line is Bresenham between two cells, used to mark the free space a beam
// passed through on its way to whatever stopped it.
// ray adds v to every cell of the Bresenham line from (x0, y0) to (x1, y1),
// the endpoint excepted: for a beam the endpoint is the obstacle, not free
// space. It is line without the list. Drawing a sweep is 360 of these and a
// redraw is every sweep the run has kept, and building a slice for each one
// was nine-tenths of a redraw's time.
func (g *grid) ray(x0, y0, x1, y1 int, v float32) {
	dx := abs(x1 - x0)
	dy := -abs(y1 - y0)
	sx, sy := -1, -1
	if x0 < x1 {
		sx = 1
	}
	if y0 < y1 {
		sy = 1
	}
	err := dx + dy
	for x0 != x1 || y0 != y1 {
		g.add(x0, y0, v)
		e2 := 2 * err
		if e2 >= dy {
			err += dy
			x0 += sx
		}
		if e2 <= dx {
			err += dx
			y0 += sy
		}
	}
}

func line(x0, y0, x1, y1 int) [][2]int {
	var out [][2]int
	dx := abs(x1 - x0)
	dy := -abs(y1 - y0)
	sx, sy := -1, -1
	if x0 < x1 {
		sx = 1
	}
	if y0 < y1 {
		sy = 1
	}
	err := dx + dy
	for {
		if x0 == x1 && y0 == y1 {
			return out // the endpoint is the obstacle, not free space
		}
		out = append(out, [2]int{x0, y0})
		e2 := 2 * err
		if e2 >= dy {
			err += dy
			x0 += sx
		}
		if e2 <= dx {
			err += dx
			y0 += sy
		}
	}
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

//-------------------------------------Publishing the map

// form renders the grid as the wire representation. Log-odds become the three
// values a consumer has to tell apart, and a cell nothing has ever touched
// stays Unknown rather than becoming free.
func (g *grid) form() forms.MapA_v1a {
	m := forms.MapA_v1a{}
	m.NewForm()
	m.Width, m.Height = g.width, g.height
	m.Resolution = g.res
	m.OriginX, m.OriginY = g.originX, g.originY
	m.Frame = "map"
	m.DistanceUnit = "<http://qudt.org/vocab/unit/M>"
	m.Cells = make([]byte, len(g.cells))
	for i, v := range g.cells {
		switch {
		case v == 0:
			m.Cells[i] = forms.MapUnknown
		case v > 0:
			m.Cells[i] = forms.MapOccupied
		default:
			m.Cells[i] = forms.MapFree
		}
	}
	return m
}

// writePGM dumps the map as a grey image, which is the form a person can
// actually look at. The values follow the convention the robotics tools use:
// black is occupied, white is free, mid-grey is unseen.
func (g *grid) writePGM(path string) error {
	buf := make([]byte, 0, g.width*g.height+64)
	buf = append(buf, []byte(fmt.Sprintf("P5\n%d %d\n255\n", g.width, g.height))...)
	// PGM runs top row first; the grid's row 0 is the bottom, so it is written
	// in reverse, otherwise the hallway comes out mirrored.
	for y := g.height - 1; y >= 0; y-- {
		for x := 0; x < g.width; x++ {
			v := g.cells[y*g.width+x]
			switch {
			case v == 0:
				buf = append(buf, 205)
			case v > 0:
				buf = append(buf, 0)
			default:
				buf = append(buf, 254)
			}
		}
	}
	return os.WriteFile(path, buf, 0o644)
}
