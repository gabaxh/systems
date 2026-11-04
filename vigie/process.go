/*******************************************************************************
 * Copyright (c) 2026 Synecdoque
 *
 * The software is licensed under the MIT License. See the LICENSE file in this
 * repository for details.
 *
 * Contributors:
 *   Jan A. van Deventer, Luleå - initial implementation
 *   Gabriel Axheim Gustafsson, Luleå - the zero measured at every standstill,
 *     gaps, and the mount from gravity
 ***************************************************************************SDG*/

package main

import (
	"log"
	"math"
	"sync"
	"time"
)

// The vehicle frame is ISO 8855, as everywhere else on the Artitrax: x forward,
// y to the left, z up, its origin on the ground below the reference point the
// mount is measured from. Angles grow to the left; pitch is positive nose down
// and roll positive with the right side down, which is what right-handed
// rotations about forward and left give.

// mount places the lidar on the vehicle: where it is, and how it is turned. It
// is assumed that the Mid-360's own frame is x forward, y left, z up, as Livox
// draws it; if the vehicle shows otherwise, the yaw, pitch and roll correct it.
type Mount struct {
	XMetres      float64 `json:"xMetres"`
	YMetres      float64 `json:"yMetres"`
	ZMetres      float64 `json:"zMetres"` // the lidar's height above the ground
	RollDegrees  float64 `json:"rollDegrees"`
	PitchDegrees float64 `json:"pitchDegrees"`
	YawDegrees   float64 `json:"yawDegrees"`
}

// rotation is the lidar-to-vehicle rotation, Rz(yaw)·Ry(pitch)·Rx(roll).
type rotation [3][3]float64

func (m Mount) rotation() rotation {
	r, p, y := m.RollDegrees*math.Pi/180, m.PitchDegrees*math.Pi/180, m.YawDegrees*math.Pi/180
	cr, sr, cp, sp, cy, sy := math.Cos(r), math.Sin(r), math.Cos(p), math.Sin(p), math.Cos(y), math.Sin(y)
	return rotation{
		{cy * cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr},
		{sy * cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr},
		{-sp, cp * sr, cp * cr},
	}
}

// transpose is the rotation back, from the vehicle to the lidar.
func (r rotation) transpose() rotation {
	var t rotation
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			t[i][j] = r[j][i]
		}
	}
	return t
}

func (r rotation) apply(v [3]float64) [3]float64 {
	return [3]float64{
		r[0][0]*v[0] + r[0][1]*v[1] + r[0][2]*v[2],
		r[1][0]*v[0] + r[1][1]*v[1] + r[1][2]*v[2],
		r[2][0]*v[0] + r[2][1]*v[1] + r[2][2]*v[2],
	}
}

// SliceConfig chooses the points that make the horizontal scan.
type SliceConfig struct {
	// The band, in metres above the ground, whose points make the scan. A
	// band rather than a plane: the Mid-360 does not sweep a plane, and a
	// wall is a wall at any height a vehicle could hit.
	MinHeightMetres float64 `json:"minHeightMetres"`
	MaxHeightMetres float64 `json:"maxHeightMetres"`
	BinDegrees      float64 `json:"binDegrees"`
	MinRangeMetres  float64 `json:"minRangeMetres"`
	MaxRangeMetres  float64 `json:"maxRangeMetres"`
	// AccumulateMs is how long points are gathered for one scan. The Mid-360
	// covers its field of view gradually, so a tenth of a second is sparse.
	AccumulateMs int `json:"accumulateMs"`
}

// Box is a region of the vehicle frame, in metres. A zero box is no box.
type Box struct {
	MinX float64 `json:"minX"`
	MaxX float64 `json:"maxX"`
	MinY float64 `json:"minY"`
	MaxY float64 `json:"maxY"`
}

func (b Box) contains(x, y float64) bool {
	if b.MinX == 0 && b.MaxX == 0 && b.MinY == 0 && b.MaxY == 0 {
		return false
	}
	return x >= b.MinX && x <= b.MaxX && y >= b.MinY && y <= b.MaxY
}

// ClearanceConfig is the corridor the vehicle will drive through.
type ClearanceConfig struct {
	HalfWidthMetres float64 `json:"halfWidthMetres"`
	MinHeightMetres float64 `json:"minHeightMetres"` // above the ground: lower is the floor
	MaxHeightMetres float64 `json:"maxHeightMetres"` // the vehicle's height and a margin
}

// sweep is one horizontal scan, in the vehicle frame: angle zero straight
// ahead, angles growing to the left, distances from the reference point.
type sweep struct {
	angles    []float64
	distances []float64
	valid     []bool
	taken     time.Time
	duration  time.Duration
}

type timedPoint struct {
	at      time.Time
	x, y, z float64 // vehicle frame
}

// cloud keeps the last window of points in the vehicle frame, and what the
// scan, the clearance and the quality are made from.
type cloud struct {
	mu      sync.Mutex
	points  []timedPoint
	returns []int // per packet: how many came back
	empties []int // and how many did not
	stamps  []time.Time
	window  time.Duration
}

func (c *cloud) add(at time.Time, pts []timedPoint, empty int) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.points = append(c.points, pts...)
	c.returns = append(c.returns, len(pts))
	c.empties = append(c.empties, empty)
	c.stamps = append(c.stamps, at)
	c.trim(at)
}

// trim drops what is older than the window.
func (c *cloud) trim(now time.Time) {
	cut := now.Add(-c.window)
	i := 0
	for i < len(c.points) && c.points[i].at.Before(cut) {
		i++
	}
	c.points = c.points[i:]
	j := 0
	for j < len(c.stamps) && c.stamps[j].Before(cut) {
		j++
	}
	c.returns, c.empties, c.stamps = c.returns[j:], c.empties[j:], c.stamps[j:]
}

func (c *cloud) snapshot() ([]timedPoint, int, int, time.Time, time.Time) {
	c.mu.Lock()
	defer c.mu.Unlock()
	pts := append([]timedPoint(nil), c.points...)
	ret, emp := 0, 0
	for i := range c.returns {
		ret += c.returns[i]
		emp += c.empties[i]
	}
	var first, last time.Time
	if len(c.stamps) > 0 {
		first, last = c.stamps[0], c.stamps[len(c.stamps)-1]
	}
	return pts, ret, emp, first, last
}

// slice makes a horizontal scan of the window: in each bin of direction, the
// nearest point within the height band, outside the vehicle's own body.
func slice(pts []timedPoint, cfg SliceConfig, body Box, first, last time.Time) sweep {
	n := int(math.Round(360 / cfg.BinDegrees))
	sw := sweep{
		angles:    make([]float64, n),
		distances: make([]float64, n),
		valid:     make([]bool, n),
		taken:     last,
		duration:  last.Sub(first),
	}
	for i := range sw.angles {
		sw.angles[i] = -180 + (float64(i)+0.5)*cfg.BinDegrees
	}
	for _, p := range pts {
		if p.z < cfg.MinHeightMetres || p.z > cfg.MaxHeightMetres || body.contains(p.x, p.y) {
			continue
		}
		d := math.Hypot(p.x, p.y)
		if d < cfg.MinRangeMetres || d > cfg.MaxRangeMetres {
			continue
		}
		a := math.Atan2(p.y, p.x) * 180 / math.Pi
		i := int(math.Floor((a + 180) / cfg.BinDegrees))
		if i < 0 {
			i = 0
		}
		if i >= n {
			i = n - 1
		}
		if !sw.valid[i] || d < sw.distances[i] {
			sw.distances[i], sw.valid[i] = d, true
		}
	}
	return sw
}

// clearanceAhead is the nearest return in the corridor ahead, measured forward
// from the reference point. ok is false when nothing in the corridor came
// back: a lidar that saw nothing there cannot say whether the way is clear or
// it was blind, and a consumer must not be handed a distance for either.
func clearanceAhead(pts []timedPoint, cfg ClearanceConfig, body Box) (float64, bool) {
	best := math.Inf(1)
	for _, p := range pts {
		if p.x <= 0 || math.Abs(p.y) > cfg.HalfWidthMetres || p.z < cfg.MinHeightMetres || p.z > cfg.MaxHeightMetres {
			continue
		}
		if body.contains(p.x, p.y) {
			continue
		}
		if p.x < best {
			best = p.x
		}
	}
	if math.IsInf(best, 1) {
		return 0, false
	}
	return best, true
}

//-------------------------------------The IMU

// motion is what the IMU says, in the vehicle frame.
//
// The gyroscope reads something even when nothing turns. On the Artitrax's
// Mid-360 that was −2.4, +0.3 and −1.4°/s about x, y and z, steady over
// minutes, against a noise of 0.05 to 0.1°/s. Integrated as it is, the heading
// drifts 86° a minute. So its zero is measured first: the samples of the first
// zeroFor are averaged, the vehicle standing still, and subtracted from every
// sample after. If the samples spread more than a still vehicle's would, the
// vehicle was moving; the measurement starts again, and the rate and heading
// stay unavailable until it succeeds.
//
// The zero does not stay put. From a cold start the same Mid-360's zero about
// the vertical moved 1.9°/s over its first hundred minutes, fastest by about
// 0.055°/s a minute, as it warmed (two hours at rest, October 2026). So it is
// not measured once: every window of zeroFor in which the vehicle stands still
// measures it again. A steady slow turn spreads as little as a standstill
// does, so a window is also judged by its mean. The first zero may be at most
// maxFirst about the vertical, which is above the cold zero the vehicle has
// shown; a later one may differ from the last by at most maxStep, plus drift
// for every minute since that one was measured, since the zero moves with
// time and not only with noise. A window further off than that is the vehicle
// turning slowly, and counted, not taken.
type motion struct {
	mu      sync.Mutex
	samples []timedIMU
	window  time.Duration
	limits  stillLimits

	// The mount, to say which way up the lidar itself is: see sayMount.
	rot   rotation
	mount Mount

	heading     float64 // degrees, integrated since the zero was known
	lastStamp   uint64  // the previous sample's stamp, ns on this host's clock
	lastRate    float64 // its rate about the vertical, zero removed, rad/s
	got         bool
	integrating bool
	recoverTo   uint64 // the heading has just lost a gap until this stamp
	gaps        int

	zeroFor time.Duration
	zero    [3]float64 // rad/s
	known   bool
	zeroAt  uint64 // stamp of the window that last measured it
	stills  int    // windows taken as standing still
	offZero int    // windows that were still but too far from the zero
	win     stillWindow
}

// stillLimits is what a standstill may look like, in rad/s and g. The
// defaults are what the gyro characterization measured on the vehicle, motors
// on and holding, with a margin.
type stillLimits struct {
	gyroStd  float64 // the most any axis may spread in a window
	accel    float64 // how far the accelerometer's magnitude may be from 1 g
	maxFirst float64 // the largest first zero about the vertical
	maxStep  float64 // how far a later zero may be from the last
	drift    float64 // and further, per minute since the last
	maxGap   time.Duration
	recover  time.Duration // how long the heading is withheld after a gap
}

// stillWindow accumulates one window of raw samples.
type stillWindow struct {
	start          uint64
	n              float64
	sumG, sumG2    [3]float64
	sumA           [3]float64
	sumAccelLength float64
}

// zeroKnown is whether the gyroscope's zero has been measured.
func (m *motion) zeroKnown() bool {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.known || m.zeroFor == 0
}

// observe takes one raw sample into the current window, and judges the window
// once it is long enough. Windows do not overlap: one that saw the vehicle
// start to move is thrown away whole.
func (m *motion) observe(s timedIMU) {
	w := &m.win
	if w.n == 0 {
		w.start = s.stamp
	}
	w.n++
	for k := 0; k < 3; k++ {
		w.sumG[k] += s.gyro[k]
		w.sumG2[k] += s.gyro[k] * s.gyro[k]
		w.sumA[k] += s.acc[k]
	}
	w.sumAccelLength += math.Sqrt(s.acc[0]*s.acc[0] + s.acc[1]*s.acc[1] + s.acc[2]*s.acc[2])
	if s.stamp < w.start || time.Duration(s.stamp-w.start) < m.zeroFor {
		return
	}
	m.judge(*w, s.stamp)
	m.win = stillWindow{}
}

// judge decides whether a full window was a standstill, and if so takes the
// zero from it.
func (m *motion) judge(w stillWindow, at uint64) {
	var mean, spread, acc [3]float64
	worst := 0.0
	for k := 0; k < 3; k++ {
		mean[k] = w.sumG[k] / w.n
		spread[k] = math.Sqrt(math.Max(0, w.sumG2[k]/w.n-mean[k]*mean[k]))
		acc[k] = w.sumA[k] / w.n
		worst = math.Max(worst, spread[k])
	}
	l := m.limits
	if worst > l.gyroStd || math.Abs(w.sumAccelLength/w.n-1) > l.accel {
		if !m.known {
			log.Printf("vigie: the vehicle moved while the gyroscope's zero was measured (spread %.2f, %.2f, %.2f °/s); measuring again",
				spread[0]*180/math.Pi, spread[1]*180/math.Pi, spread[2]*180/math.Pi)
		}
		return
	}
	if !m.known && math.Abs(mean[2]) > l.maxFirst {
		log.Printf("vigie: the gyroscope read %.2f°/s about the vertical while otherwise still — more than a zero should, "+
			"so the vehicle is taken to be turning slowly; measuring again", mean[2]*180/math.Pi)
		return
	}
	if m.known {
		minutes := time.Duration(at - m.zeroAt).Minutes()
		if math.Abs(mean[2]-m.zero[2]) > l.maxStep+l.drift*minutes {
			m.offZero++
			return
		}
	}
	first := !m.known
	m.zero, m.known, m.zeroAt = mean, true, at
	m.stills++
	if first {
		log.Printf("vigie: the gyroscope's zero is %.3f, %.3f, %.3f °/s about x, y, z; it is subtracted from every reading, "+
			"and measured again whenever the vehicle stands still",
			mean[0]*180/math.Pi, mean[1]*180/math.Pi, mean[2]*180/math.Pi)
		m.sayMount(acc)
	}
}

// sayMount says, once, which way up the lidar is in its own frame and what
// roll and pitch would level it. The yaw rate is the gyroscope's rate about
// the vehicle's vertical only if the mount says truly how the lidar is turned:
// tilted 11° and taken as level, every turn reads 2 % short and a fifth of
// every roll reads as turning. Gravity gives roll and pitch, not yaw: that the
// scan shows a person standing ahead at 0° is how the yaw is checked.
func (m *motion) sayMount(accVehicle [3]float64) {
	n := math.Sqrt(accVehicle[0]*accVehicle[0] + accVehicle[1]*accVehicle[1] + accVehicle[2]*accVehicle[2])
	if n == 0 {
		return
	}
	upVehicle := [3]float64{accVehicle[0] / n, accVehicle[1] / n, accVehicle[2] / n}
	rot := m.rot
	if rot == (rotation{}) {
		rot = Mount{}.rotation()
	}
	up := rot.transpose().apply(upVehicle)
	roll, pitch := levelling(up)
	tilted := math.Acos(math.Max(-1, math.Min(1, upVehicle[2]))) * 180 / math.Pi
	log.Printf("vigie: up, in the lidar's own frame, is [%.4f, %.4f, %.4f]; with yawDegrees %.1f, "+
		"rollDegrees %.2f and pitchDegrees %.2f would level it", up[0], up[1], up[2], m.mount.YawDegrees, roll, pitch)
	if tilted > 1 {
		log.Printf("vigie: as mounted, the vehicle reads %.1f° from level. If it stood on level ground, set the "+
			"mount's roll and pitch above: until then every turn reads short and part of every roll reads as turning", tilted)
	}
}

// levelling is the roll and pitch, in degrees, that turn up — measured in the
// lidar's own frame — to the vehicle's vertical, for the mount's order of
// rotations Rz(yaw)·Ry(pitch)·Rx(roll). The yaw does not change where up is.
func levelling(up [3]float64) (roll, pitch float64) {
	roll = math.Atan2(up[1], up[2])
	pitch = math.Atan2(-up[0], math.Hypot(up[1], up[2]))
	return roll * 180 / math.Pi, pitch * 180 / math.Pi
}

type timedIMU struct {
	at    time.Time  // when it arrived
	stamp uint64     // when the lidar took it, ns, put on this host's clock (clock.go)
	gyro  [3]float64 // rad/s
	acc   [3]float64 // g
}

func (m *motion) add(s timedIMU) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.limits == (stillLimits{}) {
		m.limits = defaultConfig().stillLimits()
	}
	// A hole longer than maxGap is a lost stream, not a stretch of turning, and
	// a clock that went back by more than a second is a restart. Neither is
	// integrated across, and for a while after one the heading is withheld, so
	// that whoever differences two readings of it cannot do so across the hole
	// unawares. A small step back is the clock map finding a packet that waited
	// less (clock.go): that sample is not integrated, and nothing is lost.
	restart := m.got && s.stamp+uint64(time.Second) < m.lastStamp
	lost := restart || m.got && s.stamp > m.lastStamp && time.Duration(s.stamp-m.lastStamp) > m.limits.maxGap
	if lost {
		m.win = stillWindow{}
		if m.integrating {
			m.gaps++
			m.recoverTo = s.stamp + uint64(m.limits.recover)
		}
	}
	if m.zeroFor > 0 {
		m.observe(s)
	}
	for k := 0; k < 3; k++ {
		s.gyro[k] -= m.zero[k]
	}
	// Integrate the yaw rate into a heading, sample by sample, by the
	// trapezoid rule, on the lidar's stamps put on this host's clock: arrival
	// times carry the network's jitter, the stamps do not, and this host's
	// clock is the one the wheels are stamped on.
	if m.known || m.zeroFor == 0 {
		rate := s.gyro[2]
		if m.integrating && !lost && s.stamp > m.lastStamp {
			dt := float64(s.stamp-m.lastStamp) / 1e9
			m.heading += 0.5 * (m.lastRate + rate) * dt * 180 / math.Pi
		}
		m.lastRate, m.integrating = rate, true
	}
	if !m.got || s.stamp > m.lastStamp || restart {
		m.lastStamp = s.stamp
	}
	m.got = true
	m.samples = append(m.samples, s)
	cut := s.at.Add(-m.window)
	i := 0
	for i < len(m.samples) && m.samples[i].at.Before(cut) {
		i++
	}
	m.samples = m.samples[i:]
}

// averages are the mean gyro and accelerometer over the window, the newest
// sample's time, and whether there was any.
func (m *motion) averages() (gyro, acc [3]float64, heading float64, at time.Time, ok bool) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if len(m.samples) == 0 {
		return gyro, acc, m.heading, at, false
	}
	for _, s := range m.samples {
		for k := 0; k < 3; k++ {
			gyro[k] += s.gyro[k]
			acc[k] += s.acc[k]
		}
	}
	n := float64(len(m.samples))
	for k := 0; k < 3; k++ {
		gyro[k] /= n
		acc[k] /= n
	}
	return gyro, acc, m.heading, m.samples[len(m.samples)-1].at, true
}

// imuState is what the services and the log say about the gyroscope beyond
// its averages.
type imuState struct {
	taken      time.Time // the newest sample's stamp, on this host's clock
	recovering bool      // just after a gap, whose turn is lost
	zeroKnown  bool
	zero       float64   // about the vertical, °/s
	zeroAt     time.Time // when it was last measured
	stills     int
	offZero    int
	gaps       int
}

func (m *motion) state() imuState {
	m.mu.Lock()
	defer m.mu.Unlock()
	return imuState{
		taken:      time.Unix(0, int64(m.lastStamp)),
		recovering: m.lastStamp < m.recoverTo,
		zeroKnown:  m.known,
		zero:       m.zero[2] * 180 / math.Pi,
		zeroAt:     time.Unix(0, int64(m.zeroAt)),
		stills:     m.stills,
		offZero:    m.offZero,
		gaps:       m.gaps,
	}
}

// tilt is pitch and roll from the accelerometer, in degrees. At rest it reads
// the reaction to gravity, one g upward; tilted nose down by θ, the upward
// vector seen from the vehicle leans back, so its x component is −sin θ.
func tilt(acc [3]float64) (pitch, roll float64) {
	pitch = math.Atan2(-acc[0], math.Hypot(acc[1], acc[2])) * 180 / math.Pi
	roll = math.Atan2(acc[1], acc[2]) * 180 / math.Pi
	return pitch, roll
}
