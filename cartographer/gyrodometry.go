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

// Gyrodometry: the heading from the wheels, except when a gyro says they are
// wrong.
//
// Wheels and a gyro go wrong in opposite ways. The wheels have no drift once
// their track and sizes are calibrated, but a moment of slip, a bump or a
// scrubbing tyre is a turn that never happened. The gyro is right over a
// fraction of a second, but its bias integrates into a heading error without
// bound. Borenstein and Feng (ICRA 1996) take the heading from the wheels, and
// from the gyro only for the short intervals in which the two disagree by
// more than a threshold. The gyro is used so rarely and so briefly that its
// bias barely enters; while the two agree, a biased gyro changes nothing.
//
// The gyro is the vigie's, the Livox Mid-360's IMU: its heading, the yaw rate
// already integrated at 200 Hz, so that two readings give the turn between
// their timestamps exactly whatever this system's polling rate. The wheels and
// the heading are read every pollMs and kept with their own timestamps; every
// intervalMs the turn over the same interval is taken from each, interpolated
// at the same two instants. The decision is made per interval, a tenth of a
// second as in the paper, so a short slip is isolated rather than swallowing
// a whole sweep. The distance always comes from the wheels.
//
// Three things differ from the paper, each switchable to compare:
//   - the threshold is a rate times the interval rather than an angle per
//     sample, since the intervals are not all equally long;
//   - a hold: a wheel spinning in a pile disagrees for seconds, wandering
//     above and below the threshold, and without a hold the slipping wheels
//     come back on every interval that falls under it. holdMs 0 is the paper;
//   - missing data falls back rather than stops. With no heading, or a hole
//     in it (the vigie withholds its heading just after a gap in its stream),
//     the turn is the wheels', exactly as without gyrodometry.
//
// This only improves the prior the scan matcher starts from. Where the walls
// fix the pose, the walls still win.

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"sync"
	"time"
)

// encoderMetresPerCount is how far a wheel rolls per encoder count: 1.335 m of
// circumference over 81 920 counts. Two counts on one wheel are the smallest
// turn the wheels can fake, and the threshold never goes below it.
const encoderMetresPerCount = 1.335 / 81920

// fastestTurn is faster than the vehicle can turn: a heading that changes
// faster is the vigie restarting, not motion.
const fastestTurn = 300 * math.Pi / 180

// GyroConfig is gyrodometry's part of the cartographer's configuration.
type GyroConfig struct {
	// Enabled switches gyrodometry on. Absent means off: it is opted into
	// until it has been tuned on the vehicle, and off the cartographer reads
	// the wheels exactly as it always has.
	Enabled *bool `json:"enabled,omitempty"`

	// Sensor picks the vigie whose heading is read.
	Sensor map[string][]string `json:"sensor"`

	// PollMs is how often the wheels and the heading are read; IntervalMs how
	// long each interval the two are compared over is, the paper's T; LagMs
	// how far behind the present the intervals end, so that readings from
	// both sides of each end have arrived.
	PollMs     int `json:"pollMs"`
	IntervalMs int `json:"intervalMs"`
	LagMs      int `json:"lagMs"`

	// MaxAgeMs is the longest hole in the heading readings an interval may
	// span and still be compared.
	MaxAgeMs int `json:"maxAgeMs"`

	// ThresholdDegreesPerSecond is how far the gyro's and the wheels' rates
	// may differ before the gyro is believed: the paper's 0.125° per 0.1 s.
	ThresholdDegreesPerSecond float64 `json:"thresholdDegreesPerSecond"`

	// HoldMs is how long the gyro keeps the heading after the last interval
	// that disagreed. Absent means 300; 0 is the paper's rule.
	HoldMs *int `json:"holdMs,omitempty"`

	// Mode is "gyrodometry", or "odometry" or "gyro" to take the turn from
	// one source always, for comparing maps.
	Mode string `json:"mode"`

	// LogPath, when set, gets one CSV line per interval: what the threshold is
	// tuned from.
	LogPath string `json:"logPath"`
}

func (c GyroConfig) on() bool { return c.Enabled != nil && *c.Enabled }

func (c GyroConfig) hold() time.Duration {
	if c.HoldMs == nil {
		return 300 * time.Millisecond
	}
	return time.Duration(*c.HoldMs) * time.Millisecond
}

var off = false

func defaultGyro() GyroConfig {
	return GyroConfig{
		Enabled:                   &off,
		Sensor:                    map[string][]string{"Model": {"Livox Mid-360"}},
		PollMs:                    50,
		IntervalMs:                100,
		LagMs:                     100,
		MaxAgeMs:                  200,
		ThresholdDegreesPerSecond: 1.25,
		Mode:                      "gyrodometry",
	}
}

func applyGyroDefaults(c *GyroConfig) {
	d := defaultGyro()
	if c.Sensor == nil {
		c.Sensor = d.Sensor
	}
	if c.PollMs <= 0 {
		c.PollMs = d.PollMs
	}
	if c.IntervalMs <= 0 {
		c.IntervalMs = d.IntervalMs
	}
	if c.LagMs <= 0 {
		c.LagMs = d.LagMs
	}
	if c.MaxAgeMs <= 0 {
		c.MaxAgeMs = d.MaxAgeMs
	}
	if c.ThresholdDegreesPerSecond <= 0 {
		c.ThresholdDegreesPerSecond = d.ThresholdDegreesPerSecond
	}
	if c.Mode == "" {
		c.Mode = d.Mode
	}
}

//-------------------------------------Readings over time

type reading struct {
	v  float64
	at time.Time
}

// history is one quantity's recent readings, oldest first.
type history struct {
	r    []reading
	keep time.Duration
}

// add keeps a reading. One that does not come after the previous is the same
// reading asked for twice, and adds nothing.
func (h *history) add(v float64, at time.Time) {
	if n := len(h.r); n > 0 && !at.After(h.r[n-1].at) {
		return
	}
	h.r = append(h.r, reading{v, at})
	cut := 0
	for cut < len(h.r)-1 && at.Sub(h.r[cut].at) > h.keep {
		cut++
	}
	h.r = append(h.r[:0], h.r[cut:]...)
}

// at is the value at when, along the straight line between the readings
// either side of it. After the newest it is the newest: a wheel standing still
// is heard from only at the loader's heartbeat, and has not moved since. The
// newest reading's time is returned too, so a caller can tell a wheel standing
// from an encoder gone quiet. Before the oldest there is nothing to go on.
func (h *history) at(when time.Time) (v float64, newest time.Time, ok bool) {
	n := len(h.r)
	if n == 0 || when.Before(h.r[0].at) {
		return 0, time.Time{}, false
	}
	last := h.r[n-1]
	if !when.Before(last.at) {
		return last.v, last.at, true
	}
	i := 1
	for h.r[i].at.Before(when) {
		i++
	}
	a, b := h.r[i-1], h.r[i]
	f := float64(when.Sub(a.at)) / float64(b.at.Sub(a.at))
	return a.v + f*(b.v-a.v), last.at, true
}

// spans is whether the readings cover [t0, t1] from both sides with no hole
// longer than maxGap: the heading must have been heard throughout, since a
// hole in it may be where the vigie withheld it after losing a piece of a
// turn.
func (h *history) spans(t0, t1 time.Time, maxGap time.Duration) bool {
	n := len(h.r)
	if n == 0 || h.r[0].at.After(t0) || h.r[n-1].at.Before(t1) {
		return false
	}
	for i := 1; i < n; i++ {
		if h.r[i].at.Before(t0) {
			continue
		}
		if h.r[i].at.Sub(h.r[i-1].at) > maxGap {
			return false
		}
		if !h.r[i-1].at.Before(t1) {
			break
		}
	}
	return true
}

//-------------------------------------The gyrodometer

// interval is one comparison, as logged.
type interval struct {
	t0, t1     time.Time
	dl, dr     float64 // metres
	turnWheels float64 // radians
	turnGyro   float64 // radians, if haveGyro
	haveGyro   bool
	fromGyro   bool
}

// motionSince is what take returns: the axle's motion since the last take.
type motionSince struct {
	axle pose
	ok   bool  // every interval since had the wheels
	err  error // why not, if not
	// The gyro's own turn, for when the wheels failed but it did not.
	gyroTurn float64
	gyroOK   bool
}

type gyrodometer struct {
	mu sync.Mutex

	mode       string
	interval   time.Duration
	lag        time.Duration
	maxGap     time.Duration
	hold       time.Duration
	threshold  float64 // rad/s
	track      float64
	maxSpeed   float64
	wheelStale time.Duration

	left, right, heading history
	last                 time.Time // where the last interval ended

	viaGyro  bool
	lastTrip time.Time

	// Since the last take.
	moved    pose
	any      bool
	failed   error
	gyroTurn float64
	gyroOK   bool

	// Since the last summary.
	intervals, fromGyro, noGyro int
	eventFrom                   time.Time
	longest                     time.Duration

	onInterval func(interval)
}

func newGyrodometer(g GyroConfig, o OdometryConfig) *gyrodometer {
	keep := 2 * time.Second
	if stale := time.Duration(o.MaxAgeMs) * time.Millisecond; stale+time.Second > keep {
		keep = stale + time.Second
	}
	return &gyrodometer{
		mode:       g.Mode,
		interval:   time.Duration(g.IntervalMs) * time.Millisecond,
		lag:        time.Duration(g.LagMs) * time.Millisecond,
		maxGap:     time.Duration(g.MaxAgeMs) * time.Millisecond,
		hold:       g.hold(),
		threshold:  g.ThresholdDegreesPerSecond * math.Pi / 180,
		track:      o.Track,
		maxSpeed:   o.MaxWheelSpeed,
		wheelStale: time.Duration(o.MaxAgeMs) * time.Millisecond,
		left:       history{keep: keep},
		right:      history{keep: keep},
		heading:    history{keep: keep},
		gyroOK:     true,
	}
}

func (g *gyrodometer) addWheels(l, r wheelSample) {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.left.add(l.metres, l.at)
	g.right.add(r.metres, r.at)
}

// addHeading keeps a reading of the vigie's heading, in degrees.
func (g *gyrodometer) addHeading(degrees float64, at time.Time) {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.heading.add(degrees*math.Pi/180, at)
}

// step closes every interval that has ended by now, less the lag.
func (g *gyrodometer) step(now time.Time) {
	g.mu.Lock()
	defer g.mu.Unlock()
	end := now.Add(-g.lag)
	if g.last.IsZero() {
		if len(g.left.r) > 0 && len(g.right.r) > 0 {
			g.last = end
		}
		return
	}
	for !g.last.Add(g.interval).After(end) {
		t0 := g.last
		g.last = t0.Add(g.interval)
		g.compare(t0, g.last)
	}
}

// compare takes the turn over [t0, t1] from the wheels and from the gyro,
// chooses, and moves the axle.
func (g *gyrodometer) compare(t0, t1 time.Time) {
	l0, _, okL0 := g.left.at(t0)
	r0, _, okR0 := g.right.at(t0)
	l1, newL, okL1 := g.left.at(t1)
	r1, newR, okR1 := g.right.at(t1)
	if !(okL0 && okR0 && okL1 && okR1) {
		g.failed = fmt.Errorf("no wheel readings around %s", t0.Format("15:04:05.000"))
		return
	}
	if age := t1.Sub(newL); age > g.wheelStale {
		g.failed = fmt.Errorf("left wheel reading is %d ms old", age.Milliseconds())
		return
	}
	if age := t1.Sub(newR); age > g.wheelStale {
		g.failed = fmt.Errorf("right wheel reading is %d ms old", age.Milliseconds())
		return
	}
	T := t1.Sub(t0).Seconds()
	iv := interval{t0: t0, t1: t1, dl: l1 - l0, dr: r1 - r0}
	if !plausible(iv.dl, iv.dr, T, g.maxSpeed) {
		g.failed = &discontinuity{fmt.Sprintf(
			"the wheels jumped %.2f and %.2f m in %.2f s — taken as the loader restarting, not as motion", iv.dl, iv.dr, T)}
		return
	}
	iv.turnWheels = (iv.dr - iv.dl) / g.track

	if g.heading.spans(t0, t1, g.maxGap) {
		h0, _, _ := g.heading.at(t0)
		h1, _, _ := g.heading.at(t1)
		if turn := h1 - h0; math.Abs(turn) <= fastestTurn*T {
			iv.turnGyro, iv.haveGyro = turn, true
		}
	}

	threshold := g.threshold*T + 2*encoderMetresPerCount/g.track
	switch g.mode {
	case "odometry":
	case "gyro":
		iv.fromGyro = iv.haveGyro
	default:
		switch {
		case !iv.haveGyro:
		case math.Abs(iv.turnGyro-iv.turnWheels) > threshold:
			iv.fromGyro, g.lastTrip = true, t1
		case g.viaGyro && t1.Sub(g.lastTrip) < g.hold:
			iv.fromGyro = true
		}
	}

	turn := iv.turnWheels
	if iv.fromGyro {
		turn = iv.turnGyro
	}
	g.moved = compose(g.moved, arcStep((iv.dl+iv.dr)/2, turn))
	g.any = true
	if iv.haveGyro {
		g.gyroTurn += iv.turnGyro
	} else {
		g.gyroOK = false
	}

	g.intervals++
	switch {
	case iv.fromGyro && !g.viaGyro:
		g.eventFrom = t0
	case !iv.fromGyro && g.viaGyro:
		g.longest = max(g.longest, t0.Sub(g.eventFrom))
	}
	if iv.fromGyro {
		g.fromGyro++
	}
	if !iv.haveGyro {
		g.noGyro++
	}
	g.viaGyro = iv.fromGyro
	if g.onInterval != nil {
		g.onInterval(iv)
	}
}

// take returns the axle's motion since the last take, and starts again.
func (g *gyrodometer) take() motionSince {
	g.mu.Lock()
	defer g.mu.Unlock()
	m := motionSince{axle: g.moved, ok: g.any && g.failed == nil, err: g.failed,
		gyroTurn: g.gyroTurn, gyroOK: g.any && g.gyroOK}
	if !g.any && g.failed == nil {
		m.err = fmt.Errorf("no interval of wheel readings yet")
	}
	g.moved, g.any, g.failed, g.gyroTurn, g.gyroOK = pose{}, false, nil, 0, true
	return m
}

// summary says how often the gyro was believed since the last summary.
func (g *gyrodometer) summary() string {
	g.mu.Lock()
	defer g.mu.Unlock()
	longest := g.longest
	if g.viaGyro && !g.eventFrom.IsZero() {
		longest = max(longest, g.last.Sub(g.eventFrom))
	}
	s := "no intervals compared"
	if g.intervals > 0 {
		n := float64(g.intervals)
		s = fmt.Sprintf("%.1f %% of %d intervals from the gyro, longest %.1f s, gyro unavailable in %.1f %%",
			100*float64(g.fromGyro)/n, g.intervals, longest.Seconds(), 100*float64(g.noGyro)/n)
	}
	g.intervals, g.fromGyro, g.noGyro, g.longest = 0, 0, 0, 0
	return s
}

//-------------------------------------The interval log

// intervalLog writes one CSV line per interval: what the threshold is tuned
// from, and what MATLAB reads.
type intervalLog struct {
	mu sync.Mutex
	f  *os.File
	w  *bufio.Writer
}

func newIntervalLog(path string) (*intervalLog, error) {
	f, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_APPEND, 0o644)
	if err != nil {
		return nil, err
	}
	w := bufio.NewWriter(f)
	if st, err := f.Stat(); err == nil && st.Size() == 0 {
		fmt.Fprintln(w, "t,T,dL,dR,turnWheelsDeg,turnGyroDeg,source")
	}
	return &intervalLog{f: f, w: w}, nil
}

func (l *intervalLog) write(iv interval) {
	gyro, source := "", "wheels"
	if iv.haveGyro {
		gyro = fmt.Sprintf("%.5f", iv.turnGyro*180/math.Pi)
	}
	if iv.fromGyro {
		source = "gyro"
	}
	l.mu.Lock()
	defer l.mu.Unlock()
	fmt.Fprintf(l.w, "%.3f,%.4f,%.6f,%.6f,%.5f,%s,%s\n", float64(iv.t1.UnixMilli())/1000, iv.t1.Sub(iv.t0).Seconds(),
		iv.dl, iv.dr, iv.turnWheels*180/math.Pi, gyro, source)
}

func (l *intervalLog) flush() error {
	l.mu.Lock()
	defer l.mu.Unlock()
	return l.w.Flush()
}

func (l *intervalLog) close() error {
	l.mu.Lock()
	defer l.mu.Unlock()
	err := l.w.Flush()
	if cerr := l.f.Close(); err == nil {
		err = cerr
	}
	return err
}
