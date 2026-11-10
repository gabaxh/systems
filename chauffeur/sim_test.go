package main

import (
	"math"
	"testing"
	"time"

	"github.com/sdoque/mbaigo/forms"
)

// The vehicle in closed loop with the mission: what the chauffeur tells the
// loader, what the machine does about it, and what comes back, late and coarse.
//
// The model takes its kinematics from the loader: the front half turns at
// (v sin γ + L2 γ̇) / (L1 cos γ + L2). Its waist behaves as the loader's angle
// loop does: nothing within a band around the target, then a constant rate. The
// pose arrives at sweep rate and old, and commands arrive late.
//
// What it leaves out, and the vehicle will not: slip, chain slack, the ramp on the
// wheels, and a scan matcher that moves the pose. The waist's rate here is a guess
// that has not been measured on the vehicle.

const (
	simL1, simL2  = 0.6175, 0.6175
	simDt         = 0.005
	simCycles     = 10 // of simDt: 20 Hz
	simSweepEvery = 40 // of simDt: 5 Hz
)

type simVehicle struct {
	x, y, th, gamma, gammaRate float64
	moving                     bool
	targetGamma                float64
	waistRate                  float64 // rad/s
	v                          float64
}

func simGammaFor(k float64) float64 {
	lo, hi := -33*math.Pi/180, 33*math.Pi/180
	for i := 0; i < 40; i++ {
		mid := (lo + hi) / 2
		if math.Sin(mid)/(simL1*math.Cos(mid)+simL2) < k {
			lo = mid
		} else {
			hi = mid
		}
	}
	return (lo + hi) / 2
}

func (s *simVehicle) advance(v, k float64) {
	// The wheels' ramp is short at these speeds: 0.5 m/s takes a quarter second.
	const accel = 2.0
	s.v += math.Max(-accel*simDt, math.Min(accel*simDt, v-s.v))

	s.targetGamma = simGammaFor(k)
	err := s.targetGamma - s.gamma
	const hold, settle = 1 * math.Pi / 180, 0.3 * math.Pi / 180
	switch {
	case !s.moving && math.Abs(err) > hold:
		s.moving = true
	case s.moving && math.Abs(err) < settle:
		s.moving = false
	}
	s.gammaRate = 0
	if s.moving {
		s.gammaRate = math.Copysign(s.waistRate, err)
	}
	s.gamma += s.gammaRate * simDt

	turn := (s.v*math.Sin(s.gamma) + simL2*s.gammaRate) / (simL1*math.Cos(s.gamma) + simL2)
	s.th += turn * simDt
	s.x += s.v * math.Cos(s.th) * simDt
	s.y += s.v * math.Sin(s.th) * simDt
}

// arcs builds a route the way the navigator's search does: arcs of the
// curvatures it uses, 15 cm a step.
func arcs(x, y, th float64, pieces ...[2]float64) *route {
	const step = 0.15
	p := &forms.PathA_v1a{X: []float64{x}, Y: []float64{y}, Frame: "map@sim", DistanceUnit: unitMetre, Timestamp: simEpoch}
	for _, piece := range pieces {
		k, metres := piece[0], piece[1]
		for s := 0.0; s < metres; s += step {
			if math.Abs(k) < 1e-9 {
				x += step * math.Cos(th)
				y += step * math.Sin(th)
			} else {
				n := th + k*step
				x += (math.Sin(n) - math.Sin(th)) / k
				y += (math.Cos(th) - math.Cos(n)) / k
				th = n
			}
			p.X, p.Y = append(p.X, x), append(p.Y, y)
		}
	}
	r, err := newRoute(p)
	if err != nil {
		panic(err)
	}
	return r
}

var simEpoch = time.Date(2026, 11, 5, 12, 0, 0, 0, time.UTC)

func simTuning() tuning {
	return tuning{
		cruise: 0.5, lookahead: 1.2, holdClearance: 0.8, stopClear: 0.4, arrive: 0.25, offRoute: 0.2,
		maxPoseAge: time.Second, maxRouteAge: time.Hour, maxClearanceAge: time.Second, brakeFor: 500 * time.Millisecond,
		routeEndMoved: 1, routeLonger: 3,
	}
}

type simResult struct {
	code        Code
	worstOffset float64
	endError    float64
	seconds     float64
	v           *simVehicle
	holdAtX     float64
}

// simDrive runs a mission along a route to its end, or until it holds or stops.
// clearanceAt, if given, is the clearance ahead from where the vehicle is.
func simDrive(r *route, v *simVehicle, poseDelaySweeps int, clearanceAt func(*simVehicle) float64) simResult {
	return simDriveWith(simTuning(), true, r, v, poseDelaySweeps, clearanceAt)
}

// simFeedforward is simTuning steering by feedforward.
func simFeedforward() tuning {
	cfg := simTuning()
	cfg.feedforward, cfg.maxArtAge = true, time.Second
	cfg.ff = ffParams{lead: 0, window: 0.6, correction: 2.5, l1: simL1, l2: simL2}
	return cfg
}

// simDriveWith is simDrive with a tuning of its own, and with the loader's
// articulation reading or without it.
func simDriveWith(cfg tuning, withArt bool, r *route, v *simVehicle, poseDelaySweeps int, clearanceAt func(*simVehicle) float64) simResult {
	m := &mission{cfg: cfg}
	now := simEpoch
	m.start(now)
	type sample struct {
		x, y, th float64
		at       time.Time
	}
	samples := []sample{{v.x, v.y, v.th, now}}
	seen := samples[0]
	cmdV, cmdK := 0.0, 0.0
	type command struct{ v, k float64 }
	latency := []command{{}, {}} // two cycles late, 100 ms
	res := simResult{v: v}
	for i := 0; i < 400000; i++ {
		v.advance(cmdV, cmdK)
		now = now.Add(time.Duration(simDt * float64(time.Second)))
		if i%simSweepEvery == 0 {
			samples = append(samples, sample{v.x, v.y, v.th, now})
			if len(samples) > poseDelaySweeps {
				seen = samples[len(samples)-1-poseDelaySweeps]
			}
		}
		if i%simCycles != 0 {
			continue
		}
		clear := 20.0
		if clearanceAt != nil {
			clear = clearanceAt(v)
		}
		a := m.step(observation{
			now: now, havePose: true, x: seen.x, y: seen.y, th: seen.th, poseFrame: "map@sim", poseAt: seen.at,
			route: r, haveClear: true, clearance: clear, clearAt: now, curveLimit: 0.48,
			haveArt: withArt, art: v.gamma, artAt: now,
		})
		if a.take {
			m.granted(a.epoch, now)
		}
		if a.release {
			m.released(a.epoch)
		}
		if a.stop {
			cmdV, cmdK = 0, 0
		}
		latency = append(latency, command{a.velocity, a.curvature})
		if !a.drive {
			latency[len(latency)-1] = command{0, cmdK}
		}
		cmdV, cmdK = latency[0].v, latency[0].k
		latency = latency[1:]

		if m.route != nil && m.at.offset > res.worstOffset {
			res.worstOffset = m.at.offset
		}
		if m.code == codeArrived || m.code.holding() || m.code == codeStoppedTooClose {
			// Let the vehicle come to rest before judging where it ended.
			for j := 0; j < 400; j++ {
				v.advance(0, cmdK)
			}
			ex, ey := r.end()
			res.code, res.endError, res.seconds, res.holdAtX = m.code, math.Hypot(v.x-ex, v.y-ey), float64(i)*simDt, v.x
			return res
		}
	}
	res.code = m.code
	return res
}

const plannerCurvature = 0.4 // what the planner uses: 80 % of the loader's 0.5

// checkFollowed drives a route to its end. The bar is the mission's own: it would
// hold at 0.2 m off the route, and the route's end is good to a quarter of a meter.
func checkFollowed(t *testing.T, name string, pieces [][2]float64, rate float64, delay int) {
	t.Helper()
	res := simDrive(arcs(0, 0, 0, pieces...), &simVehicle{waistRate: rate * math.Pi / 180}, delay, nil)
	t.Logf("%-30s waist %2.0f°/s, pose %3d ms old: worst %.3f m off, ends %.3f m from the end, %5.1f s",
		name, rate, delay*200, res.worstOffset, res.endError, res.seconds)
	if res.code != codeArrived {
		t.Errorf("%s, waist %.0f°/s, pose %d ms old: ended %q", name, rate, delay*200, res.code)
	}
	if res.endError > 0.35 {
		t.Errorf("%s: ended %.2f m from the route's end", name, res.endError)
	}
}

// Routes of every shape the navigator's search produces, with a waist that keeps
// up and the pose fresh or old.
func TestTheVehicleFollowsARoute(t *testing.T) {
	const k = plannerCurvature
	for _, route := range []struct {
		name   string
		pieces [][2]float64
	}{
		{"straight", [][2]float64{{0, 15}}},
		{"quarter turn at the tightest", [][2]float64{{0, 3}, {k, math.Pi / 2 / k}, {0, 5}}},
		{"half circle at the tightest", [][2]float64{{0, 3}, {-k, math.Pi / k}, {0, 5}}},
		{"an S", [][2]float64{{0, 2}, {k, 3}, {-k, 3}, {0, 4}}},
		{"a double S", [][2]float64{{0, 2}, {k, 4}, {-k, 4}, {k, 4}, {0, 4}}},
		{"small changes", [][2]float64{{0, 2}, {k / 2, 2}, {0, 1}, {-k / 2, 3}, {k / 2, 2}, {0, 3}}},
	} {
		for _, delay := range []int{1, 2} {
			checkFollowed(t, route.name, route.pieces, 15, delay)
		}
	}
}

// A slow waist still follows anything that does not swing it from one lock to the
// other in a hurry.
func TestASlowWaistFollowsGentleRoutes(t *testing.T) {
	const k = plannerCurvature
	checkFollowed(t, "quarter turn", [][2]float64{{0, 3}, {k, math.Pi / 2 / k}, {0, 5}}, 8, 1)
	checkFollowed(t, "half circle", [][2]float64{{0, 3}, {-k, math.Pi / k}, {0, 5}}, 8, 1)
	checkFollowed(t, "small changes", [][2]float64{{0, 2}, {k / 2, 2}, {0, 1}, {-k / 2, 3}, {k / 2, 2}, {0, 3}}, 8, 1)
}

// The planner does not know how fast the waist swings, so it can plan a reversal
// from one lock to the other that the machine cannot make: at 5°/s and 0.5 m/s the
// swing alone takes five meters. That must end in a hold, with the vehicle near the
// route, and not with it wandering off towards a wall.
func TestAWaistTooSlowForTheRouteEndsInAHold(t *testing.T) {
	const k = plannerCurvature
	r := arcs(0, 0, 0, [2]float64{0, 2}, [2]float64{k, 4}, [2]float64{-k, 4}, [2]float64{k, 4}, [2]float64{0, 4})
	res := simDrive(r, &simVehicle{waistRate: 5 * math.Pi / 180}, 2, nil)
	if res.code != codeHoldOffRoute {
		t.Fatalf("ended %q, want a hold for being off the route", res.code)
	}
	if res.worstOffset > 0.3 {
		t.Errorf("%.2f m off the route before holding", res.worstOffset)
	}
}

// A vehicle set off a little to the side of its route comes onto it.
func TestAnOffsetStartConverges(t *testing.T) {
	r := arcs(0, 0, 0, [2]float64{0, 12})
	res := simDrive(r, &simVehicle{y: 0.12, th: 5 * math.Pi / 180, waistRate: 15 * math.Pi / 180}, 1, nil)
	if res.code != codeArrived {
		t.Fatalf("ended %q, worst %.2f m off", res.code, res.worstOffset)
	}
	if math.Abs(res.v.y) > 0.1 {
		t.Errorf("ended %.2f m to the side", res.v.y)
	}
}

// Something appears ahead while driving: the vehicle holds short of it, on its
// own braking, with room to spare, and has not stopped the vehicle outright.
func TestItHoldsShortOfSomethingThatAppears(t *testing.T) {
	const obstacleAt = 8.0
	r := arcs(0, 0, 0, [2]float64{0, 15})
	clear := func(v *simVehicle) float64 { return obstacleAt - v.x }
	res := simDrive(r, &simVehicle{waistRate: 15 * math.Pi / 180}, 1, clear)
	if res.code != codeHoldObstacle {
		t.Fatalf("ended %q, want a hold for the obstacle", res.code)
	}
	if gap := obstacleAt - res.holdAtX; gap < 0.5 {
		t.Errorf("came to rest %.2f m from it", gap)
	}
}

// Something appears suddenly, already inside the distance a hold could stop in.
func TestItStopsTheVehicleForSomethingVeryClose(t *testing.T) {
	r := arcs(0, 0, 0, [2]float64{0, 15})
	clear := func(v *simVehicle) float64 {
		if v.x > 5 {
			return 0.3
		}
		return 10
	}
	res := simDrive(r, &simVehicle{waistRate: 15 * math.Pi / 180}, 1, clear)
	if res.code != codeStoppedTooClose {
		t.Fatalf("ended %q, want a stop", res.code)
	}
}

//-------------------------------------Feedforward

// The routes TestTheVehicleFollowsARoute drives.
func simRoutes() []struct {
	name   string
	pieces [][2]float64
} {
	const k = plannerCurvature
	return []struct {
		name   string
		pieces [][2]float64
	}{
		{"straight", [][2]float64{{0, 15}}},
		{"quarter turn at the tightest", [][2]float64{{0, 3}, {k, math.Pi / 2 / k}, {0, 5}}},
		{"half circle at the tightest", [][2]float64{{0, 3}, {-k, math.Pi / k}, {0, 5}}},
		{"an S", [][2]float64{{0, 2}, {k, 3}, {-k, 3}, {0, 4}}},
		{"a double S", [][2]float64{{0, 2}, {k, 4}, {-k, 4}, {k, 4}, {0, 4}}},
		{"small changes", [][2]float64{{0, 2}, {k / 2, 2}, {0, 1}, {-k / 2, 3}, {k / 2, 2}, {0, 3}}},
	}
}

// Feedforward keeps much closer to the planner's tightest routes than pursuit
// with the lookahead that stopped the waist hunting on the vehicle.
func TestFeedforwardKeepsClose(t *testing.T) {
	pursuit := simTuning()
	pursuit.lookahead, pursuit.offRoute = 2.0, 5 // to see how far it strays, not where it would hold
	for _, route := range simRoutes() {
		for _, delay := range []int{1, 2} {
			ff := simDriveWith(simFeedforward(), true, arcs(0, 0, 0, route.pieces...), &simVehicle{waistRate: 15 * math.Pi / 180}, delay, nil)
			pp := simDriveWith(pursuit, true, arcs(0, 0, 0, route.pieces...), &simVehicle{waistRate: 15 * math.Pi / 180}, delay, nil)
			t.Logf("%-30s pose %3d ms old: feedforward %.3f m off, pursuit 2.0 m %.3f m off", route.name, delay*200, ff.worstOffset, pp.worstOffset)
			if ff.code != codeArrived {
				t.Errorf("%s, pose %d ms old: ended %q, %.2f m off", route.name, delay*200, ff.code, ff.worstOffset)
			}
			if ff.worstOffset > 0.12 {
				t.Errorf("%s, pose %d ms old: %.3f m off the route", route.name, delay*200, ff.worstOffset)
			}
			if ff.endError > 0.35 {
				t.Errorf("%s: ended %.2f m from the route's end", route.name, ff.endError)
			}
		}
	}
}

// A slow waist, and a vehicle set off to the side, as for pursuit.
func TestFeedforwardWithASlowWaistAndAnOffsetStart(t *testing.T) {
	const k = plannerCurvature
	for _, pieces := range [][][2]float64{
		{{0, 3}, {k, math.Pi / 2 / k}, {0, 5}},
		{{0, 2}, {k / 2, 2}, {0, 1}, {-k / 2, 3}, {k / 2, 2}, {0, 3}},
	} {
		res := simDriveWith(simFeedforward(), true, arcs(0, 0, 0, pieces...), &simVehicle{waistRate: 8 * math.Pi / 180}, 1, nil)
		if res.code != codeArrived {
			t.Errorf("waist 8°/s: ended %q, %.2f m off", res.code, res.worstOffset)
		}
	}
	res := simDriveWith(simFeedforward(), true, arcs(0, 0, 0, [2]float64{0, 12}), &simVehicle{y: 0.12, th: 5 * math.Pi / 180, waistRate: 15 * math.Pi / 180}, 1, nil)
	if res.code != codeArrived || math.Abs(res.v.y) > 0.1 {
		t.Errorf("from 0.12 m off: ended %q, %.2f m to the side", res.code, res.v.y)
	}
}

// Without the articulation, feedforward still drives: it only no longer allows
// for the waist turning the front half.
func TestFeedforwardWithoutTheArticulationStillDrives(t *testing.T) {
	const k = plannerCurvature
	for _, pieces := range [][][2]float64{
		{{0, 15}},
		{{0, 2}, {k / 2, 2}, {0, 1}, {-k / 2, 3}, {k / 2, 2}, {0, 3}},
	} {
		res := simDriveWith(simFeedforward(), false, arcs(0, 0, 0, pieces...), &simVehicle{waistRate: 15 * math.Pi / 180}, 1, nil)
		t.Logf("without the articulation: %.3f m off", res.worstOffset)
		if res.code != codeArrived {
			t.Errorf("ended %q, %.2f m off", res.code, res.worstOffset)
		}
	}
}
