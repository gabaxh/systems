package main

import (
	"testing"
	"time"
)

// A vehicle halfway down a straight route, and everything fine.
func fineView(now time.Time, r *route) observation {
	return observation{
		now: now, havePose: true, x: 5, poseFrame: "map@sim", poseAt: now, route: r,
		haveClear: true, clearance: 10, clearAt: now, curveLimit: 0.48,
	}
}

func straightRoute(stamp time.Time) *route {
	r := arcs(0, 0, 0, [2]float64{0, 15})
	r.stamp = stamp
	return r
}

// driving is a mission that has control and is on its route.
func driving(t *testing.T) (*mission, *route, time.Time) {
	t.Helper()
	now := simEpoch
	r := straightRoute(now)
	m := &mission{cfg: simTuning()}
	m.start(now)
	a := m.step(fineView(now, r))
	if !a.take {
		t.Fatal("did not ask for control")
	}
	m.granted(a.epoch, now)
	now = now.Add(2 * time.Second) // past the settling time after setting off
	if a = m.step(fineView(now, r)); !a.drive || a.velocity != 0.5 {
		t.Fatalf("not driving: %+v, %q", a, m.code)
	}
	return m, r, now
}

func TestItDoesNothingUntilToldTo(t *testing.T) {
	m := &mission{cfg: simTuning()}
	a := m.step(fineView(simEpoch, straightRoute(simEpoch)))
	if a.take || a.drive || a.stop || a.release {
		t.Errorf("told nothing and sent %+v", a)
	}
}

// Every reason to hold, and in each: velocity zero, still sending, and it stays
// held when the reason goes away, until a person says to go on.
func TestEveryReasonToHold(t *testing.T) {
	for _, c := range []struct {
		name  string
		spoil func(*observation)
		want  Code
	}{
		{"something close", func(o *observation) { o.clearance = 0.7 }, codeHoldObstacle},
		{"blind", func(o *observation) { o.haveClear = false }, codeHoldBlind},
		{"an old clearance", func(o *observation) { o.clearAt = o.now.Add(-2 * time.Second) }, codeHoldBlind},
		{"no pose", func(o *observation) { o.havePose = false }, codeHoldPose},
		{"an old pose", func(o *observation) { o.poseAt = o.now.Add(-2 * time.Second) }, codeHoldPose},
		{"another map", func(o *observation) { o.poseFrame = "map@another" }, codeHoldFrame},
		{"no route", func(o *observation) { o.route = nil }, codeHoldRoute},
		{"off the route", func(o *observation) { o.y = 0.5 }, codeHoldOffRoute},
	} {
		m, r, now := driving(t)
		now = now.Add(100 * time.Millisecond)
		o := fineView(now, r)
		m.step(o)
		now = now.Add(100 * time.Millisecond)
		o = fineView(now, r)
		c.spoil(&o)
		a := m.step(o)
		if m.code != c.want {
			t.Errorf("%s: %q, want %q", c.name, m.code, c.want)
			continue
		}
		if !a.drive || a.velocity != 0 {
			t.Errorf("%s: sent %+v, want velocity zero and still sending", c.name, a)
		}
		// All clear again: it stays held.
		now = now.Add(100 * time.Millisecond)
		if a = m.step(fineView(now, r)); m.code != c.want || a.velocity != 0 {
			t.Errorf("%s: went on by itself: %q %+v", c.name, m.code, a)
		}
		// Until told.
		m.start(now)
		now = now.Add(100 * time.Millisecond)
		if a = m.step(fineView(now, r)); m.code != codeDriving || a.velocity != 0.5 {
			t.Errorf("%s: did not go on: %q %+v", c.name, m.code, a)
		}
	}
}

// An old route holds: it can only be old if the navigator has stopped planning.
func TestAnOldRouteHolds(t *testing.T) {
	m, r, now := driving(t)
	r.stamp = now.Add(-2 * time.Hour)
	m.step(fineView(now, r))
	if m.code != codeHoldRoute {
		t.Errorf("%q", m.code)
	}
}

// Started again with the reason still there, it holds again rather than drives.
func TestStartingAgainWithTheReasonStillThere(t *testing.T) {
	m, r, now := driving(t)
	o := fineView(now, r)
	o.clearance = 0.7
	m.step(o)
	m.start(now)
	if a := m.step(o); m.code != codeHoldObstacle || a.velocity != 0 {
		t.Errorf("%q %+v", m.code, a)
	}
}

// Very close while moving is a stop, which the mission does not undo.
func TestVeryCloseIsAStop(t *testing.T) {
	m, r, now := driving(t)
	o := fineView(now, r)
	o.clearance = 0.3
	a := m.step(o)
	if m.code != codeStoppedTooClose || !a.stop || a.drive {
		t.Fatalf("%q %+v", m.code, a)
	}
	stops := 1 // the one above
	for i := 0; i < 20; i++ {
		if a = m.step(fineView(now, r)); a.stop {
			stops++
		}
		if a.drive {
			t.Fatal("drives after stopping")
		}
	}
	if stops != stopRepeats {
		t.Errorf("%d stops, want %d", stops, stopRepeats)
	}
}

// Just told to go with something already close, it holds. The vehicle is not
// moving, and a stop would need the pad for nothing.
func TestVeryCloseOnSettingOffIsAHold(t *testing.T) {
	now := simEpoch
	r := straightRoute(now)
	m := &mission{cfg: simTuning()}
	m.start(now)
	m.granted(m.step(fineView(now, r)).epoch, now)
	o := fineView(now, r)
	o.clearance = 0.3
	if a := m.step(o); m.code != codeHoldObstacle || a.stop {
		t.Errorf("%q %+v", m.code, a)
	}
}

// A 409 is never retried: a stop stays a stop until a person undoes it.
func TestARefusalEndsTheMissionAndIsNotRetried(t *testing.T) {
	m, r, now := driving(t)
	m.refused(m.epoch, "the vehicle is stopped")
	if m.code != codeLostControl {
		t.Fatalf("%q", m.code)
	}
	for i := 0; i < 50; i++ {
		now = now.Add(100 * time.Millisecond)
		if a := m.step(fineView(now, r)); a.take || a.drive || a.release {
			t.Fatalf("sent %+v after losing control", a)
		}
	}
}

// A take refused the same way: only after a person has handed the vehicle over.
func TestATakeRefused(t *testing.T) {
	now := simEpoch
	m := &mission{cfg: simTuning()}
	m.start(now)
	a := m.step(fineView(now, straightRoute(now)))
	m.refused(a.epoch, "only gamer can take control after a stop")
	if m.code != codeLostControl {
		t.Fatalf("%q", m.code)
	}
	m.start(now) // a person handed over, and pressed start
	if a = m.step(fineView(now.Add(2*time.Second), straightRoute(now))); !a.take || m.code != codeTaking {
		t.Errorf("did not ask again: %q %+v", m.code, a)
	}
}

// A reply from before a handover cannot end the mission after it.
func TestAnOldRefusalIsIgnored(t *testing.T) {
	m, _, _ := driving(t)
	m.refused(m.epoch-1, "from before")
	if m.code != codeDriving {
		t.Errorf("%q", m.code)
	}
}

func TestAnUncalibratedWaistHolds(t *testing.T) {
	m, _, _ := driving(t)
	m.steeringRefused(m.epoch, "the waist sensor is not calibrated")
	if m.code != codeHoldSteering {
		t.Errorf("%q", m.code)
	}
}

// bringToRest runs the braking and answers the release.
func bringToRest(t *testing.T, m *mission, r *route, now time.Time) {
	t.Helper()
	for i := 0; i < 100 && m.code == codeBraking; i++ {
		now = now.Add(50 * time.Millisecond)
		a := m.step(fineView(now, r))
		if a.drive && a.velocity != 0 {
			t.Fatalf("moving while braking: %+v", a)
		}
		if a.release {
			m.released(a.epoch)
		}
	}
}

// walkTo moves the vehicle along the route in steps the search window allows.
func walkTo(m *mission, r *route, now time.Time, x float64) observation {
	o := fineView(now, r)
	for o.x = 5; o.x < x; o.x += 1 {
		m.step(o)
	}
	o.x = x
	return o
}

func TestItArrivesAtTheEndAndReleases(t *testing.T) {
	m, r, now := driving(t)
	o := walkTo(m, r, now, r.length()-0.1)
	if a := m.step(o); m.code != codeBraking || a.velocity != 0 {
		t.Fatalf("%q %+v", m.code, a)
	}
	bringToRest(t, m, r, now)
	if m.code != codeArrived {
		t.Errorf("%q", m.code)
	}
}

// The navigator drops its route once the vehicle is near enough for it, which
// can be before the vehicle's own idea of arriving. That is arriving, not a hold.
func TestARouteThatGoesAwayAtTheEndIsArrival(t *testing.T) {
	m, r, now := driving(t)
	o := walkTo(m, r, now, r.length()-0.5)
	m.step(o)
	if m.code != codeDriving {
		t.Fatalf("half a meter from the end: %q", m.code)
	}
	o.route = nil
	m.step(o)
	if m.code != codeBraking || m.after != codeArrived {
		t.Errorf("%q after %q", m.code, m.after)
	}
}

func TestCancelComesToRestAndReleases(t *testing.T) {
	m, r, now := driving(t)
	m.cancel(now)
	bringToRest(t, m, r, now)
	if m.code != codeIdle {
		t.Errorf("%q", m.code)
	}
}

// Released while a velocity already on its way is refused: that is the release
// having worked, and not a lost mission.
func TestARefusalDuringTheReleaseIsTheEnd(t *testing.T) {
	m, r, now := driving(t)
	m.cancel(now)
	for i := 0; i < 100 && m.askedAt.IsZero(); i++ {
		now = now.Add(50 * time.Millisecond)
		m.step(fineView(now, r))
	}
	m.refused(m.epoch, "nobody has control")
	if m.code != codeIdle {
		t.Errorf("%q", m.code)
	}
}

// A release that is never confirmed does not leave it braking for ever.
func TestAnUnconfirmedReleaseGivesUp(t *testing.T) {
	m, r, now := driving(t)
	m.cancel(now)
	for i := 0; i < 200 && m.code == codeBraking; i++ {
		now = now.Add(50 * time.Millisecond)
		m.step(fineView(now, r))
	}
	if m.code != codeIdle {
		t.Errorf("%q", m.code)
	}
}

// The usual replan: the same way, from further on. It is taken up without fuss.
func TestAReplanOfTheSameRouteGoesThrough(t *testing.T) {
	m, r, now := driving(t)
	next := arcs(5, 0, 0, [2]float64{0, 10})
	next.stamp = now
	if a := m.step(fineView(now, next)); m.code != codeDriving || m.route != next || a.velocity != 0.5 {
		t.Errorf("%q, route replaced %v, %+v", m.code, m.route == next, a)
	}
	_ = r
}

// A route that ends elsewhere, or goes a long way round, is one nobody has looked
// at. It holds with the new route shown, and going on accepts it.
func TestARouteNobodyHasLookedAtHolds(t *testing.T) {
	for name, pieces := range map[string][][2]float64{
		"ends elsewhere": {{0, 7}},
		"far longer":     {{0, 5}, {0.4, 7.85}, {0, 5}, {0.4, 7.85}},
	} {
		m, r, now := driving(t)
		next := arcs(5, 0, 0, pieces...)
		next.stamp = now
		if name == "far longer" {
			// Ends at the old route's end, the long way round.
			next.x[len(next.x)-1], next.y[len(next.y)-1] = 15, 0
		}
		if m.step(fineView(now, next)); m.code != codeHoldRouteChanged {
			t.Errorf("%s: %q", name, m.code)
			continue
		}
		if m.route != r {
			t.Errorf("%s: took up the new route while holding", name)
		}
		m.start(now)
		m.step(fineView(now, next))
		if m.code != codeDriving || m.route != next {
			t.Errorf("%s: going on: %q, new route %v", name, m.code, m.route == next)
		}
	}
}
