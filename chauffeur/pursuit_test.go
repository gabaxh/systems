package main

import (
	"math"
	"testing"

	"github.com/sdoque/mbaigo/forms"
)

func TestARouteMustBeUsable(t *testing.T) {
	if _, err := newRoute(&forms.PathA_v1a{}); err == nil {
		t.Error("an empty route was accepted")
	}
	if _, err := newRoute(&forms.PathA_v1a{X: []float64{1, 2}, Y: []float64{1}}); err == nil {
		t.Error("a ragged route was accepted")
	}
	if _, err := newRoute(&forms.PathA_v1a{X: []float64{1}, Y: []float64{1}, DistanceUnit: "feet"}); err == nil {
		t.Error("a route in feet was accepted")
	}
}

func TestLocateOnAStraightRoute(t *testing.T) {
	r := arcs(0, 0, 0, [2]float64{0, 12})
	p := r.locate(0, 4, 0.1)
	if math.Abs(p.along-4) > 1e-6 || math.Abs(p.offset-0.1) > 1e-6 {
		t.Errorf("%+v", p)
	}
	if math.Abs(p.remaining-(r.length()-4)) > 1e-6 {
		t.Errorf("remaining %v of %v", p.remaining, r.length())
	}
}

// A route that loops comes close to itself. The vehicle is on the stretch it has
// reached, and not on the nearer one further along.
func TestLocateDoesNotJumpAcrossALoop(t *testing.T) {
	// Out 6 m, round a circle of 2.5 m radius, and back past the start of the loop.
	r := arcs(0, 0, 0, [2]float64{0, 6}, [2]float64{0.4, 2 * math.Pi / 0.4}, [2]float64{0, 6})
	first := r.locate(0, 3, 0)
	if first.along > 3.1 {
		t.Fatalf("placed %.1f m along", first.along)
	}
	// Later, the vehicle comes back past (7, 0) on the second pass.
	later := r.locate(r.locate(0, 6, 0).index, 7, 0)
	if later.along > 8 {
		t.Errorf("jumped to %.1f m along, past the whole loop", later.along)
	}
}

func TestPursueAimsAtTheRoute(t *testing.T) {
	r := arcs(0, 0, 0, [2]float64{0, 12})
	on := r.locate(0, 4, 0)
	if k := r.pursue(on, 4, 0, 0, 1.2, 0.48); k != 0 {
		t.Errorf("on the route, curvature %v", k)
	}
	right := r.locate(0, 4, -0.2)
	if k := r.pursue(right, 4, -0.2, 0, 1.2, 0.48); k <= 0 {
		t.Errorf("to the right of the route, curvature %v, want to the left, positive", k)
	}
	left := r.locate(0, 4, 0.2)
	if k := r.pursue(left, 4, 0.2, 0, 1.2, 0.48); k >= 0 {
		t.Errorf("to the left of the route, curvature %v, want to the right, negative", k)
	}
}

// On a circle, aimed along the circle, pure pursuit asks for that curvature.
func TestPursueOnACircle(t *testing.T) {
	const k = 0.4
	r := arcs(0, 0, 0, [2]float64{k, 8})
	p := r.locate(0, r.x[20], r.y[20])
	th := math.Atan2(r.y[21]-r.y[19], r.x[21]-r.x[19])
	if got := r.pursue(p, r.x[20], r.y[20], th, 1.2, 0.48); math.Abs(got-k) > 0.02 {
		t.Errorf("curvature %.3f, want about %.3f", got, k)
	}
}

func TestPursueRespectsTheLimit(t *testing.T) {
	r := arcs(0, 0, 0, [2]float64{0, 12})
	p := r.locate(0, 4, -1)
	if k := r.pursue(p, 4, -1, -math.Pi/2, 1.2, 0.3); k != 0.3 {
		t.Errorf("curvature %v, want the limit", k)
	}
}

// Past the end the aim point carries on in the last direction, so the steering
// does not swing as the vehicle closes on the end.
func TestTheAimPointGoesOnPastTheEnd(t *testing.T) {
	r := arcs(0, 0, 0, [2]float64{0, 3})
	x, y := r.pointAt(r.length() + 1)
	if math.Abs(x-(r.length()+1)) > 1e-6 || math.Abs(y) > 1e-6 {
		t.Errorf("(%v, %v)", x, y)
	}
}

// One pose: the navigator's answer to a vehicle already there.
func TestARouteOfOnePose(t *testing.T) {
	r, err := newRoute(&forms.PathA_v1a{X: []float64{3}, Y: []float64{4}})
	if err != nil {
		t.Fatal(err)
	}
	p := r.locate(0, 3.1, 4)
	if p.remaining != 0 || math.Abs(p.offset-0.1) > 1e-9 {
		t.Errorf("%+v", p)
	}
	if k := r.pursue(p, 3.1, 4, 0, 1.2, 0.48); math.IsNaN(k) {
		t.Error("NaN")
	}
}

//-------------------------------------Feedforward

var testFF = ffParams{lead: 0, window: 0.6, correction: 2.5, l1: 0.6175, l2: 0.6175}

func TestTheRoutesCurvature(t *testing.T) {
	const k = 0.4
	r := arcs(0, 0, 0, [2]float64{0, 3}, [2]float64{k, 4}, [2]float64{-k / 2, 4})
	for _, c := range []struct {
		s, want float64
	}{
		{1, 0},              // on the straight
		{4.5, k},            // on the left arc
		{9, -k / 2},         // on the gentler right arc
		{r.length() + 1, 0}, // past the end, the route goes straight on
	} {
		if got := r.curvatureOver(c.s, 0.6); math.Abs(got-c.want) > 0.07 {
			t.Errorf("at %.1f m: %.3f, want about %.3f", c.s, got, c.want)
		}
	}
	// Straddling the start of the arc: some of it.
	if got := r.curvatureOver(2.8, 0.6); got <= 0.02 || got >= k-0.02 {
		t.Errorf("straddling the start of the arc: %.3f, want between 0 and %.1f", got, k)
	}
	// The same on an arc whatever the window, segments or not (to the chords' 0.01 %).
	for _, w := range []float64{0.1, 0.25, 0.6, 1.33} {
		if got := r.curvatureOver(4.5, w); math.Abs(got-k) > 1e-3 {
			t.Errorf("window %.2f m: %.4f, want %.4f", w, got, k)
		}
	}
}

func TestTheSideOfTheRoute(t *testing.T) {
	r := arcs(0, 0, 0, [2]float64{0, 12})
	off, heading := r.errorsAt(r.locate(0, 4, 0.2), 4, 0.2, 0.1)
	if math.Abs(off-0.2) > 1e-9 || math.Abs(heading-0.1) > 1e-9 {
		t.Errorf("left of the route and pointing left: %+.3f m, %+.3f rad", off, heading)
	}
	off, _ = r.errorsAt(r.locate(0, 4, -0.2), 4, -0.2, 0)
	if math.Abs(off+0.2) > 1e-9 {
		t.Errorf("right of the route: %+.3f m", off)
	}
}

// On the route, pointing along it, with the waist at the angle the route asks
// for: feedforward steers the route's own curvature.
func TestFeedforwardSteersTheRoutesCurvature(t *testing.T) {
	const k = 0.4
	r := arcs(0, 0, 0, [2]float64{k, 8})
	x, y := r.x[20], r.y[20]
	at := r.locate(0, x, y)
	th := r.headingAt(at.along)
	art := articulationFor(k, testFF.l1, testFF.l2)
	s := r.pursueFeedforward(at, x, y, th, art, true, testFF, 0.48)
	if math.Abs(s.curvature-k) > 0.01 || math.Abs(s.planned-k) > 1e-3 {
		t.Errorf("%+v, want %.3f", s, k)
	}
}

// Off the route, it steers back towards it, either side, on a straight.
func TestFeedforwardCorrects(t *testing.T) {
	r := arcs(0, 0, 0, [2]float64{0, 12})
	if s := r.pursueFeedforward(r.locate(0, 4, -0.2), 4, -0.2, 0, 0, true, testFF, 0.48); s.curvature <= 0 {
		t.Errorf("right of the route: %.3f, want to the left, positive", s.curvature)
	}
	if s := r.pursueFeedforward(r.locate(0, 4, 0.2), 4, 0.2, 0, 0, true, testFF, 0.48); s.curvature >= 0 {
		t.Errorf("left of the route: %.3f, want to the right, negative", s.curvature)
	}
	if s := r.pursueFeedforward(r.locate(0, 4, -1), 4, -1, -math.Pi/2, 0, true, testFF, 0.3); s.curvature != 0.3 {
		t.Errorf("%.3f, want the limit", s.curvature)
	}
}

// The front half pointing in only because the waist has swung ahead of the
// route's angle is not a heading error: the waist will swing back and take the
// front with it. Allowed for, the command is the route's; not allowed for, it
// steers out, which is the swing back that made the vehicle hunt.
func TestTheWaistTurningTheFrontIsNotAnError(t *testing.T) {
	r := arcs(0, 0, 0, [2]float64{0, 12})
	at := r.locate(0, 4, 0)
	art := 6 * math.Pi / 180                                      // swung 6° left of the straight route's 0°
	th := testFF.l2 / (testFF.l1*math.Cos(art) + testFF.l2) * art // and the front with it
	with := r.pursueFeedforward(at, 4, 0, th, art, true, testFF, 0.48)
	without := r.pursueFeedforward(at, 4, 0, th, art, false, testFF, 0.48)
	if math.Abs(with.curvature) > 1e-9 {
		t.Errorf("allowing for the waist: %.4f, want the route's 0", with.curvature)
	}
	if without.curvature >= -0.03 {
		t.Errorf("not allowing for it: %.4f, want a clear steer to the right", without.curvature)
	}
}

func TestArticulationForInvertsTheKinematics(t *testing.T) {
	for _, g := range []struct{ l1, l2 float64 }{{0.6175, 0.6175}, {0.5, 0.8}} {
		for _, k := range []float64{-0.5, -0.2, 0, 0.13, 0.4} {
			a := articulationFor(k, g.l1, g.l2)
			if got := math.Sin(a) / (g.l1*math.Cos(a) + g.l2); math.Abs(got-k) > 1e-9 {
				t.Errorf("L1 %.2f, L2 %.2f: %.3f /m gives %.2f°, which is %.6f /m", g.l1, g.l2, k, a*180/math.Pi, got)
			}
		}
	}
	if a := articulationFor(5, 0.6175, 0.6175); a != maxArticulation {
		t.Errorf("an impossible curvature gives %.1f°", a*180/math.Pi)
	}
}

func TestFeedforwardOnARouteOfOnePose(t *testing.T) {
	r, err := newRoute(&forms.PathA_v1a{X: []float64{3}, Y: []float64{4}})
	if err != nil {
		t.Fatal(err)
	}
	s := r.pursueFeedforward(r.locate(0, 3.1, 4), 3.1, 4, 0, 0.1, true, testFF, 0.48)
	if math.IsNaN(s.curvature) || s.curvature != 0 {
		t.Errorf("%+v", s)
	}
}

// Along a tight arc the route's direction is the tangent everywhere, not only
// at the middles of its 15 cm segments.
func TestTheRoutesDirectionIsSmooth(t *testing.T) {
	const k = 0.4
	r := arcs(0, 0, 0, [2]float64{0, 1}, [2]float64{k, 4})
	start := r.along[r.segAt(1.2)] // into the arc
	for s := start; s < 4; s += 0.01 {
		want := k * (s - start)
		h0 := r.headingAt(start)
		if got := wrapAngle(r.headingAt(s) - h0); math.Abs(got-want) > 0.002 {
			t.Fatalf("at %.2f m: turned %.4f rad, want %.4f", s, got, want)
		}
	}
}
