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
