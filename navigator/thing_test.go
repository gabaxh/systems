/*******************************************************************************
 * Copyright (c) 2026 Synecdoque
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, subject to the following conditions:
 *
 * The software is licensed under the MIT License. See the LICENSE file in this
 * repository for details.
 *
 * Contributors:
 *   Jan A. van Deventer, Luleå - initial implementation
 ***************************************************************************SDG*/

package main

import (
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/sdoque/mbaigo/forms"
	"github.com/sdoque/mbaigo/usecases"
)

// The vehicle at (10, 5), facing up the map's y axis.
var facingUp = here{x: 10, y: 5, th: math.Pi / 2, frame: "map@2026-09-27T10:00:00Z"}

func polar(mag, dir float64, frame string) *forms.PolarA_v1a {
	p := &forms.PolarA_v1a{Magnitude: mag, Direction: dir, Frame: frame}
	p.NewForm()
	return p
}

func near(a, b float64) bool { return math.Abs(a-b) < 1e-9 }

// From the vehicle: 0 is straight ahead and positive is to the left, whichever
// way the vehicle faces in the map.
func TestDestinationFromTheVehicle(t *testing.T) {
	for _, c := range []struct {
		dir    float64
		wx, wy float64
		what   string
	}{
		{0, 10, 9, "4 m straight ahead"},
		{90, 6, 5, "4 m to the left"},
		{-90, 14, 5, "4 m to the right"},
		{180, 10, 1, "4 m behind"},
	} {
		x, y, err := destination(polar(4, c.dir, forms.PolarVehicle), facingUp)
		if err != nil || !near(x, c.wx) || !near(y, c.wy) {
			t.Errorf("%s: (%.2f, %.2f) %v, want (%.0f, %.0f)", c.what, x, y, err, c.wx, c.wy)
		}
	}
}

// In the map: the direction is from the map's x axis, whatever the vehicle's
// heading.
func TestDestinationInTheMap(t *testing.T) {
	x, y, err := destination(polar(4, 0, forms.PolarMap), facingUp)
	if err != nil || !near(x, 14) || !near(y, 5) {
		t.Errorf("4 m along the map's x axis: (%.2f, %.2f) %v, want (14, 5)", x, y, err)
	}
	if _, _, err := destination(polar(4, 0, facingUp.frame), facingUp); err != nil {
		t.Errorf("a bearing in this map's own frame was refused: %v", err)
	}
}

// Nothing here knows where north is, so a compass bearing cannot be followed.
func TestACompassBearingIsRefused(t *testing.T) {
	for _, f := range []string{forms.PolarCompassFrom, forms.PolarCompassTo} {
		_, _, err := destination(polar(4, 0, f), facingUp)
		if err == nil || !strings.Contains(err.Error(), "north") {
			t.Errorf("%s: %v, want a refusal that says why", f, err)
		}
	}
}

// A destination from a previous run of the cartographer names somewhere else.
func TestADestinationInAnotherMapIsRefused(t *testing.T) {
	old := &forms.PoseA_v1a{X: 3, Y: 4, Frame: "map@2026-09-26T08:00:00Z"}
	old.NewForm()
	if _, _, err := destination(old, facingUp); err == nil {
		t.Error("a pose from a previous map was used")
	}
	if _, _, err := destination(polar(4, 0, "map@2026-09-26T08:00:00Z"), facingUp); err == nil {
		t.Error("a bearing from a previous map was used")
	}
	now := &forms.PoseA_v1a{X: 3, Y: 4, Frame: facingUp.frame}
	now.NewForm()
	if x, y, err := destination(now, facingUp); err != nil || x != 3 || y != 4 {
		t.Errorf("a pose in this map: (%v, %v) %v", x, y, err)
	}
}

func TestADistanceMustBeInMeters(t *testing.T) {
	p := polar(4, 0, forms.PolarVehicle)
	p.MagnitudeUnit = "<http://qudt.org/vocab/unit/FT>"
	if _, _, err := destination(p, facingUp); err == nil {
		t.Error("a distance in feet was taken for meters")
	}
}

// What the README tells a person to type must be read as they mean it.
func TestTheCurlInTheReadme(t *testing.T) {
	body := `{"magnitude": 12, "direction": 0, "frame": "vehicle", "version": "PolarA_v1.0"}`
	f, err := usecases.Unpack([]byte(body), "application/json")
	if err != nil {
		t.Fatal(err)
	}
	x, y, err := destination(f, facingUp)
	if err != nil || !near(x, 10) || !near(y, 17) {
		t.Errorf("12 m ahead: (%.2f, %.2f) %v, want (10, 17)", x, y, err)
	}
}

func TestThePictureIsWritten(t *testing.T) {
	tm := newTestMap(22, 6)
	tm.corridor(1, 2, 21, 4)
	p, err := plan(&tm.m, 0.1, evenOdds, artitrax, 2, 3, 0, 19, 3)
	if err != nil {
		t.Fatal(err)
	}
	out := filepath.Join(t.TempDir(), "plan.ppm")
	if err := writePicture(out, &tm.m, p, here{x: 2, y: 3}); err != nil {
		t.Fatal(err)
	}
	b, err := os.ReadFile(out)
	if err != nil {
		t.Fatal(err)
	}
	// Cropped to the seen corridor (0.9..21.1 by 1.9..4.1 m) and a meter of
	// margin: 445 by 85 cells at 5 cm.
	var w, h int
	if _, err := fmt.Sscanf(string(b), "P6\n%d %d\n255\n", &w, &h); err != nil {
		t.Fatalf("header %q: %v", string(b[:20]), err)
	}
	if w >= 440 && h >= 120 {
		t.Errorf("%d x %d: the picture was not cropped", w, h)
	}
	if w < 400 || h < 40 {
		t.Errorf("%d x %d: cropped into the corridor itself", w, h)
	}
	header := fmt.Sprintf("P6\n%d %d\n255\n", w, h)
	if len(b) != len(header)+w*h*3 {
		t.Errorf("%d bytes for %d x %d", len(b), w, h)
	}
}

// standing is a navigator with a destination set in a corridor, reading its map
// from a test instead of the cartographer.
func standing(t *testing.T) (*Traits, *testMap) {
	t.Helper()
	tm := newTestMap(22, 6)
	tm.corridor(1, 2, 21, 4)
	cfg := NavConfig{}
	applyDefaults(&cfg)
	cfg.PicturePath = filepath.Join(t.TempDir(), "plan.ppm")
	at := here{x: 2, y: 3, frame: tm.m.Frame}
	tr := &Traits{cfg: cfg}
	tr.readMap = func() (*forms.MapA_v1a, error) { return &tm.m, nil }
	tr.readPose = func() (here, error) { return at, nil }
	tr.readLimit = func() float64 { return 0.5 }
	if _, err := tr.planTo(19, 3, at, nil); err != nil {
		t.Fatal(err)
	}
	if tr.path == nil {
		t.Fatal("no route to begin with")
	}
	return tr, tm
}

// A door opens across the corridor. A route to the nearest point would look like
// any other, and the vehicle would arrive at the door: so no route, until it
// closes, and the destination is still there when it does.
func TestABlockedWayIsNoRouteUntilItOpens(t *testing.T) {
	tr, tm := standing(t)
	tm.fill(10, 2, 10.1, 4, forms.MapOccupied)
	tr.replanOnce()
	if tr.path != nil {
		t.Fatalf("a route with the way blocked, ending at (%.1f, %.1f)", tr.path.X[len(tr.path.X)-1], tr.path.Y[len(tr.path.Y)-1])
	}
	if tr.goal == nil {
		t.Fatal("the destination was dropped")
	}

	tm.fill(10, 2, 10.1, 4, forms.MapFree)
	tr.replanOnce()
	if tr.path == nil {
		t.Fatal("the way is open and there is no route")
	}
	if end := tr.path.X[len(tr.path.X)-1]; end < 18 {
		t.Errorf("the route ends at x=%.1f, want the destination at 19", end)
	}
}

// A plan that fails for any reason takes down the route. Before, the old one
// stayed up, and it was planned against a map that had changed.
func TestAFailedPlanTakesDownTheRoute(t *testing.T) {
	tr, _ := standing(t)
	tr.readMap = func() (*forms.MapA_v1a, error) { return nil, fmt.Errorf("no map") }
	tr.replanOnce()
	if tr.path != nil {
		t.Error("the old route is still up")
	}
	if tr.goal == nil {
		t.Error("the destination was dropped")
	}
}

// A destination the person was told is out of reach stays as they accepted it:
// the route goes on ending at the nearest point, rather than being withdrawn.
func TestAnAcceptedNearestPointKeepsItsRoute(t *testing.T) {
	tr, tm := standing(t)
	at := here{x: 2, y: 3, frame: tm.m.Frame}
	if _, err := tr.planTo(30, 3, at, nil); err != nil {
		t.Fatal(err)
	}
	if tr.goal.why == "" {
		t.Fatal("the goal outside the map was not moved")
	}
	tr.replanOnce()
	if tr.path == nil {
		t.Error("the route to the accepted nearest point was withdrawn")
	}
}

// A destination dropped while a replan is running stays dropped.
func TestAGoalDroppedDuringAPlanStaysDropped(t *testing.T) {
	tr, tm := standing(t)
	read := tr.readMap
	tr.readMap = func() (*forms.MapA_v1a, error) {
		tr.drop()
		return read()
	}
	tr.replanOnce()
	if tr.goal != nil || tr.path != nil {
		t.Errorf("the plan brought it back: goal=%v, map %s", tr.goal, tm.m.Frame)
	}
}

// Arrival still drops the destination, and a new map still does.
func TestArrivalAndANewMapDropTheGoal(t *testing.T) {
	tr, tm := standing(t)
	tr.readPose = func() (here, error) { return here{x: tr.goal.x, y: tr.goal.y, frame: tm.m.Frame}, nil }
	tr.replanOnce()
	if tr.goal != nil {
		t.Error("arrived, and the destination stands")
	}
	tr, _ = standing(t)
	tr.readPose = func() (here, error) { return here{x: 2, y: 3, frame: "map@another"}, nil }
	tr.replanOnce()
	if tr.goal != nil {
		t.Error("a new map, and the destination stands")
	}
}
