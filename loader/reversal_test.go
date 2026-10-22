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
	"bytes"
	"log"
	"net/http"
	"os"
	"strings"
	"testing"
	"time"
)

// reversalRun drives a trace cycle by cycle at 50 Hz, the count falling to the
// left as on the calibrated test waist.
type reversalRun struct {
	tr  reversalTrace
	at  time.Time
	out []string
}

func newReversalRun() *reversalRun { return &reversalRun{at: time.Unix(1_000_000, 0)} }

func (r *reversalRun) cycle(demand int, effort float64, held string, raw int, wheels string, wheelPercent float64) {
	r.at = r.at.Add(20 * time.Millisecond)
	if line := r.tr.observe(reversalCycle{now: r.at, demand: demand, effort: effort, held: held, raw: raw,
		fresh: true, leftward: -1, counts: 3, wheels: wheels, wheelPercent: wheelPercent}); line != "" {
		r.out = append(r.out, line)
	}
}

func (r *reversalRun) only(t *testing.T) string {
	t.Helper()
	if len(r.out) != 1 {
		t.Fatalf("%d lines, want one: %q", len(r.out), r.out)
	}
	return r.out[0]
}

// Left, then right: the effort goes out at once and the joint moves later,
// held by the wheels meanwhile, and the line says both.
func TestReversalTimesTheEffortAndTheJoint(t *testing.T) {
	r := newReversalRun()
	r.cycle(1, 40, "", 300, "driving", 10) // turning left
	r.cycle(-1, 0, "", 300, "driving", 10) // the stick goes right; steerLimit's zero
	for i := 0; i < 50; i++ {              // a second at 40% with the joint not moving
		r.cycle(-1, -40, "", 300, "holding", 38)
	}
	if len(r.out) != 0 {
		t.Fatalf("reported before the joint moved: %q", r.out)
	}
	r.cycle(-1, -40, "", 303, "driving", 20) // three counts up is to the right
	line := r.only(t)
	for _, want := range []string{"to the right", "effort after 20 ms", "joint moving after 1020 ms",
		"wheels driving then holding then driving", "up to 38% effort"} {
		if !strings.Contains(line, want) {
			t.Errorf("%q does not say %q", line, want)
		}
	}
}

// Effort held back says by what, and a joint that never moves is reported.
func TestReversalNamesWhatHeldTheEffort(t *testing.T) {
	r := newReversalRun()
	r.cycle(-1, -40, "", 600, "holding", 0)
	for i := 0; i < 40; i++ {
		r.cycle(1, 0, "settling after the last move", 600, "holding", 0)
	}
	for i := 0; i < 300; i++ {
		r.cycle(1, 40, "", 600, "holding", 45)
	}
	line := r.only(t)
	for _, want := range []string{"to the left", "effort after 800 ms (held: settling after the last move)",
		"joint did not move within 5000 ms"} {
		if !strings.Contains(line, want) {
			t.Errorf("%q does not say %q", line, want)
		}
	}
}

// Through the centre to the other side is a reversal; back to the same side,
// or the first steer after start-up, is not.
func TestOnlyAChangeOfSideIsAReversal(t *testing.T) {
	r := newReversalRun()
	r.cycle(1, 40, "", 450, "holding", 0) // the first steer
	r.cycle(0, 0, "no steering setpoint", 450, "holding", 0)
	r.cycle(1, 40, "", 450, "holding", 0) // the same side again
	for i := 0; i < 300; i++ {
		r.cycle(1, 40, "", 450, "holding", 0)
	}
	if len(r.out) != 0 {
		t.Fatalf("no reversal, and it reported %q", r.out)
	}
	r.cycle(0, 0, "no steering setpoint", 450, "holding", 0)
	r.cycle(-1, -40, "", 450, "holding", 0)
	r.cycle(-1, -40, "", 453, "holding", 0)
	if line := r.only(t); !strings.Contains(line, "to the right") {
		t.Errorf("left, centre, right: %q", line)
	}
}

// Reversed again before the joint moved, the first reversal is still reported.
func TestReversalReversedAgain(t *testing.T) {
	r := newReversalRun()
	r.cycle(1, 40, "", 450, "holding", 0)
	r.cycle(-1, -40, "", 450, "holding", 0)
	r.cycle(1, 40, "", 450, "holding", 0)
	if line := r.only(t); !strings.Contains(line, "reversed again before the joint moved") {
		t.Errorf("%q", line)
	}
}

// Through the drivetrain: steering by effort, left and then right, with the
// joint moving right two cycles later. The line reaches the log.
func TestTheDrivetrainLogsAReversal(t *testing.T) {
	var buf bytes.Buffer
	log.SetOutput(&buf)
	t.Cleanup(func() { log.SetOutput(os.Stderr) })

	d, _, steering := drivingDrivetrain(t, calibrated())
	withWaist(d, 450)
	if w := put(t, steering.setpointService, "gamer", "40"); w.Code != http.StatusOK {
		t.Fatalf("steering left: %d %s", w.Code, w.Body)
	}
	d.writeCycle()
	if w := put(t, steering.setpointService, "gamer", "-40"); w.Code != http.StatusOK {
		t.Fatalf("steering right: %d %s", w.Code, w.Body)
	}
	d.writeCycle()
	d.writeCycle()
	withWaist(d, 455) // the count rises to the right
	d.writeCycle()
	if !strings.Contains(buf.String(), "steering reversal to the right: effort after") ||
		!strings.Contains(buf.String(), "joint moving after") {
		t.Errorf("no reversal line in the log:\n%s", buf.String())
	}
}
