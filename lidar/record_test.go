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

import (
	"math"
	"os"
	"strconv"
	"strings"
	"testing"
	"time"
)

// The recording is what the Allan deviation is computed from, so a row must
// read back as exactly the sample that was written, float32 included.
func TestARecordedRowReadsBackExactly(t *testing.T) {
	dir := t.TempDir()
	r, err := newRecorder(dir, time.Unix(1000, 0))
	if err != nil {
		t.Fatal(err)
	}
	s := imuSample{
		gyro:  [3]float64{float64(float32(0.0123456)), float64(float32(-1e-5)), 0},
		accel: [3]float64{float64(float32(-0.4999)), 0, float64(float32(0.8661))},
		at:    time.Unix(1000, 123456789),
	}
	if err := r.write(s); err != nil {
		t.Fatal(err)
	}
	if err := r.close(); err != nil {
		t.Fatal(err)
	}

	b, err := os.ReadFile(r.path)
	if err != nil {
		t.Fatal(err)
	}
	lines := strings.Split(strings.TrimSpace(string(b)), "\n")
	if len(lines) != 2 || lines[0]+"\n" != recordHeader {
		t.Fatalf("file is\n%s", b)
	}
	f := strings.Split(lines[1], ",")
	if ns, _ := strconv.ParseInt(f[0], 10, 64); ns != s.at.UnixNano() {
		t.Errorf("time %s, want %d", f[0], s.at.UnixNano())
	}
	want := []float64{s.gyro[0], s.gyro[1], s.gyro[2], s.accel[0], s.accel[1], s.accel[2]}
	for i, w := range want {
		v, err := strconv.ParseFloat(f[i+1], 64)
		if err != nil || float32(v) != float32(w) {
			t.Errorf("column %d = %s, want %v", i+1, f[i+1], w)
		}
	}
}

// A restart must not overwrite the last recording.
func TestARecorderNeverOverwrites(t *testing.T) {
	dir := t.TempDir()
	at := time.Unix(1000, 0)
	r, err := newRecorder(dir, at)
	if err != nil {
		t.Fatal(err)
	}
	r.close()
	if _, err := newRecorder(dir, at); err == nil {
		t.Error("a second recorder in the same second opened the same file")
	}
}

// The still script is the end-to-end test of the Allan analysis, which only
// works if it really never moves.
func TestTheStillScriptNeverTurns(t *testing.T) {
	cfg := testConfig()
	cfg.SimScript = "still"
	e := testEstimator(t, cfg)
	s := newSimulatedSource(cfg)
	feed(e, s, time.Unix(1000, 0), 0, 120)
	if r := e.reading(); math.Abs(r.yaw) > 0.05 {
		t.Errorf("two minutes of the still script read %.3f° of turn", r.yaw)
	}
}
