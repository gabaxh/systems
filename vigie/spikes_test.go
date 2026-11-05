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
	"testing"
	"time"
)

func scanOf(distances []float64, valid []bool) sweep {
	n := len(distances)
	sw := sweep{angles: make([]float64, n), distances: distances, valid: valid, taken: time.Now()}
	for i := range sw.angles {
		sw.angles[i] = -180 + (float64(i)+0.5)*360/float64(n)
	}
	return sw
}

func all(n int) []bool {
	v := make([]bool, n)
	for i := range v {
		v[i] = true
	}
	return v
}

// The guetteur's cases, for the Mid-360's scan.
func TestASpikeIsDroppedAndAPostIsKept(t *testing.T) {
	cases := []struct {
		name      string
		distances []float64
		valid     []bool
		jump      float64
		want      []bool
	}{
		{"one return behind the wall is dropped",
			[]float64{2, 2, 2, 5, 2, 2}, all(6), 0.5,
			[]bool{true, true, true, false, true, true}},
		{"a post in front of the wall is kept",
			[]float64{2, 2, 2, 0.8, 2, 2}, all(6), 0.5,
			all(6)},
		{"a step no larger than the jump is a surface, not a spike",
			[]float64{2, 2, 2, 2.4, 2, 2}, all(6), 0.5,
			all(6)},
		{"further than only one neighbour is an edge, not a spike",
			[]float64{2, 2, 2, 5, 5, 5}, all(6), 0.5,
			all(6)},
		{"beside a bin with no return there is nothing to compare with",
			[]float64{2, 2, 0, 5, 2, 2}, []bool{true, true, false, true, true, true}, 0.5,
			[]bool{true, true, false, true, true, true}},
		{"a negative jump turns the filter off",
			[]float64{2, 2, 2, 5, 2, 2}, all(6), -1,
			all(6)},
		{"two spikes side by side judge each other as the scan was, and both stay",
			[]float64{2, 2, 5, 5, 2, 2}, all(6), 0.5,
			all(6)},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			in := scanOf(c.distances, c.valid)
			before := append([]bool(nil), in.valid...)
			out := rejectSpikes(in, c.jump)
			for i := range c.want {
				if out.valid[i] != c.want[i] {
					t.Fatalf("valid %v, want %v", out.valid, c.want)
				}
			}
			for i := range before {
				if in.valid[i] != before[i] {
					t.Fatal("rejectSpikes changed the scan it was given")
				}
			}
		})
	}
}

// The scan is a whole turn: the bins either side of the seam behind the
// vehicle are neighbours, and a spike there is a spike.
func TestASpikeAtTheSeamIsDropped(t *testing.T) {
	d := []float64{6, 2, 2, 2, 2, 2}
	if out := rejectSpikes(scanOf(d, all(6)), 0.5); out.valid[0] {
		t.Error("a spike in the first bin, with the last bin as its other neighbour, was kept")
	}
	d = []float64{2, 2, 2, 2, 2, 6}
	if out := rejectSpikes(scanOf(d, all(6)), 0.5); out.valid[5] {
		t.Error("a spike in the last bin, with the first bin as its other neighbour, was kept")
	}
}

// A corner of a 4 × 4 m room seen from its middle, with one bin's beam come
// back from behind the corner. That bin goes, and nothing else does.
func TestTheStrayReturnInACornerGoes(t *testing.T) {
	const n = 360
	d := make([]float64, n)
	for i := range d {
		a := (-180 + float64(i) + 0.5) * math.Pi / 180
		// Distance to the walls of a square of half-width 2 m.
		d[i] = 2 / math.Max(math.Abs(math.Cos(a)), math.Abs(math.Sin(a)))
	}
	corner := 180 + 45 // 45° to the left: the corner, at 2.83 m
	d[corner] = 4.6
	out := rejectSpikes(scanOf(d, all(n)), 0.5)
	for i := range out.valid {
		if want := i != corner; out.valid[i] != want {
			t.Errorf("bin %d at %.2f m: valid %v, want %v", i, d[i], out.valid[i], want)
		}
	}
}

func TestSpikeRejectionIsOnByDefault(t *testing.T) {
	c := VigieConfig{}
	applyDefaults(&c)
	if c.Slice.SpikeJumpMetres != 0.5 {
		t.Errorf("spikeJumpMetres defaults to %v, want 0.5", c.Slice.SpikeJumpMetres)
	}
	c = VigieConfig{Slice: SliceConfig{SpikeJumpMetres: -1}}
	applyDefaults(&c)
	if c.Slice.SpikeJumpMetres != -1 {
		t.Errorf("a negative spikeJumpMetres, to turn the filter off, became %v", c.Slice.SpikeJumpMetres)
	}
}
