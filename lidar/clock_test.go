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
	"math/rand/v2"
	"testing"
	"time"
)

// Packets wait between 1 and 4 ms on their way here. The map should keep the
// device's even spacing and sit at the least delay, never later than receipt.
func TestTheClockMapTakesTheLeastDelayedPacket(t *testing.T) {
	c := newClockMap(clockWindow)
	rng := rand.New(rand.NewPCG(3, 4))
	base := time.Unix(5000, 0)
	const least = time.Millisecond
	for k := range 2000 {
		device := int64(k) * int64(5*time.Millisecond)
		delay := least + time.Duration(rng.Float64()*float64(3*time.Millisecond))
		if k%50 == 0 {
			delay = least
		}
		received := base.Add(time.Duration(device) + delay)
		got := c.toHost(device, received)
		if got.After(received) {
			t.Fatalf("packet %d mapped to after it was received", k)
		}
		if k > 0 {
			if err := got.Sub(base.Add(time.Duration(device) + least)); err < 0 || err > 3*time.Millisecond {
				t.Fatalf("packet %d mapped %v from the least delay", k, err)
			}
		}
		if k >= 50 {
			if err := got.Sub(base.Add(time.Duration(device) + least)); err != 0 {
				t.Fatalf("packet %d mapped %v from the least delay, after the least-delayed packet was seen", k, err)
			}
		}
	}
	if s := c.spread(); s < 2*time.Millisecond || s > 3*time.Millisecond {
		t.Errorf("spread = %v, want about the 3 ms of jitter", s)
	}
}

// A device that restarts begins counting from zero again. The old offset
// would put its samples hours in the past.
func TestTheClockMapStartsOverWhenTheDeviceRestarts(t *testing.T) {
	c := newClockMap(clockWindow)
	base := time.Unix(5000, 0)
	for k := range 100 {
		device := int64(time.Hour) + int64(k)*int64(5*time.Millisecond)
		c.toHost(device, base.Add(time.Duration(k)*5*time.Millisecond))
	}
	later := base.Add(2 * time.Second)
	got := c.toHost(0, later)
	if !got.Equal(later) {
		t.Errorf("after a restart the first packet mapped to %v from its receipt, want 0", got.Sub(later))
	}
	if c.resets != 1 {
		t.Errorf("%d resets, want 1", c.resets)
	}
}

// The minimum is over a window, so a device clock that runs slow, whose
// offset grows, is followed rather than left behind.
func TestTheClockMapFollowsDrift(t *testing.T) {
	c := newClockMap(time.Second)
	base := time.Unix(5000, 0)
	var device int64
	var received time.Time
	for k := range 1000 { // 5 s at 200 Hz, the device losing 100 ppm
		received = base.Add(time.Duration(k) * 5 * time.Millisecond)
		device = int64(float64(k) * float64(5*time.Millisecond) * (1 - 100e-6))
		c.toHost(device, received)
	}
	if lag := received.Sub(c.toHost(device, received)); lag > 150*time.Microsecond {
		t.Errorf("the map lags receipt by %v, want at most a window's drift", lag)
	}
}
