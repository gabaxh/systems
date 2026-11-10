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
	"math/rand"
	"testing"
	"time"
)

// Packets wait between 1 and 4 ms on their way. The map keeps the lidar's even
// spacing and sits at the least wait, never later than the packet arrived.
func TestTheClockMapTakesTheLeastDelayedPacket(t *testing.T) {
	c := newClockMap(clockWindow)
	rng := rand.New(rand.NewSource(3))
	base := time.Unix(5000, 0)
	const least = time.Millisecond
	for k := 0; k < 2000; k++ {
		stamp := int64(k) * int64(5*time.Millisecond)
		wait := least + time.Duration(rng.Float64()*float64(3*time.Millisecond))
		if k%50 == 0 {
			wait = least
		}
		arrived := base.Add(time.Duration(stamp) + wait)
		got := c.toHost(stamp, arrived)
		if got.After(arrived) {
			t.Fatalf("packet %d put after it arrived", k)
		}
		if k >= 50 {
			if off := got.Sub(base.Add(time.Duration(stamp) + least)); off != 0 {
				t.Fatalf("packet %d put %v from the least wait", k, off)
			}
		}
	}
}

// A lidar that restarts counts from zero again; the old offset would put its
// samples an hour in the past.
func TestTheClockMapStartsOverWhenTheLidarRestarts(t *testing.T) {
	c := newClockMap(clockWindow)
	base := time.Unix(5000, 0)
	for k := 0; k < 100; k++ {
		c.toHost(int64(time.Hour)+int64(k)*int64(5*time.Millisecond), base.Add(time.Duration(k)*5*time.Millisecond))
	}
	later := base.Add(2 * time.Second)
	if got := c.toHost(0, later); !got.Equal(later) {
		t.Errorf("after a restart the first packet was put %v from its arrival", got.Sub(later))
	}
	if c.restarts != 1 {
		t.Errorf("%d restarts, want 1", c.restarts)
	}
}

// The Artitrax's Mid-360 stamps its samples about 0.55 % fast against this
// host's clock. Integrated on its raw stamps, a heading reads every turn that
// much large, which a 0.3 % scale check would refuse. Put on this host's clock
// first, it reads true — whichever clock is the faster.
func TestTheHeadingKeepsThisHostsTime(t *testing.T) {
	for _, rate := range []float64{1.0055, 1 / 1.0055} {
		c := newClockMap(clockWindow)
		mapped := &motion{window: 100 * time.Millisecond}
		raw := &motion{window: 100 * time.Millisecond}
		rng := rand.New(rand.NewSource(5))
		base := time.Unix(5000, 0)
		const yaw = 30 * math.Pi / 180
		n := 60 * 200
		for k := 0; k <= n; k++ {
			host := time.Duration(k) * 5 * time.Millisecond
			stamp := int64(float64(host) * rate)
			wait := 200*time.Microsecond + time.Duration(rng.Float64()*float64(3*time.Millisecond))
			arrived := base.Add(host + wait)
			s := timedIMU{at: arrived, gyro: [3]float64{0, 0, yaw}, acc: [3]float64{0, 0, 1}}
			s.stamp = uint64(c.toHost(stamp, arrived).UnixNano())
			mapped.add(s)
			s.stamp = uint64(stamp)
			raw.add(s)
		}
		want := 30 * 60.0
		_, _, got, _, _ := mapped.averages()
		_, _, gotRaw, _, _ := raw.averages()
		if rel := math.Abs(got/want - 1); rel > 0.0005 {
			t.Errorf("lidar clock ×%.4f: 60 s at 30°/s read %.2f° on this host's clock, %.3f %% off", rate, got, 100*rel)
		}
		if g := mapped.state().gaps; g != 0 {
			t.Errorf("lidar clock ×%.4f: the map made %d gaps", rate, g)
		}
		if rel := math.Abs(gotRaw/want - 1); rel < 0.004 {
			t.Errorf("lidar clock ×%.4f: the raw stamps read only %.3f %% off; the test does not test the map", rate, 100*rel)
		}
	}
}
