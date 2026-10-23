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

// The device's clock on this computer's.
//
// The cartographer compares the gyro with the wheels over the same interval,
// and the wheels are stamped on this computer's clock, so the gyro must be
// too. The device stamps each packet itself, in nanoseconds since it powered
// on, and those stamps are evenly spaced; the time a packet is received here
// is on the right clock but jitters with the network and the scheduler. So
// the spacing is taken from the device and the epoch from this computer: the
// offset between the two is the smallest (received − device) seen recently,
// which is the packet that waited least. Every sample is then device time plus
// that offset, never later than it was received.
//
// The minimum is over a sliding window rather than for all time so that it
// follows the two crystals drifting apart. The point cloud will be stamped by
// the same map. PTP between the device and this computer can replace it if the
// point cloud ever needs better.

import (
	"sync"
	"time"
)

// clockWindow is how far back the minimum offset is looked for. At the 50 ppm
// of a cheap crystal, ten seconds lets the map fall at most half a millisecond
// behind.
const clockWindow = 10 * time.Second

type offsetAt struct {
	received int64 // host unix nanoseconds
	offset   int64 // received − device, nanoseconds
}

type clockMap struct {
	mu     sync.Mutex
	window time.Duration
	// mins holds increasing offsets, oldest first: the front is the minimum
	// of the window. maxs is the same for the maximum, for the spread.
	mins, maxs []offsetAt
	lastDevice int64
	resets     int
}

func newClockMap(window time.Duration) *clockMap {
	return &clockMap{window: window}
}

// toHost maps a device timestamp, from a packet received at received, to
// this computer's clock.
func (c *clockMap) toHost(device int64, received time.Time) time.Time {
	c.mu.Lock()
	defer c.mu.Unlock()
	if len(c.mins) > 0 && device < c.lastDevice-int64(time.Second) {
		// The device's clock went back: it restarted, and the old offset
		// means nothing any more.
		c.mins, c.maxs = c.mins[:0], c.maxs[:0]
		c.resets++
	}
	c.lastDevice = device

	now := received.UnixNano()
	o := offsetAt{received: now, offset: now - device}
	for len(c.mins) > 0 && c.mins[len(c.mins)-1].offset >= o.offset {
		c.mins = c.mins[:len(c.mins)-1]
	}
	c.mins = append(c.mins, o)
	for len(c.maxs) > 0 && c.maxs[len(c.maxs)-1].offset <= o.offset {
		c.maxs = c.maxs[:len(c.maxs)-1]
	}
	c.maxs = append(c.maxs, o)
	oldest := now - int64(c.window)
	for c.mins[0].received < oldest {
		c.mins = c.mins[1:]
	}
	for c.maxs[0].received < oldest {
		c.maxs = c.maxs[1:]
	}
	return time.Unix(0, device+c.mins[0].offset)
}

// spread is how much the delay of a packet varied over the window: the
// jitter that taking the device's own spacing removes.
func (c *clockMap) spread() time.Duration {
	c.mu.Lock()
	defer c.mu.Unlock()
	if len(c.mins) == 0 {
		return 0
	}
	return time.Duration(c.maxs[0].offset - c.mins[0].offset)
}
