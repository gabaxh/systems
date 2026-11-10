/*******************************************************************************
 * Copyright (c) 2026 Synecdoque
 *
 * The software is licensed under the MIT License. See the LICENSE file in this
 * repository for details.
 *
 * Contributors:
 *   Gabriel Axheim Gustafsson, Luleå - the lidar's clock on this host's
 ***************************************************************************SDG*/

package main

import (
	"sync"
	"time"
)

// The lidar's clock on this host's.
//
// The lidar stamps every packet itself, in nanoseconds since it powered up,
// and those stamps are evenly spaced; the time a packet arrives here is on the
// right clock but carries the network's and the scheduler's jitter. So the
// spacing is taken from the lidar and the epoch from this host: the offset
// between the two is the smallest (arrival − stamp) seen lately, the packet
// that waited least, and every stamp is put on this host's clock by adding
// it. It is never later than the packet arrived.
//
// The two clocks do not run at the same rate. On the Artitrax's Mid-360 the
// lidar's stamps advance about 0.55 % faster than this host's clock (2 h of
// IMU samples at rest, October 2026), so a heading integrated on the raw stamps
// reads every turn 0.55 % large. Mapped, the stamps keep this host's rate: the
// minimum is taken over a sliding window, which follows the drift. The window
// is short because when the lidar's clock is the slower one the minimum is held
// for the whole window and then jumps; over 2 s at 0.55 % that is 11 ms, well
// inside the gap a stream may have before it counts as lost.
//
// What the cartographer compares the heading with — the loader's wheels — is
// stamped on this host's clock too, which is the other reason for doing it.

// clockWindow is how far back the least-delayed packet is looked for.
const clockWindow = 2 * time.Second

type clockOffset struct {
	arrived int64 // this host's unix nanoseconds
	offset  int64 // arrived − stamp, nanoseconds
}

// clockMap puts the lidar's stamps on this host's clock.
type clockMap struct {
	mu        sync.Mutex
	window    time.Duration
	mins      []clockOffset // increasing offsets, oldest first: the front is the window's minimum
	lastStamp int64
	restarts  int
}

func newClockMap(window time.Duration) *clockMap {
	return &clockMap{window: window}
}

// toHost maps a stamp from a packet that arrived at arrived.
func (c *clockMap) toHost(stamp int64, arrived time.Time) time.Time {
	c.mu.Lock()
	defer c.mu.Unlock()
	if len(c.mins) > 0 && stamp < c.lastStamp-int64(time.Second) {
		// The lidar's clock went back: it restarted, and the old offset means
		// nothing any more.
		c.mins = c.mins[:0]
		c.restarts++
	}
	c.lastStamp = stamp

	now := arrived.UnixNano()
	o := clockOffset{arrived: now, offset: now - stamp}
	for len(c.mins) > 0 && c.mins[len(c.mins)-1].offset >= o.offset {
		c.mins = c.mins[:len(c.mins)-1]
	}
	c.mins = append(c.mins, o)
	for c.mins[0].arrived < now-int64(c.window) {
		c.mins = c.mins[1:]
	}
	return time.Unix(0, stamp+c.mins[0].offset)
}
