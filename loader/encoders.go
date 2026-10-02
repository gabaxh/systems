/*******************************************************************************
 * Copyright (c) 2026 Synecdoque
 *
 * The software is licensed under the MIT License. See the LICENSE file in this
 * repository for details.
 *
 * Contributors:
 *   Jan A. van Deventer, Luleå - initial implementation
 ***************************************************************************SDG*/

package main

// Reading what the machine actually did, as opposed to what it was told to do.
//
// Every constant here is taken from the artitrax can_dds bridge, which is the
// tested reference against this hardware:
//
//	can0, 500 kbit/s   motors 0x601-0x605, wheel encoders 0x18B-0x18E
//	can1, 250 kbit/s   waist angle sensor, polled at 0x700, answers at 0x701
//
// The distinction matters more than it looks. Until now this system published
// the setpoint it was holding and called it speed, which is the commanded value
// and not a measurement: a stalled motor, a slipping wheel and a working one all
// report the number that was asked for. Anything downstream that integrates it —
// odometry, and therefore a map — would be integrating a wish.

import (
	"context"
	"errors"
	"log"
	"sync"
	"time"
)

var errCANTimeout = errors.New("no CAN frame within the timeout")

const (
	// Wheel encoders report on their own once started, one CAN ID each, in the
	// order the motors are numbered: front left, front right, back left, back
	// right.
	encoderBaseID = 0x18B
	encoderCount  = 4

	// 4096 counts per encoder revolution through a 20:1 gearbox.
	encoderCountsPerRev = 4096.0
	gearboxRatio        = 20.0
	countsPerOutputRev  = encoderCountsPerRev * gearboxRatio // 81 920

	// The speed field counts encoder edges in a 5 ms window.
	speedWindowMs = 5.0

	// The waist sensor is polled and answers with a 10-bit value.
	waistPollID  = 0x700
	waistReplyID = 0x701
)

// initEncoders configures the encoders and starts them, as can_dds does.
//
// They are CANopen nodes, and a CANopen node comes up pre-operational and says
// nothing until it is told to start. This system first assumed the encoders
// reported unsolicited, and on the vehicle they reported nothing; the students
// found the missing step in the reference and ported it (fix-encoder-init).
//
// Each node gets three SDO writes and then an NMT start:
//
//	0x2005 = 2    PDO1 carries position, speed and acceleration
//	0x6200 = 50   send it every 50 ms
//	0x6003 = 0    preset the position to zero
//
// The preset means the position counts from when this system started, not from
// when the encoder powered up.
//
// Nodes 0x0B to 0x0F: the reference starts five, one more than there are
// wheels. Nothing here reads the fifth (it would report on 0x18F); it is
// started because the reference starts it, and a node left pre-operational is
// the kind of difference that is expensive to find later.
//
// The SDO frames are seven bytes, exactly as the reference sends them. CANopen
// specifies eight, and a stricter node could refuse them; these do not.
//
// preset is for start-up only. A restart of encoders that have gone quiet
// leaves their position alone: the travel counted so far is carried on from
// where it was (see unwrapLocked), not started again.
func configureEncoders(fd int, preset bool) {
	send := func(id uint32, data []byte, what string, node byte) {
		if err := sendCAN(fd, id, data); err != nil {
			log.Printf("loader: encoder 0x%02X: %s: %v", node, what, err)
		}
	}
	for node := byte(0x0B); node <= 0x0F; node++ {
		sdo := uint32(0x600) + uint32(node)
		send(sdo, []byte{0x2F, 0x05, 0x20, 0x00, 0x02, 0x00, 0x00}, "select PDO type 2", node)
		time.Sleep(10 * time.Millisecond)
		send(sdo, []byte{0x2B, 0x00, 0x62, 0x00, 0x32, 0x00, 0x00}, "set the 50 ms cycle", node)
		if preset {
			send(sdo, []byte{0x23, 0x03, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}, "preset the position to zero", node)
		}
		time.Sleep(10 * time.Millisecond)
		send(0x000, []byte{0x01, node}, "NMT start", node)
	}
	if preset {
		log.Println("loader: wheel encoders configured and started")
	}
}

// wheelReading is one wheel's own account of itself.
type wheelReading struct {
	// revolutions is continuous: the encoder's 24-bit counter wraps every
	// 204.8 revolutions, and the listener unwraps it, so a consumer can simply
	// difference two readings. It counts from when this system started and
	// preset the counter; a restart of this system starts it again at zero.
	revolutions float64
	rpm         float64 // of the output shaft, after the gearbox
	at          time.Time

	count uint32 // the frame's own 24-bit counter, before unwrapping
}

// feedback is everything the machine reports back, shared by the assets that
// serve it.
type feedback struct {
	mu sync.RWMutex

	wheels [encoderCount]wheelReading

	// The unwrapping: each wheel's last counter value and its running total.
	lastCount [encoderCount]uint32
	total     [encoderCount]int64
	started   [encoderCount]bool

	waistRaw   int // the 10-bit sensor value, before any scaling
	waistAt    time.Time
	waistFresh bool

	staleAfter time.Duration
	// clock is what freshness is judged against: the wall clock, except in a
	// test that runs the loader on simulated time.
	clock func() time.Time

	// Called with every fresh reading, outside the lock, so that the services
	// can hand it to whoever follows them. Set once, before the listeners start.
	onWheel func(index int, r wheelReading)
	onWaist func(raw int, at time.Time)
}

func newFeedback(staleAfter time.Duration) *feedback {
	return &feedback{staleAfter: staleAfter, clock: time.Now}
}

// decodeWheel unpacks one encoder frame.
//
// Position is a free-running 24-bit counter in bytes 0-2 and speed is a signed
// count of edges per 5 ms window in bytes 4-5, both little-endian.
func decodeWheel(f canFrame) (index int, r wheelReading, ok bool) {
	index = int(f.ID) - encoderBaseID
	if index < 0 || index >= encoderCount {
		return 0, wheelReading{}, false
	}
	if f.DLC < 6 {
		return 0, wheelReading{}, false
	}
	raw := uint32(f.Data[2])<<16 | uint32(f.Data[1])<<8 | uint32(f.Data[0])
	rawSpeed := int16(uint16(f.Data[4]) | uint16(f.Data[5])<<8)

	r.count = raw
	r.revolutions = float64(raw) / countsPerOutputRev
	r.rpm = float64(rawSpeed) * 1000 * 60 / (speedWindowMs * countsPerOutputRev)

	// The left-hand encoders are mounted facing the other way and count
	// backwards, so their sign is flipped here rather than in every consumer.
	// Indices 0 and 2 are front left and back left.
	if index%2 == 0 {
		r.revolutions = -r.revolutions
		r.rpm = -r.rpm
	}
	r.at = time.Now()
	return index, r, true
}

//-------------------------------------The listeners

// listenEncoders folds every wheel-encoder frame on the motor bus into the
// shared feedback. It shares the bus with the motor commands, which is why it
// reads on its own socket: a reader and a writer on one socket would have to
// take turns, and the command loop must never wait for a sensor.
func (fb *feedback) listenEncoders(ctx context.Context, fd int) {
	for {
		select {
		case <-ctx.Done():
			return
		default:
		}
		f, err := recvCAN(fd, 250*time.Millisecond)
		if err != nil {
			if errors.Is(err, errCANTimeout) {
				continue
			}
			log.Printf("loader: encoder bus read failed: %v", err)
			time.Sleep(time.Second)
			continue
		}
		if i, r, ok := decodeWheel(f); ok {
			fb.mu.Lock()
			r = fb.unwrapLocked(i, r)
			fb.wheels[i] = r
			fb.mu.Unlock()
			if fb.onWheel != nil {
				fb.onWheel(i, r)
			}
		}
	}
}

// unwrapLocked replaces a frame's wrapping count with the wheel's continuous
// total. Frames arrive every 50 ms, far more often than a wheel could turn half
// the counter's range, so the shortest way round is always the right one.
// Caller holds the lock.
//
// After a silence of encoderSilence or more the frame is not differenced at all:
// the total carries on from where it was. An encoder that went quiet had most
// likely reset — a dip in the supply under stall current, on 1 October 2026 —
// and comes back counting from wherever its counter now starts. Differenced, that
// was a jump of up to half the counter's range, 100 revolutions, in travel,
// distance and the cartographer's odometry. What the wheel turned during the
// silence is lost either way.
func (fb *feedback) unwrapLocked(i int, r wheelReading) wheelReading {
	const span = int64(1) << 24
	prev := fb.wheels[i].at
	resync := fb.started[i] && !prev.IsZero() && r.at.Sub(prev) >= encoderSilence
	switch {
	case !fb.started[i]:
		fb.total[i], fb.started[i] = int64(r.count), true
	case resync:
		// The total stands, and counting goes on from this frame.
	default:
		d := int64(r.count) - int64(fb.lastCount[i])
		switch {
		case d > span/2:
			d -= span
		case d < -span/2:
			d += span
		}
		fb.total[i] += d
	}
	fb.lastCount[i] = r.count
	r.revolutions = float64(fb.total[i]) / countsPerOutputRev
	// The left-hand encoders count backwards; see decodeWheel.
	if i%2 == 0 {
		r.revolutions = -r.revolutions
	}
	return r
}

// pollWaist asks the articulation sensor for its angle and records the answer.
func (fb *feedback) pollWaist(ctx context.Context, fd int, period time.Duration) {
	tick := time.NewTicker(period)
	defer tick.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-tick.C:
		}
		if err := sendCAN(fd, waistPollID, nil); err != nil {
			log.Printf("loader: cannot poll the waist sensor: %v", err)
			continue
		}
		f, err := recvCAN(fd, 200*time.Millisecond)
		if err != nil {
			if !errors.Is(err, errCANTimeout) {
				log.Printf("loader: waist sensor read failed: %v", err)
			}
			fb.mu.Lock()
			fb.waistFresh = false
			fb.mu.Unlock()
			continue
		}
		if f.ID != waistReplyID || f.DLC < 2 {
			continue
		}
		raw, at := int(f.Data[0]&0x03)<<8|int(f.Data[1]), time.Now()
		fb.mu.Lock()
		fb.waistRaw, fb.waistAt, fb.waistFresh = raw, at, true
		fb.mu.Unlock()
		if fb.onWaist != nil {
			fb.onWaist(raw, at)
		}
	}
}

// encoderSilence is how long a wheel encoder may say nothing before it is
// restarted. It reports every 50 ms, so a second is twenty frames missed.
const encoderSilence = time.Second

// encoderRestartEvery is the least time between two restarts: the
// configuration takes a tenth of a second to send, and a node that is still
// booting gets a moment to finish.
const encoderRestartEvery = 2 * time.Second

// superviseEncoders restarts wheel encoders that have gone quiet.
//
// They are CANopen nodes, and a node that resets comes up pre-operational and
// says nothing until it is started again. This system started them once, at
// start-up, so an encoder that reset stayed silent until the loader itself was
// restarted: twice on 1 October 2026, on a slope, with the motors stalled. The
// speed loop fell back to open loop as it should, but every wheel's speed and
// travel were gone. Now a wheel quiet for encoderSilence gets the configuration
// and NMT start again, without the position preset.
func (fb *feedback) superviseEncoders(ctx context.Context, fd int) {
	tick := time.NewTicker(time.Second)
	defer tick.Stop()
	var lastRestart time.Time
	quietBefore := false
	for {
		select {
		case <-ctx.Done():
			return
		case <-tick.C:
		}
		now := time.Now()
		quiet := fb.quietWheels(now)
		if len(quiet) == 0 {
			if quietBefore {
				log.Println("loader: the wheel encoders report again")
				quietBefore = false
			}
			continue
		}
		if now.Sub(lastRestart) < encoderRestartEvery {
			continue
		}
		if !quietBefore {
			log.Printf("loader: wheel encoder(s) %v silent for over %v — restarting them", quiet, encoderSilence)
			quietBefore = true
		}
		configureEncoders(fd, false)
		lastRestart = now
	}
}

// quietWheels names the wheels whose encoder has said nothing for
// encoderSilence, including any that never has.
func (fb *feedback) quietWheels(now time.Time) []string {
	names := [encoderCount]string{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}
	fb.mu.RLock()
	defer fb.mu.RUnlock()
	var quiet []string
	for i, w := range fb.wheels {
		if w.at.IsZero() || now.Sub(w.at) >= encoderSilence {
			quiet = append(quiet, names[i])
		}
	}
	return quiet
}

//-------------------------------------Reading it back

// wheel returns one wheel's reading and whether it is recent enough to use.
func (fb *feedback) wheel(index int) (wheelReading, bool) {
	fb.mu.RLock()
	defer fb.mu.RUnlock()
	if index < 0 || index >= encoderCount {
		return wheelReading{}, false
	}
	r := fb.wheels[index]
	if r.at.IsZero() || fb.clock().Sub(r.at) > fb.staleAfter {
		return r, false
	}
	return r, true
}

// waistLatest returns the newest reading and when it arrived, whatever its age;
// the steering guard judges freshness by its own, much shorter, standard.
func (fb *feedback) waistLatest() (raw int, at time.Time, have bool) {
	fb.mu.RLock()
	defer fb.mu.RUnlock()
	return fb.waistRaw, fb.waistAt, fb.waistFresh
}

// waist returns the raw sensor value and whether it is recent enough to use.
func (fb *feedback) waist() (int, time.Time, bool) {
	fb.mu.RLock()
	defer fb.mu.RUnlock()
	if !fb.waistFresh || fb.clock().Sub(fb.waistAt) > fb.staleAfter {
		return fb.waistRaw, fb.waistAt, false
	}
	return fb.waistRaw, fb.waistAt, true
}
