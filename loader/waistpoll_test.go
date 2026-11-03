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
	"errors"
	"testing"
	"time"
)

// sensorBus is can1 as the waist socket sees it: a queue of frames, and a
// sensor that answers each poll with whatever answer says, if anything.
type sensorBus struct {
	queue  []canFrame
	answer func() []canFrame
}

func reply(raw int) canFrame {
	return canFrame{ID: waistReplyID, DLC: 2, Data: [8]byte{byte(raw >> 8), byte(raw)}}
}

func (b *sensorBus) poll(timeout time.Duration) (int, int, error) {
	raw, _, discarded, err := pollOnce(
		func() error {
			if b.answer != nil {
				b.queue = append(b.queue, b.answer()...)
			}
			return nil
		},
		func(time.Duration) (canFrame, error) {
			if len(b.queue) == 0 {
				return canFrame{}, errCANTimeout
			}
			f := b.queue[0]
			b.queue = b.queue[1:]
			return f, nil
		},
		func() int {
			n := len(b.queue)
			b.queue = nil
			return n
		},
		timeout)
	return raw, discarded, err
}

// The backlog that made the steering follow an angle seconds old: answers to
// earlier polls, and frames nobody asked for, waiting ahead of this one.
func TestPollTakesTheAnswerToThisPoll(t *testing.T) {
	b := &sensorBus{queue: []canFrame{reply(400), reply(401), {ID: 0x123, DLC: 8}, reply(402)}}
	b.answer = func() []canFrame { return []canFrame{reply(450)} }
	raw, discarded, err := b.poll(waistReplyTimeout)
	if err != nil {
		t.Fatal(err)
	}
	if raw != 450 {
		t.Errorf("read count %d, want 450, the answer to this poll", raw)
	}
	if discarded != 4 {
		t.Errorf("discarded %d frames, want the 4 that were waiting", discarded)
	}
	if len(b.queue) != 0 {
		t.Errorf("%d frames left behind", len(b.queue))
	}
}

// A sensor that answers twice must not leave every later poll one answer
// behind, as it did when one frame was read per poll.
func TestPollDoesNotFallBehind(t *testing.T) {
	count := 300
	b := &sensorBus{answer: func() []canFrame {
		count++
		return []canFrame{reply(count), reply(count)}
	}}
	for i := 0; i < 50; i++ {
		raw, _, err := b.poll(waistReplyTimeout)
		if err != nil {
			t.Fatal(err)
		}
		if raw != count {
			t.Fatalf("poll %d read count %d, want %d: it is reading old answers", i, raw, count)
		}
	}
}

// An answer that comes after its poll has given up is not the next poll's.
func TestLateAnswerIsNotTakenByTheNextPoll(t *testing.T) {
	b := &sensorBus{}
	if _, _, err := b.poll(waistReplyTimeout); !errors.Is(err, errCANTimeout) {
		t.Fatalf("no answer gave %v, want a timeout", err)
	}
	b.queue = append(b.queue, reply(380)) // the late one
	b.answer = func() []canFrame { return []canFrame{reply(460)} }
	raw, discarded, err := b.poll(waistReplyTimeout)
	if err != nil {
		t.Fatal(err)
	}
	if raw != 460 || discarded != 1 {
		t.Errorf("read %d after discarding %d, want 460 after discarding the late 380", raw, discarded)
	}
}

// A frame on the answer's ID too short to carry a count, a heartbeat say, is
// passed over, not read as an angle.
func TestShortFrameIsNotAnAnswer(t *testing.T) {
	b := &sensorBus{answer: func() []canFrame {
		return []canFrame{{ID: waistReplyID, DLC: 1, Data: [8]byte{0x05}}, reply(470)}
	}}
	raw, discarded, err := b.poll(waistReplyTimeout)
	if err != nil {
		t.Fatal(err)
	}
	if raw != 470 || discarded != 1 {
		t.Errorf("read %d after discarding %d, want 470 after passing over the short frame", raw, discarded)
	}
}

func TestPollGivesUpAtItsDeadline(t *testing.T) {
	b := &sensorBus{}
	if _, _, err := b.poll(time.Nanosecond); !errors.Is(err, errCANTimeout) {
		t.Errorf("got %v, want a timeout", err)
	}
}
