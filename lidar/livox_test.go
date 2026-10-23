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
	"bytes"
	"errors"
	"hash/crc32"
	"math"
	"net"
	"strings"
	"testing"
	"time"
)

// These tests are built from the same reading of the Livox documents as the
// code. They catch a slip in the code, not a misreading of the documents; the
// packets in testdata, once captured, are what catch that.

func TestTheChecksumsGiveTheirCheckValues(t *testing.T) {
	check := []byte("123456789")
	if got := crc16CCITT(check); got != 0x29B1 {
		t.Errorf("CRC-16/CCITT-FALSE = 0x%04X, want 0x29B1", got)
	}
	if got := crc32.ChecksumIEEE(check); got != 0xCBF43926 {
		t.Errorf("CRC-32 = 0x%08X, want 0xCBF43926", got)
	}
}

func TestAControlFrameRoundTrips(t *testing.T) {
	f := controlFrame{seq: 7, cmdID: cmdParamConfig, cmdType: cmdTypeRequest, senderType: senderHost, data: []byte{1, 2, 3}}
	b := f.marshal()
	if b[0] != 0xAA || len(b) != 27 || le.Uint16(b[2:]) != 27 {
		t.Fatalf("header % x, want 0xAA and a length of 27", b[:4])
	}
	g, err := parseControlFrame(b)
	if err != nil {
		t.Fatal(err)
	}
	if g.seq != 7 || g.cmdID != cmdParamConfig || !bytes.Equal(g.data, f.data) {
		t.Errorf("round trip gave %+v, want %+v", g, f)
	}
	for _, i := range []int{4, 25} { // one byte of the header, one of the data
		bad := append([]byte(nil), b...)
		bad[i] ^= 0x01
		if _, err := parseControlFrame(bad); !errors.Is(err, errBadCRC) {
			t.Errorf("a frame with byte %d flipped gave %v, want a CRC mismatch", i, err)
		}
	}
}

// imuPacket builds an IMU packet the way the documents describe it.
func imuPacket(udpCnt uint16, timestamp uint64, gyro, accel [3]float32) []byte {
	b := make([]byte, dataHeaderLen+imuRecordLen)
	b[0] = 0
	le.PutUint16(b[1:], uint16(len(b)))
	le.PutUint16(b[5:], 1)
	le.PutUint16(b[7:], udpCnt)
	b[10] = dataTypeIMU
	le.PutUint64(b[28:], timestamp)
	for i := range 3 {
		le.PutUint32(b[36+4*i:], math.Float32bits(gyro[i]))
		le.PutUint32(b[48+4*i:], math.Float32bits(accel[i]))
	}
	le.PutUint32(b[24:], crc32.ChecksumIEEE(b[28:]))
	return b
}

func TestAnIMUPacketDecodes(t *testing.T) {
	b := imuPacket(42, 123456789, [3]float32{0.1, -0.2, 0.3}, [3]float32{0.5, 0, 0.866})
	h, data, err := parsePacket(b)
	if err != nil {
		t.Fatal(err)
	}
	if h.udpCnt != 42 || h.timestamp != 123456789 || h.dataType != dataTypeIMU || h.dotNum != 1 {
		t.Errorf("header %+v", h)
	}
	recs, err := parseIMU(data, h.dotNum)
	if err != nil {
		t.Fatal(err)
	}
	if len(recs) != 1 {
		t.Fatalf("%d records, want 1", len(recs))
	}
	if r := recs[0]; float32(r.gyro[1]) != -0.2 || float32(r.accel[2]) != 0.866 {
		t.Errorf("record %+v", r)
	}
}

func TestBrokenPacketsAreCountedNotUsed(t *testing.T) {
	s := &livoxSource{clock: newClockMap(clockWindow)}
	now := time.Now()
	good := imuPacket(1, 0, [3]float32{}, [3]float32{0, 0, 1})

	corrupt := append([]byte(nil), good...)
	corrupt[40] ^= 0xFF
	points := imuPacket(2, 0, [3]float32{}, [3]float32{})
	points[10] = 1 // a point-cloud packet on the IMU port
	le.PutUint32(points[24:], crc32.ChecksumIEEE(points[28:]))

	for _, b := range [][]byte{corrupt, good[:20], points} {
		if got := s.decode(b, now); got != nil {
			t.Errorf("a broken packet gave %d samples", len(got))
		}
	}
	if s.badCRC.Load() != 1 || s.malformed.Load() != 1 || s.foreign.Load() != 1 || s.packets.Load() != 0 {
		t.Errorf("counted %s", s.status())
	}
}

func TestLostPacketsAreCountedFromTheUDPCounter(t *testing.T) {
	s := &livoxSource{clock: newClockMap(clockWindow)}
	now := time.Now()
	for _, cnt := range []uint16{65534, 65535, 2, 3} { // 0 and 1 lost, across the wrap
		ts := uint64(cnt) * uint64(5*time.Millisecond)
		if got := s.decode(imuPacket(cnt, ts, [3]float32{}, [3]float32{0, 0, 1}), now); len(got) != 1 {
			t.Fatalf("packet %d gave %d samples", cnt, len(got))
		}
	}
	if s.lost.Load() != 2 {
		t.Errorf("%d lost, want 2", s.lost.Load())
	}
}

// fakeDevice answers control frames on loopback, as a MID-360 would.
func fakeDevice(t *testing.T, answer func(controlFrame) []byte) (*net.UDPAddr, <-chan controlFrame) {
	t.Helper()
	conn, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1)})
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { conn.Close() })
	got := make(chan controlFrame, 8)
	go func() {
		buf := make([]byte, 1500)
		for {
			n, from, err := conn.ReadFromUDP(buf)
			if err != nil {
				return
			}
			req, err := parseControlFrame(buf[:n])
			if err != nil {
				t.Errorf("the device received a bad frame: %v", err)
				continue
			}
			got <- req
			if data := answer(req); data != nil {
				ack := controlFrame{seq: req.seq, cmdID: req.cmdID, cmdType: cmdTypeAck, senderType: 1, data: data}
				conn.WriteToUDP(ack.marshal(), from)
			}
		}
	}()
	return conn.LocalAddr().(*net.UDPAddr), got
}

func testDevice(t *testing.T, at *net.UDPAddr) *livoxDevice {
	t.Helper()
	ctl, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1)})
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { ctl.Close() })
	return &livoxDevice{ctl: ctl, addr: at, host: net.IPv4(127, 0, 0, 1).To4(), timeout: 50 * time.Millisecond}
}

func TestConfigureSendsTheIMUKeys(t *testing.T) {
	addr, got := fakeDevice(t, func(controlFrame) []byte { return []byte{0, 0, 0} })
	d := testDevice(t, addr)
	if _, err := d.configure(testConfig()); err != nil {
		t.Fatal(err)
	}
	req := <-got
	if req.cmdID != cmdParamConfig || req.cmdType != cmdTypeRequest {
		t.Fatalf("command 0x%04x type %d, want a 0x0100 request", req.cmdID, req.cmdType)
	}
	want := paramConfig([]keyValue{
		{0x0007, []byte{127, 0, 0, 1, 0x51, 0xDC, 0x50, 0xDC}}, // to 127.0.0.1:56401, from 56400
		{0x001C, []byte{1}},
		{0x001A, []byte{1}},
	})
	if !bytes.Equal(req.data, want) {
		t.Errorf("configuration\n% x\nwant\n% x", req.data, want)
	}
}

func TestConfigureReportsARefusal(t *testing.T) {
	addr, _ := fakeDevice(t, func(controlFrame) []byte { return []byte{0x01, 0x1C, 0x00} })
	_, err := testDevice(t, addr).configure(testConfig())
	if err == nil || !strings.Contains(err.Error(), "0x001c") {
		t.Errorf("a refusal at key 0x001C gave %v", err)
	}
}

func TestConfigureGivesUpOnSilence(t *testing.T) {
	addr, got := fakeDevice(t, func(controlFrame) []byte { return nil })
	if _, err := testDevice(t, addr).configure(testConfig()); err == nil {
		t.Fatal("a silent device was taken as configured")
	}
	if len(got) != 3 {
		t.Errorf("the request was sent %d times, want 3", len(got))
	}
}
