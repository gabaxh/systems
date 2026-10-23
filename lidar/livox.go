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

// The Livox MID-360 over Ethernet, in pure Go, without the C++ SDK.
//
// Written from the Livox SDK2 header livox_lidar_def.h and the MID-360
// Ethernet protocol page of the Livox wiki. A capture of a MID-360 on
// 2026-10-23, configured by this system, confirmed the points below except
// where marked; a test built from the same reading of the documents cannot
// catch a misreading of them, so check any new point against a capture too.
//
// From the documents, and confirmed on the capture:
//   - a data packet (points or IMU) has a 36-byte little-endian header:
//     version u8, length u16, time_interval u16 (0.1 µs), dot_num u16,
//     udp_cnt u16, frame_cnt u8, data_type u8, time_type u8, 12 reserved
//     bytes, crc32 u32 over the timestamp and the data, timestamp u64 in ns;
//   - data_type 0 is IMU: gyro x, y, z as float32 rad/s, then the
//     accelerometer x, y, z as float32 in g, not m/s²;
//   - time_type 0 is nanoseconds since the device powered on, 1 PTP, 2 GPS;
//   - a control frame is 0xAA, version 0, length u16 of the whole frame,
//     seq u32, cmd_id u16, cmd_type u8 (0 request, 1 acknowledgement),
//     sender_type u8 (0 host, 1 the device), 6 reserved bytes, a CRC-16/CCITT-FALSE of the
//     18 bytes before it, a CRC-32 of the data (0 when there is none), then
//     the data;
//   - 0x0100 sets parameters, given as key_num u16, 2 reserved bytes, and
//     key u16, length u16, value for each;
//   - key 0x0007 is where the IMU is sent: this computer's IPv4 address, its
//     port, and the device's own sending port; 0x001C switches the IMU stream
//     on; 0x001A is the work mode, 0x01 sampling and 0x02 idle;
//   - the device listens for commands on 56100 and discovery on 56000 (not
//     yet seen on a capture), and sends the IMU from 56400;
//   - an acknowledgement of 0x0100 carries ret_code u8 (0 is success) and the
//     failing key u16, 0 when none failed;
//   - one IMU sample per 60-byte packet, dot_num 1 and time_interval 0, at
//     200 Hz. Several are still handled, spread over time_interval, in case
//     dot_num says so.
//
// Assumed, and to check on a capture:
//   - an acknowledgement of discovery carries ret_code u8, dev_type u8, the
//     serial number in 16 bytes, the device's IPv4 address and its command
//     port; when it is shorter, the sender's address is used;
//   - whether the IMU flows when the device is idle (the workMode setting).

import (
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"hash/crc32"
	"math"
	"net"
	"strings"
	"sync/atomic"
	"time"
)

const (
	livoxSOF         = 0xAA
	controlHeaderLen = 24
	dataHeaderLen    = 36
	imuRecordLen     = 24

	cmdDiscovery   = 0x0000
	cmdParamConfig = 0x0100

	cmdTypeRequest = 0x00
	cmdTypeAck     = 0x01
	senderHost     = 0x00

	keyIMUHostIPConfig = 0x0007
	keyWorkMode        = 0x001A
	keyIMUDataEnable   = 0x001C

	dataTypeIMU = 0

	// The device's own ports. Its point cloud comes from 56300, for the
	// point-cloud asset.
	lidarDiscoveryPort = 56000
	lidarCommandPort   = 56100
	lidarIMUPort       = 56400
)

var le = binary.LittleEndian

// workModes are the work modes a host may ask for, by their configuration name.
var workModes = map[string]uint8{"sampling": 0x01, "idle": 0x02}

var errBadCRC = errors.New("CRC mismatch")

//-------------------------------------Checksums

// crc16CCITT is CRC-16/CCITT-FALSE: polynomial 0x1021, initial 0xFFFF, no
// reflection, no final XOR. The CRC-32 is the ordinary IEEE one, which
// hash/crc32 computes.
func crc16CCITT(b []byte) uint16 {
	crc := uint16(0xFFFF)
	for _, c := range b {
		crc ^= uint16(c) << 8
		for range 8 {
			if crc&0x8000 != 0 {
				crc = crc<<1 ^ 0x1021
			} else {
				crc <<= 1
			}
		}
	}
	return crc
}

//-------------------------------------Control frames

type controlFrame struct {
	seq        uint32
	cmdID      uint16
	cmdType    uint8
	senderType uint8
	data       []byte
}

func (f controlFrame) marshal() []byte {
	b := make([]byte, controlHeaderLen+len(f.data))
	b[0] = livoxSOF
	le.PutUint16(b[2:], uint16(len(b)))
	le.PutUint32(b[4:], f.seq)
	le.PutUint16(b[8:], f.cmdID)
	b[10] = f.cmdType
	b[11] = f.senderType
	le.PutUint16(b[18:], crc16CCITT(b[:18]))
	if len(f.data) > 0 {
		le.PutUint32(b[20:], crc32.ChecksumIEEE(f.data))
	}
	copy(b[controlHeaderLen:], f.data)
	return b
}

func parseControlFrame(b []byte) (controlFrame, error) {
	if len(b) < controlHeaderLen {
		return controlFrame{}, fmt.Errorf("a control frame of %d bytes is shorter than its header", len(b))
	}
	if b[0] != livoxSOF {
		return controlFrame{}, fmt.Errorf("a control frame starting 0x%02x, not 0xAA", b[0])
	}
	n := int(le.Uint16(b[2:]))
	if n < controlHeaderLen || n > len(b) {
		return controlFrame{}, fmt.Errorf("a control frame claiming %d bytes in %d", n, len(b))
	}
	if crc16CCITT(b[:18]) != le.Uint16(b[18:]) {
		return controlFrame{}, fmt.Errorf("control frame header: %w", errBadCRC)
	}
	data := b[controlHeaderLen:n]
	if len(data) > 0 && crc32.ChecksumIEEE(data) != le.Uint32(b[20:]) {
		return controlFrame{}, fmt.Errorf("control frame data: %w", errBadCRC)
	}
	return controlFrame{
		seq:        le.Uint32(b[4:]),
		cmdID:      le.Uint16(b[8:]),
		cmdType:    b[10],
		senderType: b[11],
		data:       append([]byte(nil), data...),
	}, nil
}

type keyValue struct {
	key   uint16
	value []byte
}

// paramConfig is the data of a 0x0100 request.
func paramConfig(kvs []keyValue) []byte {
	b := make([]byte, 4, 64)
	le.PutUint16(b, uint16(len(kvs)))
	for _, kv := range kvs {
		b = le.AppendUint16(b, kv.key)
		b = le.AppendUint16(b, uint16(len(kv.value)))
		b = append(b, kv.value...)
	}
	return b
}

// hostIPConfig is the value of a host IP key: send to ip:port, from the
// device's port from.
func hostIPConfig(ip net.IP, port, from int) []byte {
	b := make([]byte, 8)
	copy(b, ip.To4())
	le.PutUint16(b[4:], uint16(port))
	le.PutUint16(b[6:], uint16(from))
	return b
}

//-------------------------------------Data packets

type packetHeader struct {
	length, timeInterval, dotNum, udpCnt uint16
	frameCnt, dataType, timeType         uint8
	timestamp                            uint64 // nanoseconds
}

// parsePacket checks a data packet and splits it into its header and data.
func parsePacket(b []byte) (packetHeader, []byte, error) {
	if len(b) < dataHeaderLen {
		return packetHeader{}, nil, fmt.Errorf("a packet of %d bytes is shorter than its header", len(b))
	}
	if crc32.ChecksumIEEE(b[28:]) != le.Uint32(b[24:]) {
		return packetHeader{}, nil, errBadCRC
	}
	return packetHeader{
		length:       le.Uint16(b[1:]),
		timeInterval: le.Uint16(b[3:]),
		dotNum:       le.Uint16(b[5:]),
		udpCnt:       le.Uint16(b[7:]),
		frameCnt:     b[9],
		dataType:     b[10],
		timeType:     b[11],
		timestamp:    le.Uint64(b[28:]),
	}, b[dataHeaderLen:], nil
}

type imuRecord struct {
	gyro  [3]float64 // rad/s
	accel [3]float64 // g
}

// parseIMU reads the IMU records of a packet's data, at most dotNum of them
// when dotNum is set.
func parseIMU(data []byte, dotNum uint16) ([]imuRecord, error) {
	n := len(data) / imuRecordLen
	if n == 0 {
		return nil, fmt.Errorf("%d bytes of IMU data, less than one record", len(data))
	}
	if dotNum > 0 && int(dotNum) < n {
		n = int(dotNum)
	}
	recs := make([]imuRecord, n)
	for i := range recs {
		r := data[i*imuRecordLen:]
		for j := range 3 {
			recs[i].gyro[j] = float64(math.Float32frombits(le.Uint32(r[4*j:])))
			recs[i].accel[j] = float64(math.Float32frombits(le.Uint32(r[12+4*j:])))
		}
	}
	return recs, nil
}

//-------------------------------------The device

// livoxDevice is the command side of one MID-360. It is used from one
// goroutine at a time.
type livoxDevice struct {
	ctl     *net.UDPConn
	addr    *net.UDPAddr // the device's command port; nil until found
	host    net.IP       // nil means the local address on the route to it
	serial  string
	seq     uint32
	timeout time.Duration
}

func openDevice(cfg LidarConfig) (*livoxDevice, error) {
	d := &livoxDevice{timeout: time.Second}
	if cfg.DeviceIP != "" {
		ip := net.ParseIP(cfg.DeviceIP).To4()
		if ip == nil {
			return nil, fmt.Errorf("deviceIP %q is not an IPv4 address", cfg.DeviceIP)
		}
		d.addr = &net.UDPAddr{IP: ip, Port: lidarCommandPort}
	}
	if cfg.HostIP != "" {
		if d.host = net.ParseIP(cfg.HostIP).To4(); d.host == nil {
			return nil, fmt.Errorf("hostIP %q is not an IPv4 address", cfg.HostIP)
		}
	}
	lc := net.ListenConfig{Control: allowBroadcast}
	pc, err := lc.ListenPacket(context.Background(), "udp4", fmt.Sprintf(":%d", cfg.ControlPort))
	if err != nil {
		return nil, fmt.Errorf("control port %d: %w", cfg.ControlPort, err)
	}
	d.ctl = pc.(*net.UDPConn)
	return d, nil
}

func (d *livoxDevice) close() error { return d.ctl.Close() }

// request sends a command and waits for its acknowledgement, three times
// before giving up. It returns the acknowledgement's data and who sent it.
func (d *livoxDevice) request(cmd uint16, data []byte, to *net.UDPAddr) ([]byte, *net.UDPAddr, error) {
	d.seq++
	frame := controlFrame{seq: d.seq, cmdID: cmd, cmdType: cmdTypeRequest, senderType: senderHost, data: data}.marshal()
	buf := make([]byte, 1500)
	for range 3 {
		if _, err := d.ctl.WriteToUDP(frame, to); err != nil {
			return nil, nil, fmt.Errorf("sending command 0x%04x to %s: %w", cmd, to, err)
		}
		deadline := time.Now().Add(d.timeout)
		for {
			if err := d.ctl.SetReadDeadline(deadline); err != nil {
				return nil, nil, err
			}
			n, from, err := d.ctl.ReadFromUDP(buf)
			if err != nil {
				if ne, ok := err.(net.Error); ok && ne.Timeout() {
					break
				}
				return nil, nil, err
			}
			ack, err := parseControlFrame(buf[:n])
			if err != nil || ack.cmdType != cmdTypeAck || ack.cmdID != cmd || ack.seq != d.seq {
				continue // a stray frame, or the answer to an earlier try
			}
			return ack.data, from, nil
		}
	}
	return nil, nil, fmt.Errorf("no answer to command 0x%04x from %s", cmd, to)
}

// discover finds the device by broadcast.
func (d *livoxDevice) discover() error {
	data, from, err := d.request(cmdDiscovery, nil, &net.UDPAddr{IP: net.IPv4bcast, Port: lidarDiscoveryPort})
	if err != nil {
		return fmt.Errorf("no MID-360 answered the discovery broadcast; set deviceIP: %w", err)
	}
	ip, port := from.IP, lidarCommandPort
	if len(data) >= 24 {
		d.serial = strings.TrimRight(string(data[2:18]), "\x00")
		ip = net.IPv4(data[18], data[19], data[20], data[21])
		if p := int(le.Uint16(data[22:])); p != 0 {
			port = p
		}
	}
	d.addr = &net.UDPAddr{IP: ip, Port: port}
	logf("found the MID-360 %s at %s", d.serial, ip)
	return nil
}

// configure tells the device where to send its IMU, to send it, and which work
// mode to be in, all in one request. It returns the address the IMU now goes
// to.
func (d *livoxDevice) configure(cfg LidarConfig) (net.IP, error) {
	if d.addr == nil {
		if err := d.discover(); err != nil {
			return nil, err
		}
	}
	host := d.host
	if host == nil {
		var err error
		if host, err = localIPToward(d.addr); err != nil {
			return nil, err
		}
	}
	data := paramConfig([]keyValue{
		{keyIMUHostIPConfig, hostIPConfig(host, cfg.IMUPort, lidarIMUPort)},
		{keyIMUDataEnable, []byte{1}},
		{keyWorkMode, []byte{workModes[cfg.WorkMode]}},
	})
	ack, _, err := d.request(cmdParamConfig, data, d.addr)
	if err != nil {
		return nil, err
	}
	if len(ack) == 0 {
		return nil, errors.New("the MID-360 acknowledged the configuration with nothing in it")
	}
	if ack[0] != 0 {
		key := ""
		if len(ack) >= 3 {
			key = fmt.Sprintf(" at key 0x%04x", le.Uint16(ack[1:]))
		}
		return nil, fmt.Errorf("the MID-360 refused the configuration (return code 0x%02x%s)", ack[0], key)
	}
	return host, nil
}

// localIPToward is this computer's address on the route to addr. Dialling UDP
// sends nothing; it only picks the route.
func localIPToward(addr *net.UDPAddr) (net.IP, error) {
	c, err := net.DialUDP("udp4", nil, addr)
	if err != nil {
		return nil, fmt.Errorf("no route to the MID-360 at %s: %w", addr.IP, err)
	}
	defer c.Close()
	return c.LocalAddr().(*net.UDPAddr).IP, nil
}

//-------------------------------------The IMU stream

// livoxSource reads the MID-360's IMU stream and, unless told not to, keeps
// the device configured to send it.
type livoxSource struct {
	cfg   LidarConfig
	dev   *livoxDevice
	imu   *net.UDPConn
	clock *clockMap
	out   chan imuSample

	lastPacket                          atomic.Int64 // host unix nanoseconds
	packets, badCRC, malformed, foreign atomic.Int64
	lost                                atomic.Int64
	lastCnt                             uint16 // the reading goroutine's own
	haveCnt                             bool
}

func newLivoxSource(cfg LidarConfig) (*livoxSource, error) {
	dev, err := openDevice(cfg)
	if err != nil {
		return nil, err
	}
	imu, err := net.ListenUDP("udp4", &net.UDPAddr{Port: cfg.IMUPort})
	if err != nil {
		dev.close()
		return nil, fmt.Errorf("IMU port %d: %w", cfg.IMUPort, err)
	}
	return &livoxSource{cfg: cfg, dev: dev, imu: imu, clock: newClockMap(clockWindow), out: make(chan imuSample, 64)}, nil
}

func (s *livoxSource) samples(ctx context.Context) (<-chan imuSample, error) {
	go s.read(ctx)
	if *s.cfg.ConfigureDevice {
		go s.keepConfigured(ctx)
	}
	return s.out, nil
}

func (s *livoxSource) close() error {
	return errors.Join(s.imu.Close(), s.dev.close())
}

func (s *livoxSource) status() string {
	return fmt.Sprintf("%d IMU packets, %d bad CRC, %d lost, %d malformed, %d of another type, clock offset spread %.2f ms",
		s.packets.Load(), s.badCRC.Load(), s.lost.Load(), s.malformed.Load(), s.foreign.Load(),
		float64(s.clock.spread())/float64(time.Millisecond))
}

func (s *livoxSource) read(ctx context.Context) {
	defer close(s.out)
	buf := make([]byte, 2048)
	for ctx.Err() == nil {
		if err := s.imu.SetReadDeadline(time.Now().Add(time.Second)); err != nil {
			logf("the IMU port: %v", err)
			return
		}
		n, _, err := s.imu.ReadFromUDP(buf)
		received := time.Now()
		if err != nil {
			if ne, ok := err.(net.Error); ok && ne.Timeout() {
				continue
			}
			if ctx.Err() == nil {
				logf("reading the IMU stream: %v", err)
			}
			return
		}
		for _, smp := range s.decode(buf[:n], received) {
			select {
			case s.out <- smp:
			case <-ctx.Done():
				return
			}
		}
	}
}

// decode turns one packet into samples on this computer's clock, and counts
// what is wrong with the packets that are not.
func (s *livoxSource) decode(b []byte, received time.Time) []imuSample {
	h, data, err := parsePacket(b)
	switch {
	case errors.Is(err, errBadCRC):
		s.badCRC.Add(1)
		return nil
	case err != nil:
		s.malformed.Add(1)
		return nil
	case h.dataType != dataTypeIMU:
		s.foreign.Add(1)
		return nil
	}
	recs, err := parseIMU(data, h.dotNum)
	if err != nil {
		s.malformed.Add(1)
		return nil
	}
	s.packets.Add(1)
	s.lastPacket.Store(received.UnixNano())
	if s.haveCnt {
		// A jump of a thousand or more is a restart rather than a loss.
		if missed := h.udpCnt - s.lastCnt - 1; missed > 0 && missed < 1000 {
			s.lost.Add(int64(missed))
		}
	}
	s.lastCnt, s.haveCnt = h.udpCnt, true

	first := s.clock.toHost(int64(h.timestamp), received)
	step := time.Duration(h.timeInterval) * 100 * time.Nanosecond / time.Duration(len(recs))
	out := make([]imuSample, len(recs))
	for i, r := range recs {
		out[i] = imuSample{gyro: r.gyro, accel: r.accel, at: first.Add(time.Duration(i) * step)}
	}
	return out
}

// keepConfigured configures the device at start-up, and again whenever its
// IMU stream has been quiet for a few seconds: a device that restarted, or
// was plugged in late, is set up again without anyone noticing.
func (s *livoxSource) keepConfigured(ctx context.Context) {
	const quiet, retry = 3 * time.Second, 5 * time.Second
	tick := time.NewTicker(time.Second)
	defer tick.Stop()
	var lastTry, lastLog time.Time
	tries := 0    // since the last packet
	start := true // configure once even if a stream left by an earlier run is already flowing
	for {
		since := time.Since(time.Unix(0, s.lastPacket.Load()))
		if (start || since > quiet) && time.Since(lastTry) > retry {
			start = false
			lastTry = time.Now()
			tries++
			host, err := s.dev.configure(s.cfg)
			switch {
			case err == nil && tries == 1:
				logf("the MID-360 at %s sends its IMU to %s:%d, in %s mode", s.dev.addr.IP, host, s.cfg.IMUPort, s.cfg.WorkMode)
			case tries > 1 && time.Since(lastLog) < statusEvery:
				// said recently enough
			case err != nil:
				logf("cannot configure the MID-360: %v", err)
				lastLog = time.Now()
			default:
				logf("the MID-360 took its configuration %d times and still no IMU packet has reached port %d: check hostIP and the firewall",
					tries, s.cfg.IMUPort)
				lastLog = time.Now()
			}
		} else if since <= quiet {
			tries = 0
		}
		select {
		case <-ctx.Done():
			return
		case <-tick.C:
		}
	}
}
