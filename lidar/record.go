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

// Every raw sample to a CSV file, for characterizing the gyro: its bias, its
// noise and its Allan deviation (gyro_characterization_guide_2026-10-23.md).
// The estimator keeps only an integral, and an Allan deviation needs every
// sample, so it is recorded here, as the device sent it: no bias removed, no
// projection on the up vector.

import (
	"bufio"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"time"
)

const recordHeader = "host_unix_ns,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,accel_x_g,accel_y_g,accel_z_g\n"

// recorder writes samples to one file. It is used from one goroutine only.
type recorder struct {
	f    *os.File
	w    *bufio.Writer
	row  []byte
	path string
}

// newRecorder opens a new file in dir, named for the time it was opened, so a
// restart starts a new file rather than overwriting the last.
func newRecorder(dir string, now time.Time) (*recorder, error) {
	if err := os.MkdirAll(dir, 0o755); err != nil {
		return nil, err
	}
	path := filepath.Join(dir, "imu_"+now.Format("20060102T150405")+".csv")
	f, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0o644)
	if err != nil {
		return nil, err
	}
	r := &recorder{f: f, w: bufio.NewWriterSize(f, 64<<10), path: path}
	if _, err := r.w.WriteString(recordHeader); err != nil {
		f.Close()
		return nil, err
	}
	return r, nil
}

// write adds one row. Nine significant digits are exact for the device's
// float32 values.
func (r *recorder) write(s imuSample) error {
	b := strconv.AppendInt(r.row[:0], s.at.UnixNano(), 10)
	for _, v := range [6]float64{s.gyro[0], s.gyro[1], s.gyro[2], s.accel[0], s.accel[1], s.accel[2]} {
		b = append(b, ',')
		b = strconv.AppendFloat(b, v, 'g', 9, 64)
	}
	b = append(b, '\n')
	r.row = b
	_, err := r.w.Write(b)
	return err
}

func (r *recorder) flush() error { return r.w.Flush() }

func (r *recorder) close() error {
	err := r.w.Flush()
	if cerr := r.f.Close(); err == nil {
		err = cerr
	}
	if err != nil {
		return fmt.Errorf("closing %s: %w", r.path, err)
	}
	return nil
}
