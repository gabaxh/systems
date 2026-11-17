/*******************************************************************************
 * Copyright (c) 2025 Synecdoque
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, subject to the following conditions:
 *
 * The software is licensed under the MIT License. See the LICENSE file in this repository for details.
 *
 * Contributors:
 *   Jan A. van Deventer, Luleå - initial implementation
 *   Thomas Hedeler, Hamburg - initial implementation
 ***************************************************************************SDG*/

package main

import (
	"bufio"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"log"
	"mime/multipart"
	"net/http"
	"net/textproto"
	"os"
	"os/exec"
	"strconv"
	"sync/atomic"
	"time"

	"github.com/sdoque/mbaigo/components"
	"github.com/sdoque/mbaigo/usecases"
)

// -------------------------------------Define the unit asset
// Traits are Asset-specific configurable parameters and variables
type Traits struct {
	// Backend selects rpicam (the existing Pi camera) or ffmpeg (Linux V4L2).
	Backend string `json:"backend"`
	// Device should be a /dev/v4l/by-id path for V4L2, since video numbers change.
	Device string `json:"device,omitempty"`
	Width  int    `json:"width"`
	Height int    `json:"height"`
	FPS    int    `json:"fps"`
	// Rotation is applied during capture, independently of RealSense Viewer.
	Rotation int `json:"rotation"`
	active   atomic.Bool
	owner    *components.System `json:"-"`
	name     string             `json:"-"`
}

//-------------------------------------Instantiate a unit asset template

// initTemplate initializes a UnitAsset with default values.
func initTemplate() *components.UnitAsset {
	stream := components.Service{
		Definition:  "stream",
		SubPath:     "start",
		Details:     map[string][]string{"Forms": {"mpeg"}},
		Description: " provides a video stream from the camera",
	}

	return &components.UnitAsset{
		Name:    "PiCam",
		Mission: components.MissionMeasurement,
		Details: map[string][]string{"Model": {"PiCam v3 NoIR"}},
		ServicesMap: components.Services{
			stream.SubPath: &stream,
		},
		Traits: &Traits{Backend: "rpicam", Width: 640, Height: 480, FPS: 15},
	}
}

//-------------------------------------Instantiate the unit assets based on configuration

// newResource creates the Resource resource with its pointers and channels based on the configuration
func newResource(configuredAsset usecases.ConfigurableAsset, sys *components.System) (*components.UnitAsset, func()) {
	t := &Traits{
		owner: sys,
		name:  configuredAsset.Name,
	}
	if len(configuredAsset.Traits) > 0 {
		if err := json.Unmarshal(configuredAsset.Traits[0], t); err != nil {
			log.Fatalf("filmer: invalid camera configuration: %v", err)
		}
	}
	t.applyDefaults()
	if err := t.validate(); err != nil {
		log.Fatalf("filmer: invalid camera configuration: %v", err)
	}

	ua := &components.UnitAsset{
		Name:        configuredAsset.Name,
		Mission:     configuredAsset.Mission,
		Mobility:    configuredAsset.Mobility,
		TetheredTo:  configuredAsset.TetheredTo,
		Owner:       sys,
		Details:     configuredAsset.Details,
		ServicesMap: usecases.MakeServiceMap(configuredAsset.Services),
		Traits:      t,
	}
	ua.ServingFunc = func(w http.ResponseWriter, r *http.Request, servicePath string) {
		serving(t, w, r, servicePath)
	}

	return ua, func() {
		log.Println("disconnecting from sensors")
	}
}

//-------------------------------------Unit asset's resource functions

// StartStreamURL returns the URL to start the video stream.
func (t *Traits) StartStreamURL() string {
	ip := t.owner.Husk.Host.IPAddresses[0]
	port := t.owner.Husk.ProtoPort["http"]
	return fmt.Sprintf("http://%s:%d/filmer/%s/stream", ip, port, t.name)
}

func (t *Traits) applyDefaults() {
	if t.Backend == "" {
		t.Backend = "rpicam"
	}
	if t.Width == 0 {
		t.Width = 640
	}
	if t.Height == 0 {
		t.Height = 480
	}
	if t.FPS == 0 {
		t.FPS = 15
	}
}

func (t *Traits) validate() error {
	if t.Backend != "rpicam" && t.Backend != "ffmpeg" {
		return fmt.Errorf("backend must be rpicam or ffmpeg")
	}
	if t.Backend == "ffmpeg" && t.Device == "" {
		return fmt.Errorf("ffmpeg requires a camera device path")
	}
	if t.Width <= 0 || t.Height <= 0 || t.FPS <= 0 {
		return fmt.Errorf("width, height and fps must be positive")
	}
	if t.Rotation != 0 && t.Rotation != 180 {
		return fmt.Errorf("rotation must be 0 or 180 degrees")
	}
	return nil
}

func (t *Traits) captureCommand(ctx context.Context) (*exec.Cmd, error) {
	if err := t.validate(); err != nil {
		return nil, err
	}
	if t.Backend == "rpicam" {
		args := []string{"-t", "0", "--codec", "mjpeg", "--width", strconv.Itoa(t.Width),
			"--height", strconv.Itoa(t.Height), "--framerate", strconv.Itoa(t.FPS)}
		if t.Rotation == 180 {
			args = append(args, "--rotation", "180")
		}
		return exec.CommandContext(ctx, "rpicam-vid", append(args, "-o", "-")...), nil
	}
	args := []string{"-hide_banner", "-loglevel", "warning", "-nostdin",
		"-f", "v4l2", "-input_format", "yuyv422", "-video_size", fmt.Sprintf("%dx%d", t.Width, t.Height),
		"-framerate", strconv.Itoa(t.FPS), "-i", t.Device}
	if t.Rotation == 180 {
		args = append(args, "-vf", "hflip,vflip")
	}
	args = append(args, "-an", "-c:v", "mjpeg", "-q:v", "5", "-f", "image2pipe", "pipe:1")
	return exec.CommandContext(ctx, "ffmpeg", args...), nil
}

// nextJPEG reconstructs frames across pipe reads, with a bound on malformed output.
// Capture backends must emit JPEG images, not arbitrary video containers.
func nextJPEG(r *bufio.Reader) ([]byte, error) {
	const maxFrameBytes = 8 * 1024 * 1024
	previous := byte(0)
	for {
		b, err := r.ReadByte()
		if err != nil {
			return nil, err
		}
		if previous == 0xff && b == 0xd8 {
			break
		}
		previous = b
	}
	frame := []byte{0xff, 0xd8}
	for len(frame) < maxFrameBytes {
		b, err := r.ReadByte()
		if err == io.EOF {
			return nil, io.ErrUnexpectedEOF
		}
		if err != nil {
			return nil, err
		}
		frame = append(frame, b)
		if frame[len(frame)-2] == 0xff && b == 0xd9 {
			return frame, nil
		}
	}
	return nil, fmt.Errorf("JPEG frame exceeds %d bytes", maxFrameBytes)
}

// StreamTo owns one capture process until the viewer leaves or the system stops.
// A second viewer must not start a competing process on the same camera.
func (t *Traits) StreamTo(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		w.Header().Set("Allow", http.MethodGet)
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}
	if !t.active.CompareAndSwap(false, true) {
		http.Error(w, "camera already in use", http.StatusConflict)
		return
	}
	defer t.active.Store(false)

	ctx, cancel := context.WithCancel(r.Context())
	defer cancel()
	if t.owner != nil {
		stop := context.AfterFunc(t.owner.Ctx, cancel)
		defer stop()
	}
	cmd, err := t.captureCommand(ctx)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	cmd.Stderr = os.Stderr
	stdout, err := cmd.StdoutPipe()
	if err != nil {
		http.Error(w, "failed to create capture pipe", http.StatusInternalServerError)
		return
	}
	if err := cmd.Start(); err != nil {
		stdout.Close()
		log.Printf("filmer: start capture: %v", err)
		http.Error(w, "failed to start camera capture", http.StatusServiceUnavailable)
		return
	}
	defer func() {
		cancel()
		stdout.Close()
		// Wait reaps the child before allowing another viewer to open the camera.
		if err := cmd.Wait(); err != nil {
			log.Printf("filmer: capture ended: %v", err)
		}
	}()

	reader := bufio.NewReader(stdout)
	frame, err := nextJPEG(reader)
	if err != nil {
		log.Printf("filmer: first frame: %v", err)
		http.Error(w, "camera did not provide a frame; check capture log", http.StatusServiceUnavailable)
		return
	}
	writer := multipart.NewWriter(w)
	writer.SetBoundary("frame")
	w.Header().Set("Content-Type", "multipart/x-mixed-replace; boundary=frame")
	w.Header().Set("Cache-Control", "no-cache")
	w.Header().Set("Access-Control-Allow-Origin", "*")
	controller := http.NewResponseController(w)
	for {
		// Renew the server's one-minute deadline for a long-lived stream, while
		// bounding writes to a viewer that has stopped reading.
		if err := controller.SetWriteDeadline(time.Now().Add(10 * time.Second)); err != nil && !errors.Is(err, http.ErrNotSupported) {
			log.Printf("filmer: stream deadline: %v", err)
			return
		}
		header := make(textproto.MIMEHeader)
		header.Set("Content-Type", "image/jpeg")
		header.Set("Content-Length", strconv.Itoa(len(frame)))
		part, err := writer.CreatePart(header)
		if err != nil {
			return
		}
		if _, err := part.Write(frame); err != nil {
			return
		}
		if f, ok := w.(http.Flusher); ok {
			f.Flush()
		}
		frame, err = nextJPEG(reader)
		if err != nil {
			if ctx.Err() == nil && err != io.EOF {
				log.Printf("filmer: read capture: %v", err)
			}
			writer.Close()
			return
		}
	}
}
