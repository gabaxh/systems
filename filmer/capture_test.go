package main

import (
	"bufio"
	"bytes"
	"context"
	"io"
	"mime"
	"mime/multipart"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
	"testing/iotest"
	"time"
)

func TestCaptureCommands(t *testing.T) {
	pi := &Traits{}
	pi.applyDefaults()
	cmd, err := pi.captureCommand(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	want := []string{"rpicam-vid", "-t", "0", "--codec", "mjpeg", "--width", "640", "--height", "480", "--framerate", "15", "-o", "-"}
	if !reflect.DeepEqual(cmd.Args, want) {
		t.Fatalf("Pi compatibility: %v", cmd.Args)
	}
	cam := &Traits{Backend: "ffmpeg", Device: "/dev/v4l/by-id/test-camera", Width: 640, Height: 480, FPS: 30, Rotation: 180}
	cmd, err = cam.captureCommand(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	args := strings.Join(cmd.Args, " ")
	for _, fragment := range []string{"-input_format yuyv422", "-video_size 640x480", "-framerate 30", "-i /dev/v4l/by-id/test-camera", "-vf hflip,vflip", "-f image2pipe pipe:1"} {
		if !strings.Contains(args, fragment) {
			t.Fatalf("missing %q in %s", fragment, args)
		}
	}
	cam.Rotation = 0
	cmd, _ = cam.captureCommand(context.Background())
	if strings.Contains(strings.Join(cmd.Args, " "), "-vf") {
		t.Fatal("rotation enabled for upright camera")
	}
	cam.Rotation = 90
	if _, err := cam.captureCommand(context.Background()); err == nil {
		t.Fatal("accepted unsupported rotation")
	}
	cam.Rotation = 0
	cam.Device = ""
	if _, err := cam.captureCommand(context.Background()); err == nil {
		t.Fatal("accepted missing device")
	}
}

func TestJPEGFraming(t *testing.T) {
	jpeg := []byte{0xff, 0xd8, 1, 2, 3, 0xff, 0xd9}
	data := append([]byte("prefix"), jpeg...)
	data = append(data, jpeg...)
	reader := bufio.NewReader(iotest.OneByteReader(bytes.NewReader(data)))
	for i := 0; i < 2; i++ {
		frame, err := nextJPEG(reader)
		if err != nil || !bytes.Equal(frame, jpeg) {
			t.Fatalf("frame %d: %x, %v", i, frame, err)
		}
	}
	if _, err := nextJPEG(reader); err != io.EOF {
		t.Fatalf("end: %v", err)
	}
	if _, err := nextJPEG(bufio.NewReader(bytes.NewReader(jpeg[:5]))); err != io.ErrUnexpectedEOF {
		t.Fatalf("truncation: %v", err)
	}
	oversized := io.MultiReader(bytes.NewReader(jpeg[:2]), bytes.NewReader(bytes.Repeat([]byte{1}, 8*1024*1024)))
	if _, err := nextJPEG(bufio.NewReader(oversized)); err == nil {
		t.Fatal("unbounded JPEG accepted")
	}
}

// Stand in for FFmpeg so process cleanup is tested without a physical camera.
func fakeFFmpeg(t *testing.T, body string) {
	t.Helper()
	dir := t.TempDir()
	if err := os.WriteFile(filepath.Join(dir, "ffmpeg"), []byte("#!/bin/sh\n"+body+"\n"), 0700); err != nil {
		t.Fatal(err)
	}
	t.Setenv("PATH", dir+string(os.PathListSeparator)+os.Getenv("PATH"))
}

type cancelOnFrame struct {
	*httptest.ResponseRecorder
	cancel context.CancelFunc
}

func (w *cancelOnFrame) Flush() { w.ResponseRecorder.Flush(); w.cancel() }

func TestStreamCancellationReleasesCamera(t *testing.T) {
	fakeFFmpeg(t, "printf '\\377\\330\\001\\377\\331'\nexec sleep 30")
	cam := &Traits{Backend: "ffmpeg", Device: "test", Width: 640, Height: 480, FPS: 30, Rotation: 180}
	for run := 0; run < 2; run++ {
		ctx, cancel := context.WithCancel(context.Background())
		w := &cancelOnFrame{httptest.NewRecorder(), cancel}
		r := httptest.NewRequest(http.MethodGet, "/stream", nil).WithContext(ctx)
		done := make(chan struct{})
		go func() { cam.StreamTo(w, r); close(done) }()
		select {
		case <-done:
		case <-time.After(3 * time.Second):
			cancel()
			t.Fatal("capture did not exit on viewer disconnect")
		}
		cancel()
		if cam.active.Load() {
			t.Fatal("camera not released")
		}
		if w.Code != http.StatusOK {
			t.Fatalf("HTTP %d: %s", w.Code, w.Body.String())
		}
		_, params, err := mime.ParseMediaType(w.Header().Get("Content-Type"))
		if err != nil {
			t.Fatal(err)
		}
		part, err := multipart.NewReader(bytes.NewReader(w.Body.Bytes()), params["boundary"]).NextPart()
		if err != nil {
			t.Fatal(err)
		}
		frame, err := io.ReadAll(part)
		if err != nil || !bytes.Equal(frame, []byte{0xff, 0xd8, 1, 0xff, 0xd9}) {
			t.Fatalf("bad HTTP JPEG: %x, %v", frame, err)
		}
	}
}

func TestStreamBusyAndFailedCapture(t *testing.T) {
	cam := &Traits{Backend: "ffmpeg", Device: "test", Width: 640, Height: 480, FPS: 30}
	cam.active.Store(true)
	w := httptest.NewRecorder()
	cam.StreamTo(w, httptest.NewRequest(http.MethodGet, "/stream", nil))
	if w.Code != http.StatusConflict {
		t.Fatalf("busy: HTTP %d", w.Code)
	}
	cam.active.Store(false)
	fakeFFmpeg(t, "exit 1")
	w = httptest.NewRecorder()
	cam.StreamTo(w, httptest.NewRequest(http.MethodGet, "/stream", nil))
	if w.Code != http.StatusServiceUnavailable || cam.active.Load() {
		t.Fatalf("failed capture: HTTP %d, busy=%v", w.Code, cam.active.Load())
	}
}

func TestStreamOutlivesServerWriteTimeout(t *testing.T) {
	fakeFFmpeg(t, "printf '\\377\\330\\001\\377\\331'\nsleep 0.15\nprintf '\\377\\330\\002\\377\\331'\nexec sleep 30")
	cam := &Traits{Backend: "ffmpeg", Device: "test", Width: 640, Height: 480, FPS: 30}
	server := httptest.NewUnstartedServer(http.HandlerFunc(cam.StreamTo))
	server.Config.WriteTimeout = 50 * time.Millisecond
	server.Start()
	defer server.Close()
	client := &http.Client{Timeout: 3 * time.Second}
	response, err := client.Get(server.URL)
	if err != nil {
		t.Fatal(err)
	}
	defer response.Body.Close()
	_, params, err := mime.ParseMediaType(response.Header.Get("Content-Type"))
	if err != nil {
		t.Fatal(err)
	}
	reader := multipart.NewReader(response.Body, params["boundary"])
	for i := byte(1); i <= 2; i++ {
		part, err := reader.NextPart()
		if err != nil {
			t.Fatal(err)
		}
		frame := make([]byte, 5)
		if _, err := io.ReadFull(part, frame); err != nil {
			t.Fatal(err)
		}
		if frame[2] != i {
			t.Fatalf("wrong frame: %x", frame)
		}
	}
	response.Body.Close()
	deadline := time.Now().Add(3 * time.Second)
	for cam.active.Load() && time.Now().Before(deadline) {
		time.Sleep(time.Millisecond)
	}
	if cam.active.Load() {
		t.Fatal("camera held after HTTP client closed")
	}
}
