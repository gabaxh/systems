package main

import (
	"context"
	"os"
	"path/filepath"
	"testing"
	"time"
)

func hidFixture(t *testing.T) (string, string, string) {
	t.Helper()
	root := t.TempDir()
	sys, dev := filepath.Join(root, "sys"), filepath.Join(root, "dev")
	parent := filepath.Join(sys, "devices/controllerA")
	for _, dir := range []string{filepath.Join(parent, "input/input1"), filepath.Join(sys, "class/input/js0"), filepath.Join(sys, "class/hidraw/hidraw1"), dev} {
		if err := os.MkdirAll(dir, 0755); err != nil {
			t.Fatal(err)
		}
	}
	if err := os.WriteFile(filepath.Join(parent, "uevent"), []byte("HID_ID=0005:0000054C:00000CE6\nHID_NAME=Test DualSense\n"), 0644); err != nil {
		t.Fatal(err)
	}
	if err := os.Symlink(filepath.Join(parent, "input/input1"), filepath.Join(sys, "class/input/js0/device")); err != nil {
		t.Fatal(err)
	}
	if err := os.Symlink(parent, filepath.Join(sys, "class/hidraw/hidraw1/device")); err != nil {
		t.Fatal(err)
	}
	return sys, dev, parent
}

func TestHIDDiscoveryTracksRenumbering(t *testing.T) {
	sys, dev, parent := hidFixture(t)
	path, err := discoverHID(sys, dev, "/dev/input/js0", "")
	if err != nil || path != filepath.Join(dev, "hidraw1") {
		t.Fatalf("discovery: %q, %v", path, err)
	}
	if err := os.RemoveAll(filepath.Join(sys, "class/hidraw/hidraw1")); err != nil {
		t.Fatal(err)
	}
	if err := os.MkdirAll(filepath.Join(sys, "class/hidraw/hidraw7"), 0755); err != nil {
		t.Fatal(err)
	}
	if err := os.Symlink(parent, filepath.Join(sys, "class/hidraw/hidraw7/device")); err != nil {
		t.Fatal(err)
	}
	path, err = discoverHID(sys, dev, "/dev/input/js0", "Test DualSense")
	if err != nil || path != filepath.Join(dev, "hidraw7") {
		t.Fatalf("rediscovery: %q, %v", path, err)
	}
}

func TestHIDDiscoveryRejectsWrongOrAmbiguousDevice(t *testing.T) {
	t.Run("wrong configured name", func(t *testing.T) {
		sys, dev, _ := hidFixture(t)
		if _, err := discoverHID(sys, dev, "/dev/input/js0", "Other pad"); err == nil {
			t.Fatal("wrong name accepted")
		}
	})
	t.Run("same name different controller", func(t *testing.T) {
		sys, dev, _ := hidFixture(t)
		other := filepath.Join(sys, "devices/controllerB")
		if err := os.MkdirAll(other, 0755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(filepath.Join(other, "uevent"), []byte("HID_NAME=Test DualSense\n"), 0644); err != nil {
			t.Fatal(err)
		}
		link := filepath.Join(sys, "class/hidraw/hidraw1/device")
		if err := os.Remove(link); err != nil {
			t.Fatal(err)
		}
		if err := os.Symlink(other, link); err != nil {
			t.Fatal(err)
		}
		if _, err := discoverHID(sys, dev, "/dev/input/js0", ""); err == nil {
			t.Fatal("another controller vouched for js0")
		}
	})
	t.Run("multiple matching interfaces", func(t *testing.T) {
		sys, dev, parent := hidFixture(t)
		dir := filepath.Join(sys, "class/hidraw/hidraw2")
		if err := os.MkdirAll(dir, 0755); err != nil {
			t.Fatal(err)
		}
		if err := os.Symlink(parent, filepath.Join(dir, "device")); err != nil {
			t.Fatal(err)
		}
		if _, err := discoverHID(sys, dev, "/dev/input/js0", ""); err == nil {
			t.Fatal("ambiguous interfaces accepted")
		}
	})
	t.Run("missing joystick", func(t *testing.T) {
		sys, dev, _ := hidFixture(t)
		if _, err := discoverHID(sys, dev, "/dev/input/js9", ""); err == nil {
			t.Fatal("missing joystick accepted")
		}
	})
	t.Run("missing HID name", func(t *testing.T) {
		sys, dev, parent := hidFixture(t)
		if err := os.WriteFile(filepath.Join(parent, "uevent"), []byte("HID_ID=123\n"), 0644); err != nil {
			t.Fatal(err)
		}
		if _, err := discoverHID(sys, dev, "/dev/input/js0", ""); err == nil {
			t.Fatal("missing name accepted")
		}
	})
}

func TestHIDFreshnessStopsPilotWithCachedStick(t *testing.T) {
	for _, age := range []time.Duration{0, 600 * time.Millisecond} {
		pad := &pad{state: stick(connected(), speedAxis, 20000), lastHIDReport: time.Now().Add(-age)}
		s := pad.snapshot()
		if !pad.hidFresh(500 * time.Millisecond) {
			s.connected = false
		}
		pilot := newTestPilot()
		pilot.inControl = true
		a := pilot.step(s, time.Now())
		if age == 0 && (!a.drive || a.stop) {
			t.Fatal("fresh report prevented driving")
		}
		if age > 0 && (!a.stop || a.drive || pilot.inControl) {
			t.Fatal("stale reports allowed cached stick command")
		}
	}
	if (&pad{}).hidFresh(time.Second) {
		t.Fatal("no report treated as fresh")
	}
}

func TestHIDReaderClearsFreshnessOnEOFAndIdentityLoss(t *testing.T) {
	for _, valid := range []bool{true, false} {
		r, w, err := os.Pipe()
		if err != nil {
			t.Fatal(err)
		}
		p := &pad{lastHIDReport: time.Now()}
		done := make(chan struct{})
		checked := make(chan bool, 1)
		go func() {
			p.readHID(context.Background(), r, func() bool { checked <- p.hidFresh(time.Second); return valid })
			close(done)
		}()
		if _, err := w.Write([]byte{1, 2}); err != nil {
			t.Fatal(err)
		}
		select {
		case fresh := <-checked:
			if fresh {
				t.Fatal("old freshness not cleared before validation")
			}
		case <-time.After(time.Second):
			t.Fatal("report not read")
		}
		if valid {
			deadline := time.Now().Add(time.Second)
			for !p.hidFresh(time.Second) && time.Now().Before(deadline) {
				time.Sleep(time.Millisecond)
			}
			if !p.hidFresh(time.Second) {
				t.Fatal("validated report not credited")
			}
		}
		w.Close()
		select {
		case <-done:
		case <-time.After(time.Second):
			t.Fatal("reader did not exit")
		}
		if p.hidFresh(time.Second) {
			t.Fatal("reader left stale freshness behind")
		}
	}
}

func TestHIDReaderCancellationUnblocksSilentDevice(t *testing.T) {
	r, w, err := os.Pipe()
	if err != nil {
		t.Fatal(err)
	}
	defer w.Close()
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	p := &pad{}
	done := make(chan struct{})
	go func() { p.readHID(ctx, r, func() bool { return true }); close(done) }()
	cancel()
	select {
	case <-done:
	case <-time.After(time.Second):
		t.Fatal("silent HID reader did not stop")
	}
	if p.hidFresh(time.Second) {
		t.Fatal("cancelled reader looks connected")
	}
}
