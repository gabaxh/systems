package main

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
)

// discoverHID reads HID_NAME from sysfs and requires the hidraw device to
// belong to the selected joystick's HID parent. Names alone are not unique:
// a second connected DualSense must not vouch for a disconnected first one.
// Roots are parameters so discovery can be tested without Linux hardware.
func discoverHID(sysRoot, devRoot, joystick, wantedName string) (string, error) {
	input, err := filepath.EvalSymlinks(filepath.Join(sysRoot, "class/input", filepath.Base(joystick), "device"))
	if err != nil {
		return "", fmt.Errorf("joystick HID parent: %w", err)
	}
	parent, name := "", ""
	for dir := input; ; dir = filepath.Dir(dir) {
		if b, err := os.ReadFile(filepath.Join(dir, "uevent")); err == nil {
			for _, line := range strings.Split(string(b), "\n") {
				if strings.HasPrefix(line, "HID_NAME=") {
					parent, name = dir, strings.TrimPrefix(line, "HID_NAME=")
					break
				}
			}
		}
		if parent != "" || dir == filepath.Dir(dir) {
			break
		}
	}
	if name == "" {
		return "", fmt.Errorf("no HID_NAME for joystick %s", joystick)
	}
	if wantedName != "" && wantedName != name {
		return "", fmt.Errorf("joystick HID_NAME %q differs from configured %q", name, wantedName)
	}
	entries, err := filepath.Glob(filepath.Join(sysRoot, "class/hidraw/*/device/uevent"))
	if err != nil {
		return "", err
	}
	found := ""
	for _, entry := range entries {
		device, err := filepath.EvalSymlinks(filepath.Dir(entry))
		if err != nil || device != parent {
			continue
		}
		b, err := os.ReadFile(entry)
		if err != nil {
			continue
		}
		matches := false
		for _, line := range strings.Split(string(b), "\n") {
			if line == "HID_NAME="+name {
				matches = true
			}
		}
		if !matches {
			continue
		}
		if found != "" {
			return "", fmt.Errorf("multiple hidraw interfaces for joystick %s", joystick)
		}
		found = filepath.Join(devRoot, filepath.Base(filepath.Dir(filepath.Dir(entry))))
	}
	if found == "" {
		return "", fmt.Errorf("no matching hidraw device for joystick HID_NAME %q", name)
	}
	return found, nil
}
