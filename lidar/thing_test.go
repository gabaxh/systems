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
	"encoding/json"
	"math"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"

	"github.com/sdoque/mbaigo/forms"
)

// testTraits runs a script into a fresh estimator so that its last sample
// lands age ago.
func testTraits(t *testing.T, seconds float64, age time.Duration, script ...motion) *Traits {
	t.Helper()
	cfg := testConfig()
	e := testEstimator(t, cfg)
	start := time.Now().Add(-age - time.Duration(seconds*float64(time.Second)))
	feed(e, scripted(cfg, script...), start, 0, seconds)
	return &Traits{cfg: cfg, est: e}
}

func get(handler func(http.ResponseWriter, *http.Request)) *httptest.ResponseRecorder {
	w := httptest.NewRecorder()
	handler(w, httptest.NewRequest(http.MethodGet, "/", nil))
	return w
}

// A yaw with an unknown bias in it drifts from the first second. There is no
// float that means "not yet"; the only honest answer is not to answer.
func TestYawIsUnavailableBeforeTheBias(t *testing.T) {
	tr := testTraits(t, 5, 0, motion{seconds: 5, yawRate: 20})
	for name, h := range map[string]func(http.ResponseWriter, *http.Request){
		"yaw": tr.yawService, "yawRate": tr.yawRateService, "bias": tr.biasService,
	} {
		if w := get(h); w.Code != http.StatusServiceUnavailable {
			t.Errorf("%s answered %d before any standstill, want 503", name, w.Code)
		}
	}
}

func TestYawIsServedAfterAStandstill(t *testing.T) {
	tr := testTraits(t, 3, 0, motion{seconds: 3})
	w := get(tr.yawService)
	if w.Code != http.StatusOK {
		t.Fatalf("yaw answered %d (%s), want 200", w.Code, w.Body.String())
	}
	var f forms.SignalA_v1a
	if err := json.Unmarshal(w.Body.Bytes(), &f); err != nil {
		t.Fatal(err)
	}
	if math.Abs(f.Value) > 0.05 || f.Unit != unitDegree {
		t.Errorf("yaw = %v %s, want about 0 degrees", f.Value, f.Unit)
	}
	if !f.Timestamp.Equal(tr.est.last) {
		t.Errorf("stamped %v, want the newest sample's %v", f.Timestamp, tr.est.last)
	}
	if w := get(tr.biasService); w.Code != http.StatusOK {
		t.Errorf("bias answered %d after a standstill", w.Code)
	}
}

// A yaw that has stopped moving because the stream stopped is not a vehicle
// that has stopped turning.
func TestYawIsUnavailableWhenTheStreamIsStale(t *testing.T) {
	tr := testTraits(t, 3, time.Second, motion{seconds: 3})
	if w := get(tr.yawService); w.Code != http.StatusServiceUnavailable {
		t.Errorf("yaw answered %d with the newest sample a second old, want 503", w.Code)
	}
	// The bias is still what it was, and says when it was measured.
	if w := get(tr.biasService); w.Code != http.StatusOK {
		t.Errorf("bias answered %d with a stale stream, want 200", w.Code)
	}
}
