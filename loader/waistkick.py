#!/usr/bin/env python3
# waistkick.py — find the effort that moves the waist.
#   waistkick.py left          walk the effort up, 2% at a time, until the joint moves
#   waistkick.py right 30      hold 30% and time how long the joint takes to move
import json, sys, time, urllib.error, urllib.request

H = "http://127.0.0.1:20197/loader"

def put(path, v):
    body = json.dumps({"value": v, "version": "SignalA_v1.0"}).encode()
    req = urllib.request.Request(f"{H}/{path}", body, method="PUT",
                                 headers={"Content-Type": "application/json"})
    try:
        urllib.request.urlopen(req, timeout=1).read()
    except urllib.error.HTTPError as e:
        sys.exit(f"{path}: {e.code} {e.read().decode().strip()}")

def waist():
    return json.load(urllib.request.urlopen(f"{H}/Steering/waist", timeout=1))["value"]

side = 1 if sys.argv[1] == "left" else -1   # positive effort is left
steps = [float(sys.argv[2])] if len(sys.argv) > 2 else range(2, 52, 2)

put("Vehicle/control", 1)
try:
    for pct in steps:
        start, t0 = waist(), time.time()
        while time.time() - t0 < 1.5:
            put("Steering/setpoint", side * pct)
            w = waist()
            if abs(w - start) >= 4:
                print(f"MOVED at {pct}%: count {start} -> {w} after {time.time() - t0:.2f} s")
                sys.exit()
            time.sleep(0.05)
        print(f"{pct}%: not moving (count {start} -> {w})")
finally:
    put("Vehicle/stop", 1)
