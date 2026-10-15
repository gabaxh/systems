#!/usr/bin/env python3
# holdspeed.py — drive all four wheels at a steady speed, for tuning the speed loop.
#   holdspeed.py                 10 RPM for 60 s
#   holdspeed.py 15              15 RPM for 60 s
#   holdspeed.py 10 30           10 RPM for 30 s
#   holdspeed.py -- -10          10 RPM in reverse
#
# It commands the Vehicle's velocity, not the wheels' setpoints: a wheel set
# directly is open loop, and the speed loop would not be tested at all. The
# velocity is worked out from the wheel circumference in systemconfig.json.
# Five times a second it prints each wheel's target/measured RPM, with the time.
#
# Quit the gamer first: it sends velocity 50 times a second and the two would
# take turns. Ctrl-C stops the vehicle; if this script dies, the loader stops on
# its own after half a second without a command. LOADER=http://<host>:20197/loader
# points it at a loader on another machine.
import json, os, sys, time, urllib.error, urllib.request

H = os.environ.get("LOADER", "http://127.0.0.1:20197/loader")
WHEELS = ["FrontLeft", "FrontRight", "BackLeft", "BackRight"]

def put(path, v):
    body = json.dumps({"value": v, "version": "SignalA_v1.0"}).encode()
    req = urllib.request.Request(f"{H}/{path}", body, method="PUT",
                                 headers={"Content-Type": "application/json"})
    try:
        urllib.request.urlopen(req, timeout=1).read()
    except urllib.error.HTTPError as e:
        sys.exit(f"{path}: {e.code} {e.read().decode().strip()}")

def get(path):
    try:
        return json.load(urllib.request.urlopen(f"{H}/{path}", timeout=1))["value"]
    except (urllib.error.URLError, ValueError, KeyError):
        return None

def circumference():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "systemconfig.json")
    try:
        with open(path) as f:
            traits = json.load(f)["unit_assets"][0]["traits"][0]
        return traits["geometry"]["wheelCircumferenceMetres"]
    except (OSError, ValueError, KeyError, IndexError):
        return 1.335  # the loader's own default

def fmt(v):
    return "-" if v is None else f"{v:.1f}"

args = [a for a in sys.argv[1:] if a != "--"]
rpm = float(args[0]) if args else 10.0
seconds = float(args[1]) if len(args) > 1 else 60.0
c = circumference()
velocity = rpm * c / 60

art = get("Vehicle/articulation")
print(f"{rpm:g} RPM is {velocity:.3f} m/s at a {c} m circumference, for {seconds:g} s; Ctrl-C stops")
if art is not None and abs(art) > 2:
    print(f"the waist is at {art:.1f}°, so the wheels' targets will differ: straighten it for equal ones")

put("Vehicle/control", 1)
start = time.time()
next_print = start
try:
    while time.time() - start < seconds:
        put("Vehicle/velocity", velocity)
        now = time.time()
        if now >= next_print:
            row = "  ".join(f"{w:<10} {fmt(get(w + '/setpoint')):>5}/{fmt(get(w + '/speed')):<5}" for w in WHEELS)
            print(f"{now - start:6.1f} s  {row}", flush=True)
            next_print += 0.2
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    put("Vehicle/stop", 1)
    print("stopped")
