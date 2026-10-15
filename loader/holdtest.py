#!/usr/bin/env python3
# holdtest.py — stop on a slope and see whether the vehicle stays there.
#   holdtest.py                  up at 10 RPM for 5 s, then the stick centred for 60 s
#   holdtest.py 10 5 60          the same, spelled out: RPM, seconds driving, seconds holding
#   holdtest.py 10 5 60 stop     and then a stop, and another 60 s of watching
#
# Point the vehicle up the slope. It drives up, then commands zero — the stick
# centred, with control kept — while the loader holds it. With "stop" it then
# stops the vehicle, which sets every motor to zero, and watches whether it
# rolls back with its motors off: the answer to whether holdRelaxAfterSeconds
# can be set. The loader's log says when it rests, catches a creep, or warns
# about a hold costing heat.
#
# Five times a second it prints the time, the phase, and for each wheel its
# measured RPM and how far it has rolled since the stick was centred, in cm
# (negative is back down the slope).
#
# Quit the gamer first: it sends velocity 50 times a second and the two would
# take turns. Have someone ready to catch the vehicle. Ctrl-C stops it; if this
# script dies, the loader stops on its own after half a second without a
# command. LOADER=http://<host>:20197/loader points it at another machine.
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
    except urllib.error.URLError as e:
        sys.exit(f"{path}: cannot reach the loader at {H}: {e.reason}")

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

args = [a for a in sys.argv[1:] if a != "--"]
stop_after = "stop" in args
args = [a for a in args if a != "stop"]
rpm = float(args[0]) if args else 10.0
driving = float(args[1]) if len(args) > 1 else 5.0
holding = float(args[2]) if len(args) > 2 else 60.0
velocity = rpm * circumference() / 60

def row(start, phase, origin):
    cells = []
    for w in WHEELS:
        speed, distance = get(w + "/speed"), get(w + "/distance")
        moved = "-" if distance is None or origin.get(w) is None else f"{(distance - origin[w]) * 100:+.1f}"
        cells.append(f"{w:<10} {'-' if speed is None else f'{speed:.1f}':>5} RPM {moved:>6} cm")
    print(f"{time.time() - start:6.1f} s  {phase:<8} " + "  ".join(cells), flush=True)

print(f"up at {rpm:g} RPM ({velocity:.3f} m/s) for {driving:g} s, then holding for {holding:g} s"
      + (", then a stop and as long again watching" if stop_after else "") + "; Ctrl-C stops")
put("Vehicle/control", 1)
start = time.time()
origin = {}
stopped = False
try:
    phase_end = start + driving
    next_print = start
    while time.time() < phase_end:
        put("Vehicle/velocity", velocity)
        if time.time() >= next_print:
            row(start, "driving", origin)
            next_print += 0.2
        time.sleep(0.1)
    origin = {w: get(w + "/distance") for w in WHEELS}
    phase_end = time.time() + holding
    while time.time() < phase_end:
        put("Vehicle/velocity", 0)
        if time.time() >= next_print:
            row(start, "holding", origin)
            next_print += 0.2
        time.sleep(0.1)
    if stop_after:
        put("Vehicle/stop", 1)
        stopped = True
        phase_end = time.time() + holding
        while time.time() < phase_end:
            if time.time() >= next_print:
                row(start, "stopped", origin)
                next_print += 0.2
            time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    if not stopped:
        put("Vehicle/stop", 1)
    print("stopped")
