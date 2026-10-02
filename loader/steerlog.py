#!/usr/bin/env python3
# steerlog.py — step the steering by curvature and record what the waist does.
#   steerlog.py                  straight, 20° left, 20° right, straight; 5 s each, standing still
#   steerlog.py 0.2              the same, rolling at 0.2 m/s
#   steerlog.py 0.2 35 8         rolling at 0.2 m/s, steps of 35°, 8 s each
#
# Twenty times a second it prints the time, the target angle and the measured
# articulation, so a swing shows as the measured angle crossing the target
# back and forth, and growing or not. The targets are angles; the curvature for
# each is worked out from the geometry in systemconfig.json.
#
# Quit the gamer first: it sends steering 50 times a second and the two would
# take turns. Ctrl-C stops the vehicle; if this script dies, the loader stops on
# its own after half a second without a command. LOADER=http://<host>:20197/loader
# points it at a loader on another machine.
import json, math, os, sys, time, urllib.error, urllib.request

H = os.environ.get("LOADER", "http://127.0.0.1:20197/loader")

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

def geometry():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "systemconfig.json")
    try:
        with open(path) as f:
            g = json.load(f)["unit_assets"][0]["traits"][0]["geometry"]
        return g["jointToFrontAxleMetres"], g["jointToRearAxleMetres"]
    except (OSError, ValueError, KeyError, IndexError):
        return 0.6175, 0.6175  # the loader's own default

def curvature(degrees):
    l1, l2 = geometry()
    g = math.radians(degrees)
    return math.sin(g) / (l1 * math.cos(g) + l2)

args = [a for a in sys.argv[1:] if a != "--"]
velocity = float(args[0]) if args else 0.0
step = float(args[1]) if len(args) > 1 else 20.0
hold = float(args[2]) if len(args) > 2 else 5.0
targets = [0.0, step, -step, 0.0]

print(f"targets {targets}°, {hold:g} s each, at {velocity:g} m/s; Ctrl-C stops")
print("     time  target  measured")
put("Vehicle/control", 1)
start = time.time()
try:
    for target in targets:
        k = curvature(target)
        until = time.time() + hold
        while time.time() < until:
            put("Vehicle/velocity", velocity)
            put("Vehicle/curvature", k)
            a = get("Vehicle/articulation")
            print(f"{time.time() - start:7.2f} s {target:6.1f}°  " + ("-" if a is None else f"{a:6.1f}°"), flush=True)
            time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    put("Vehicle/stop", 1)
    print("stopped")
