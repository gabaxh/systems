#!/usr/bin/env python3
# wheel10.py — turn one wheel until its encoder says N revolutions, for checking
# the counts-per-revolution against a chalk mark. Vehicle on blocks, gamer quit.
#   wheel10.py FrontLeft            10 revolutions at 5 RPM
#   wheel10.py BackRight 10 3       10 revolutions at 3 RPM
# LOADER=http://<host>:20197/loader points it at another machine.
import json, os, sys, time, urllib.error, urllib.request

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
    return json.load(urllib.request.urlopen(f"{H}/{path}", timeout=1))["value"]

wheel = sys.argv[1]
revs = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
rpm = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

input(f"Line up the chalk mark on {wheel} with the pointer, then press Enter")
start = get(f"{wheel}/travel")
put("Vehicle/control", 1)
try:
    while True:
        put(f"{wheel}/setpoint", rpm)        # re-sent often: the loader stops after 0.5 s of silence
        done = abs(get(f"{wheel}/travel") - start)
        print(f"\r{done:7.3f} / {revs} rev", end="", flush=True)
        if done >= revs:
            break
        time.sleep(0.1)
finally:
    put("Vehicle/stop", 1)

time.sleep(2)                                 # let it come to rest before the final reading
final = abs(get(f"{wheel}/travel") - start)
print(f"\nstopped; encoder says {final:.3f} rev. Now see where the chalk mark is.")