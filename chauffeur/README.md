# chauffeur

Drives the route the [navigator](../navigator/) has planned. It takes control of
the [loader](../loader/), follows the route by velocity and curvature, and holds
for anything that makes the route unsafe to follow, until a person says to go on.

It plans nothing, and it decides nothing about where to go. The
[commander](../commander/) is the page that tells it when to go; so can `curl`.

> **Tested in simulation and on the bench, not on the vehicle.** The bench ran the
> real systems against a stand-in loader. **Keep the pad in hand on every run:** it
> is the safety stop. Work through [first runs](#first-runs-on-the-vehicle) in order.

## Services

| service | form | what it does |
|---|---|---|
| `engage` | `SignalA_v1a` | **PUT** `1` to drive the route, or to go on from a hold; **PUT** `0` to come to rest and release control; **GET** `1` while it has or wants control |
| `drivingState` | `SignalA_v1a` | what it is doing and why, as a code below |

It consumes the navigator's `path`, the cartographer's `pose`, the guetteur's or
the vigie's `clearance`, and the loader's `control`, `velocity`, `curvature`, `stop`
and `curvatureLimit`.

In a cloud with certificates, `engage` refuses a caller that cannot be identified,
as the loader does. Otherwise the plain HTTP port would let anyone on the network
start the vehicle. In an authorized cloud the policy must allow all of the above.

## What it does when

| code | state | what happens | how it ends |
|---|---|---|---|
| 0 | idle | nothing | `engage 1` |
| 1 | asking for control | `PUT control 1` | the loader answers |
| 2 | driving | follows the route | any of the below |
| 3 | arrived | at rest, control released | `engage 1`, no pad needed |
| 4 | coming to rest | velocity 0, then releases control | by itself |
| 10 | hold: something closer than `holdClearanceMetres` | velocity 0, keeps control | `engage 1` |
| 11 | hold: no clearance reading | | |
| 12 | hold: no recent pose | | |
| 13 | hold: route and pose in different maps | | |
| 14 | hold: no route, or an old one | | |
| 15 | hold: further than `offRouteMetres` from the route | | |
| 16 | hold: the loader refused a curvature, usually an uncalibrated waist | | |
| 17 | hold: the route ends elsewhere, or is much longer | | |
| 20 | stopped: something closer than `stopClearanceMetres` while moving | `PUT stop` | the pad, then `engage 1` |
| 21 | lost control: the loader answered `409` | nothing, and never retried | the pad, then `engage 1` |

**A hold stays a hold.** It does not end when the reason does: a person looks, and
says to go on. If the reason is still there, it holds again at once.

**A hold is not a stop.** The chauffeur keeps sending velocity 0 and keeps control,
because a stop can only be undone with the pad.

**The route's end is judged here.** The navigator checks for arrival only when it
replans, every five seconds, which is 2.5 m at walking pace.

**A route nobody has looked at is not driven.** Replans that tidy up the same route
go through. One that ends elsewhere or goes a long way round is a hold (17), and
going on accepts it.

If the chauffeur crashes, the loader stops the vehicle after half a second of
silence. If it is closed while driving, it stops the vehicle itself.

## How it steers

**Pure pursuit:** aim at a point `lookaheadMetres` further along the route, and
drive the circle through that point: `κ = 2y / d²`, where `d` is the distance to the
point and `y` how far to the side it lies. Each half of an articulated loader is a
rigid body on an axle that does not slide sideways, which is what this assumes.

**Check that the scanner is over the front axle.** The pose is the scanner's, and
the loader's curvature is the front axle's. This treats them as one point.

The simulator (`sim_test.go`) closes the loop with the loader's kinematics, a waist
with the angle loop's hold band, a pose at 5 Hz that is up to 400 ms old, and late
commands. On the planner's tightest routes, at 0.5 m/s:

| lookahead | 15°/s waist, hard S | 8°/s waist, hard S | old pose |
|---|---|---|---|
| 0.8 m | 0.04 m off the route | 1.5 m off | 0.24 m off |
| **1.2 m** | **0.16 m off** | 0.29 m off | 0.15 m off |
| 1.5 m | 0.23 m off | 0.21 m off | 0.21 m off |

**The waist's speed decides what can be followed, and it has not been measured.**
15°/s is a guess. The planner does not know the waist takes time, so it can plan a
swing from one lock to the other in less distance than the machine needs: at 8°/s,
about three meters. The result is a hold for being off the route, which is safe and
does not finish the route. If that happens, drive slower or use a longer lookahead.

The simulator leaves out slip, chain slack and a scan matcher that moves the pose.

## First runs on the vehicle

**Before any of this:**

- The waist and the wheels are [calibrated](../loader/README.md#calibrating).
- Measure how fast the waist swings, lock to lock, and the lowest speed that turns
  the wheels reliably. Set `cruiseMetresPerSecond` and `lookaheadMetres` from these.
- Set the navigator's `clearanceMetres` to about 0.6. Its default of 0.4 leaves
  2.5 cm on a vehicle 0.75 m wide. Half the width and `offRouteMetres` must fit
  inside it.
- Map the area by hand, and look at some routes in `plan.ppm`.

**Then, with someone holding the pad, and each step clean before the next:**

1. **On blocks.** Hand over with the pad, set a goal, `engage 1`. Check the commands
   in the loader's log, the pad's stop, and that killing the chauffeur stops the
   wheels within half a second.
2. **Open floor,** 5 m straight. Then put something in the way, and `engage 1` when
   it has gone.
3. **One curve.** Note how far it strays, and tune.
4. **A corridor,** and then a loop.

```bash
H=http://<host>:20206/chauffeur/Driver
curl -X PUT "$H/engage" -H "Content-Type: application/json" -d '{"value": 1, "version": "SignalA_v1.0"}'
```

```bash
curl -s "$H/drivingState"
```

```bash
curl -X PUT "$H/engage" -H "Content-Type: application/json" -d '{"value": 0, "version": "SignalA_v1.0"}'
```

**Handing over:** the vehicle starts stopped, and after a stop only the pad can take
it. Take control with the pad, then release it, and the chauffeur can take it.

**`clearance` is only a first check.** It looks ±20° straight ahead, which is
narrower than the vehicle within about a meter, and a wall ahead in a curve can
trigger it. If curves hold for no reason, lower `holdClearanceMetres` with care.

## Configuration

| field | default | |
|---|---|---|
| `cruiseMetresPerSecond` | 0.5 | |
| `lookaheadMetres` | 1.2 | see [how it steers](#how-it-steers) |
| `commandHz` | 20 | the loader stops after 0.5 s of silence |
| `holdClearanceMetres` / `stopClearanceMetres` | 0.8 / 0.4 | starting values |
| `arriveMetres` | 0.25 | how near the route's end counts as there |
| `offRouteMetres` | 0.2 | |
| `maxPoseAgeMs` / `maxClearanceAgeMs` | 1000 / 1000 | |
| `maxRouteAgeSeconds` | 15 | three of the navigator's replans |
| `routeEndMovedMetres` / `routeLongerMetres` | 1 / 3 | how far a replanned route may differ |
| `maxCurvature` | 0.4 | used until the loader says |
| `vehicle` | `{"Model": ["artitrax"]}` | picks the loader |
| `clearanceSensor` | none | picks the guetteur or the vigie, if both run |

## Not here yet

- **Going on by itself** when an obstacle has gone.
- **Slowing for curves,** which the wheels' lowest speed leaves little room for.
- **Checking the route's corridor against the live scan,** in place of `clearance`.
- **Reversing.** The navigator plans forward only.
