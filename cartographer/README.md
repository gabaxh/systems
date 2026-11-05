# cartographer

Consumes sweeps from a [guetteur](../guetteur) and builds an occupancy map,
working out as it goes where each sweep was taken from.

It maps and it localizes; it does not drive. Between sweeps it reads how far
the [loader](../loader)'s wheels have turned, and matches each sweep against the
map built so far starting from there. Without the wheels it starts from constant
velocity instead, and refuses to map where the view alone cannot place it.

## Services

| service | form | what it is |
|---|---|---|
| `map` | `MapA_v1a` | the occupancy grid built so far |
| `pose` | `PoseA_v1a` | where in the map the sensor is believed to be |
| `coverage` | `SignalA_v1a` | the share of the grid that has been observed at all |

**Each run is its own map.** The origin is wherever the first sweep was taken,
so after a restart the same coordinates name another place. The `map` and `pose`
forms carry the run in their frame — `map@2026-09-27T06:13:50Z`, the start time —
and anything planned in one run, like a [navigator](../navigator/) route, must be
refused in another.

A `map.pgm` is also written to disk every ten seconds. That is the artefact a
person looks at; the `map` service is what a system consumes.

## One sweep, start to finish

```mermaid
sequenceDiagram
    autonumber
    participant G as Guetteur
    participant L as Loader, front axle wheels
    participant C as Cartographer
    participant D as map.pgm

    loop every pollMs, 200 ms
        C->>G: GET scan
        alt the guetteur is blind or stale
            G-->>C: 503, nothing to map this time
        else
            G-->>C: the sweep: angles, distances, valid
            C->>C: too few returns to be evidence? skip it
            C->>L: GET distance (left wheel)
            C->>L: GET distance (right wheel)
            L-->>C: meters rolled, stamped when the frame arrived
            C->>C: the two wheels give the axle's motion,<br/>moved to where the scanner sits
            C->>C: prior = the last map pose, moved by that
            C->>C: match the sweep against the map from the prior,<br/>then probe the answer
            alt the peak is sharp
                C->>C: take the match
            else the view cannot say (a plain corridor)
                C->>C: with the wheels, keep the prior.<br/>Without them, refuse to map and say so
            end
            C->>C: integrate, if the vehicle has moved far enough
        end
    end

    loop every ten seconds
        C->>D: write the picture a person looks at
    end
```

## What it is, and what it is not

By default this is a SLAM **front end**, with no pose graph and no loop
closure. A long circuit of a building will not snap shut when it returns to
where it started: the drift collected on the way round simply stays. For a
hallway driven up and back, which is the experiment the students are being
given, that is the right amount of machinery.

With `loop.enabled` set, a back end is added: keyframes, a pose graph, and a
map redrawn when the vehicle comes back to where it has been. See *Loop
closure* below.

## Three things that are not obvious, and were each found by a failing test

**1. Walls must be recorded as surfaces, not points.** A sweep samples a wall at
intervals that grow with range, so recording only beam endpoints leaves a wall
as a row of dots. A later sweep then scores best by dropping its endpoints back
into those same dots — which is the pose the vehicle has already left. The
matcher fits the sampling pattern instead of the geometry and reports that
nothing has moved. `integrate` therefore joins adjacent returns that are within
30 cm of each other, and the range jump at a doorway is what stops it drawing a
wall across the opening.

**2. Matching scores against a distance transform, not the occupancy grid.**
A Euclidean distance field makes a wall uniformly attractive along its length,
so the along-wall direction carries no false evidence.

**3. A straight corridor cannot say how far along it you are.** Two parallel
walls look identical from everywhere between them. This is the aperture problem
and it is a property of the geometry, not a defect to be tuned away.

So the matcher probes its own answer: it nudges the winning pose and asks
whether the score actually falls. A flat peak is reported as flat, and then:

- **with odometry**, the odometry's position is taken and the matcher is not
  allowed to slide it along the direction the map is indifferent to. The
  **heading** is a separate question: parallel walls fix it even when they
  cannot fix the position, so where the turning probes show a clear peak the
  walls set the heading and the wheels supply only how far the vehicle went.
  The wheels are worst at heading — a few millimetres of tyre or a moment of
  slip is a turn that never happened — and left to them it bends every wall
  drawn after it. `"headingFromWalls": false` turns this off, to compare maps;
- **without odometry**, the sweep is *not* integrated at all, and the system
  says so. A corridor mapped from a pose that cannot be known comes out the
  wrong length and looks entirely convincing.

## Odometry

**The consumed services are the loader's wheel `distance`** — meters rolled by
the two wheels of the scanner's own axle, found by definition `distance`, the
wheel's `NodeID` and the vehicle's `Model`. Between two sweeps the change in
each says how far each side moved. The wheel's size is the loader's business:
it converts revolutions to meters with the circumference in its own
configuration, so calibrating the wheel size means changing one number, there.

**One axle is enough, and the articulation angle is not needed.** An
articulated loader bends in the middle, so its two halves point in different
directions; but each half is a rigid body on two wheels that do not slide
sideways, which is exactly a differential-drive robot. Its axle moves at the
mean of the two wheel speeds and turns at their difference over the track. So
the cartographer tracks the half the scanner is mounted on, from that half's
wheels, and does not need the waist sensor.
The motion is integrated as an arc, and moved from the axle's center to where
the scanner sits (`scannerMount`).

**The motion is applied to the map, not the odometry's position.** The wheel
counters start from wherever the loader was switched on; the map starts from
the first sweep. The prior for each sweep is the previous map pose moved by the
wheels' change. (The first version used an odometry pose directly as the map
pose, and would also have discovered its *own* `pose` service as the odometry
source. Neither ever ran, because nothing provided a pose.)

**Where the view cannot place the vehicle, the wheels do.** Down a plain
corridor the matcher's peak is flat, and the wheel-derived pose is taken as it
is. `odometry_test.go` drives a simulated corridor: with the wheels the estimate
follows the vehicle to the centimeter; without them every sweep is refused.

**What the wheels get wrong, and what guards against it:**

- *Slip.* The loader drives each wheel at the speed the geometry asks for, but
  tires still creep and scrub, most in tight turns, so the heading from the
  encoders drifts. That is why the matcher still corrects wherever the view
  allows.
- *Counter wrap.* The encoder's own counter wraps every 204.8 revolutions; the
  loader unwraps it, so `distance` does not.
- *A loader restart* starts the distances again from zero. A jump larger than
  `maxWheelSpeedMetresPerSecond` allows in the time between readings is taken
  as a restart, not motion, and logged.
- *A dead encoder.* The loader re-sends its latest reading once a second even
  when nothing changes, stamped with when it arrived. A reading older than
  `maxAgeMs` (1.5 s) is not used. The loader and the cartographer must
  therefore agree on the time to within that — run NTP if they are on different
  hosts.
- *Timing.* The wheels are read when a sweep is processed, not when it was
  taken, and a sweep is not de-skewed for motion during it. At walking pace
  that is a few centimeters.

| `odometry` field | default | |
|---|---|---|
| `enabled` | true | absent means true |
| `vehicle` | `{"Model": ["artitrax"]}` | picks the loader |
| `leftNodeID` / `rightNodeID` | 1 / 2 | the **front** axle; 3 / 4 for the back |
| `trackMetres` | 0.6275 | measured |
| `scannerMount` | 0, 0, 0° | forward, left, and yaw of the scanner from that axle's center |
| `maxAgeMs` | 1500 | longer than the loader's one-second heartbeat |
| `maxWheelSpeedMetresPerSecond` | 3 | bounds plausible motion between readings |

**Check the defaults against the vehicle:** they assume the SF45/B is on the
**front** half, over the axle, facing forward. If it is mounted elsewhere, say
so in `scannerMount`, and pick the axle it is on.

## Gyrodometry

**Off unless `gyro.enabled` is true**, and off the cartographer reads the
wheels exactly as described above. On, the heading between sweeps comes from
the wheels except where the [vigie](../vigie)'s gyro says they are wrong —
Borenstein and Feng's *gyrodometry* (ICRA 1996). The reasoning, and how the
gyro was characterized, are in `gyrodometry_proposal_2026-10-20.md`.

Wheels and a gyro fail in opposite ways. The wheels do not drift, once
UMBmark has calibrated the track and the wheel sizes, but a spinning wheel or
a scrubbing tyre is a turn that never happened. The gyro is right over a
fraction of a second, but its bias adds up. So the turn is taken from the
wheels, and from the gyro only over the short intervals in which the two
disagree by more than `thresholdDegreesPerSecond`. The distance is always the
wheels'.

```mermaid
sequenceDiagram
    autonumber
    participant L as Loader, the front wheels
    participant V as Vigie, the Mid-360's gyro
    participant C as Cartographer

    loop every pollMs (50 ms)
        C->>L: GET distance, left and right
        C->>V: GET heading
        C->>C: keep each with its own timestamp
        C->>C: for every 100 ms interval that has closed:<br/>the turn from the wheels and from the heading,<br/>both at the same two instants
        alt they differ by more than the threshold, or did within holdMs
            C->>C: the gyro's turn
        else
            C->>C: the wheels' turn
        end
    end
    C->>C: each sweep: the motion gathered since the last is the prior
```

- **The heading, not the rate.** The vigie integrates its gyro at 200 Hz and
  serves the angle, so two readings give the turn between their timestamps
  exactly, however seldom they are read.
- **Per interval, not per sweep.** A tenth of a second, as in the paper, so a
  short slip is isolated rather than spoiling a whole sweep.
- **A hold.** A wheel spinning in a pile disagrees for seconds, wandering
  above and below the threshold; the gyro keeps the turn for `holdMs` after
  the last disagreement. `holdMs: 0` is the paper's rule.
- **Holes fall back to the wheels.** No vigie, a stale heading, or the 503 the
  vigie gives just after a gap in its own stream: the intervals across it take
  the wheels' turn, as without gyrodometry. If the wheels fail and the gyro
  does not, the sweep starts from constant velocity with the gyro's turn.
- **The matcher still has the last word.** Gyrodometry improves the prior;
  where the walls fix the pose, the walls win.

The tests drive a scripted vehicle (`gyrodometry_test.go`). A wheel slipping
20 % for a second while driving straight turns the wheels' heading 18°, and
gyrodometry's 0.65° — the gyro's bias over the event. A biased gyro changes
nothing while the two agree, and the joint swinging at standstill is taken
from the gyro.

| `gyro` field | default | |
|---|---|---|
| `enabled` | false | absent means off |
| `sensor` | `{"Model": ["Livox Mid-360"]}` | picks the vigie |
| `pollMs` | 50 | how often the wheels and the heading are read |
| `intervalMs` | 100 | the paper's T |
| `lagMs` | 100 | how far behind the present the intervals end, so readings from both sides have arrived |
| `maxAgeMs` | 200 | the longest hole in the heading an interval may span |
| `thresholdDegreesPerSecond` | 1.25 | the paper's 0.125° per 0.1 s; to be checked on the vehicle |
| `holdMs` | 300 | 0 is the paper's rule |
| `mode` | `gyrodometry` | `odometry` or `gyro` take the turn from one source always, to compare maps |
| `logPath` | | one CSV line per interval: `t,T,dL,dR,turnWheelsDeg,turnGyroDeg,source` |

Every thirty seconds the log says how often the gyro was believed:

```
cartographer: gyrodometry — 2.1 % of 300 intervals from the gyro, longest 0.6 s, gyro unavailable in 0.0 %
```

**Before trusting it on the vehicle:** set the vigie's mount from gravity (its
README, *Check the mount*), calibrate the wheels with UMBmark
(`umbmark_guide_2026-10-21.md`), and set the threshold from the CSV of a
normal drive: above the 99.5th percentile of |turnGyro − turnWheels| / T.

## Loop closure

**Off unless `loop.enabled` is true**, and off the cartographer maps exactly
as described above. On, it adds a back end: it remembers where each part of
the map was seen from, notices when the vehicle is back somewhere it has
been, and takes the drift collected in between back out of the map. The
reasoning, the maths and the measurements are in
`loop_closure_proposal_2026-11-05.md`.

```mermaid
sequenceDiagram
    autonumber
    participant F as Front end (consume)
    participant M as Memory
    participant L as Loop closer (its own goroutine)

    F->>M: every sweep drawn: kept, hanging off a keyframe;<br/>every 0.5 m or 15°, a new keyframe, with the<br/>wheels' uncertainty since the last
    loop every 500 ms
        L->>M: a snapshot, if there are new keyframes
        L->>L: for each: the older keyframes 10 m or more back<br/>along the path and near by the map; match against<br/>a submap drawn from them
        alt scores well, fixes two directions, and twice in a row
            L->>L: into the pose graph; optimise; its χ² must agree<br/>with the rest of the graph
            L->>L: draw the map again from the keyframes, outside the lock
            L->>F: swap the map in, and move the vehicle with it
        end
    end
```

- **Keyframes.** Every sweep the front end draws is kept, as float32, hanging
  off a keyframe by its offset. Every `keyframeTravelMetres` or
  `keyframeTurnDegrees` one of them becomes a keyframe: a node of the pose
  graph. Between keyframes, each step's uncertainty depends on how the front
  end placed that sweep, and the total is the weight of the edge between them:
  - **The map fixed the pose:** the uncertainty is reset to `matchedMetres` and
    `matchedDegrees`.
  - **In a corridor, where the walls set the heading:** the heading is reset to
    `matchedDegrees`, and the position along the corridor grows with
    `corridorAlongFraction`.
  - **Wheels alone (or gyrodometry):** the uncertainty grows with `noise`.

  This matters for where a loop's error goes. On 5 November, weighting every
  step as the wheels' random walk made the heading look loose and the distance
  along a corridor tight, which is the opposite of what the front end gets
  wrong. Closing 1.8 m of along-track drift, the optimiser bent the corridor
  between the two squares instead of stretching it.
- **Finding a loop.** A keyframe is matched against the older keyframes at
  least `minGapMetres` back along the path, and near enough by the map that the
  drift could explain the rest. The match is against a submap drawn from the
  older keyframe and its neighbours, not against the whole map: the whole map
  already holds the doubled wall the loop is meant to remove.
- **Believing it.** Corridors repeat, and a convincing match in the wrong place
  folds the map. A match has to pass every one of these:
  - **Score:** at least `minScore`, counted over the returns that land where
    the old submap has seen something. A return in unknown ground says
    nothing either way. At a junction approached from a new direction, most of
    what the vehicle sees was never seen before.
  - **Overlap:** at least `minOverlap` of the returns must land on known
    ground.
  - **Directions:** it must fix at least two. One that fixes only two, as in
    a corridor, keeps only its heading. The direction the corridor does not
    fix is never found exactly, and the rest of the match is not to be
    trusted.
  - **Twice:** it must be found again from the next keyframe with the same
    correction, compared in the directions both matches fix.
  - **Position:** a match's position goes into the graph only once a second
    match with at least `positionOverlap` of its returns on known ground
    agrees with it. Until then it gives its heading only. On half a sweep a
    match still has the heading right, but its place along a corridor can be
    off by a door's spacing.
  - **χ²:** once in the graph, it must agree with the rest, within
    `maxChi2`.
- **Optimising.** Levenberg–Marquardt on the graph, by conjugate gradients
  preconditioned by the chain of keyframes. A new loop starts from the headings
  and then the positions solved alone, and is judged without a robust kernel;
  loops already accepted get a Cauchy kernel (`robustScale`), so that one
  later found to be wrong cannot fold the map.
- **Redrawing.** When some keyframe moves more than `redrawMetres`, the grid is
  drawn again from every kept sweep, outside the lock, and swapped in. The
  vehicle's pose moves with the newest keyframe, and a sweep matched against
  the old grid during the swap is moved too. The frame stays
  `map@<start time>`; the `pose` service jumps once, by the size of the
  correction.

The tests drive the ring corridor of `loops_test.go`, 20 × 10 m round a block,
with a twentieth of a degree of phantom turn every metre:
- the end keyframe goes from 0.25 m out to 4 mm;
- the share of drawn walls within 10 cm of a real one goes from 60 % to 98 %.

A plain corridor driven up and back with wheels reading 3 % long accepts
23 loops, none of which moves a keyframe along the corridor. A 1 km loop of
2000 keyframes with 100° of drift in it closes in one iteration and 8 ms
(`BenchmarkALongLoop`).

| `loop` field | default | |
|---|---|---|
| `enabled` | false | absent means off |
| `keyframeTravelMetres` / `keyframeTurnDegrees` | 0.5 / 15 | |
| `noise` | 3 % along, 2 % across, 1.5 °/m, 10 % of a turn | lower `headingDegreesPerMeter` with gyrodometry |
| `maxSweeps` | 20000 | past it the oldest are thinned; keyframes are kept |
| `minGapMetres` | 10 | how far back along the path a loop must reach |
| `searchRadiusMetres` / `driftFraction` | 2 / 0.05 | how near by the map, growing with the path between |
| `maxCandidates` | 3 | |
| `submapMetres` / `submapKeyframes` | 16 / 5 | |
| `search` | ±1.2 m, ±20°, 0.10 m / 2° → 0.0125 m / 0.5° | as the front end's `search` |
| `minScore` | 0.55 | per return that lands where the old submap has seen something; 1 is every such return on a wall |
| `minOverlap` | 0.3 | the share of returns that must land where the old submap has seen something for a match to count at all |
| `positionOverlap` | 0.75 | the share needed before a match's position is trusted, and not just its heading. A second such match must agree |
| `consistentMatches` | 2 | keyframes in a row |
| `maxChi2` | 7.8 | χ² of three degrees of freedom at 95 % |
| `robustScale` | 3 | Cauchy, in standard deviations, on loops already accepted |
| `maxIterations` | 20 | |
| `redrawMetres` | 0.05 | |
| `matchedMetres` / `matchedDegrees` | 0.02 / 0.05 | how far a step may be off where the map fixed the pose; the heading's also where the walls set it |
| `corridorAlongFraction` | 0.25 | along a corridor, the position's uncertainty per step as a fraction of the step. Large, because the wheels' error there is mostly a scale error, the same every metre |
| `logPath` | | one CSV line per match tried, and one more for each put to the graph: `t,i,j,score,seen,dx,dy,dthetaDeg,chi2,believed,accepted,overlap`. `chi2` is the one `maxChi2` judges, before the robust kernel; 0 on lines for matches not put to the graph. `overlap` is the share of returns on known ground |
| `graphPath` | | the pose graph in g2o format, after each closure |

**First tried on the vehicle on 5 November**, on a laptop, with a 250 m grid.
The first loop closed. Later in the run, closures found from one stale
snapshot stacked on each other and turned the map 20–30°. That is fixed:
see the proposal, section 3.6, and `TestALaggingCloserClosesTheLoopOnce`.
What else that run changed:
- **The size of the grid no longer costs anything.** The likelihood field is
  rebuilt only over the part of the grid anything has been drawn in, plus a
  margin. On a 250 m grid (25 million cells) a rebuild took 466 ms, after
  every sweep drawn and after every redraw. It now takes 2.5 ms
  (`BenchmarkRebuildFieldOn250Metres`).
- **Drawing allocates nothing.** Every beam used to build a list of cells.
  A redraw of the ring on a 250 m grid went from 327 ms to 122 ms
  (`BenchmarkRedrawOn250Metres`), and the front end draws each sweep faster
  too.
- **`maxCandidates`** is how many older keyframes each new keyframe is tried
  against, nearest first. Each try is a submap and a wide search. If the log
  says the loop closer fell behind and skipped keyframes, 1 is a third of the
  work.
- **The scanner:** the cartographer's `scan` cervice still takes any provider,
  guetteur or vigie, and it should be pinned to the vigie.
