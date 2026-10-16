# Algorithms and maths of the tennis collector

This document describes how the controller in
`controllers/tennis_collector/tennis_collector.py` (and the learning code in
`policy.py` and `tools/train_policy.py`) decides what to do, with the formulas
and the constants it actually uses. Function names are given so each part can
be checked against the code. Constants are quoted as they are in the source;
if they disagree, the source wins.

Contents

1. The control loop
2. Frames and conventions
3. Pose estimation
4. Attitude
5. Lidar, obstacle map and clearance field
6. Ball sensing: camera, memory and coverage
7. Lift beams: capture and throat
8. Planning and driving
9. The state machine
10. Recovery and stuck detection
11. The learned policy (Q-learning)
12. Experience buffer and offline training
13. Referee and metrics
14. Known gaps

---

## 1. The control loop

The world's basic time step is 32 ms (`basicTimeStep 32`), so the loop runs at
about 31 Hz. Each iteration of `run()` does, in order:

| Step | Function | What it does |
|---|---|---|
| 1 | `read_pose()` | dead-reckon the robot pose (section 3) |
| 2 | `obstacle_points_world()` | lidar scan to world-frame hit points and rays (5) |
| 3 | `update_map()` | every `MAP_PERIOD` = 0.16 s: rebuild the obstacle grid and clearance field (5) |
| 4 | `update_odometry()`, `monitor_attitude()` | wheel speed, roll/pitch/tilt (4) |
| 5 | `update_ball_states()` | referee only: which balls really are in the bin (13) |
| 6 | `update_beams()` | lift break-beams: capture events (7) |
| 7 | `sense_balls()` | every `SENSE_PERIOD` = 0.25 s: camera, ball memory, coverage (6) |
| 8 | `check_attitude()`, `check_walls()` | fall and wall guards (9, 10) |
| 9 | state dispatch | `run_seek()`, `run_align()`, `run_collect()`, ... (9) |
| 10 | `detect_stuck()`, `rl_tick()` | stuck and idle watchdogs (10), policy housekeeping (11) |

Two environment switches change the behaviour:

* `TENNIS_GROUND_TRUTH=1`: the old supervisor shortcuts (exact pose, exact ball
  positions, exact throat sensor, self-right teleport). Default is off, i.e.
  real sensing.
* `TENNIS_NO_POLICY=1`: the learned policy never drives; DWA always does. This
  is the recommended setting today (section 11).

---

## 2. Frames and conventions

* **World frame**: Webots ENU: x east, y north, z up. The robot's allowed region
  is `--bounds xmin xmax ymin ymax`, default (-5.75, -0.70, -2.70, 2.70): one
  half of the court, net side at x = 0.
* **Robot frame**: origin at the robot's reference point, x forward, y left,
  z up. Heading h is the yaw of the robot's x axis in the world.
* **Grid**: square cells of `CELL` = 0.10 m; cell (i, j) covers
  [i·CELL, (i+1)·CELL) × [j·CELL, (j+1)·CELL), centre ((i+½)·CELL, (j+½)·CELL)
  (`cell_of`, `cell_center`).

World to robot (`to_robot`), with c = cos h, s = sin h:

$$
\begin{aligned}
l_x &= (w_x - x)\,c + (w_y - y)\,s \\
l_y &= -(w_x - x)\,s + (w_y - y)\,c
\end{aligned}
$$

Robot to world (`to_world`): w_x = x + l_x c − l_y s, w_y = y + l_x s + l_y c.

All navigation uses the **estimated** pose (x, y, h). The referee uses the true
pose (`true_x`, `true_y`, `true_h`, `truth_to_robot`), and only for scoring.

---

## 3. Pose estimation (`read_pose`)

Dead reckoning from the two wheel encoders and the yaw gyro: what an ESP32 with
two encoder motors and an MPU6050 can do.

Per step, with encoder angles θ_L, θ_R (rad), wheel radius r = `WHEEL_RADIUS`
= 0.09 m, gyro yaw rate ω_z and time step Δt:

$$
\begin{aligned}
d_L &= r\,\Delta\theta_L, \qquad d_R = r\,\Delta\theta_R, \qquad d = \tfrac12 (d_L + d_R) \\
\Delta h &= (\omega_z + b)\,\Delta t \\
x &\leftarrow x + d \cos\!\left(h + \tfrac12 \Delta h\right) \\
y &\leftarrow y + d \sin\!\left(h + \tfrac12 \Delta h\right) \\
h &\leftarrow \operatorname{wrap}(h + \Delta h)
\end{aligned}
$$

* Heading comes from the gyro, not from the wheel difference (d_R − d_L)/track:
  skid-steer wheels slip in turns, so the wheel difference over-reads the turn.
* b is a fixed per-run gyro bias drawn from N(0, `GYRO_BIAS_SD`² = 0.002²) rad/s
  (seeded by `--seed`), on top of the per-sample gyro noise set in the PROTO
  (`lookupTable` noise 0.005, about 0.3°/s rms). A calibrated MPU6050 is in
  this range.
* The start pose is known (the robot is placed at a known spot, as on a real
  court). After that nothing corrects the estimate: there is no GPS, no
  landmark matching and no scan matching.
* `pose_err` = ‖(x, y) − (x_true, y_true)‖ is logged (status line `perr`,
  trace fields `ex`, `ey`). Measured peak errors: 0.1 to 0.5 m over 1 to 5
  minute runs.

---

## 4. Attitude (`monitor_attitude`)

From the `imu` InertialUnit: roll φ and pitch θ (the yaw output is ignored).
Tilt is the angle between the body z axis and the vertical:

$$
\text{tilt} = \arccos(\cos\varphi \cos\theta)
$$

Thresholds: `TILT_WARN` = 0.35 rad, `TILT_FALLEN` = 0.70 rad. Above
`TILT_FALLEN` the robot enters `DOWN`; with real sensing there is no self-right,
so it tries to drive off the obstacle and, if still down after `FALL_GIVE_UP`
= 8 s, ends the run.

Tilt also gates the obstacle memory (section 5): above `MAP_MAX_TILT` = 1.5°
no lidar hit is remembered.

---

## 5. Lidar, obstacle map and clearance field

### 5.1 Beams to points (`obstacle_points_world`)

Three single-layer lidar fans (`FAN_DEFS`), each `(name, sx, sy, yaw, fov,
res, maxr)`:

| Fan | Mount (sx, sy) | Yaw | FOV | Beams | Max range |
|---|---|---|---|---|---|
| lidarLeft | (−0.28, +0.17), z 0.50 | +0.85 rad | 2.6 rad | 128 | 8 m |
| lidarRight | (−0.28, −0.17), z 0.50 | −0.85 rad | 2.6 rad | 128 | 8 m |
| bumperLidar | (0.50, 0), z 0.30 | 0 | 2.8 rad | 96 | 1.2 m |

Webots orders a range image left to right, so beam k of N has angle

$$
a_k = \text{yaw} + \frac{\text{fov}}{2} - \left(k + \tfrac12\right)\frac{\text{fov}}{N}
$$

(Up to commit 558f108 the code used yaw − fov/2 + (k+½)·fov/N, which mirrored
every scan. `TENNIS_LIDAR_PROBE=1` checks this against the court walls: 9 mm
median error the right way round, about 1 m mirrored.)

A beam with range ρ lands at robot-frame (sx + ρ cos a, sy + ρ sin a), then
goes to the world frame with `to_world`. Points inside the robot's own
footprint box (`SELF_X` = (−0.60, 0.90), |y| ≤ `SELF_Y` = 0.38) are dropped.
A beam at the range cap is "no hit" but still marks free space.

### 5.2 Grids (`update_map`)

* `occ`: cells containing a hit **in the current scan only**. Rebuilt every
  `MAP_PERIOD`. This is what planning uses.
* `known_free`: cells any beam has passed through.
* `known_occ`: cells that were hits. A hit is added only while tilt <
  `MAP_MAX_TILT` (when pitched, the 0.50 m-high fans hit the floor inside
  their 8 m range), and a later beam passing through a remembered cell removes
  it. Used only by the coverage search (6.4) and saved in the map file.

Both remembered sets are pruned to the local window once they exceed 4000
cells.

### 5.3 Clearance field (`build_clearance`)

A multi-source Dijkstra from every `occ` cell over an 8-connected grid inside
the local window (half-size `WINDOW` = 3.2 m around the robot):

$$
D(c) = \min_{o \in \text{occ}} \; \text{path length}(o \to c), \qquad
\text{edge cost} = \text{CELL} \times \{1, \sqrt2\}
$$

capped at `WINDOW`. `clearance(x, y)` returns D at that cell, or `WINDOW` if
the cell was never reached (unknown counts as clear). This is a cheap
approximation of the Euclidean distance transform (the octile metric
over-estimates true distance by at most about 8 %).

---

## 6. Ball sensing

### 6.1 Camera model (`cam_project`, `cam_ground`)

`ballCam`: 160 × 120 pixels, horizontal field of view F = 1.0 rad, mounted at
C = (−0.25, 0, 0.58) in the robot frame, pitched down by θ_c = 30°
(`CAM_PITCH` = 0.5236). Pinhole focal length in pixels:

$$
f = \frac{W/2}{\tan(F/2)} \approx 146.4
$$

Camera axes in the robot frame: forward (cos θ_c, 0, −sin θ_c), left
(0, 1, 0), up (sin θ_c, 0, cos θ_c).

**Projection** of a robot-frame point P, with Δ = P − C:

$$
\begin{aligned}
\text{fwd} &= \Delta_x \cos\theta_c - \Delta_z \sin\theta_c \\
\text{up} &= \Delta_x \sin\theta_c + \Delta_z \cos\theta_c \\
u &= \tfrac{W}{2} - f\,\frac{\Delta_y}{\text{fwd}}, \qquad
v = \tfrac{H}{2} - f\,\frac{\text{up}}{\text{fwd}}
\end{aligned}
$$

**Back-projection** of pixel (u, v) onto the plane z = R_ball = 0.0335 m (the
height of a ball's centre):

$$
\begin{aligned}
\ell &= \frac{W/2 - u}{f}, \qquad \upsilon = \frac{H/2 - v}{f} \\
\mathbf{d} &= (\cos\theta_c + \upsilon \sin\theta_c,\;\; \ell,\;\; -\sin\theta_c + \upsilon \cos\theta_c) \\
t &= \frac{R_\text{ball} - C_z}{d_z}, \qquad P = C + t\,\mathbf{d} \quad (d_z < 0)
\end{aligned}
$$

### 6.2 Detection (`detect_balls`)

1. **Colour test** per pixel (8-bit r, g, b):
   g > 90, b < 0.55 g and 0.55 g < r < g. Tennis-ball felt is about
   (212, 230, 51): r/g ≈ 0.9 in the material, about 0.75 rendered. The test
   rejects court blue and green (r too low), clay (r > g), white lines (b too
   high) and the robot's orange fan (r > g).
2. **Blobs**: 4-connected components of the mask (depth-first flood fill).
3. A blob of at least `CAM_MIN_PIXELS` = 2 pixels is back-projected from its
   centroid (6.1). It is kept if `CAM_MIN_X` = 0.30 m ≤ l_x (nearer than this
   is the robot's own intake and lift) and range ≤ `CAM_MAX_RANGE` = 3.5 m.

A ball 3.5 m away is about f · 0.067 / 3.5 ≈ 2.8 px wide, which sets the
useful range. Occlusion is real (the camera cannot see through the net or
another ball). The camera runs at about 4 Hz (enabled every 8 steps).

### 6.3 Ball memory (`sense_balls`)

Each detection is converted to the world frame with the **estimated** pose,
then matched against memory:

* nearest unconsumed entry within `MEMORY_MATCH` = 0.30 m → update it;
  otherwise create a new entry;
* update is a running average with gain

$$
k = \begin{cases}
0.6 & \text{if the sighting is within 1.2 m of the robot (ball may be moving)} \\
1/\min(n+1, 5) & \text{otherwise, } n = \text{sightings so far}
\end{cases}
\qquad \hat p \leftarrow \hat p + k\,(z - \hat p)
$$

* **Negative evidence**: an entry that should be visible (inside the camera
  view with a 6 px inner margin, l_x ≥ 0.45 m, range ≤ 3.1 m) but was not seen
  gets a miss; after `MEMORY_MISSES` = 4 misses it is forgotten.
* A capture (7) marks the nearest entry within 0.6 m of the capture position as
  collected (`consume_memory_near`).

Navigation reads only this memory (`loose_balls()`); it never reads the true
ball list.

### 6.4 Coverage map (`mark_sensed`, `coverage_waypoint`)

`sensed` is the set of grid cells whose centre has been inside the camera view
(same test as 6.2, without the shrink). `SEARCH` drives to the nearest cell
inside the bounds (minus `SEARCH_MARGIN` = 0.45 m) that is not in `sensed`,
not in `known_occ` and has clearance ≥ `ROBOT_RADIUS`. When there is none and
memory holds no reachable ball, the run is DONE.

Coverage fraction = |sensed ∩ inner region| / |inner region|.

### 6.5 Saved map (`save_map`)

Every `MAP_SAVE_PERIOD` = 5 s and at the end of the run:
`logs/map_<world>.json` with `occupied` (known_occ cells), `looked_at`
(sensed cells), `balls` (memory, estimated frame), `captures` and the referee
count. `tools/plot_map.py` draws it. **The robot never loads it**: each run
starts with an empty map.

---

## 7. Lift beams (`update_beams`, `ball_in_throat`)

Two downward IR distance sensors over the lift rail (rail normal
n = (0.707, 0, 0.707), sensor axis −n):

| Beam | Position | Empty reading | With a ball |
|---|---|---|---|
| liftLow | (0.190, 0, 0.167) | 0.23 m | ≈ 0.10 m |
| liftTop | (0.035, 0, 0.322) | 0.30 m (range cap) | ≈ 0.11 m |

(Webots skips the robot's own body when casting the ray, so an empty beam
reads past the rail. A ball is a separate object and is seen.)

* A beam is "blocked" when its reading < `BEAM_BALL` = 0.15 m.
* **Capture** = rising edge of liftTop. Its floor position is the current
  target if that is within (−0.2 < l_x < 1.0, |l_y| < 0.4), otherwise the
  intake mouth (0.40, 0) in the robot frame. The bin count the robot believes
  (`bag`) is the number of liftTop rising edges.
* **Throat** (a ball is on its way up): liftLow blocked now, or blocked less
  than `LIFT_TRANSIT` = 3 s ago without a liftTop edge since. While the throat
  is busy the COLLECT timeout and the under-body abort are suspended.

---

## 8. Planning and driving

Three layers: target choice, global A* path, local DWA command.

### 8.1 Target choice (`choose_target`)

Among remembered, reachable (inside bounds ± 0.20 m), not banned balls:

$$
\text{cost}(b) = \lVert (l_x, l_y) \rVert + 1.2\,[\,l_x < 0.2\,]
$$

i.e. the nearest ball, with a penalty for balls beside or behind the intake.
Bans (`avoid_until`) come from failed pickups (6 s) and stuck events
(`STUCK_AVOID` · 2^(n−1) s, abandoned after `STUCK_GIVE_UP` = 4).

### 8.2 A* (`astar`)

8-connected search on the local grid. A cell is blocked when its clearance
< `ROBOT_RADIUS` = 0.30 m (configuration-space inflation). Edge cost
1 or √2, plus 0.6 if the cell's clearance < 1.5 · ROBOT_RADIUS (prefer a
margin). Octile heuristic

$$
h(a, b) = (\Delta_x + \Delta_y) + (\sqrt2 - 2)\min(\Delta_x, \Delta_y)
$$

which is admissible for the edge costs without the margin penalty. Blocked
start or goal cells are moved to the nearest free cell. Search is capped at
40000 expansions. Replanning happens every `REPLAN_PERIOD` = 0.5 s or when the
goal changes (`maybe_replan`).

### 8.3 DWA (`dwa`)

The local goal is the first path point beyond `LOOKAHEAD` = 0.45 m (or the
goal itself, capped at 0.6 m, when there is no path), at bearing g_dir.
Candidate commands are the lattice

v ∈ {0, 0.16, 0.32, 0.47, 0.62} m/s, ω ∈ {−2.4, −1.6, −0.8, 0, 0.8, 1.6, 2.4} rad/s,

each rolled out for `DWA_HORIZON` = 0.6 s in 6 steps with the unicycle model
h += ω Δt, x += v cos h Δt, y += v sin h Δt. A rollout is rejected if any
point has clearance < ROBOT_RADIUS, or if v > safe_speed(min clearance).
Survivors are scored, lowest wins:

$$
J = 1.4\,|\operatorname{wrap}(\omega T - g_\text{dir})| + 1.0\,\lVert p_T - g \rVert
  + 2.5\,v\,[\,|g_\text{dir}| > 1.05\,] + 0.35\,(v_\max - v)
  + 0.8\,\max(0,\, 0.45 - d_\min) + 5\,[\,v = 0 \wedge \omega = 0\,]
$$

The third term makes the robot pivot first when the goal is behind it. If
every rollout is rejected: pivot toward the goal at ω = clamp(1.8 g_dir), at
least 1 rad/s, reversing at 0.10 m/s when the goal is close and behind.

### 8.4 Safe speed (`safe_speed`)

The speed from which the robot can still stop before the keep-off gap, with
assumed deceleration a = `WALL_DECEL` = 1.6 m/s²:

$$
v_\text{safe}(d) = \sqrt{2a\,(d - R - d_\text{stop})}, \qquad
R = 0.30,\; d_\text{stop} = \texttt{WALL\_STOP\_DIST} = 0.18
$$

(0 when the root is negative).

### 8.5 Drive limits (`drive`)

1. No motion during the first `SETTLE_TIME` = 0.4 s.
2. Under `wall_contact`, forward speed ≤ 0; otherwise v ≤ v_safe(min_clear).
3. Turn limit against scrubbing and rollover:
   |ω| ≤ ω_max · max(0, 1 − |v| / 0.95).
4. Traction scaling (v, ω) · traction (traction is 1 with real sensing, as
   there is no slip estimate without GPS).
5. Differential drive: v_L = v − ω T/2, v_R = v + ω T/2 with track T = 0.37 m.
6. Wheel acceleration ramp: each wheel's speed moves toward its target by at
   most `WHEEL_ACCEL` · Δt = 1.2 · Δt m/s per step.

---

## 9. The state machine

```
              ball within 0.8 m and ±32°
   SEARCH ──► SEEK ───────────────────────► ALIGN ──centred──► COLLECT
     ▲         │ ▲                            │ timeout 6 s       │ liftTop edge
     │ nothing │ │ capture, bin not full      ▼                   ▼
     └─────────┘ └──────────────────────── (pickup failed) ◄── capture → SEEK
                                                                bin full → DONE
   any state: WALL_BACK (contact or out of bounds), RECOVER (stuck),
              ESCAPE (ball under the deck), DOWN (fallen)
```

| State | Behaviour | Exit |
|---|---|---|
| SEARCH | A* + DWA to `coverage_waypoint()` | a reachable ball in memory → SEEK; nothing uncovered → DONE |
| SEEK | `choose_target()`, A* + DWA (or the policy, if trusted) toward it | within 0.8 m and ±32° → ALIGN; no target → SEARCH; bin full (14) → DONE |
| ALIGN | pivot until \|l_y\| < 0.05 and \|bearing\| < 0.12; then v = clamp(0.6 (l_x − 0.62), 0, 0.14); reverse at 0.045 m/s if l_x < 0.40 | centred and l_x ≤ 0.85 → COLLECT; 6 s → pickup failed |
| COLLECT | belts and fan on; v = 0.16 (l_x > 0.50) else 0.06; ω = clamp(0.9 atan2(l_y, l_x), ±0.5) | capture → SEEK or DONE; back to ALIGN if l_x > 0.55 and \|l_y\| > 0.10; 6 s (and throat empty) → pickup failed |
| WALL_BACK | reverse (or drive forward if out of bounds but already facing in) | timer done and inside bounds for 0.3 s → SEEK |
| RECOVER | reverse 0.8 s, spin toward the clearer side 0.8 s, then drive out; longer each time it repeats within 0.5 m | back to the interrupted state |
| ESCAPE | reverse 1.4 s, then spin, until the ball under the deck is 0.30 m behind | → SEEK, ball banned 30 s |
| DONE | park | |

The intake (side belts at 0.80 m/s, fan at 24 rad/s) runs in SEARCH, SEEK,
ALIGN and COLLECT; the lift belts (0.50 m/s) run in every state except DONE and
DOWN, so a ball already on the lift always finishes the climb.

---

## 10. Recovery and stuck detection (`detect_stuck`, `idle_watchdog`)

* **Displacement test**: every `STUCK_WINDOW` = 1.5 s compare the pose with the
  one 1.5 s earlier. If it moved < 0.05 m, turned < 0.35 rad and was commanded
  to drive (|v| > 0.05), add 1.5 s to `stuck_time`. Above `STUCK_TIME` = 4 s:
  ban the target (doubling), clear the map, RECOVER.
* **Wall stall**: commanded v = 0 because of the safety cap, clearance under
  R + d_stop, not moving, for `STALL_TIME` = 3 s → WALL_BACK.
* **Idle watchdog**: nothing commanded for `IDLE_TIME` = 2 s → pivot at
  0.9 rad/s (above the yaw static-friction deadzone) toward the region centre
  for 1.2 s. No state transition for `NO_TRANSIT_TIME` = 8 s → RECOVER.
* **Bounds guard** (`check_walls`): estimated pose more than `BOUND_MARGIN` =
  0.05 m outside the bounds → WALL_BACK for `BOUND_BACK_TIME` = 2 s.
* **Contact guard**: min clearance ≤ R + 0.03 m, or every frontal bumper beam
  swallowed → WALL_BACK for `WALL_BACK_TIME` = 0.9 s.

---

## 11. The learned policy (`policy.py`)

**Status: not in use.** Run with `TENNIS_NO_POLICY=1`. The weights in
`data/policy_weights.json` were trained on the old robot (before the container
body, the camera and the lidar fix), and AGENTS.md says the policy must not
drive on its own judgement until it beats DWA. It only ever drives SEEK and
SEARCH navigation; ALIGN, COLLECT and all recoveries are hand-written.

### 11.1 Observation (`rl_observation`)

| Feature | Source | Bins |
|---|---|---|
| dist | range to the target ball (robot frame) | edges 0.35, 0.70, 1.20, 2.00, 3.50 m → 6 |
| bearing | atan2(l_y, l_x) | 8 sectors of 45° |
| clearance | `min_clear` | edges 0.45, 0.75, 1.20 m → 4 |
| bag | balls in the bin | 0, 1, 2, 3+ → 4 |
| slip | slip estimate | 3 (always 0 with real sensing) |
| mode | SEEK / DROP / IDLE | 3 (DROP is no longer produced) |
| stuck | stuck_time > 1.5 s | 2 |
| escape | in RECOVER / ESCAPE / WALL_BACK | 2 |

### 11.2 Actions

15 (v, ω) pairs: hold; forward 0.16, 0.32, 0.62 straight; the same speeds at
ω = ±0.80; pivot ±1.20; reverse −0.12 straight; reverse −0.22 at ω = ±0.80.
Every action passes through `rl_guard` (safe speed, wall latch, turn limit)
before it reaches the wheels.

### 11.3 Q-function

A factored linear model over one-hot bins, with action interactions only where
the choice depends on the state:

$$
\begin{aligned}
Q(s, a) =\;& w_0 + w^\text{dist}_{d} + w^\text{bear}_{b} + w^\text{clear}_{c}
          + w^\text{bag}_{g} + w^\text{slip}_{\sigma} + w^\text{mode}_{m}
          + w^\text{stuck}_{k} + w^\text{esc}_{e} + w^\text{act}_{a} \\
         &+ w^\text{dist,act}_{d,a} + w^\text{bear,act}_{b,a} + w^\text{clear,act}_{c,a}
\end{aligned}
$$

That is 1 + 6 + 8 + 4 + 4 + 3 + 3 + 15 + 2 + 2 + (6 + 8 + 4)·15 = 318 weights.

### 11.4 Rewards (`shaped_reward`)

$$
r = R_\text{step} + R_\text{event} + R_\text{shape}
$$

| Term | Value | Fires on (today) |
|---|---|---|
| R_step | −0.01 per decision | every transition |
| capture | +10 | liftTop rising edge on the target (`rl_note('capture')`) |
| deliver | +25 | never (no drop zone any more) |
| stopped in zone | +5 | never (no drop zone any more) |
| stuck | −6 | stuck, wall stall, idle pivot |
| escape fail | −4 | ESCAPE timed out with the ball still under the deck |
| out of bounds | −5 | bounds guard |
| R_shape | +0.6 per metre closed, −0.6 per metre lost | SEEK → SEEK transitions only; steps where \|Δdist\| > 0.10 m are ignored (target switches, teleports) |

Scale check that matters: a 2 m approach earns about +1.2 of shaping while
costing about −0.01 per step; earlier versions where step cost dominated
taught the policy to stand still (FINDINGS 5.39).

### 11.5 Learning (`LinearQPolicy.update`)

Semi-gradient TD(0) Q-learning, α = 0.05, γ = 0.97:

$$
\delta = r + \gamma \max_{a'} Q(s', a') - Q(s, a), \qquad
w_i \leftarrow w_i + \alpha\,\delta \quad \text{for each of the 13 active weights}
$$

(The gradient of a sum of one-hot weights is 1 for each active weight.)
Online, the first 3 steps of a run do not update.

### 11.6 Exploration (`choose`)

ε-greedy with ε starting at 0.30, multiplied by 0.998 every 20 steps (online)
down to 0.05. Plus directed exploration: in a state cell (dist, bear, clear,
mode) visited fewer than 12 times, choose the best-valued action among those
tried fewer than 2 times in that cell.

### 11.7 Trust gate (`rl_ready`)

The policy drives a mode only after that mode has received
`RL_MIN_UPDATES` = 400 updates. Until then DWA drives and its commands are
recorded as transitions, so the policy learns from the planner (bootstrap).

---

## 12. Experience buffer and offline training

### 12.1 Buffer (`record_experience`)

`controllers/tennis_collector/data/experience.jsonl`, append-only and shared
(never delete; see AGENTS.md). One row per transition:

```
{x, y, rng, v, w, bag, clr, mode, nx, ny, reward, done}
```

x, y: target ball in the robot frame; v, w: executed command (after the
drive limits); nx, ny: next state's ball position; reward: +1 capture
(terminal), −1 failed pickup (terminal), 0 otherwise.

Reductions: keep 1 routine step in `EXP_DECIMATE` = 4 (about 8 Hz); drop a
step whose ball position moved < 4 mm with the same action; always keep
events. The file is capped at 16 MB.

### 12.2 Offline trainer (`tools/train_policy.py`)

```
python tools/train_policy.py [buffer] [--epochs 20 --alpha 0.05 --gamma 0.97]
```

1. Rebuild (s, a, s') from each row: dist and bearing from (x, y), clearance
   from `clr`; slip = 0; stuck and escape = true only on failed terminals.
2. Action = the lattice entry nearest to the logged (v, w) in squared error.
3. Reward recomputed by `reward_from_row`: terminal +1 → R_capture + R_step,
   terminal −1 → R_escape_fail + R_step, otherwise R_step + distance shaping.
4. Shuffle and sweep for N epochs with the same TD update.
5. Print the greedy action in six test situations, so a run that learned
   nothing is obvious.

Online and offline rewards differ: offline has no stuck, out-of-bounds or wall
events (the buffer does not store them) and maps every failed pickup to
R_escape_fail.

---

## 13. Referee and metrics

The supervisor reads true ball positions every step (`update_ball_states`). A
ball counts as **in the bin** once its centre has been inside the bin volume
(x ∈ (−0.238, 0.038), |y| < 0.143, z ∈ (0.09, 0.30) in the true robot frame)
for 3 consecutive steps. That number (`del` in the status line,
`count_delivered()`) is the score. The robot's own count (`bag`) comes from
the beams and can differ (one extra beam count was seen on train_run).

Per run: status line every 2 s (`log`), events (`>>` lines), trace
`logs/trace_<world>.jsonl` (true pose x, y, h, estimated ex, ey, every ball),
map `logs/map_<world>.json`.

---

## 14. Known gaps

* **No pose correction.** Dead reckoning only; drift grows with time and
  wheel slip. A real build should add lidar scan matching or court-line
  landmarks.
* **The map is not reused.** It is saved but never loaded.
* **Three lidar fans** in simulation; a real build would use one 360° unit.
* **Colour thresholds** are tuned to Webots rendering, not to a real camera
  under sunlight and shadow.
* **The policy is stale** (trained on the old robot and on mirrored lidar
  clearance) and its DROP mode and delivery rewards are dead code.
* **Single runs.** The results in FINDINGS 6.11 are one seed per world; a
  multi-seed baseline is still to do.
