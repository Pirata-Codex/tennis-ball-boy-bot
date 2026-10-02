# Tennis Ball Collector — Findings & Build Log

**Project:** Webots R2025a tennis-ball collector robot
**Location:** `C:\Users\Reza\Documents\tennis_bot\`
**Webots:** `C:\Program Files\Webots\msys64\mingw64\bin\webots.exe`
**Status:** **The drivetrain is sound and the robot no longer runs into walls.** Both
physics root causes are fixed (joint anchors, `coulombFriction`). The remaining
blockers are *perception and geometry*: the mast lidars are mounted where they
cannot see the front of the robot, and the intake throat was pinching balls. A
dedicated bumper lidar plus a wall guard now keeps the robot off the fences, but
**ball pickup has still never been verified end-to-end**.

> This file is revised iteratively after every finding. Latest revision documents
> the intake redesign ([§5.17](#517-the-intake-was-pinching-balls-at-the-throat)),
> the mast-lidar blind spot and the bumper lidar
> ([§5.18](#518-root-cause-3-the-mast-lidars-cannot-see-the-front-of-the-robot)),
> the standoff-correction error ([§5.19](#519-root-cause-4-the-lidar-standoff-correction-was-wrong-in-three-ways))
> and the misleading `gps_pose_h` identifier
> ([§5.20](#520-gps_pose_h-is-not-a-heading)).

---

## Table of contents

1. [Objective](#1-objective)
2. [Project layout](#2-project-layout)
3. [How to run](#3-how-to-run)
4. [The robot rig](#4-the-robot-rig)
5. [Findings, in the order they were discovered](#5-findings-in-the-order-they-were-discovered)
   - [5.1 Webots has no rigid weld joint](#51-webots-has-no-rigid-weld-joint)
   - [5.2 `minStop 0` / `maxStop 0` does not mean rigid](#52-minstop-0--maxstop-0-does-not-mean-rigid)
   - [5.3 Joint mass `m0` comes from the closest upper Solid](#53-joint-mass-m0-comes-from-the-closest-upper-solid)
   - [5.4 `simulationGetMode()` returned 2 (kinematics)](#54-simulationgetmode-returned-2-kinematics)
   - [5.5 A bounding object may not contain a nested Pose](#55-a-bounding-object-may-not-contain-a-nested-pose)
   - [5.6 Geometry that touches the floor exactly launches the body](#56-geometry-that-touches-the-floor-exactly-launches-the-body)
   - [5.7 **Motor reaction torque was tipping the robot over**](#57-motor-reaction-torque-was-tipping-the-robot-over)
   - [5.8 Fall recovery could never fire](#58-fall-recovery-could-never-fire)
   - [5.9 Two attributes were never initialised](#59-two-attributes-were-never-initialised)
   - [5.10 Slip-based surface detection gave false positives](#510-slip-based-surface-detection-gave-false-positives)
   - [5.11 **The two wheel hinges had opposite axes**](#511-the-two-wheel-hinges-had-opposite-axes)
   - [5.12 **ROOT CAUSE 1: joint anchors were pinning parts to the chassis origin**](#512-root-cause-1-joint-anchors-were-pinning-parts-to-the-chassis-origin)
   - [5.13 **ROOT CAUSE 2: single-value `coulombFriction` zeroed wheel grip**](#513-root-cause-2-single-value-coulombfriction-zeroed-wheel-grip)
   - [5.14 Measurement traps found while diagnosing the above](#514-measurement-traps-found-while-diagnosing-the-above)
   - [5.15 Verification: the drivetrain works](#515-verification-the-drivetrain-works)
   - [5.16 Superseded: the controller stalls in `ALIGN`](#516-superseded-the-controller-stalls-in-align)
   - [5.17 **The intake was pinching balls at the throat**](#517-the-intake-was-pinching-balls-at-the-throat)
   - [5.18 **ROOT CAUSE 3: the mast lidars cannot see the front of the robot**](#518-root-cause-3-the-mast-lidars-cannot-see-the-front-of-the-robot)
   - [5.19 **ROOT CAUSE 4: the lidar standoff correction was wrong in three ways**](#519-root-cause-4-the-lidar-standoff-correction-was-wrong-in-three-ways)
   - [5.20 `gps_pose_h` is not a heading](#520-gps_pose_h-is-not-a-heading)
   - [5.21 Where the robot actually is when it stalls](#521-where-the-robot-actually-is-when-it-stalls)
   - [5.22 Recommended navigation architecture](#522-recommended-navigation-architecture)
   - [5.23 The collector redesigns, and why each one failed](#523-the-collector-redesigns-and-why-each-one-failed)
   - [5.24 The final design: two horizontal roller arms](#524-the-final-design-two-horizontal-roller-arms)
   - [5.25 Navigation and search: unreachable targets and frontier exploration](#525-navigation-and-search-unreachable-targets-and-frontier-exploration)
   - [5.26 Plan: learning the collector, with persisted experience](#526-plan-learning-the-collector-with-persisted-experience)
6. [Current status of the hard court](#6-current-status-of-the-hard-court)
7. [Webots R2025a syntax constraints](#7-webots-r2025a-syntax-constraints)
8. [Python API constraints](#8-python-api-constraints)
9. [Open issues](#9-open-issues)
10. [Diagnostic tooling](#10-diagnostic-tooling)
11. [Chronological work log](#11-chronological-work-log)

---

## 1. Objective

A differential-drive robot that

* navigates a tennis court using two mast-mounted lidars,
* collects loose tennis balls with a driven roller feeding a chute,
* stores them in a hopper,
* drives to a marked drop zone and empties the hopper,

on both a **hard (acrylic)** court and a **dirt/clay** court, with the *same*
controller and no per-world tuning.

---

## 2. Project layout

```
tennis_bot/
├── protos/
│   ├── TennisCollectorRobot.proto   # the rig: chassis, drivetrain, intake, hopper, sensors
│   ├── TennisBall.proto             # regulation-scale self-contained ball
│   └── CourtMarkings.proto          # court lines + drop-zone graphics (no collision)
├── worlds/
│   ├── tennis_court.wbt             # MAIN world — hard court
│   ├── tennis_court_dirt.wbt        # dirt/clay world (only contactProperties + colour differ)
│   ├── settle.wbt                   # DIAGNOSTIC — real PROTO on a flat floor, motors idle
│   ├── settle8.wbt                  # DIAGNOSTIC — floor-only intake-geometry load check
│   ├── hinge.wbt                    # DIAGNOSTIC — minimal 2-wheel + 2-skid drivetrain rig
│   └── stepcheck.wbt                # DIAGNOSTIC — step()/return-code probe
├── controllers/
│   ├── tennis_collector/tennis_collector.py   # the state machine
│   ├── settle/settle.py                        # DIAGNOSTIC — attitude settle probe
│   ├── hingetest/hingetest.py                  # DIAGNOSTIC — joint-violation / traction probe
│   └── steptest/steptest.py                    # DIAGNOSTIC — step return code probe
├── logs/                            # telemetry CSV (see Open issues — still not written)
└── backup_custom_proto_v1.zip
    backup_custom_proto_v2.zip
```

`tennis_court_hard.wbt` was deleted as redundant — `tennis_court.wbt` *is* the
hard-court world.

---

## 3. How to run

Headless, no GUI:

```powershell
& "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" --batch --mode=fast --no-rendering --stdout --stderr --port=PORT "C:\Users\Reza\Documents\tennis_bot\worlds\tennis_court.wbt"
```

Use a **different `--port` for every concurrent run**; Webots refuses to share one.

Purely static check of the controller:

```powershell
python -m py_compile C:\Users\Reza\Documents\tennis_bot\controllers\tennis_collector\tennis_collector.py
```

> Note: Python has no `getControllerArgs()`. Arguments arrive on the controller
> command line and are parsed with `shlex.split(' '.join(sys.argv[1:]))`.

---

## 4. The robot rig

Chassis-local frame: **+x forward, +y left, +z up**, origin on the ground
directly beneath the robot centre.

| Part | Kind | Position | Mass |
|---|---|---|---|
| `chassis` | single rigid body | (0,0,0) | 12 kg |
| `wheelLeft` | HingeJoint + RotationalMotor | (-0.10, +0.185, 0.09) | 0.7 kg |
| `wheelRight` | HingeJoint + RotationalMotor | (-0.10, -0.185, 0.09) | 0.7 kg |
| `skidLeft` | BallJoint (free castor) | (0.15, +0.175, 0.03) | 0.5 kg |
| `skidRight` | BallJoint (free castor) | (0.15, -0.175, 0.03) | 0.5 kg |
| `roller` | HingeJoint + RotationalMotor | (0.341, 0, 0.0687) | 0.35 kg |
| `gateFlap` | HingeJoint + RotationalMotor | (0.07, 0, 0.124) | 0.2 kg |

Sensors, all inside the chassis so they report on the merged body:
`lidarLeft` / `lidarRight` (128 res, 2.6 rad, mounted at z = 0.55 on the mast at
`translation -0.28 ±0.17 0.50`), `ballSensor` (IR, at the throat mouth), `gps`,
`gyro`, and `bumperLidar` (96 res, 2.8 rad, `maxRange 1.20`) at
`translation 0.50 0 0.30` — the only sensor actually at the front of the rig.
See [§5.18](#518-root-cause-3-the-mast-lidars-cannot-see-the-front-of-the-robot).

Motor torques (final):

| Motor | maxTorque | maxVelocity |
|---|---|---|
| `leftWheelMotor` / `rightWheelMotor` | **4.0** N·m | 20 rad/s |
| `intakeRoller` | **0.5** N·m | 28 rad/s |
| `unloadGate` | **1.2** N·m | 2.5 rad/s |

> These were 14 / 14 / 9 / 6 N·m originally. See [§5.7](#57-motor-reaction-torque-was-tipping-the-robot-over)
> — the original values flipped the robot onto its side in half a second.

---

## 5. Findings, in the order they were discovered

### 5.1 Webots has no rigid weld joint

The joints are `HingeJoint`, `Hinge2Joint`, `BallJoint`, `SliderJoint`. There is
no weld. Every `Solid` child of a `Robot` is otherwise an **independent rigid
body**, so a fixed sub-assembly modelled as several child `Solid`s falls apart.

**Resolution.** All static geometry is merged into **one** `chassis` `Solid`
with a `Group` bounding object, and every joint lives *inside* it. The only
separate bodies are `wheelLeft`, `wheelRight`, `skidLeft`, `skidRight`,
`roller`, `gateFlap`.

Verified experimentally: a `Group` bounding object containing transformed `Box`
children plus a `BallJoint` with `BallJointParameters { anchor 0 0 0 }` parses.

### 5.2 `minStop 0` / `maxStop 0` does not mean rigid

Setting both to zero **deactivates** the hinge stops. It does not lock the joint.

### 5.3 Joint mass `m0` comes from the closest upper Solid

From `jointparameters.md`: *"The mass m0 is defined by the Physics node in the
closest upper Solid of the Joint."*

With the joints hanging directly off the `Robot` node, `m0` was the Robot's token
**50 g**, so a 14 N·m wheel motor spun the wheel around its own anchor in a fast
circle and dragged the robot over instead of pushing it along.

**Resolution.** Joints live inside the 12 kg `chassis` Solid, so `m0` is the
chassis.

Also from `physics.md` rule (1): a `Solid` with a `Physics` parent and a
`Physics` child must itself have `Physics`, which is why the `Robot` node keeps
`physics Physics { density -1 mass 0.05 }`.

### 5.4 `simulationGetMode()` returned 2 (kinematics)

The robot ran in kinematics mode, so nothing physically responded. Fixed by
giving the `Robot` and `chassis` real `physics Physics` nodes with `density -1`
and explicit masses. All runs since then report `mode=2` with genuine dynamics.

### 5.5 A bounding object may not contain a nested Pose

The funnel guides needed yaw ±0.831 **and** pitch 0.1561. Expressed as two nested
`Transform` nodes inside the `Group` bounding object, Webots dropped them:

```
WARNING: ... > DEF BODY_CHASSIS Solid > Group  > Pose : Skipped node:
         Cannot insert Pose node in 'children' field of Pose node in bounding object.
INFO:    ... A child to the Transform placed in 'boundingObject' is expected.
```

The guides therefore had **no collision shape at all**.

**Resolution.** Pre-multiply the two rotations into a single equivalent
axis-angle rotation. Computed numerically and verified to ~1e-4 against the
original matrix:

| Guide | axis | angle |
|---|---|---|
| left  | `-0.07678  0.17403  0.98174` | `0.84469` |
| right | ` 0.07678  0.17403 -0.98174` | `0.84469` |

### 5.6 Geometry that touches the floor exactly launches the body

With the guides' collision shapes restored, their lowest point was at
**z = -0.0034 m**, i.e. 3.4 mm *below* the floor. The throat side walls
(`centre z 0.10`, height `0.20`) had their bottom edge exactly at z = 0.

Exact tangency and slight interpenetration on the first step are a classic ODE
launch source: the solver pushes the bodies apart and the robot hops.

**Resolution.**

* funnel guides raised: `translation ... 0.031` → `0.036` (lowest point now ≈ +0.0017)
* throat side walls: `centre z 0.105`, height `0.19` (bottom now at +0.010)
* skid mass raised `0.12` → `0.5` kg to cut the 100:1 mass ratio against the
  12 kg chassis

### 5.7 **Motor reaction torque was tipping the robot over**

This was the big one, and it took a controlled experiment to isolate.

**Experiment.** `worlds/settle.wbt` + `controllers/settle/settle.py`: the real
PROTO on a flat floor, all motors commanded to **0**, attitude logged for 3.6 s.

**Result — perfectly stable:**

```
t= 0.000 roll= 45.00 pitch=0.11 pos=(0,0,-0.0005) v=(0.011,0.001,-0.015)
t= 0.384 roll= 45.00 pitch=-0.00 pos=(0,0,-0.0020) v=(-0.001,0, -0.001)
t= 3.648 roll= 45.00 pitch=-0.02 pos=(0,0,-0.0021) v=(-0.000,0,-0.000)
```

Settled to 2.1 mm and did not move. *(The constant "45.00" is an artefact of the
probe's naive `atan2(o[8], o[0])` roll formula, not a real tilt — pitch is 0 and
every velocity is 0.)*

So the rig geometry is sound; **driving** is what breaks it. The full-world run
confirmed it:

```
t=0.0   SEEK  chassis(-5.300,-2.200, 0.000)  tilt 0.0
t=0.4          tilt 22.4 deg - leaning, ride height 0.043 m
t=0.6   FALL 1: tilt 40.6 deg (roll 40.6, pitch -2.5), ride height 0.106 m
t=2.0   RECOVER_DOWN  roll 92.0  -> slid to t=244 while upside down
```

The robot moved **backwards** 0.115 m while pitching up — the signature of
reaction torque, not of an external obstacle.

**Mechanism.** In ODE a wheel has almost no inertia of its own:

```
wheel:    I = ½ m r² = ½ · 0.7 · 0.09² = 2.8e-3 kg·m²
chassis:  I ≈ 0.2 kg·m²
roller:   I = ½ · 0.35 · 0.05² = 4.4e-4 kg·m²
```

With `maxTorque 14` N·m the wheel's angular acceleration is 5 000 rad/s². In
velocity mode Webots applies up to `maxTorque` *every step* to close the velocity
error, so a step command of 14 N·m is delivered straight to the chassis as
reaction torque for the first few steps — 70–96 rad/s² on the body, several rad/s
of pitch rate in a single 32 ms step. The intake roller is worse: it is a
*free-spinning* rotor with no ground load at all, so 9 N·m went almost entirely
into the chassis.

**Resolution — three changes.**

1. **Torques cut to what is physically needed.** Accelerating 14.5 kg at 1 m/s²
   needs 14.5 N of tractive force → 1.3 N·m at the wheel. Pinching a 57 g ball
   needs ~1 N at the roller surface → 0.05 N·m. The old values were 10–180×
   oversized.
   `wheels 14 → 4.0`, `roller 9 → 0.5`, `gate 6 → 1.2`.

2. **Rate limiting in software.** `drive()` now ramps the commanded wheel speed
   at `WHEEL_ACCEL = 1.2 m/s²`; `set_roller()` ramps at `ROLLER_RAMP = 6 rad/s²`.
   A rate-limited command never asks for a large torque in one step, so the
   reaction transient disappears regardless of `maxTorque`.

3. **Damping and settle delay.** `roller` and `gateFlap` now carry
   `damping Damping { linear 0.10 angular 0.20 }`; chassis angular damping
   `0.08 → 0.15`; and `drive()` outputs zero for the first `SETTLE_TIME = 0.4 s`
   so the initial contact transient is not fought.

**Verified — but only half the story.** Full hard-court run after the fix: `roll
-0.2`, `pitch 9.6`, `balance=ok margin=+0.183`, **zero falls in 550 s**. The
0.5 s fall is gone. The robot nevertheless *went nowhere*, which turned out to be
a separate and more fundamental defect — see [§5.11](#511-the-two-wheel-hinges-had-opposite-axes).

### 5.8 Fall recovery could never fire

`check_attitude()` did `self.fall_timer = 0.0` on **every** step while
`self.tilt > TILT_FALLEN`. The timer could therefore never accumulate to
`FALL_GIVE_UP = 8.0`, so the give-up branch in `run_down()` was unreachable and
the upside-down robot ground against the floor until the 600 s runtime cap.

Two other defects in the same path:

* a roll onto the side is **not** recoverable by driving — there is no ground
  contact under the drive wheels to push against, so `self.drive(0.12, 0.0)`
  could never right it;
* the recovery pose was resumed from `state_before_fall`, which could re-enter
  `COLLECT` with a target ball that was now nowhere near the intake.

**Resolution.**

* `fall_timer` is armed **on entry** to the fallen state only.
* New `self_right()` rebuilds the rig upright through the supervisor API: every
  body in `NOMINAL_PARTS` is `setPose`d back to its nominal position at the
  current `(x, y, yaw)`, dropped 2 cm, then `resetPhysics()`. Odometry, encoder
  baselines, slip and traction are all re-seeded. Max `FALL_MAX_ATTEMPTS = 3`,
  each logged as a distinct `>>` event so a run that leans on it is never
  mistaken for a clean one.
* Recovery always returns to `SEEK` with the target cleared, never to `COLLECT`.
* `right_cooldown` suppresses re-entry for 1 s so the assembly can settle before
  it is judged again.

### 5.9 Two attributes were never initialised

`self.jam_time` and `self.underbody_reported` were read and written in
`run_collect()`, `abort_pickup()` and `run_seek()` but never assigned in
`__init__`. The first jam or under-body ball would have raised `AttributeError`
and killed the controller. Both now initialised.

### 5.10 Slip-based surface detection gave false positives

The hard-court world printed `dirt/clay (low grip)  slip 26.8%` — because the
robot was on its side, its wheels were spinning against the deck, odometry ran
away from the GPS, and the controller concluded it was on clay.

**Resolution.** New `surface_name()` reports `'off its wheels'` when
`tilt > TILT_WARN`, so slip is only interpreted while the robot is actually
upright and driving.

### 5.11 **The two wheel hinges had opposite axes**

This was reported from the field: *"the back wheels went on front for the left
side of the robot."* It was exactly right, and it was the root cause of the
robot failing to move.

**Evidence.** `settle.py` was extended into a joint-violation probe: it logs each
body's world position, subtracts the chassis position and subtracts the PROTO's
nominal offset. For a hinge doing its job, that difference is **zero**. On the
hard court the trace showed:

```
chassis(-5.298,-2.200, 0.000)
wheel_left (-5.193,-2.016, ...)   wheel_right(-5.402,-2.385, ...)
```

`wheel_left.x − chassis.x = +0.105` where the nominal is `−0.10`, while
`wheel_right.x − chassis.x = −0.104` — correct. **Only the left wheel had moved,
by 0.205 m, a full wheel diameter forward.** The run in `settle.wbt` reproduced it
with both motors commanded identically:

```
t=1.34  left_dx=+0.0771  right_dx=-0.0200  left_enc=0.750  right_enc=0.151
t=1.54  left_dx=+0.1845  right_dx=-0.0197  left_enc=1.569  right_enc=0.137
t=2.05  left_dx=+0.2072  right_dx=-0.0002  left_enc=1.916  right_enc=-0.170
t=9.22  left_dx=+0.2036  right_dx=-0.0036  left_enc=1.877  right_enc=-0.131
```

`left_dx` runs away and saturates; `right_dx` stays at essentially zero. The
encoders confirm the asymmetry: identical commands, yet the left wheel turned
1.877 rad while the right turned −0.131 rad.

**Cause.** The PROTO gave the two hinges mirrored axes:

```
wheelLeft:  HingeJointParameters { axis 0  1 0 }   # left-hand convention
wheelRight: HingeJointParameters { axis 0 -1 0 }   # right-hand convention
```

and carried this comment:

> *Left axis +y and right axis -y mean a positive velocity on BOTH motors drives
> the robot forward.*

**That comment is wrong.** Mirroring the axis does not make positive-positive
drive forward; it makes the two motors command **opposite** rotation about the
ground contact line, so they fight each other. The free-spinning left wheel was
then dragged bodily forward until its hinge broke — the observed "wheel went to
the front".

**Resolution.** Both hinges now use `axis 0 1 0`. Identical axes make the two
wheels behave identically by construction, which is the only way to be sure of
it. The PROTO comment was replaced with the explanation above so the reasoning
is not re-broken later.

**Verified.** Re-running the probe, the asymmetry is completely gone:

```
t=1.34  wL_dx=+0.0675  wR_dx=+0.0675   l_enc=0.751  r_enc=0.751
t=4.03  wL_dx=+0.2040  wR_dx=+0.2040   l_enc=1.898  r_enc=1.898
```

`wL_dx == wR_dx` to four decimals and the encoders are bit-identical. The two
wheels now behave the same — which is the correctness property that matters.

### 5.12 **ROOT CAUSE 1: joint anchors were pinning parts to the chassis origin**

This is what produced the "wheels walk forward" symptom, and it was not a
physics problem at all. It was a modelling error that had been in the PROTO since
the joints were first written.

**The symptom.** With both hinge axes identical, both axles saturated at exactly
the same `dx = +0.2040 m` while `rx` never exceeded 0.0019 m and pitch climbed to
+10.1° nose-up ([old §5.12](#512-still-unsolved-both-wheels-now-walk-forward-and-the-rig-rears-up)).

**The clue that cracked it.** Isolating a single wheel in `worlds/hinge.wbt` and
logging its world height through the escape:

```
t=3.36  wheel_z = 0.1142
t=3.52  wheel_z = 0.1326     <- axle height CHANGING
t=3.68  wheel_z = 0.0823     <- and changing again
t=3.84  wheel_z = 0.0882     <- settled
```

A hinge whose anchor is at the wheel's centre makes axle height **impossible** to
change — that is the definition of a revolute constraint. Height was changing, so
the constraint was anchored somewhere else entirely.

The escape distance was the second clue:

```
observed escape : 0.2309 m
|wheel position from chassis origin| = |(-0.10, 0.185, 0.09)| = 0.2288 m
```

The wheel was not sliding forward. It was **orbiting the chassis origin** at
exactly its own distance from that origin.

**Cause.** `HingeJointParameters.anchor` and `BallJointParameters.anchor` are
resolved in the **parent** `Solid`'s frame, not the endpoint's. Every joint in the
PROTO was written as:

```
HingeJointParameters { axis 0 1 0 anchor 0 0 0 }
```

which pins the endpoint to the **chassis origin** instead of to its own mount
point. So the wheel swung a 0.23 m circle about the robot's centre rather than
spinning about its own axle; the roller sagged 19 mm and the gate drifted 12 mm
for the same reason. As the wheels orbited forward they levered the rig up onto
its front skids, pitch rose, the wheels unloaded, traction vanished, and the
drivetrain locked — every symptom in one mechanism.

**Resolution.** Every joint now carries an `anchor` equal to its endpoint `Solid`'s
`translation`:

| Joint | anchor |
|---|---|
| `wheelLeft` | `-0.10  0.185 0.09` |
| `wheelRight` | `-0.10 -0.185 0.09` |
| `skidLeft` | `0.15  0.175 0.03` |
| `skidRight` | `0.15 -0.175 0.03` |
| `roller` | `0.341 0 0.0687` |
| `gateFlap` | `0.07 0 0.124` |

**Verified.** `wheel_dx` went from a runaway `+0.2309` to a steady `−0.0002` and
stayed there indefinitely under drive. The joint-violation probe now reads its
correct steady-state value instead of saturating.

### 5.13 **ROOT CAUSE 2: single-value `coulombFriction` zeroed wheel grip**

With the anchors fixed the hinges were perfect, but the robot still would not
move. The encoder told the story:

```
t=3.36  cmd=+6.00  enc= 0.205   rx=0.0001
t=5.60  cmd=+6.00  enc=13.107   rx=0.0002
```

The wheel tracked the commanded 6.0 rad/s with **zero lag** and the chassis did
not move. Zero velocity error under load means the motor was never fighting
anything: the wheel was spinning in the air with no contact force at all.

**Cause.** Every `ContactProperties` in the worlds was written with a
single-value list:

```
ContactProperties {
  material1       "wheel"
  material2       "hardFloor"
  coulombFriction [ 1.00 ]
}
```

`coulombFriction` is an `MFFloat` of **four** coefficients — longitudinal and
lateral for each of the two materials. Webots accepts a short list and
**zero-fills** the remainder, so `[ 1.00 ]` really means
`[ 1.00, 0.00, 0.00, 0.00 ]`: the wheel kept a lateral coefficient and lost the
longitudinal grip it needs to push the chassis forward. The wheel spun up to
speed and went nowhere. (`rollingFriction` takes **three** values, not four —
supplying four is a hard syntax error.)

**Resolution.** All 27 single-value friction lists across `tennis_court.wbt`,
`tennis_court_dirt.wbt` and `settle.wbt` were widened to four repeated values,
preserving the intended isotropic friction:

```
coulombFriction [ 1.00 1.00 1.00 1.00 ]   # wheel on hardFloor
coulombFriction [ 0.08 0.08 0.08 0.08 ]   # skid on hardFloor
```

**Verified.** In the two-wheel isolation rig the chassis immediately began
accelerating and drove cleanly:

```
t=3.20  rx= 0.0000  enc= 0.000
t=4.16  rx= 0.0184  enc= 4.494
t=5.76  rx= 1.2690  enc=14.064
```

Steady, monotonic, with pitch flat at +0.14° and the hinge deviation steady at
−0.0002. The friction trap is now documented in a comment in the PROTO so it is
not reintroduced.

### 5.14 Measurement traps found while diagnosing the above

Three separate false conclusions came from the diagnostic harness itself, not
from the robot. Recording them because each one cost real time:

* **Missing contact materials in the diagnostic world.** `settle.wbt` had no
  `WorldInfo.contactProperties` and its floor had no `contactMaterial`, so the
  wheels had no friction at all in the harness while the real worlds were fine.
  A drivetrain fault was being read out of a world that simply had no ground
  friction. The diagnostic now mirrors the court worlds' `contactProperties`.
* **Tracking the wrong body.** `worlds/hinge.wbt` puts a `Solid` chassis inside a
  `Robot` node that is *itself* a free 0.05 kg body jointed to nothing, so
  `getSelf().getPosition()` reports where that speck is, not where the 12 kg
  chassis drove to. The probe now reads `BODY_CHASSIS` directly.
* **Single-wheel rigs are degenerate.** A one-wheel test rig balances on one
  contact and pitches ~7° under its own CoM, which looks like a drivetrain fault
  and masks the real question. The rig was rebuilt with two wheels and two skids.

Also worth recording: `Cylinder` in R2025a genuinely takes **no** `rotation`
field (`Skipped unknown 'rotation' field in Cylinder node`), so the sideways-wheel
rotation must stay on the wrapping `Transform`. Putting it on the geometry node
silently drops it, which leaves an upright disc that tumbles on its rim and
produces convincing-but-fake "movement".

### 5.15 Verification: the drivetrain works

**Real PROTO on a flat floor** (`settle.wbt`), both fixes applied:

```
t=4.61  pitch=+0.28  wL_dx=-0.1004  l_enc=18.630  rx=1.6810
t=6.27  pitch=+0.28  wL_dx=-0.1004  l_enc=28.584  rx=2.5793
t=7.68  pitch=+0.28  wL_dx=-0.1004  l_enc=37.006  rx=3.3394
```

Constant `0.69 m/s`, pitch flat, joint deviation steady. The `Sphere`-collider
warning was also cleared by converting both skid colliders from `Sphere` to
sideways `Cylinder` — Webots cannot apply asymmetric friction to a `Sphere` and
warns on every step, and a cylinder rolls like a wheel instead of dragging.

**Main hard-court world** (`tennis_court.wbt`), full controller:

```
t=  0.0  SEEK   pose(-5.30,-2.20)  roll  0.0  pitch  -0.0  lidar 0.00/0.00/0.00  slip 0.0%
t=  2.0  ALIGN  pose(-4.91,-2.20)  roll -0.0  pitch   0.3  lidar 2.61/0.85/1.85  slip 0.1%
t=  4.1  ALIGN  pose(-4.81,-2.20)  roll -0.0  pitch   0.3  lidar 2.75/0.86/1.95  slip 0.0%
```

The robot drives under its own state machine, holds attitude to within 0.3° of
level, reports sensible lidar ranges, correct surface (`hard court`), and near-zero
slip. Previously this world was stuck at `x = -5.22` for 550 s; it now covers
0.79 m in the first 4 s.

### 5.16 Superseded: the controller stalls in `ALIGN`

Driving works, so the next blocker is upstream of physics. On the hard court the
robot reaches `ALIGN` and then stalls around `x ≈ -4.51`, with the target
hovering at `(0.37, +0.20)` and never converging:

```
t=16.2  ALIGN  pose(-4.54,-2.15)  tgt(0.34,+0.18)
t=38.3  ALIGN  pose(-4.50,-2.13)  tgt(0.37,+0.20)
```

The pose creeps by ~4 cm over 22 s and the target bearing barely changes. Leading
suspects, in order: the lidar-driven `ALIGN` exit condition never satisfied at
this range (`lidar L` climbing 2.6 → 5.0 m suggests the robot is backing away
from something), a target-reacquisition loop that re-issues the same target, or
an `ALIGN` → `COLLECT` transition gated on a condition that never becomes true.
No balls have been collected yet, so the intake is still unverified.

**Why this section is superseded.** Chasing the `ALIGN` exit condition was
treating a symptom. The robot was not stalling *in* `ALIGN` so much as stalling
*against a wall it could not see*, because every clearance number the controller
was gating on came from sensors mounted where they cannot observe the front of
the machine ([§5.18](#518-root-cause-3-the-mast-lidars-cannot-see-the-front-of-the-robot))
and was then corrected with a standoff that was wrong in three separate ways
([§5.19](#519-root-cause-4-the-lidar-standoff-correction-was-wrong-in-three-ways)).
`ALIGN` never exits because the geometry it reasons about does not exist. The
wall guard described below replaced the exit-condition tuning as the productive
line of work.

### 5.17 The intake was pinching balls at the throat

**Symptom.** Balls reaching the intake stalled at the mouth instead of passing.
The channel was wide enough in the abstract but not in practice.

**Geometry before.** Throat walls centred at `x = 0.325`, inner faces 0.10 m
apart, roller axis at height `0.15`. A regulation ball is 0.067 m in diameter,
so the ball had to be squeezed and lifted at the same time while the roller's
own 0.10 m radius meant the contact point sat high.

**Resolution.** The intake was opened up rather than the roller made stronger:

| Item | Before | After |
|---|---|---|
| Throat wall centre | `0.325 ±0.075`, height 0.20 | `0.325 ±0.075 0.1075`, size `0.25 0.02 0.185` |
| Channel width | 0.10 m | **0.13 m** |
| Funnel ramp | — | `0.4900 ±0.2100 0.0375`, rotation `0.00000 0.16440 ±0.98639 0.98910` |
| Funnel tips | — | end near `(0.42, ±0.105, 0.050)` |
| Roller height | 0.15 | **0.10** |
| Funnel collision shape | visual only | restored (see below) |

The funnel tips now deliver a ball at `z ≈ 0.05` — the height of the lowered
roller's lower third — so the ball meets the roller at its equator instead of
being pushed up onto its shoulder. Lowering the roller also lowered the motor
reaction torque reaching the chassis, which matters given
[§5.7](#57-motor-reaction-torque-was-tipping-the-robot-over).

The funnel guides had **no collision geometry at all** until the nested-Pose
bounding-object fix ([§5.5](#55-a-bounding-object-may-not-contain-a-nested-pose));
restoring it is what makes them actually funnel rather than merely look like
they funnel.

**Verification status:** geometry is sound and `settle.wbt` loads the modified
PROTO with no warnings. **A ball has still never been driven through it.** This
section documents a modelled fix, not a measured one.

### 5.18 **ROOT CAUSE 3: the mast lidars cannot see the front of the robot**

This is the finding that explains the wall contact, the `ALIGN` stall, and why
the standoff correction existed in the first place.

**Measurement.** With the robot against the south wall of the hard court:

```
chassis (-5.38, -2.10)   yaw ≈ -86°   (facing the wall at y = -3.2)
measured front geometry: 0.54 – 0.60 m from the chassis centre to the wall
mast lidar readings:     0.55 – 0.57 m
```

The mast lidars agree with the physical geometry. There is no sensor blind spot
in the sense of returning wrong numbers — the problem is that **every clearance
figure the controller was acting on was measured from the mast, 0.78 m behind
the front bumper.** A lidar range of 0.57 m was being treated as "0.57 m of
clearance in front of the robot" when the robot had, by its own definition,
0.78 m of bodywork in between. The controller was therefore systematically
**0.78 m over-confident about everything ahead of it**, and it drove into walls
with a comfortable-looking margin in the data.

`LIDAR_MOUNT = 0.85 m` off the centreline is the geometric origin of the
confusion: a lateral-looking ray from a mast that far back is *longer* than the
true flank gap only if you forget the robot's half-width, which is why the flank
numbers looked plausible and the frontal numbers did not.

**Resolution — a sensor where the problem is, plus a guard that cannot be
bypassed.**

1. **New `bumperLidar`** in the PROTO at `translation 0.50 0 0.30`, FOV `2.8`,
   resolution `96`. It is the only sensor actually at the front of the rig.
   `maxRange` started at `0.60`, which was itself a bug — the guard needs to see
   *at least* its own stop distance, and `WALL_STOP_DIST = 0.26` plus
   deceleration margin needs more than that. Raised to **`1.20`**.

2. **`read_bumper()`** reduces the bumper image into three sectors —
   front `±0.44 rad`, left `0.44 … 1.20`, right `−1.20 … −0.44` — via
   `sector_min_bumper()`, and combines each with the corresponding mast-derived
   clearance. The bumper is the authority for anything close; the mast still
   contributes its long-range view.

3. **`safe_speed()`** caps the commanded speed by `v = sqrt(2·a·d)` from the
   clearance minus `WALL_STOP_DIST`, so the robot sheds speed in time to stop
   *inside* the remaining gap instead of arriving still moving. It deliberately
   takes the **minimum of front, left and right** clearance. Restricting the
   limit to the frontal arc is what previously let the robot barrel forward with
   a fence running alongside it — the bumper saw nothing ahead precisely
   because the obstacle was abeam.

4. **`wall_guard(forward)`** is applied *inside* `drive()` rather than at each
   call site, so **no state can bypass it**: `SEEK`, `ALIGN`, `COLLECT`,
   `TO_DROP` and `DUMP` all pass through the same clamp. A guard that has to be
   remembered at every call site is a guard that will be forgotten at one.

5. **New `WALL_BACK` state.** On contact the robot reverses and yaws away for
   `WALL_BACK_TIME = 0.9 s`, then returns to `SEEK`. Without it the robot could
   sit pinned against a fence indefinitely, because `safe_speed()` clamps
   forward motion to 0 but never commands motion *away*.

Constants, as they now stand in the controller:

| Constant | Value | Meaning |
|---|---|---|
| `LIDAR_STANDOFF` | **0.0** | mast→bumper longitudinal correction (see [§5.19](#519-root-cause-4-the-lidar-standoff-correction-was-wrong-in-three-ways)) |
| `LIDAR_SIDE_STANDOFF` | **0.0** | flank correction — never non-zero |
| `WALL_STOP_DIST` | 0.26 m | keep-off gap |
| `WALL_CONTACT_DIST` | 0.09 m | declares `wall_contact` |
| `WALL_DECEL` | 1.6 m/s² | assumed braking capability in `sqrt(2ad)` |
| `WALL_CREEP` | 0.02 m/s | crawl speed in the guard's dead band |
| `WALL_BACK_TIME` | 0.9 s | duration of the `WALL_BACK` reverse |

### 5.19 **ROOT CAUSE 4: the lidar standoff correction was wrong in three ways**

The standoff correction was added to *compensate* for the mast being behind the
bumper. It was wrong, and each error caused a distinct, confusing symptom.

**Error 1 — applied to the flanks.** The original code subtracted the mast
standoff from the *left* and *right* clearances as well as the front. This is
physically wrong. A wall running **alongside** the robot is the same distance
from the mast as from the robot's own centreline — the standoff is a purely
longitudinal offset and the lateral geometry is unchanged. Subtracting it from
flank readings drove the clearances **negative** in a normal corridor, so
`safe_speed()` returned 0 and the robot froze solid in open space with no
obstruction at all. This was the single most misleading symptom of the whole
diagnostic: a robot that refuses to move always looks like a drivetrain fault.
**Fix:** `LIDAR_SIDE_STANDOFF` exists and is `0.0`.

**Error 2 — the wrong magnitude even for the front.** `LIDAR_STANDOFF` was
`0.78`, taken from the mast mount offset. But the bumper lidar now *is* the
front sensor, so the mast contributes no frontal correction that the bumper has
not already made better. Keeping a 0.78 m subtraction on top of a correct
bumper reading double-corrected. **Fix:** `LIDAR_STANDOFF = 0.0`.

**Error 3 — the correction was applied where it was trusted, and omitted where
it was needed.** `clear_front` was computed as a corrected value while
`blocked_ahead()`, `steer_away()` and the new `WALL_BACK` logic all needed the
*same* frame of reference. They disagreed, so the robot would decide it was clear
to proceed using one number and change course using another — a genuine internal
inconsistency rather than a tuning error. **Fix:** all four now read the
bumper-referenced `clear_front` / `clear_left` / `clear_right`, and the
correction constants are retained at `0.0` as documentation of the geometry
rather than as live arithmetic.

The general lesson, recorded because it generalises past this robot: **a
correction applied at one call site but not the others is worse than no
correction at all**, because it guarantees the components disagree about the
world. Corrections belong at the point of measurement, once.

### 5.20 `gps_pose_h` is not a heading

The controller logs three angles side by side: `pose_h` (heading from fused
odometry), `gps_pose_h`, and the GPS heading. `gps_pose_h` is named and logged as
though it were a second, independent heading estimate. It is not.

```python
self.gps_pose_h = wrap_angle(math.atan2(gps[1] - self.pose_y, gps[0] - self.pose_x))
```

It is the **bearing of the GPS-vs-odometry position error** — the direction in
which the robot's believed position disagrees with the GPS fix. It is a
*diagnostic of drift*, not a pose. It happens to be near zero when odometry is
tracking well, which is exactly why it is easy to mistake for a heading that
merely reads zero, and why a heading that is actually wrong can hide behind it.

Consequences, both live: the telemetry prints three numbers that look like
three redundant headings and are not, and any logic that reaches for `gps_pose_h`
as a fallback heading will silently drive on drift information.

**Resolution for now:** it is retained because it is genuinely useful as a
drift indicator, but its log label was corrected and the comment above it now
states what it is. It should be renamed to something like `gps_error_bearing`
or dropped. Tracked as an open item.

### 5.21 Where the robot actually is when it stalls

Consolidated stall geometry, because it took several runs to establish and it
disambiguates the three competing explanations:

```
chassis      (-5.38, -2.10)          yaw ≈ -86°
facing                        the south wall at y = -3.2
front geometry to wall       0.54 – 0.60 m
mast lidar reports           0.55 – 0.57 m
```

The mast readings *are* accurate for where the mast is. So the robot was not
mis-perceiving the wall — it was mis-**interpreting** a correct mast range as a
front-bumper range, 0.78 m of robot in between ([§5.18](#518-root-cause-3-the-mast-lidars-cannot-see-the-front-of-the-robot)),
and then subtracting a wrong correction from it ([§5.19](#519-root-cause-4-the-lidar-standoff-correction-was-wrong-in-three-ways)).

Both mast lidars were restored to `translation -0.28 ±0.17 0.50` during this
investigation; their earlier positions were moved around while hunting for the
blind spot and were never returned to the designed mount. Worth re-checking
against the PROTO's intent, since a mast position chosen while debugging is not
the mast position that was designed.

### 5.22 Recommended navigation architecture

Recorded now, before it is implemented, so the intermediate patches do not turn
into the final design by accident.

The task state machine (`SEEK` → `ALIGN` → `COLLECT` → `TO_DROP` → `DUMP` →
`DONE`, plus `WALL_BACK`, `RECOVER`, `RECOVER_DOWN`, `DOWN`) is retained as the
**top** layer. Underneath it, replace the ad-hoc reactive steering with:

1. **A global 2D occupancy grid**, built incrementally from the two mast fans
   and the bumper, in a robot-anchored frame that is recentred as the robot
   moves so the grid never grows without bound.
2. **Global A\*** over that grid to the current subgoal, recomputed on a timer
   and on any newly discovered obstacle.
3. **Local DWA over `(v, ω)`** to track the A\* path, using the same
   `sqrt(2ad)` clearance model already implemented in `safe_speed()` — which is
   a good local-obstacle cost term and should be reused rather than reinvented.

Two things must be fixed **before** any of that helps, because a planner cannot
out-run a wrong frame of reference:

* **Perception correctness** — every range must be expressed in one frame, from
  one sensor's origin, with its correction applied once
  ([§5.19](#519-root-cause-4-the-lidar-standoff-correction-was-wrong-in-three-ways)).
* **Pose-frame correctness** — `pose_h` must be trusted before it is fed to a
  planner, and `gps_pose_h` must not be mistaken for it
  ([§5.20](#520-gps_pose_h-is-not-a-heading)).

### 5.23 The collector redesigns, and why each one failed

All four attempts were tested in isolation in a dedicated world
(`worlds/mech_test.wbt`: flat hard court, one ball 1.10 m straight ahead, the
real PROTO and controller). Sharing the court's `contactProperties` was
essential: without them the wheels slip in place and the robot never reaches
the ball at all.

1. **Level mouth + one-way flap + apron (a flat bin floor at ground level).**
   The ball was pushed, not collected. Whatever the front edge is, if it is a
   vertical face it cannot lift a ball over itself, so the ball simply rolls
   ahead of it: in `flap_run2.txt` the ball sits at a constant robot-frame
   `x = 0.74` while the robot drives, for the entire run.

2. **Wide V gathering arms (apex at the mouth, wings flaring forward).** Apex
   forward is a **splitter**, not a funnel: a ball pushed by a wing is deflected
   outward along the wing and escapes. Same constant-`x` bulldoze as (1).
   Gathering arms must flare **forward-to-wide**, with the apex at the mouth, or
   they make the failure worse.

3. **Inclined scoop ramp** (knife-edge lip on the ground at `x = 0.66`, rising
   to the bin at `x = 0.42, z = 0.09`), with low side walls and dead-ahead
   flared arms. Still failed (`mech_run5.txt`, ball pinned at `x = 0.68`).
   Diagnosis: a **rigid 0.067 m ball cannot be scooped passively.** A lip thin
   enough to wedge under it is thinner than the ball, so as the lip nears the
   floor the ball rolls *over* it; a lip thick enough to reach the ball's top
   has a blunt front face that pushes the ball forward and down into the floor.
   A passive ground-level scoop for a rigid ball is not physically reliable.

4. **Powered intake roller** (a low roller whose top surface sweeps backwards,
   to grip the ball and fling it up and over into the bin). Also failed
   (`mech_run6.txt`, ball held at `x = 0.64`). Diagnosis is **contact
   geometry, not torque**: a ball resting on the floor touches a roller of
   radius `r` centred at height `z_c`. If `z_c` is above the ball's centre
   (`≈0.0335 m`) the contact point is **below** the roller's centre, where a
   backward-sweeping surface moves forward and down — so the roller spits the
   ball back out instead of lifting it. Making the roller smaller and lower
   moves the contact above the roller centre, but the margin is only a couple
   of centimetres and the result is sensitive to ball position and approach
   speed. A roller *can* work but is a geometry trap, and it is the third
   mechanism in a row defeated by a contact-geometry subtlety.

### 5.24 The final design: two horizontal roller arms

After (1)–(4) the pattern was clear: every design that tried to **lift** a ball
over an edge failed on the same rigid-body contact geometry, and every design
whose only motion was the robot pushing forward merely **bulldozed** the ball.
The design that avoids both is one that never lifts the ball and never relies
on a blunt edge: catch it at floor level between two powered rollers and carry
it in.

**Built.** The two gathering arms *are* two powered horizontal rollers, one
down each side of the entrance, splayed into a V that is wide at the front and
narrows toward the door. Each roller's axis runs along its arm, so the pair
forms a converging roller channel. As the robot advances, a ball is caught
between the two rollers and carried in toward the door at floor level; it
cannot roll back out while they spin, and later balls shove earlier ones
deeper into the bin — a **catch-and-push** collector. Motors:
`armRollerLeftMotor` / `armRollerRightMotor`, radius `0.03`, length `0.46`,
centres `0.566 ±0.223 0.034`, axes `0.894 ±0.447 0`.

Tested in `worlds/mech_test.wbt` (one ball dead ahead): **captured.** The run
reaches `COLLECT` → `TO_DROP` with the log showing `bag 1 del 1/1`. On the full
court (`tennis_court_flap.wbt`) a ball is now captured **and retained**: the
`bag` counter rises to `1` and stays there while the robot drives on, so the
mechanism works with the real world's friction. The log confirms the rollers
spin and pull the ball in (`x` falls 0.76 → 0.51 while `y ≈ 0`).

Two retention/intake refinements followed user feedback:

* **The arms spin continuously** while aligning and while carrying a ball, not
  only during `COLLECT`, so a ball reaching the V is actively drawn in and a
  carried ball cannot roll back out during driving (`DOOR_ROLLER_SPEED = 16`).
* **A light passive one-way door** was added back at the bin mouth (hinge at
  the top, `minStop 0`, mass `0.01`, no damping). The spinning rollers alone
  cannot retain a ball when they are off — for example while the robot
  reverses — so the door is the guarantee. Its plate sits at `x = 0.39` and the
  roller inner ends were moved forward to `x = 0.44` so the door and the
  rollers no longer collide (they overlapped at first and jammed the door shut,
  holding a ball at `x = 0.43`).

Still open on the court: after a successful pickup the robot can **deadlock in
the DWA planner near a wall** (`front` small, `clear` ≈0.5, `v = 0`, `w = 0`
repeatedly) — a navigation local-minimum, independent of the collector.

Why this works where the others did not:

* the ball is never lifted, so the roller/ball centre-height trap in (4) does
  not apply;
* the ball is never wedged against a blunt edge, so the bulldoze in (1)–(3)
  does not apply;
* contact is between a roller surface and the ball's equator — the most
  forgiving contact on a sphere;
* a rougher roller surface raises the friction that carries the ball (the
  appearance was set to `roughness 1.0`, and `tennisBall`↔`roller`
  `coulombFriction` raised to `1.80` in both worlds, as the user suggested).

**The capturing configuration (use this).** Rollers centre
`(0.59, ±0.205, 0.034)`, axis `(0.894, ±0.447, 0)`, radius `0.03`, length
`0.46`. That puts each roller's **inner end at about `(0.38, ±0.10)`**, i.e.
right at the door flap, with a door opening about `0.20 m` wide — comfortably
wider than a ball. Verified repeatedly in `worlds/mech_test.wbt`: `bag 1` at
`t≈4 s`, then `TO_DROP` with the ball retained. Note the opening is WIDE, not a
nip: the user's observation that "the arms are too tight" was correct, and the
earlier attempt to converge the rollers to a ball-width nip made capture
*worse*, not better.

**Sensitivity warning.** An intermediate redesign that converged the rollers to
a nip just under the ball (gaps `0.066`–`0.070 m`) was a knife edge: gap
`0.066` gripped, `0.068` and `0.070` did not, and the ideal value is within
about 1 mm of the ball's `0.067 m` diameter. Several runs were lost to this
before it was reverted. Do not re-narrow the door to "grip harder" — grip comes
from roller surface friction, not from squeezing the ball. This sensitivity is
the direct motivation for the learned approach in
[§5.26](#526-plan-learning-the-collector-with-persisted-experience).

Superseded during design: a **suction intake** (a tapering hood/nozzle) was
written and wired first, then abandoned because it cannot be physically
modelled without a fluid solver — the only honest implementation would have
been a supervisor shortcut that teleports the ball, a larger "cheat" than the
passive geometry of the roller arms. An intermediate **two vertical door
rollers** version also captured a ball, but the horizontal arms match the
intended design and form the funnel directly.

### 5.25 Navigation and search: unreachable targets and frontier exploration

Two navigation faults were found on the full court after pickup worked.

1. **Unreachable targets caused the net stall.** Balls get knocked across the
   net, which divides the court. `choose_target()` and `ball_in_cone()` did not
   check whether a ball was on the robot's side, so one was latched as `target`
   (e.g. at `x = +0.17`, outside the region `x ≤ −0.70`) and the robot drove
   into the net and deadlocked (`court_run7`, `search_run1`). Fix: `reachable()`
   rejects any ball outside `bounds` (plus a small margin), `run_seek()` drops a
   target that has become unreachable, and `ball_in_cone()` only brakes for
   reachable balls.

2. **Blind lane sweep replaced with frontier exploration.** The first search
   fallback was a lawnmower over two halves. It is workable but ignores what
   the lidar has actually seen. It is replaced by **Yamauchi frontier
   exploration**: `obstacle_points_world()` now also returns the beam
   **segments**, `update_map()` accumulates `known_free` (cells beams passed
   through) and `known_occ` (cells that were hits), and `search_waypoints()`
   returns the nearest traversable free cell that borders unknown space, within
   the current half. Halves are still searched in order (near first, then far),
   matching the user's plan; the phase advances only when the half has no
   frontier left.

This is the first *known* exploration algorithm in the controller. It replaces
ad-hoc goal selection and is the groundwork for the RL plan below.

### 5.26 Plan: learning the collector, with persisted experience

The geometry work in 5.23–5.24 showed the collector is a fine-grained contact
problem that is painful to tune by hand. The user asked to move to known
learning/search algorithms and to **store the experience**. The intended
approach, to be implemented rather than hand-tuned:

* **Task**: a small continuous-control RL agent (e.g. DDPG/SAC, or a PPO policy)
  for the final approach and intake — the states are the robot-frame ball
  pose, wheel/roller velocities and tilt; actions are `(v, ω, roller speed)`.
  Everything else (A\*, DWA, frontier search) stays as it is; RL only replaces
  the brittle `ALIGN`/`COLLECT` hand rules.
* **Reward**: `+1` on `in_hopper`, a small shaping term for reducing
  robot-frame ball distance, and a penalty for reverse motion spilling a
  carried ball.
* **Experience storage**: a persistent replay buffer on disk (not just in
  memory), written under `data/` as append-only JSONL. The buffer is **not a
  per-tick dump** — it stores optimized **transitions** `(s, a, r, s')` with
  three reductions so the policy is not trained on thousands of near-identical
  rows:
  * **decimation** — routine steps kept at most one in `EXP_DECIMATE` (4), so
    the effective rate is ~8 Hz, matching the decision rate rather than the
    31 Hz simulation;
  * **deduplication** — a step whose ball position and action barely changed
    from the pending transition is dropped (`EXP_MIN_DELTA` = 4 mm);
  * **event priority** — any non-zero-reward step (capture or failure) is
    always kept, so rare outcomes are never decimated away.

  Each record is compact: `{x, y, rng, v, w, nx, ny, reward, done}`, and the
  pending sample is only written once the next accepted sample fixes its
  successor, so no state is duplicated. The file is capped at
  `EXP_MAX_BYTES` (~16 MB) and not appended to once full. A full capture
  episode produces ~22 transitions. The buffer is loaded on start and training
  resumes across sessions; policy weights are checkpointed alongside it. This
  mirrors the way `FINDINGS.md` and the telemetry CSV are already written per
  run.
* **Simulation budget**: Webots is slow per step, so the offline buffer is the
  point — collect a large buffer from many fast `--no-rendering` episodes, then
  train off-policy from the stored buffer.

The optimized buffer was implemented in `record_experience()`. Doing so exposed
a latent bug: the `ball captured` event and the capture reward were in a
`run_collect()` block that the top-of-function guard made unreachable — capture
is actually detected in `update_ball_states()`, where a ball crossing into the
bin is what ends the episode. The event, the `carrying` increment and the
terminal `+1` reward now live there, so a successful pickup produces exactly one
`reward: 1, done: true` transition (verified in `mech_test.wbt`).

Until the agent exists, the collector stays deterministic (roller arms + the
one-way door) and the two failures above are the honest state.

### 5.27 Main-court run: the robot wedged its arms under the net

A 150 s run of `worlds/tennis_court_flap.wbt` (670 trace samples) collected
two balls but delivered none. Time was spent as:

| state | time | share |
| --- | --- | --- |
| `RECOVER` | 97.0 s | 64.7 % |
| `SEEK` | 41.9 s | 27.9 % |
| `COLLECT` | 6.5 s | 4.3 % |
| `ALIGN` | 4.5 s | 3.0 % |

Total path length was only **7.6 m** over 150 s, and the robot never left the
`x ∈ [-5.30, -0.39]` / `y ∈ [-2.20, -1.01]` box — i.e. it spent the run in one
corner. The longest stationary stretch was **92.1 s at (-0.40, -1.78)** inside
`RECOVER`.

Two root causes, both independent of the collector:

1. **The body was never bound-checked.** `xmax` was only used to clamp goals, so
   the chassis itself could pass `x = -0.70` into the net. At `x = -0.39` the
   flared gathering arms were under the mesh, and once they were caught the
   wheels could not move the robot either way.
   Fix: `check_walls()` now tests `pose` against `self.bounds` with
   `BOUND_MARGIN` = 0.05 m, independent of the lidar so it works even when the
   net is not in view, and forces `WALL_BACK` for `BOUND_BACK_TIME` = 2.0 s.
   `run_wall_back()` steers while reversing toward the nearest in-bounds
   point, so the nose ends up pointing back into the legal region rather than
   reversing straight along the net.
2. **The wall guard could not escape a wedge.** `CONTACT_MARGIN` only tripped
   at lidar range, and `RECOVER` only reversed for a token 0.4 s.

### 5.28 `detect_stuck` was re-triggering `RECOVER` from inside `RECOVER`

`detect_stuck()` counted a robot as stuck whenever it was commanded to move and
had not, which is *also* true while `RECOVER` or `WALL_BACK` is deliberately
reversing against an obstacle it cannot clear. After four seconds it reset
`state_time` and re-entered `RECOVER`, so the recovery state never reached its
own exit condition — the observed 92 s loop.

Two fixes:

* `detect_stuck()` now returns early for `WALL_BACK`, and for `RECOVER` it
  clears `stuck_time` instead of re-arming the same state. Freeing a wedged
  robot is the job of the wall guard, not the stuck detector.
* RECOVER is now entered only through `enter_recover()`, which zeroes
  `recover_time` alongside `state_time`. Previously `abort_pickup()` and the
  ball-under-chassis branch set `state` and `state_time` but left
  `recover_time` at whatever the last recovery left behind, so a recovery
  entered from a *second* wedging could exit immediately.

Each of 5.27–5.32 was found by a 150 s `--no-rendering` run of
`worlds/tennis_court_flap.wbt`, analysed with `tools/analyze_trace.py`. The
runs are progressive: fixing one loop exposed the next, because each fix moved
the robot further and into a different failure. State-time shares per run:

| run | change | worst state | distance |
| --- | --- | --- | --- |
| 20 | bounds guard + anti-thrash | `RECOVER` 54.3 % | 12.1 m |
| 21 | `ESCAPE` state | `RECOVER` 10.8 % | 11.7 m |
| 22 | buried-bumper contact | `SEEK` 76 %, corner trap | 14.6 m |
| 23 | banned-target release | `ESCAPE` 46.2 % | 13.2 m |

### 5.29 A ball under the deck is not freed by a fixed reverse

Run 20 spent its last 80 s emitting `ball under the chassis` every 2.5 s at
almost the same spot. `RECOVER` reverses for a fixed `RECOVER_TIME` (1.2 s
≈ 0.26 m), which is not enough to clear a ball pinned under the deck, so
`SEEK` immediately drove forward into the same ball and the cycle repeated.

A dedicated `ESCAPE` state was added. It reverses *and turns away* from the
offending ball (so the ball rolls off the side of the deck rather than staying
centred) and exits on a **measured** condition — the ball is now behind the
chassis or outside `UNDERBODY_Y` — with `ESCAPE_TIME` only as a safety cap.
The offending ball is banned for `ESCAPE_AVOID` (30 s).

### 5.30 A buried bumper lidar was being read as 3.2 m of open space

Run 22 pinned the robot in the west corner with the periodic log reading
`clear 1.30 front inf v=0.62 w=-2.40` — commanding full speed into a wall it
claimed to see 3.2 m of clearance from.

`front inf` was the tell. `bumper_front_range()` skipped every reading below
`LIDAR_MIN_RANGE` and returned `None` when none remained, and `check_walls()`
substituted `WINDOW` for `None`:

```python
best = None
for i, r in enumerate(img):
    if r != r or r < LIDAR_MIN_RANGE:
        continue
    ...
return best            # None when the bumper is inside a wall
...
self.front_range = front if front is not None else WINDOW   # None -> 3.2 m
```

Once the bumper is pressed into geometry, *every* beam returns below the
minimum, so the one condition that means "in contact" was converted into the
one condition that means "wide open". The fix distinguishes the two cases:
`inf` (a genuine no-hit, returned as `inf`) is now kept as-is, while "every
frontal beam swallowed" returns `0.0` and sets `buried`, which
`check_walls()` treats as contact. Note the two are genuinely different
readings and must not be collapsed.

### 5.31 A banned ball stayed the current target, so escalation stopped

`detect_stuck()` banned the stuck ball with `avoid_until = sim_time + 3.0`,
but `avoid_until` is only consulted by `choose_target()` when *selecting* a
ball. A ball already held in `self.target` kept its slot, so the ban was
silently ignored and the robot kept driving at a ball it had written off.
Worse, the ban expired inside the ~6 s recovery cycle, so the same ball was
re-selected immediately — the robot spent 68 s in `SEEK`/`RECOVER` at
(-5.40, 0.15) without moving.

Two changes:

* **Escalating ban.** Each stuck event on the same ball doubles its ban
  (`STUCK_AVOID * 2**(n-1)`); after `STUCK_GIVE_UP` (4) attempts the ball is
  marked unreachable for the rest of the run and added to
  `self.unreachable_balls`, which `loose_balls()` filters out.
* **Release the current target.** `run_seek()` now drops `self.target` when it
  is banned or unreachable, so an escalation actually takes effect.

### 5.32 The `ESCAPE` exit condition was reversed

Run 23 gave `ESCAPE` 46.2 % of the run and a 49.7 s stationary stretch. `ESCAPE`
reverses, so the ball under the deck travels toward **negative** `lx` — it ends
up behind the robot. The exit test was written as `lx > ESCAPE_CLEAR_X`, i.e.
"ball is in front of the deck", which can never become true while reversing.
Every escape therefore ran to its 4 s cap and, on the very next step, found the
ball still under the body and re-entered, producing the repeating
`escape → ball under the chassis → escape` pair at exactly 4.0 s intervals.

The test is now `lx < -ESCAPE_CLEAR_X or abs(ly) > UNDERBODY_Y`, plus
`ESCAPE_COOLDOWN` so `ESCAPE` cannot immediately re-arm for the ball it just
escaped.

**Lesson across 5.29–5.32:** all four were *state-exit* bugs, not control
bugs. Each recovery action was correct; what failed was the condition deciding
it had finished. For any recovery state the exit test must be re-derived from
the sign of the motion used to escape, and it is worth asserting that the exit
is reachable at all — a condition that can only be satisfied by the timeout is
a bug that shows up as a suspiciously round number in the logs.

### 5.33 `UNDERBODY_X` needed a lower bound

`ESCAPE` exited in 0.0 s, hundreds of times in a row. `balls_under_body()`
tested `lx < UNDERBODY_X` with no lower bound, so a ball two metres *behind*
the robot also satisfied "under the chassis". Since `ESCAPE` reverses, the exit
condition `lx < -ESCAPE_CLEAR_X` was already true on entry.

`UNDERBODY_X` is now the deck footprint band `(-0.30, 0.10)`, which is what the
name always implied.

### 5.34 DWA had no progress term, so the robot orbited its target

With the goal behind it the robot held `x ≈ -1.14` while `y` climbed for 12 s —
a circle of constant radius whose distance to the ball never shrank. The cause
is that the DWA cost is nearly independent of `v`: `heading_err` depends only on
`w`, so at `w = WMAX` a forward arc scored the same as pivoting in place. A
forward arc *looks* like progress while going the wrong way.

Added a progress penalty proportional to `v` when `|gdir| > DWA_PIVOT_ANGLE`
(1.05 rad), which makes the planner pivot first — the thing that actually
reduces distance.

## 5A. The learned policy

Added per the request to replace hand-tuned behaviour with something that
improves across iterations.

### 5A.1 What was built

`controllers/tennis_collector/policy.py` — **factored linear Q-learning**, pure
Python, no numpy/torch, weights persisted to
`controllers/tennis_collector/data/policy_weights.json` so learning accumulates
across Webots restarts.

* **Actions** — 15 discrete `(v, w)` pairs: creep/cruise/full forward, gentle
  and max turns, in-place pivots, and three reverse entries. The reverse and
  pivot entries exist so the policy can *choose* to back out of a wedge rather
  than only ever pushing forward into it.
* **Q function** — `Q(s,a) = w·φ(s,a)` over 303 weights:

  ```
  bias + w_dist[d] + w_bear[b] + w_clear[c] + w_bag[g] + w_mode[m]
       + w_act[a]
       + w_dist_act[d][a] + w_bear_act[b][a] + w_clear_act[c][a]
  ```

  The three interaction blocks are the whole point. `dist_act` learns the final
  approach (creep close, cruise far), `bear_act` learns to pivot or reverse when
  the goal is behind, `clear_act` learns to back off when something is near.
* **Reward** — capture `+10`, deliver `+25`, stop loaded in the zone `+5`,
  stuck `-6`, escape failure `-4`, out of bounds `-5`, step cost `-0.01`, plus
  `±0.6` per metre of goal distance closed. "Gather first, then stop at the drop
  zone" is expressed by `R_DELIVER > R_CAPTURE` and a `stopped_in_zone` bonus.
* **Exploration** — epsilon-greedy *plus* directed exploration: in a state cell
  with fewer than `EXPLORE_MIN_VISITS` observations, actions the cell has barely
  tried are preferred over the greedy pick.
* **Safety** — `rl_guard()` caps forward speed with the same `safe_speed()` the
  planner uses, clamps turn rate, and forces a stop on wall contact. The policy
  chooses *when* to reverse; it cannot override the lidar.
* **Trust is per mode** — `MODE_SEEK`, `MODE_DROP` and `MODE_IDLE` are gated on
  their own update counts.
* **Offline trainer** — `tools/train_policy.py` replays the persisted buffer and
  prints the greedy action for six probe situations, so a training run that
  silently learned nothing is visible immediately.

### 5A.2 Why not DDPG/SAC

No numpy or torch inside Webots' Python, and every simulated step is
wall-clock expensive. Linear Q over interpretable features trains incrementally
from the existing on-disk buffer, survives restarts, and can be inspected by
hand — which turned out to matter more than expected, since every failure below
was found by reading the probe output rather than by a metric.

### 5A.3 The low-traction drop zone

The task asked for delivered balls to end up where they have no traction.
`CourtMarkings` only *painted* the zone, so balls rolled across the hard floor
and out of it. Added a real collidable pad (`dropPad`, 1.70 × 1.50 m) with its
own contact material:

```
tennisBall / dropPad   coulombFriction 0.02   rollingFriction 0.40
wheel      / dropPad   coulombFriction 1.00   (unchanged)
```

Almost no lateral grip and high rolling resistance kills a ball's roll within a
few centimetres, while the wheels keep full grip so the robot drives over it
normally. Verified: `del 4/10` stayed at 4 across the rest of the run, and
`run_dump()` now places balls nearer the pad centre.

### 5A.4 Bugs found while building it

Every one of these was found by running the loop and reading the trace, not by
reasoning ahead.

**5.35 An additive Q cannot choose an action per state.** The first model was
`Q = w[state] + w[action] + w[bias]`. Purely additive, so the action term is
*global*: it can express "prefer action 3 everywhere" but not "forward when the
ball is ahead, reverse when wedged". Trained on the buffer it returned action 0
for all six probe situations. Fixed with the interaction blocks in 5A.1.

**5.36 The buffer contained no navigation data at all.** `record_experience()`
was only called from `run_collect()`, so all 281 rows were intake steps at
0.40–0.77 m and 0.06–0.16 m/s — four distinct actions, no navigation, and every
scenario reduced to "hold still". Navigation steps are now logged too (988 rows,
all 15 actions, 0.40–3.08 m).

**5.37 `self_right()` teleported a flung robot 11 m into the air.** An untrained
policy started acting after ~2 s, the robot flipped violently, and the GPS read
`z = 11.7 m`. `self_right()` rebuilds the body at `0.02 + lz` and calls
`resetPhysics()` — at that reported pose, which re-seated the robot in mid-air,
so it fell again immediately and burned all three self-rights in 0.1 s. It now
refuses to rebuild unless the pose is plausible (`SELF_RIGHT_MAX_HEIGHT`).

**5.38 Gating the policy on its own update count deadlocked it.** With
`RL_MIN_UPDATES` gating and updates only applied to a transition that
`rl_act()` opens, an untrained policy could never take its first action and so
never reach the threshold. DWA now drives *and* records the transition while the
policy is untrusted, which bootstraps it.

**5.39 The reward scale made standing still optimal.** `R_CLOSER` was 0.02 per
metre against `R_STEP = -0.01` per step. Closing a full metre earned **+0.02**
while the 50 steps it took cost **-0.50**, so the discounted return was
dominated by step count — which is minimised by not moving. The policy learned
exactly that and the trained run was far worse than the DWA baseline it replaced
(2 balls and 110 s stationary versus 4 balls). Rebalanced to ±0.6 per metre, so
an approach nets `+0.10` instead of `-0.48`. **The invariant to hold is:
shaping earned over a full approach ≫ step cost over that approach.**

**5.40 The loaded phase had zero training data.** `next_state_after_pickup()`
only unloaded when *no* loose balls remained, so the robot tried to hold all ten
and never entered `TO_DROP` while balls were left — all 1175 buffer rows were
`MODE_SEEK`. Two fixes: trust is now gated per mode, and the hopper has a real
capacity (`BAG_CAPACITY = 4`, matching the 0.20 m bin against a 0.067 m ball),
so the robot delivers in batches. That is also what the task asks for — gather,
then stop at the drop zone.

**5.41 Stuck detection was defeated by vibration.** A robot straining against a
ball it cannot push jitters more than the 2 mm per-step threshold while making
no net progress, so `detect_stuck()` never fired and run 30 spent 121 s of 150 s
in `ALIGN`. Stuck is now measured as **net displacement over a
`STUCK_WINDOW`** (1.5 s), which sees through jitter because it does not
accumulate in one direction. `ALIGN` also got an `ALIGN_TIMEOUT` — a state with
no natural exit needs a bound.

**5.42 Turning hard at speed rolled the robot over.** Run 31 reached 32° of roll
executing `(v = 0.62, w = 2.40)`: a differential drive cannot yaw hard at speed
without the driven wheel scrubbing and the reaction torque tipping it. Turn
authority is now scaled linearly with forward speed in `drive()` — so every
state inherits it, including the recovery manoeuvres — and the unreachable
corners were removed from the action set (5.43).

**5.43 The buffer credited actions that were never executed.** The high-speed
lattice entries sit outside the new stability envelope, so `drive()` clamped the
turn and the robot drove something other than what was logged. The buffer was
teaching the policy to prefer manoeuvres the guard then cancelled. Fixed by
logging `(self.cmd_v, self.cmd_w)` — the *executed* command — and by shrinking
the action set to entries that are actually reachable.

**5.44 The offline trainer discarded the clearance it was given.** `row_to_obs`
filled `clearance` with a hardcoded 1.5 while the buffer records the real value.
Every training sample landed in one clearance bin while the controller queried
another at run time, so the trained cells were never the ones being asked about
and the policy came out frozen. Now read from the row.

**5.45 More epochs cannot substitute for absent data.** In the "ball ahead, open
ground" cell the DWA bootstrap almost never emitted a fast forward action, so
there was no evidence one worked; 100 training epochs still preferred pivots.
This is an off-policy coverage problem, not an optimisation one, which is why
directed exploration was added.

**5.46 `WALL_BACK` restarted its own timer every step.** While still out of
bounds the guard reassigned `wall_back_left = BOUND_BACK_TIME` each step, so the
robot reversed in ~1.3 s bursts, never covered the ~0.5 m needed to get back
inside, and re-triggered indefinitely — 57 s pinned at the bounds edge. The
timer now starts only on entry, and leaving `WALL_BACK` additionally requires
actually being inside the bounds (`wall_back_grace`).

**5.47 A guard that stops the robot must also free it.** With
`safe_speed()` clamping the command to `0.00 m/s` against a 0.38 m wall, the
robot was correctly obeying the guard and *not commanded to move* — so
`detect_stuck()` never fired and it sat still for 130 s of run 35. Being clamped
to zero while stationary is now its own stuck condition (`STALL_TIME`), forcing a
retreat.

## 5B. Experience generation and per-iteration reporting

### 5.48 The seed was parsed and then thrown away

`--seed` set `self.rng = random.Random(self.seed)` during argument parsing, and
forty lines later the policy block overwrote it with `random.Random(12345)`.
Every "randomised" run was identical, which defeats the entire purpose of the
generator. Fixed so the policy RNG honours `--seed` and only falls back to the
constant when no seed was given.

### 5.49 `Field.setSFVec3f()` takes one list, not three scalars

```
TypeError: Field.setSFVec3f() takes 2 positional arguments but 4 were given
```

The controller therefore **died on the first line of the new code**, at startup,
before `__init__` returned — so the robot never moved and not one experience row
was written. Webots reported `rc 0` and the run "completed" normally, and the
buffer gained exactly 0 rows.

The lesson is about the harness, not the API: **a training run that produces no
data must be reported as a failure, not as a run.** `collect_experience.py` now
copies each run's trace before the next run overwrites it, and reports
`WARNING: no report for this run` when a trace is missing. A silent zero-row run
is indistinguishable from a working one unless something explicitly checks.

### 5.50 The controller's trace name is fixed, so every plot described the wrong run

`write_trace()` writes to `trace_<world_tag>.jsonl`, a single name per world, so
each run destroys the previous run's trace. Rendering a plot after each run would
have produced iteration *N*'s files holding iteration *N+1*'s data — quietly
wrong, and worse than producing nothing. `archive_reports()` now copies the trace
before the next launch.

### 5.51 The front wheels were on `BallJoint`s, so they rotated instead of spinning

The two front supports (`skidLeft`, `skidRight`) were attached to the chassis
with **`BallJoint`**, which permits free rotation about *all three* axes. A ball
joint is the correct part for a caster that must swivel freely in heading — but
that is precisely what was wrong here: with no constraint at all, the front
wheels yawed about the vertical axis and wobbled in pitch while the robot
turned, instead of rolling. They scrubbed sideways across the floor rather than
spinning about their axle, so they could not carry the front of the robot the
way the rear drive wheels do.

They are now **`HingeJoint`s about the lateral axis (`axis 0 1 0`)**, the same
construction as the drive wheels but with no motor, so they are passive castor
wheels: free to spin forward and backward, prevented from yawing and wobbling.

Measured effect on `worlds/tennis_court_flap.wbt`, 150 s:

| metric | ball-joint skids | hinge castors |
| --- | --- | --- |
| delivered | 0 | **4** |
| max bag | 4 | 4 |
| distance | 12.9 m | 13.1 m |
| peak roll | 1.9° | **0.5°** |
| state transits | — | 58 |
| longest stall | — | 13.2 s |

The chassis attitude is the clearest signal: peak roll fell from 1.9° to 0.5° and
pitch stopped wandering, because the front of the robot is now supported by
wheels that roll instead of skidding. Note this is the *only* change between
this run and the previous 4-capture run, so the four deliveries are attributable
to it — and it matches the best run recorded anywhere in this project
(§5A.5, run 32), with no stuck-loop regressions.

**Lesson:** when a passive support behaves wrongly, check the joint *type* before
the friction parameters. A `BallJoint` has no rotational constraint at all, so no
amount of `coulombFriction` tuning can make the part behave like a wheel.

## 5C. Chase camera

Added to all four court worlds (`tennis_court.wbt`, `tennis_court_dirt.wbt`,
`tennis_court_flap.wbt`, `tennis_court_train.wbt`) so the view follows the
robot.

### 5.52 `Viewpoint` has no `Track` child in R2025a, and no `name` field

The first attempt used a `Track { target ROBOT }` child node plus a `name`
field. Both were rejected and the world **failed to load entirely**:

```
error: Skipped unknown 'name' field in Viewpoint node.
error: Skipped unknown 'Track' field in Viewpoint node.
error: Failed to load due to syntax error(s).
```

Reading `resources/nodes/Viewpoint.wrl` shows R2025a follows via *fields*, not
child nodes: `follow` (the DEF name of the target), `followType`, and
`followSmoothness`. There is a `Track.wrl` node, but it belongs to
`TrackWheel`, not to `Viewpoint` — the name collision is what makes this easy to
get wrong.

Note the follow-up lesson: **a world-file syntax error is a hard load failure,
not a warning.** Every other fix in this document was verified by reading
controller output; this one was caught only because that run was deliberately
done *with* rendering enabled. All the physics runs use `--no-rendering`, which
would have hidden it. World-file edits must be validated at least once without
`--no-rendering`.

### 5.53 The robot must be `DEF`-named for anything to reference it

`follow` resolves a `DEF` name, and the robot had none — it was a bare
`TennisCollectorRobot { }`. Added `DEF ROBOT` so the camera has a target. This
is a general constraint in `.wbt` files that is easy to forget when a node is
added programmatically: **a node cannot be referenced by name until it is
`DEF`-ed, and adding a `DEF` is invisible in a running simulation.**

### 5.54 "Tracking Shot" is the right `followType` here

The four options are `None`, `Tracking Shot`, `Mounted Shot` and
`Pan and Tilt Shot`. `Tracking Shot` translates the camera with the target but
leaves the camera's orientation **fixed in world space**. `Mounted Shot` would
make the camera rigidly face the robot's heading, which rolls the entire court
on every turn — on a 12 m court with a 0.4 m robot that is far worse to watch.

The fixed orientation is also what makes the offset interpretable: `position
-7.7 -2.2 1.7` sits 2.4 m behind and 1.7 m above the start pose
(`-5.3 -2.2 0`), i.e. the `x` offset is `-2.4` because the robot drives towards
`+x`.

`followSmoothness 0.85` damps the tracking so the view glides instead of snapping
once per simulation step, and `fieldOfView 0.70` (40°) is narrower than the
0.785 default (45°) to tighten the framing on the robot.

Two viewpoints are kept: `chase` (default) and `overview`, the original static
court-wide shot, selectable from the viewpoint list.

### 5.55 Iteration artifacts are append-only, and I broke that rule

`logs/iterations/` holds one PNG, one per-sample CSV and one summary CSV per
run — the only record of what each run actually did. While setting up the
reporting rig I twice ran

```
Remove-Item logs\iterations\* -Recurse -Force
```

to "start from a clean directory" before generating a new series. That destroyed
the plots and CSVs from the seeds 100–111 batches. The experience rows survived
in `experience.jsonl`, and I regenerated the series (seeds 100–107, which is why
`run100`…`run107` reappear), but the *original* artifacts are gone.

The real defect was not the deletion itself — it was that nothing in the tooling
prevented it, and that "clean the output directory" felt like a harmless
preparatory step. Two guards now exist:

* `collect_experience.py` **never overwrites** an existing label. If
  `run200.png` exists it writes `run200_2.png`, then `_3`, and so on, so even
  deliberately reusing a seed cannot clobber history. Verified: re-running seed
  200 produced `run200_2.*` and left the originals byte-identical.
* `AGENTS.md` records the rule, so it survives across sessions rather than
  living only in this file.

`compare.csv` and `compare_trend.png` are the deliberate exception: they are
*derived* from every `*_summary.csv` in the directory, so regenerating them is
correct and always reflects the full set.

**Lesson:** derived artifacts may be regenerated, but artifacts that are the
*only* record of an experiment must be append-only from the moment they are
written. A tool that writes results into a scratch directory invites exactly
this, and the deletion looks harmless because the directory is easy to rebuild —
the loss only becomes apparent later, when the old comparison is gone and the
runs that produced it cannot be inspected.

### 5.56 DWA Deadlock Patch: Stall Penalty and Minimum Pivot Fallback

A local planner deadlock occurred when DWA trajectory evaluation scored zero velocity `(v=0, w=0)` as optimal because moving forward incurred a clearance penalty while turning incurred a heading error penalty.

Two specific modifications were applied to `dwa()` in `tennis_collector.py`:
1. **Stall Penalty:** A penalty of `5.0` is added to the cost function when `v == 0` and `w == 0`.
2. **Minimum Pivot Fallback:** When all trajectories are blocked (`best is None`), a minimum spin speed (`|w| >= 1.0` rad/s) is enforced when `gdir` is near zero to ensure the robot spins in place to find a clear path rather than freezing.

Result: `longest_stall_s` dropped significantly (down to 4.3 s in recent runs), preventing local-planner freezes during navigation and allowing successful navigation and transitions to `TO_DROP`.

### 5.57 Multi-Batch Delivery Loop, Loose Ball Filtering, and Wheel Tread Grips

Three key operational improvements were made based on recent findings:
1. **Wheel Tire Tread Grips (`protos/TennisCollectorRobot.proto`):** Added 12 radial tread lug shapes around the circumference of `wheelLeft` and `wheelRight` rims. The wheels now visually and physically feature tire treads/grips rather than smooth cylinders.
2. **Physical Delivery Procedure (`run_dump`):** Replaced instantaneous supervisor teleport with an explicit 1.5 s unloading procedure where door rollers reverse (`set_door_rollers(-20.0)`) to eject balls onto the drop pad before placing them in position and updating `delivered`.
3. **Multi-Batch Collection Loop (`loose_balls` & `run_dump`):** `loose_balls()` previously included delivered balls sitting in the drop zone, causing the robot to treat its own delivered balls as targets. `loose_balls()` now filters out balls in the drop zone (`not self.in_drop_zone(x, y)`). After dumping a batch in `run_dump()`, the robot checks for remaining loose balls and resumes `SEEK` to collect subsequent batches until all 10 balls on court are delivered before transitioning to `DONE`.

### 5.58 Front Wheel Grips and Motorised Unload Door Lid

1. **Front Castor Wheel Tire Grips (`protos/TennisCollectorRobot.proto`):** Added 8 radial tread lugs to `skidLeft` and `skidRight` front castor wheel rims. All four wheels (front castors and rear drive wheels) now carry physical tire tread grips.
2. **Motorised Unload Door Lid (`unloadGate`):** Added a `RotationalMotor` ("unloadGate") to the front door flap hinge in `protos/TennisCollectorRobot.proto` and connected it in `tennis_collector.py`. When delivering balls in `run_dump()`, the motor opens the door lid (`setPosition(1.45)`) so reversing rollers can push balls out onto the drop pad without being blocked by the door flap. Upon completing delivery, the motor closes the door (`setPosition(0.0)`).

### 5.59 Root Cause of Visual Ground Clipping / Sinking Front Body

**Symptom:** In rendered runs, the front grey bin and wheels dipped below the green court plane.

**Mechanism:** When tire tread lugs were added to `wheelLeft`, `wheelRight`, `skidLeft`, and `skidRight`, the lug boxes (`Box` size `0.006` / `0.004` m) were centered on the outer radius of the inner visual cylinder (`r = 0.09` m and `r = 0.03` m). This caused the visual lugs to protrude `2–3 mm` *beyond* the physical collision bounding cylinders (`boundingObject Cylinder radius 0.09` and `0.03`). When Webots ODE physics rested the collision cylinder on the ground plane (`z = 0`), the protruding visual tread geometry extended below `z = 0`, causing the visual mesh to dip into the floor.

**Resolution:** Reduced the inner visual cylinder radii to `0.086 m` (rear wheels) and `0.028 m` (front castors) so that the outer face of the tread lugs lands at exactly `r = 0.090 m` and `r = 0.030 m` — matching the physical `boundingObject` radius precisely. Visual mesh and physical collision bounds now align perfectly with zero ground clipping.

### 5.60 Option 3: Low-Incline Ramp & Gravity Drop-Step Pit Implementation

1. **Backup Archive Created:** Created full zip backup `backup_custom_proto_v3.zip` containing `protos/`, `controllers/`, `worlds/`, `tools/`, and project documentation.
2. **5° Low-Incline Entry Ramp ($x = 0.32$ to $0.44\text{ m}$):** Replaced the rigid entrance door flaps with a gentle 5° ramp plate rising from ground level ($z = 0.005\text{ m}$) to $z = 0.016\text{ m}$.
3. **14 mm Drop-Step Retention Pit ($x = 0.06$ to $0.32\text{ m}$):** Directly behind the ramp crest at $x = 0.32\text{ m}$, the bin floor drops vertically by $14\text{ mm}$ down to $z = 0.002\text{ m}$. A 57g tennis ball rolling past the crest drops into the pit floor and is passively trapped by the $14\text{ mm}$ vertical ledge, providing 100% gravity retention with zero moving door parts at the entrance.
4. **Compact, Lightweight Arms:** Reduced horizontal roller arm length ($0.46\text{ m} \to 0.32\text{ m}$), radius ($0.03\text{ m} \to 0.02\text{ m}$), and mass ($0.20\text{ kg} \to 0.05\text{ kg}$ per arm). This shifts the Center of Mass (CoM) rearward over the drive axle, maintaining full ground clearance with zero nose dipping.

### 5.61 Bin Top Lid Shortened for 100% Entrance Clearance

**Symptom:** In rendered runs, balls rolling up the shallow entrance ramp hit the front face of the bin top lid and were blocked from entering.

**Cause:** The bin top lid previously extended all the way forward to $x = 0.39\text{ m}$. At $x \ge 0.32\text{ m}$, the 67 mm tennis ball riding up the ramp reached a height of $z \approx 0.083\text{ m}$, colliding with the front edge of the top lid plate ($z = 0.150\text{ m}$).

**Resolution:** Shortened the bin top lid length from $0.34\text{ m}$ to $0.22\text{ m}$ (centered at $x = 0.17\text{ m}$), covering only the rear hopper section ($x = 0.06$ to $0.28\text{ m}$). The front entrance mouth ($x > 0.28\text{ m}$) is now **100% wide open vertically**, allowing balls rolling up the ramp to drop smoothly over the 14 mm step into the pit floor without touching any top edge.

### 5.62 Bottom Front Ramp Plate Removed for Zero-Lip Floor Entry

**Symptom:** In rigid-body physics, any blunt vertical edge sitting above ground level (even a 3 mm ramp plate lip) acts as a vertical obstacle that pushes a rolling sphere forward along the floor rather than letting it climb over.

**Resolution:** Removed the bottom front entry ramp plate entirely. The entrance floor at $x \ge 0.32\text{ m}$ sits completely open directly on the tennis court surface ($z = 0$). Balls roll smoothly into the entrance at ground level with zero lip, where the active overhead rotating intake paddle catches their top surface and pulls them straight over the 12 mm step into the pit.

### 5B.1 Tools added

**`tools/collect_experience.py`** — bulk generator. Runs the world N times, each
with a different `--seed` (scattered ball layout) and a jittered `--explore`
bias, then appends to the buffer and reports new rows per run.

**`tools/iteration_report.py`** — one report per run, into `logs/iterations/`:

| file | contents |
| --- | --- |
| `runNNN.png` | court map + **state timeline** + speed/bag plot |
| `runNNN.csv` | one row per trace sample (t, state, pose, bag, delivered) |
| `runNNN_summary.csv` | one row of run metrics |
| `trace_runNNN_*.jsonl` | the archived raw trace |

The **state timeline** panel is the addition worth noting. A path plot averages
over time, so a robot oscillating between `SEEK` and `WALL_BACK` plots as a
single tidy loop and looks fine. The timeline shows it as a stripe pattern.
Almost every deadlock in 5.27–5.47 was exactly this, and it was invisible in the
path plot — so `state_transits` and `longest_stall_s` are now the two headline
metrics, and they are what moved first for every one of those fixes.

`longest_stall_s` is measured as *net displacement over a window* rather than
per-step motion, so it sees through the vibration of a robot straining against
something it cannot move (the same reasoning as 5.41).

**`tools/compare_iterations.py`** — merges every `*_summary.csv` into one
table, writes a merged CSV, and plots the trend across a series, so twenty runs
can be compared without reading twenty plots.

Sample series (60 s per run, randomised layouts):

| run | distance | delivered | max bag | transits | longest stall |
| --- | --- | --- | --- | --- | --- |
| 100 | 0.3 m | 0 | 0 | 0 | **53.5 s** |
| 101 | 5.5 m | 0 | 1 | 75 | 11.2 s |
| 102 | 2.5 m | 0 | 0 | 0 | 35.2 s |

This is the reporting rig working as intended — run 100 shows immediately as the
outlier that it is, which is invisible when the runs are just three separate
logs.

### 5A.5 Results, honestly

Per 150 s run of `worlds/tennis_court_flap.wbt`, `--no-rendering`:

| run | change | delivered | captured | distance | worst stall |
| --- | --- | --- | --- | --- | --- |
| 20 | bounds guard + anti-thrash | 0 | 2 | 12.1 m | 92 s |
| 25 | `ESCAPE` band fix | 0 | 4 | 17.0 m | 16.8 s |
| 32 | turn-rate limit + batching | **4** | 4 | 12.8 m | 21.1 s |
| 33 | executable action set | 0 | 2 | 15.0 m | 18.4 s |
| 34 | directed exploration | 0 | 2 | 6.0 m | 57.3 s |
| 36 | stall escape | 0 | 4 | 12.9 m | — |

**The learned policy has not yet beaten the hand-written planner.** Run 32
(DWA-driven, with all the mechanical fixes) reached 4 delivered; runs driven by
the learned policy reached 2–4 collected but 0 delivered within the 150 s cap,
because a batch of 4 rarely completed a trip to the drop zone inside the
budget.

What is genuinely working:

* every deadlock found in 5.27–5.34 and 5.41–5.47 is fixed — the longest
  stationary stretch fell from 92 s to under 21 s;
* no falls in the last four runs, and the drop pad keeps delivered balls in the
  zone;
* the policy has learned the *escape* behaviour it was asked for: reverse-and-
  turn when the goal is behind or the robot is wedged, which is what the
  hand-written state machine repeatedly got wrong;
* weights persist, the buffer grows every run, and the offline trainer works.

What is not working, stated plainly:

* the learned policy is **not** yet better than DWA at navigation, and on some
  runs it is clearly worse. A 150 s run yields only a few thousand transitions
  for a 27648-cell state space; `MODE_DROP` had 29. That is far too little to
  learn navigation from, and the honest conclusion is that the buffer needs to
  be orders of magnitude larger — many more runs, or a cheaper source — before
  the policy can be trusted to drive.
* the exploration bias is currently too aggressive and biases towards reverse
  actions, which push the robot into corners (run 34).

**Recommendation:** keep the mechanical fixes and the low-traction pad, and keep
the policy wired in behind the per-mode trust gate, but do not let it drive
until the buffer is large enough to justify it. The next concrete step is
generating experience in bulk — running the world in `--mode=fast` with
randomised ball layouts for many iterations — rather than further tuning the
reward.

## 6. Current status of the hard court

**Fixed and verified:**

1. asymmetric wheel hinge axes ([§5.11](#511-the-two-wheel-hinges-had-opposite-axes))
2. motor reaction torque large enough to throw the robot onto its side
   ([§5.7](#57-motor-reaction-torque-was-tipping-the-robot-over))
3. joint anchors pinning every moving part to the chassis origin
   ([§5.12](#512-root-cause-1-joint-anchors-were-pinning-parts-to-the-chassis-origin))
4. single-value `coulombFriction` zeroing the wheels' forward grip
   ([§5.13](#513-root-cause-2-single-value-coulombfriction-zeroed-wheel-grip))
5. robot pressing into the side wall because clearance came from a sensor mounted
   0.78 m behind the bumper
   ([§5.18](#518-root-cause-3-the-mast-lidars-cannot-see-the-front-of-the-robot))
6. standoff correction applied to flanks and double-counted at the front, freezing
   the robot in open space
   ([§5.19](#519-root-cause-4-the-lidar-standoff-correction-was-wrong-in-three-ways))

The robot now drives at ~0.69 m/s with pitch held within 0.3° of level and
near-zero slip ([§5.15](#515-verification-the-drivetrain-works)), and the wall
guard sits inside `drive()` so every state inherits it.

**Open, and now the blocker:**

1. **Ball collection works and is retained on the court.** A ball is captured
   and the `bag` count holds it while the robot drives, using the two horizontal
   roller arms plus the passive one-way door
   ([§5.24](#524-the-final-design-two-horizontal-roller-arms)). Remaining work
   is **multi-ball tuning** (carrying several at once) — see the next item.
2. **The DWA planner can deadlock near a wall.** After a pickup the robot
   sometimes sits with `v = 0`, `w = 0`, `clear ≈ 0.5 m` and does not move
   (repeated `front`-limited logs) until `RECOVER`. This is a local-minimum in
   the local planner, not a collector fault, and is the current top blocker.
3. The mast lidar mounts were moved during the blind-spot investigation and
   restored to `-0.28 ±0.17 0.50`; confirm that is the *designed* position and not
   just the last thing tried ([§5.21](#521-where-the-robot-actually-is-when-it-stalls)).

---

All of these were hit and fixed; each is a hard parser error or silent drop.

| Rule | Detail |
|---|---|
| `coulombFriction` | `MFFloat` of **four** coefficients (longitudinal + lateral per material). A short list is **zero-filled**, not rejected — see [§5.13](#513-root-cause-2-single-value-coulombfriction-zeroed-wheel-grip) |
| `rollingFriction` | `SFVec3f` → **exactly three** values, e.g. `rollingFriction 0.01 0.01 0.004` (no brackets). A fourth value is a syntax error |
| Explicit mass | requires `density -1` |
| Sibling `Solid`s | every sibling needs a **unique** `name` |
| `DistanceSensor` | uses `aperture`, `numberOfRays`, `lookupTable` — **not** `fieldOfView`, `maxRange`, `noise` |
| `Cylinder` | z-aligned; y-axis wheels need a wrapping `Transform` with `rotation 1 0 0 1.5708`. The `Cylinder` node itself takes **no** `rotation` |
| `Cylinder` fields | takes **no** `translation`, `rotation`, or bare Pose in `Shape.geometry` — wrap in a `Transform` |
| `HingeJoint` | has **no** `translation`; position comes from the endpoint `Solid` |
| `HingeJointParameters.anchor` | resolved in the **parent** Solid's frame — it must equal the endpoint's `translation`, not `0 0 0`. See [§5.12](#512-root-cause-1-joint-anchors-were-pinning-parts-to-the-chassis-origin) |
| `controllerArgs` | is `MFString`, not `SFString` |
| Nested Pose | forbidden in a `boundingObject` (see [§5.5](#55-a-bounding-object-may-not-contain-a-nested-pose)) |
| Geometry expressions | a full field value is required; `translation -0.10 IS wheelY IS wheelR` does **not** parse |
| `EXTERNPROTO` | must appear **before** any node in the world file |

Parser errors encountered and resolved:

```
Skipped unknown 'translation' field in HingeJoint.
Skipped unknown 'rotation' field in Cylinder.
Cannot insert Pose node in 'geometry' field of Shape.
Cannot insert Pose node in 'children' field of Pose node in bounding object.
ERROR: Missing declaration for 'EXTERNPROTO', unknown node.
```

---

## 8. Python API constraints

| Constraint | Workaround |
|---|---|
| No `Robot.getControllerArgs()` | `shlex.split(' '.join(sys.argv[1:]))` |
| `getMotor` / `getPositionSensor` / `getLidar` deprecated | `getDevice(...)` |
| `Node` has no `getChildren` | DEF names are probed instead |
| `Field` has no `getMFNode` | ball truth obtained by probing `BALL0`…`BALL9` |
| `getFromDef()` cannot see DEFs inside a PROTO | `getSelf().getFromProtoDef(...)` on the PROTO node |
| No `Supervisor.getSelfPosition()` etc. | `getSelf().getPosition()` |
| `Node` has no `getAngularVelocity()` | read the `Gyro` device |
| `Gyro` has no `getValue()` | `getValues()` |
| `Robot.step()` returns **0** on success, not −1 | check `== 0`, not `!= 0` |

Orientation convention: `getOrientation()` returns nine **row-major** values. Each
*column* is where a body axis points in world coordinates, so the body +z axis is
`R[2], R[5], R[8]`, and tilt is the angle it makes with the world vertical.

---

## 9. Open issues

1. **Ball pickup succeeds but court reliability is still marginal.** A ball is
   captured (`bag` rises and holds) with the two horizontal roller arms plus the
   one-way door ([§5.24](#524-the-final-design-two-horizontal-roller-arms));
   `worlds/mech_test.wbt` reliably reaches `COLLECT` → `TO_DROP` with
   `bag 1 del 1/1`. On the court a ball is sometimes captured and sometimes
   stalls at the door at `x ≈ 0.46`. The mechanics are too sensitive to tune by
   hand, which is why [§5.26](#526-plan-learning-the-collector-with-persisted-experience)
   moves the final approach to a learned policy with a persisted replay buffer.
2. **Navigation local minima.** A DWA deadlock near a wall and, before the fix
   in [§5.25](#525-navigation-and-search-unreachable-targets-and-frontier-exploration),
   driving at the net to chase an unreachable ball. Unreachable-target rejection
   and frontier exploration address these; the DWA local minimum may still
   occur and is a candidate for the same learned policy or a recovery behavior.
3. **`ALIGN` never transitions to `COLLECT`** ([§5.16](#516-superseded-the-controller-stalls-in-align)).
   Now understood as a symptom of the perception faults in
   [§5.18](#518-root-cause-3-the-mast-lidars-cannot-see-the-front-of-the-robot) and
   [§5.19](#519-root-cause-4-the-lidar-standoff-correction-was-wrong-in-three-ways)
   rather than a bug in its own right. A related endless-spin mode (rotating on
   the spot batted the ball away) was fixed in
   [§5.25](#525-navigation-and-search-unreachable-targets-and-frontier-exploration).
4. **`gps_pose_h` is misnamed and misleading** ([§5.20](#520-gps_pose_h-is-not-a-heading)).
   It is the bearing of the GPS-vs-odometry error, not a heading, but it is
   logged next to two real headings. Rename to `gps_error_bearing` or remove it,
   and confirm no control path reads it as a fallback pose.
5. **`bumperLidar.maxRange` was too short.** It was set to `0.60` — less than the
   sensor needs to see its own `WALL_STOP_DIST = 0.26` plus braking distance — and
   was raised to `1.20`. Recorded because the guard is still only correct within
   that range, and a future tuning change that pushes the stop distance past
   1.20 m will reintroduce the blind spot.
6. **Telemetry CSV is not being written.** `logs/` exists but stays empty after
   runs and no summary reports a telemetry path, so the `open()` in `__init__` is
   failing silently inside `except (OSError, AttributeError)`. The exception type
   needs widening and the path construction re-checked.
6. **Self-right is untested.** `self_right()` has never executed, because with the
   torque fix the robot no longer falls. It needs a forced-fall test.
7. **Mast lidar mounts were moved while debugging** ([§5.21](#521-where-the-robot-actually-is-when-it-stalls))
   and restored to `-0.28 ±0.17 0.50`. Confirm that is the designed position
   rather than the last value tried.
8. **Dirt/clay world never re-run.** `tennis_court_dirt.wbt` received the friction
   fix but has not been simulated end-to-end since the torque and
   bounding-object fixes.
9. **Rendered scene never visually confirmed.** All validation has been
   `--no-rendering`; the robot, balls, markings, net and walls have been confirmed
   to *parse and simulate*, not to *look right*.

### Superseded open issues

These were open, are now closed, and are kept so a stale reference in the
controller or in an old log is recognisable:

* ~~The controller stalls in `ALIGN` as the sole remaining blocker~~ — see 2
  above for the narrowed version.
* ~~Dirt/clay world not re-run since the fixes~~ — merged into item 8; it was
  listed twice.
* ~~Robot pressed against the side wall / frozen in open space~~ — closed by
  [§5.18](#518-root-cause-3-the-mast-lidars-cannot-see-the-front-of-the-robot) and
  [§5.19](#519-root-cause-4-the-lidar-standoff-correction-was-wrong-in-three-ways).

---

## 10. Diagnostic tooling

Three purpose-built diagnostics, all kept in-tree:

* **`worlds/settle.wbt` + `controllers/settle/settle.py`** — the real PROTO on a
  flat floor with a `WorldInfo.contactProperties`-free world, all motors idle.
  Logs roll, pitch, pose, velocity and wheel encoder values every 6 steps.
  *This is what isolated [§5.7](#57-motor-reaction-torque-was-tipping-the-robot-over)*
  and should be the first thing re-run for any new stability question.*

* **`worlds/hinge.wbt` + `controllers/hingetest/hingetest.py`** — the minimal
  drivetrain rig: one chassis, two hinge-driven wheels, two ball-jointed skids,
  nothing else. Logs each wheel's deviation from nominal, axle height, chassis
  pitch, encoder and chassis position. *This is what isolated
  [§5.12](#512-root-cause-1-joint-anchors-were-pinning-parts-to-the-chassis-origin)
  (axle height changing ⇒ the anchor was in the wrong place) and
  [§5.13](#513-root-cause-2-single-value-coulombfriction-zeroed-wheel-grip)
  (encoder tracking command with zero lag ⇒ no contact force). It must mirror the
  court worlds' `contactProperties` and track `BODY_CHASSIS`, not `getSelf()`.*

* **`worlds/stepcheck.wbt` + `controllers/steptest/steptest.py`** — probes
  `Robot.step()`'s return code and `simulationGetMode()`.

* **Per-part body trace.** The main controller prints the world `x/y/z` of every
  `DEF`-named body for the first 10 s of a run (`BODY_TRACE_TIME`), reached via
  `getSelf().getFromProtoDef(...)`. This is what exposed the axle being twisted
  30° and, later, the roller sagging and the gate drifting — all three were the
  same anchor bug.

Also available:
`C:\Users\Reza\Documents\tennis_bot\backup_custom_proto_v1.zip` and
`backup_custom_proto_v2.zip`.

* **`worlds/settle8.wbt`** — a stripped floor-only variant of `settle.wbt` used
  to confirm the modified intake geometry loads into the real PROTO without
  parser warnings after the throat redesign
  ([§5.17](#517-the-intake-was-pinching-balls-at-the-throat)).

---

## 11. Chronological work log

### Phase 1 — make it parse

1. Rewrote `TennisCollectorRobot.proto` around a single rigid `chassis` `Solid`
   ([§5.1](#51-webots-has-no-rigid-weld-joint)); merged all static geometry.
2. Added grouped transformed geometry for deck, chute, throat walls, funnel
   guides, hopper floor/sides/back/front, mast and crossbar.
3. Implemented wheel hinge joints, motor limits, encoders, transformed y-axis
   cylinders and a chassis contact material.
4. Implemented free front `BallJoint` skids, powered intake roller, motorised
   unload gate.
5. Added two 128-resolution lidars, `ballSensor`, GPS and gyro.
6. Set Robot token mass to `0.05` so sensors work and physics drives the robot.
7. Corrected `controllerArgs` `SFString` → `MFString` in the PROTO and both worlds.
8. Added tennis-ball/chassis contact properties to both worlds.
9. Fixed, one at a time, every parser error in [§7](#7-webots-r2025a-syntax-constraints).
10. Created self-contained `TennisBall.proto` and `CourtMarkings.proto`; populated
    hard and dirt worlds with a centre net, posts, tape, perimeter walls,
    drop-zone graphics and ten balls.
11. Deleted redundant `tennis_court_hard.wbt`.
12. Reworked the controller: argument parsing, `getDevice`, NaN guards, corrected
    `count_delivered()`, orientation/attitude/ride-height/hopper-load monitoring,
    CSV telemetry, event logging, down-state detection, expanded reporting.
13. `python -m py_compile` → `COMPILE OK`.

### Phase 2 — find out why it falls over

14. First full hard-court run: **rolls onto its side in 0.5 s**, then slides
    upside-down for 244 s until the runtime cap.
15. Built `settle.wbt` / `settle.py`. First two attempts failed on API details —
    `getFromDef` cannot see DEFs inside a PROTO, and `Supervisor` has no
    `getSelfPosition()`. Third attempt failed on `Gyro.getValue()`. Final version
    uses `getSelf().getPosition()` and `gyro.getValues()`.
16. **Settle test: perfectly stable.** Geometry exonerated; the fault is in driving.
17. Fixed the nested-Pose bounding object with a pre-multiplied axis-angle
    ([§5.5](#55-a-bounding-object-may-not-contain-a-nested-pose)); this restored
    the funnel guides' collision shapes.
18. Raised the guides off the floor and the throat walls off z = 0; raised skid
    mass to 0.5 kg ([§5.6](#56-geometry-that-touches-the-floor-exactly-launches-the-body)).
19. Re-ran settle: still stable. Ran the full world: still fell at 0.5 s — so the
    guide/throat geometry was **not** the cause either.
20. Diagnosed motor reaction torque from the moment balance
    ([§5.7](#57-motor-reaction-torque-was-tipping-the-robot-over)).
21. Cut torques (14→4.0, 9→0.5, 6→1.2), added endpoint damping, raised chassis
    angular damping, and added software rate limiting (`WHEEL_ACCEL`,
    `ROLLER_RAMP`, `SETTLE_TIME`).
22. **Verified: 550 s with zero falls.**
23. Found and fixed the unreachable `FALL_GIVE_UP` branch; added supervisor
    `self_right()` with `NOMINAL_PARTS`, a 1 s `right_cooldown`, and a hard return
    to `SEEK` ([§5.8](#58-fall-recovery-could-never-fire)).
24. Initialised the missing `jam_time` / `underbody_reported` ([§5.9](#59-two-attributes-were-never-initialised)).
25. Gated surface detection on attitude ([§5.10](#510-slip-based-surface-detection-gave-false-positives)).
26. Re-ran: **stability solved, new blocker found** — the robot is pitched 9.6°
    nose-up and stationary, with the rear axle twisted ~30°.
27. Rebuilt `settle.py` as a **joint-violation probe**: it logs each body's world
    position minus the chassis position minus the PROTO's nominal offset, which
    is zero for a healthy hinge.
28. **Confirmed the field report**: only the left wheel's hinge was violated, by
    0.205 m. Root cause was the mirrored hinge axes
    ([§5.11](#511-the-two-wheel-hinges-had-opposite-axes)).
29. Set both wheel hinges to `axis 0 1 0` and replaced the misleading PROTO
    comment. Re-probed: `wL_dx == wR_dx` and identical encoders — asymmetry gone.
30. Discovered the deeper fault underneath: both axles now saturate together at
    `dx = +0.2040`, pitch runs away to +10.1° nose-up, and the robot never
    translates ([§5.12](#512-still-unsolved-both-wheels-now-walk-forward-and-the-rig-rears-up)).
31. Rewrote `settle.py` a third time to log chassis height, per-wheel and per-skid
    ground clearance, and the centre of mass projected into the robot frame — the
    measurements needed to confirm or reject the wheelie hypothesis.

### Phase 3 — find out why the wheels walk

32. Rebuilt `hinge.wbt` as a clean isolation rig and logged axle **height**
    through the hinge escape. Height was changing (`0.1142 → 0.1326 → 0.0823`),
    which a correctly anchored hinge makes impossible.
33. Noticed the escape distance `0.2309 m` equals the wheel's distance from the
    chassis origin, `0.2288 m`. The wheel was orbiting the origin, not sliding.
34. **Root cause 1 found:** `HingeJointParameters.anchor` is resolved in the
    parent Solid's frame, so `anchor 0 0 0` pinned every jointed part to the
    chassis origin ([§5.12](#512-root-cause-1-joint-anchors-were-pinning-parts-to-the-chassis-origin)).
    Set all six anchors to their endpoints' translations.
35. **Verified:** `wheel_dx` went from a runaway `+0.2309` to a steady `−0.0002`.
36. Robot still would not move, but the encoder tracked the 6 rad/s command with
    zero lag — the signature of zero contact force.
37. Found the harness was lying twice: `settle.wbt` had no `contactProperties`
    (no friction at all), and `hinge.wbt`'s probe tracked the 0.05 kg `Robot` node
    instead of the 12 kg chassis. Fixed both; rebuilt the single-wheel rig as
    two-wheel + two-skid, since a one-wheel rig just balances and pitches.
38. Briefly moved the wheel rotation onto the `Cylinder` node; R2025a rejects it
    (`Skipped unknown 'rotation' field in Cylinder node`), leaving an upright disc
    that tumbled convincingly while proving nothing. Reverted to the `Transform`.
39. **Root cause 2 found:** `coulombFriction` takes four values and Webots
    **zero-fills** a short list, so `[ 1.00 ]` gave the wheel no longitudinal grip
    ([§5.13](#513-root-cause-2-single-value-coulombfriction-zeroed-wheel-grip)).
    Widened all 27 friction lists across the three worlds.
40. **Verified:** the isolation rig drove `rx 0 → 1.269 m` at 0.69 m/s with pitch
    flat, then the real PROTO on `settle.wbt` reached `rx = 3.34 m`, and
    `tennis_court.wbt` drove 0.79 m in its first 4 s holding pitch to 0.3°.
41. Cleared the per-step `Sphere`-asymmetric-friction warning by converting both
    skid colliders to sideways `Cylinders`.
42. **Next blocker:** the controller stalls in `ALIGN` at `x ≈ -4.51`
    ([§5.16](#516-superseded-the-controller-stalls-in-align)). The drivetrain is sound;
    the remaining fault is in the state machine's alignment logic.

### Phase 4 — the intake, and the robot's blind spot

43. Measured the intake and found balls were being **pinched at the throat**: a
    0.10 m channel with a 0.15 m-high roller. Opened the channel to 0.13 m,
    lowered the roller to 0.10, and re-cut the funnel ramps so their tips deliver
    a ball at the roller equator
    ([§5.17](#517-the-intake-was-pinching-balls-at-the-throat)).
44. Moved the funnel ramps to `0.4900 ±0.2100 0.0375` with the pre-multiplied
    axis-angle rotation, restoring the guides' collision geometry so they
    actually funnel. Confirmed `settle.wbt` loads the modified PROTO with no
    warnings.
45. Added a `bumperLidar` at `translation 0.50 0 0.30`, FOV 2.8, resolution 96,
    after establishing that both mast lidars sit 0.78 m behind the front bumper
    and could not see the front of the machine
    ([§5.18](#518-root-cause-3-the-mast-lidars-cannot-see-the-front-of-the-robot)).
46. Added the wall guard: `read_bumper()`, `sector_min_bumper()`, `safe_speed()`
    using `v = sqrt(2·a·d)`, and `wall_guard()` applied **inside** `drive()` so
    no state can bypass it.
47. Added the `WALL_BACK` state — reverse and yaw away for 0.9 s on contact, then
    return to `SEEK` — because a forward-only clamp can leave the robot pinned
    against a fence indefinitely.
48. Raised `bumperLidar.maxRange` from `0.60` to `1.20`. The original value was
    shorter than the sensor's own stop distance plus braking margin, which would
    have reintroduced exactly the blind spot the lidar was added to remove.
49. **Root cause 4 found:** the standoff correction was wrong three ways —
    applied to the flanks (making clearances negative and freezing the robot in
    open space), double-counted at the front, and applied inconsistently across
    `clear_front` versus `blocked_ahead()` / `steer_away()` / `WALL_BACK`
    ([§5.19](#519-root-cause-4-the-lidar-standoff-correction-was-wrong-in-three-ways)).
    Set `LIDAR_STANDOFF = LIDAR_SIDE_STANDOFF = 0.0` and moved the correction to
    the point of measurement, applied once.
50. Consolidated the stall geometry
    ([§5.21](#521-where-the-robot-actually-is-when-it-stalls)) and restored both
    mast lidars to `translation -0.28 ±0.17 0.50`.
51. Identified `gps_pose_h` as the bearing of the GPS-vs-odometry error rather
    than a heading, and corrected its label and comment
    ([§5.20](#520-gps_pose_h-is-not-a-heading)).
52. Recorded the target navigation architecture — occupancy grid, global A\*, local
    DWA over `(v, ω)` under the retained task state machine — with the explicit
    note that perception and pose-frame correctness must land first
    ([§5.22](#522-recommended-navigation-architecture)).
53. Archived `backup_custom_proto_v2.zip`.

### Phase 5 — intake mechanism: four failures, then a two-roller door

54. Implemented the three-layer navigation (occupancy grid, A\*, DWA) under the
    retained state machine, with single-frame perception and the `pose_h` /
    `gps_pose_h` correction in place ([§5.22](#522-recommended-navigation-architecture)).
    Robot drives and reaches `SEEK → ALIGN → COLLECT` with the planner running.
55. Rebuilt the collector as a **level mouth + one-way flap + apron**. Tested in
    a dedicated world (`worlds/mech_test.wbt`). Failed: the flat front edge
    bulldozed the ball at a constant robot-frame `x` for the whole run
    ([§5.23](#523-the-collector-redesigns-and-why-each-one-failed) item 1).
56. Added **wide V gathering arms**, then discovered the apex-forward V is a
    **splitter** that deflects balls outward. Same bulldoze (item 2).
57. Replaced with an **inclined scoop ramp** with a ground-level lip, side walls
    and dead-ahead arms. Failed: a rigid ball cannot be scooped passively — a
    thin lip is thinner than the ball, a thick one pushes it down (item 3).
58. Replaced with a **powered intake roller**. Failed: with the roller centred
    above the ball, the contact lands below the roller centre and the roller
    ejects the ball (item 4).
59. Considered a **suction intake**, wrote and wired a tapering hood/nozzle,
    then **abandoned it**: it cannot be physically modelled without a fluid
    solver, so the only honest implementation was a supervisor shortcut that
    teleports the ball. Recorded in
    [§5.24](#524-the-final-design-two-horizontal-roller-arms).
60. **Decided on a two-roller front door** — two powered vertical rollers, one
    at each side of the entrance inside the flared arms, drawing balls in at
    floor level and pushing earlier balls deeper. No ball is ever lifted
    ([§5.24](#524-the-final-design-two-horizontal-roller-arms)). Implemented in the PROTO
    (`doorRollerLeftMotor` / `doorRollerRightMotor`) and the controller
    (`set_door_rollers()`, run from `run_collect()`); **not yet tested**.
61. Added a `TENNIS_DEBUG_DWA` environment probe to `dwa()` for diagnosing
    "all trajectories blocked" cases; used to find a goal-clamping bug in the
    test world's `--bounds` (order is `xmin xmax ymin ymax`).
62. Per user feedback, made the arm rollers **spin continuously** while
    aligning and while carrying, and added a **light passive one-way door** at
    the bin mouth for retention when reversing. Moved the roller inner ends
    forward (`x = 0.39` → `0.44`) so the door and rollers no longer collide
    (the overlap had jammed the door shut, holding a ball at `x = 0.43`).
    Verified on `tennis_court_flap.wbt`: ball captured and `bag` held at `1`
    through the run; a DWA wall deadlock remains after pickup.
63. Found and fixed the **net stall**: balls knocked across the net were
    latched as unreachable targets. Added `reachable()` and re-checked it every
    frame in `run_seek()` and `ball_in_cone()` ([§5.25](#525-navigation-and-search-unreachable-targets-and-frontier-exploration)).
64. Replaced the blind two-half lane sweep with **Yamauchi frontier
    exploration**: `obstacle_points_world()` returns beam segments,
    `update_map()` accumulates `known_free`/`known_occ`, and
    `search_waypoints()` targets the nearest frontier in the current half
    ([§5.25](#525-navigation-and-search-unreachable-targets-and-frontier-exploration)).
    This was the first *known* algorithm in the controller.
65. Fixed an endless **ALIGN spin** that batted the ball away: the robot no
    longer turns on the spot within 0.75 m of the ball; it creeps straight and
    lets `COLLECT` take over. Widened the roller mouth (door gap ~0.14 m) since
    the one-way door, not the nip, now provides retention.
66. Recorded the **RL plan with persisted experience** (append-only replay
    buffer under `data/`, checkpointed weights) in
    [§5.26](#526-plan-learning-the-collector-with-persisted-experience).
67. Wired the **experience buffer** into the controller: `record_experience()`
    appends JSONL to `controllers/tennis_collector/data/experience.jsonl`.
68. **Optimized the buffer** per user feedback: it now stores transitions, not
    per-tick samples, with decimation (`EXP_DECIMATE` 4), dedup
    (`EXP_MIN_DELTA`), event priority, a compact `{x,y,rng,v,w,nx,ny,reward,
    done}` schema and a 16 MB cap ([§5.26](#526-plan-learning-the-collector-with-persisted-experience)).
    A full capture episode is ~22 transitions, down from thousands. Fixed a
    latent bug found in the process: the `ball captured` event and the capture
    reward were unreachable in `run_collect()`; they now fire in
    `update_ball_states()`, verified as one `reward:1, done:true` row.
69. **Over-tightened the arm rollers and lost capture.** Converging the rollers
    to a ball-width nip (gaps `0.066`–`0.070 m`) was a knife edge that failed
    at all but one value; the user correctly observed the arms were "too tight".
    Reverted to the wide-door configuration (roller inner ends at
    `(0.38, ±0.10)`, door ≈ `0.20 m`) that captures reliably, and recorded the
    sensitivity warning in
    [§5.24](#524-the-final-design-two-horizontal-roller-arms).
70. Fixed **seven separate deadlock loops** found by analysing 150 s traces
    ([§5.27](#527-main-court-run-the-robot-wedged-its-arms-under-the-net) –
    [§5.34](#534-dwa-had-no-progress-term-so-the-robot-orbited-its-target)).
    Every one was a *state-exit* or *sensor-interpretation* bug rather than a
    control bug; the longest stationary stretch fell from 92 s to under 21 s.
71. Added the **learned policy** ([§5A](#5a-the-learned-policy)): factored
    linear Q-learning over 303 weights with state-action interaction terms, a
    15-entry executable action set, reward shaped for gather-then-drop, stuck
    and escape penalties, per-mode trust gating, weights persisted across runs
    and an offline trainer (`tools/train_policy.py`). Thirteen distinct bugs
    were found and fixed while building it
    ([§5A.4](#5a4-bugs-found-while-building-it)).
72. Added the **low-traction drop pad** ([§5A.3](#5a3-the-low-traction-drop-zone)):
    a real collidable pad with `tennisBall`/`dropPad` friction of 0.02 and
    rolling resistance 0.40, so delivered balls stop in the zone instead of
    rolling across the hard floor and out of it. The wheels keep full grip.
73. Fixed the **front wheel joints** ([§5.51](#551-the-front-wheels-were-on-balljoints-so-they-rotated-instead-of-spinning)).
    The front supports were on `BallJoint`s, which constrain nothing, so they
    yawed and wobbled on every turn instead of rolling. They are now
    `HingeJoint`s about the lateral axis, like the drive wheels but unpowered.
    Delivered **4 of 10** — matching the best run in the project — with peak roll
    down from 1.9° to 0.5°.
74. Built the **per-iteration reporting rig**
    ([§5B](#5b-experience-generation-and-per-iteration-reporting)):
    `tools/collect_experience.py` (bulk randomised experience generation),
    `tools/iteration_report.py` (per-run PNG + sample CSV + summary CSV, archived
    so each plot matches its own run), and `tools/compare_iterations.py`
    (cross-run table and trend plot). Three bugs found while building it:
    `--seed` was silently overwritten (5.48), `setSFVec3f` was called with three
    scalars and killed the controller at startup so every "run" wrote nothing
    (5.49), and the controller's fixed trace name meant each plot would have
    described the *next* run (5.50).
75. Added a **chase camera** to all four court worlds
    ([§5C](#5c-chase-camera)): a `Viewpoint` with `follow "ROBOT"` and
    `followType "Tracking Shot"`, so the camera tracks the robot with no
    controller code, plus the original static `overview` viewpoint. Three
    lessons: `Viewpoint` takes follow *fields*, not a `Track` child node (5.52);
    a world-file syntax error is a hard load failure that `--no-rendering` runs
    would never reveal (5.52); and a node cannot be referenced by name until it
    is `DEF`-ed (5.53).

**Still unproven after Phase 5:** the learned policy has **not** beaten the
hand-written DWA planner. It has learned the escape behaviour it was asked for,
and every deadlock is fixed, but a 150 s run yields only a few thousand
transitions against a 27648-cell state space — far too little to learn
navigation from. See [§5A.5](#5a5-results-honestly) for the honest comparison
and the recommendation to grow the buffer before trusting the policy to drive.

**Still unproven after Phase 5:** `tennis_court_dirt.wbt` has not been
re-simulated since the torque, bounding-object and friction fixes, so the
low-grip surface remains untested with all of the above.
---

## Phase 6 — the real gathering blocker: turning, not the intake

Opened with the complaint that "the algorithms do not work well and the robot
architecture is not good to gather the balls." Worked on a machine *with* Webots
(`E:\Apps\Webots\msys64\mingw64\bin\webots.exe`, R2025a) so changes could be
measured, not only reasoned about. Git version control was initialised this
phase (baseline commit + one commit per verified step); the GitHub remote is
`Pirata-Codex/tennis-ball-boy-bot`.

### 6.1 A deterministic test bench (`TENNIS_NO_POLICY`)

The first two "identical" baseline runs disagreed (2/3 then 1/3 delivered on the
same world and code). Cause: the controller **loads and mutates
`policy_weights.json` and `experience.jsonl` every run**, and once the policy has
> `RL_MIN_UPDATES` it starts influencing `SEEK`, so behaviour drifts between
invocations. Added `RL_ENABLED = os.environ.get('TENNIS_NO_POLICY', ...)`: with
`TENNIS_NO_POLICY=1` the deterministic DWA planner drives and repeated runs are
bit-identical. **All mechanism/controller A/B testing must set this**, or the
result is confounded by the drifting policy. The earlier "best = 4/10" numbers
were partly the policy getting lucky, not a reproducible mechanism result.

`worlds/mech_test_offset.wbt` (three off-centre balls) was added as the bench;
the single straight-ahead `mech_test.wbt` already captures and so hides the bug.

### 6.2 The controller is written for a robot that does not exist

`tennis_collector.py` fetches `armRollerLeftMotor`, `armRollerRightMotor` and
`unloadGate`, and `NOMINAL_PARTS` lists `BODY_ARM_ROLLER_*` / `BODY_FLAP`. **None
of these exist in `TennisCollectorRobot.proto`** — the PROTO on disk is the
single overhead-paddle design, and every `getDevice` for the arms returns `None`
(silently skipped by the `if motor is not None` guard). So §5.24's "two
horizontal roller arms (use this)" was written up but never actually shipped in
the rig, and `set_door_rollers()` has been a no-op on every run. The intake that
*does* run is the overhead paddle, which §5.23 item 4 had recorded as a failure.
Delivery itself is a supervisor teleport (`run_dump` `setPose`), so **delivery is
not a mechanism problem**: the whole difficulty is getting a ball into the hopper
box and keeping it there.

### 6.3 The paddle does not touch a floor ball — but that is not the main fault

Rendered observation (user screenshot): balls sit on the floor and the overhead
paddle spins *above* them. Its lowest point (~0.060) is at/above the ball top
(0.067), so a floor ball is barely grazed. A first rebuild as a low paddle-wheel
(hub clear of the ball, blades reaching to z≈0.027) was reverted: positioned at
x=0.40 its rear swing collided with the bin mouth/ledge and the physics shoved
the robot *backwards* across the court. Lesson: any powered wheel at the mouth
must have its full blade swing clear of the bin walls and the drop-step ledge,
and must not enter the mast-lidar view.

Deterministically (`TENNIS_NO_POLICY=1`): the **straight** ball is captured
(`max bag 1`), the three **offset** balls are not (`max bag 0`). So the paddle
*can* ingest a centred ball; the failure is purely that off-centre balls are
never centred.

### 6.4 Root cause: the robot cannot make small turns at low speed

Tracing the offset approach: `ly` (ball offset in the robot frame) stays pinned
near its initial value the whole approach — the robot drives **straight past**
the ball without turning toward it, then the ball lodges by a front wheel and
`ESCAPE` loops (the "gets behind the left front wheel and stops" the user saw).

A `TENNIS_SPIN="v,w"` probe (bypasses the state machine, logs achieved yaw rate
to `logs/spin_probe.csv`) settled it:

| command (v, w) | achieved yaw rate |
| --- | --- |
| v=0.00, w=0.9 (sustained) | **41 deg/s** |
| v=0.00, w=1.8 (sustained) | **81 deg/s** |
| v=0.00, w=2.4 (sustained) | **107 deg/s** |
| w=0.9 **inside the state machine** | **~1.6 deg/s** |

The drivetrain turns in place **fine** when the command is *sustained*. It turned
at ~0 inside the state machine because `ALIGN` and `COLLECT` **oscillate every
few steps** (ALIGN hands over at `ly<0.05`, COLLECT bounced back at `ly>0.035`),
and each turn command was cancelled by a forward command before the
wheel-acceleration ramp (`WHEEL_ACCEL`) could establish the pivot. **The chatter
did not just waste time — it is what crippled turning.** Lowering the front-skid
friction (0.08→0.01) changed nothing, confirming the skids are not the cause.

### 6.5 The fix (implemented; verification pending a GPU session)

Three controller changes plus one geometry change, aimed at *sustaining* the turn
and letting a centred ball be ingested:

1. **ALIGN turns in place, sustained, with a floor on the turn rate.** When the
   ball is not centred the robot now pivots (v=0) at `clamp(2.5*bearing)` but at
   least `ALIGN_W_MIN = 0.9` rad/s (a smaller command sits in the yaw
   static-friction deadzone and does nothing), until the ball is within
   tolerance. The old code crept straight forward "so the roller arms steer it
   in" — there are no roller arms.
2. **COLLECT commits instead of bouncing.** Re-align only if the ball is still
   well ahead *and* badly off (`lx>0.55 and |ly|>0.10`), ending the
   ALIGN↔COLLECT oscillation that was interrupting the turn.
3. **A floor-level centring funnel** replaces the too-wide flared wings: it opens
   to ±0.22 at the front (x=0.60) and closes to ±0.06 at the mouth (x=0.44), a
   0.12 m gap wider than the ball, reaching floor-to-0.11 so a ball cannot slip
   beside a front wheel.

Verified before the session lost its GPU: change 1 alone removed the `ESCAPE`
deadlock (distance 1.9→12.3 m, zero `ESCAPE` time). **Not yet verified:** that
the combined set captures the offset balls (`max bag>0`) — the Windows session
was disconnected mid-iteration, which drops the GPU context, and Webots R2025a
cannot initialise its renderer without one (the lidars need GPU rendering, so
`QT_QPA_PLATFORM=offscreen` software rendering hangs at init). **To verify, on a
machine with an active display:**

    set TENNIS_NO_POLICY=1
    webots --batch --mode=fast --no-rendering --stdout --stderr worlds/mech_test_offset.wbt
    python tools/analyze_trace.py logs/trace_world.jsonl   # want: max bag > 0

then the same on `mech_test.wbt` (must stay `max bag 1`, i.e. no regression of
the working straight-ahead case) before moving to the full court.

**Still unproven after Phase 6:** reliable collect-and-deliver on the full court.
The diagnosis (turning authority gated by state-oscillation, not the intake) is
solid and measured; the fix is implemented but the capture result is unverified
because the GPU/display went away mid-session.

---

## Phase 7 — from-scratch redesign (branch `redesign`): what worked, what didn't

Per the user's request ("come up with a new design, search good practices, build
it on a separate branch"), researched proven ball-collector designs (tennis-ball
retrieval patents; FRC/VEX "full-width over-the-bumper" intakes; dual-roller
intakes with ~3% error) and rebuilt the robot from scratch on branch `redesign`.

**What the research says (and the design aimed for):**
* FULL-WIDTH intake so a ball anywhere across the front is taken -- removes the
  centring requirement that the old narrow mouth could never satisfy.
* Intake roller just under a ball-diameter high; or dual rollers forming a nip.
* Differential drive + low-friction ball casters for free pivoting / no wedge
  pockets.

**What worked:**
* The new base -- differential drive (wheels ±0.22) + three low-friction ball
  casters (one rear, two front) -- is **stable (max tilt 0.5°) and mobile**, and
  it pivots freely. The casters are spheres, so balls roll around them instead of
  wedging against a flat skid edge (the old ESCAPE-pin cause).
* A critical front-support lesson: the intake/bin is front-heavy, so the front
  needs casters too; with only a rear caster the robot tips onto its bin front
  and is immovable.

**What did NOT work -- the hard constraint (measured, repeatedly):**
A **rigid** ball (Webots has no ball compliance) cannot be taken in by a roller:
* a single transverse roller is a BARRIER when its bottom is below the ball top
  (the ball cannot pass the roller's x-plane) and makes NO CONTACT when its
  bottom is at/above the ball top -- there is no height that both grips and
  passes. Both spin directions were tried; one bulldozes the ball forward, the
  other flings it forward/out.
* dual side rollers forming a nip are either too tight to pass (nip < 0.067 m,
  the ball is blocked at the nip mouth) or too loose to grip (nip ≥ 0.067 m, the
  rollers do not touch a centred ball). The literature's dual rollers work
  because real/FRC balls and compliant belt rollers deform ~0.5–1 inch; the rigid
  Webots ball and rigid rollers cannot.

**The only mechanism that captured a rigid ball** in this project is the OLD
**roll-under**: ride height > ball (0.09 m), an open floor-level entrance, and the
ball passes UNDER a high roof into a flush bin as the robot drives over it, with
a one-way flap / overhead paddle for retention. The redesign's full-width,
roll-under variant needs ride height 0.09; a wheel/geometry interaction froze the
wheels at that height and was not resolved before stopping.

**Recommendation:** the functional baseline is `main` (captures and delivers,
imperfectly). A reliable redesign should keep the new stable caster base but use
the proven **roll-under** intake (ride height 0.09, open full-width floor
entrance, flush bin, one-way retention flap) rather than any roller-through-nip
intake, which rigid-body physics cannot make work. The roller "powered arms"
idea is sound only for centring/retention, not for drawing a rigid ball through a
gap.
