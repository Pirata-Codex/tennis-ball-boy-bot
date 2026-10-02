#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
tennis_collector.py - Webots controller for TennisCollectorRobot.

Navigation architecture
-----------------------
Three layers, plus the task state machine on top:

  1. MAPPING      A local 2D occupancy grid, expressed in world cells but
                  pruned to a bounded window around the robot, so it is
                  robot-anchored and cannot grow without bound. Every lidar
                  and bumper beam is converted ONCE from (sensor mount,
                  sample angle, range) into a single robot-frame point, then
                  into a world point. There is no standoff correction: the
                  sensor mount offset is intrinsic to the sensor pose, so
                  applying it exactly once *is* the correction.

  2. PLANNING     Global A* over that grid to the current goal, recomputed on
                  a timer and whenever the goal changes.

  3. CONTROL      Local Dynamic Window Approach over (v, w). Candidate
                  trajectories are scored on path alignment, heading, speed and
                  clearance. The sqrt(2 a d) braking model that used to live in
                  safe_speed() is reused here as a hard velocity constraint *and*
                  as a clearance cost, so the robot sheds speed in time to stop
                  inside the remaining gap.

Task state machine (unchanged in spirit):

    SEEK    -> pick the nearest ball that is not in the hopper, A*+DWA to it
    ALIGN   -> close the last few centimetres and centre the ball in the throat
    COLLECT -> roller on + slow creep until the ball is inside the hopper
    RECOVER -> back out and retry if a pickup fails
    TO_DROP -> A*+DWA to the drop zone
    DUMP    -> open the unload gate, let the sloped floor feed the balls out
    DONE    -> park

Perception and pose
-------------------
The robot is a Supervisor. Following the same deliberate prototype shortcut the
project already uses for ball truth, the robot's own pose (x, y, yaw) is read
from the supervisor each step, so the planner can trust it absolutely. Wheel
encoders + GPS are still integrated for the slip/traction estimate that makes
the dirt-court behaviour correct. `gps_error_bearing` is the bearing of the
GPS-versus-odometry error and is a drift diagnostic only - it is NEVER used as
a heading.

The freeze fix
--------------
On the first controller iteration the lidars have not been stepped yet and
return an all-zero image. Treating 0.0 as a real range latched wall_contact and
pinned the robot for the whole run. Every range is now validated (finite and
>= minRange) before it can affect the map or the guard, and the guard is only
armed once at least one valid frame has been seen.
"""

from controller import Supervisor

import heapq
import json
import math
import os
import random
import shlex
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import policy as rl

# ============================== tunables ==============================

WHEEL_RADIUS = 0.09
TRACK = 0.37                 # wheel-centre to wheel-centre

# speeds, m/s and rad/s
VMAX = 0.62
WMAX = 2.4
ALIGN_WMAX = 1.8
# Minimum in-place turn rate during final alignment. Below this the robot's yaw
# static friction swallows the command and it does not rotate at all, so small
# heading errors never get corrected (the robot drove past off-centre balls).
ALIGN_W_MIN = 0.9
CREEP_SPEED = 0.045
BACK_SPEED = 0.22

# Acceleration limit, m/s^2 of wheel surface speed. A wheel carries almost no
# inertia, so an un-ramped step command goes straight into the chassis as
# reaction torque and used to tip the robot onto its side. Ramping removes it.
WHEEL_ACCEL = 1.2
SETTLE_TIME = 0.4            # s before the first drive command

# intake
# The front door is flanked by two powered vertical rollers, one at each side
# of the entrance. Each roller's door-facing surface sweeps backwards (-x), so
# a ball that reaches the door is drawn in behind the rollers; it cannot roll
# back out because the surfaces keep pushing it in, and new balls shove the
# earlier ones deeper into the bin. Nothing has to be wedged under the ball.
DOOR_ROLLER_SPEED = 16.0     # rad/s magnitude; signs set in set_door_rollers()
COLLECT_TIMEOUT = 6.0        # s
COLLECT_ABORT_X = 0.20       # m: closer than this the ball is under the body

# Overrun guard: a 67 mm ball against a 90 mm ride height lifts the wheels.
SEEK_BRAKE_DISTANCE = 0.80
SEEK_BRAKE_ANGLE = math.radians(32.0)

# ---- intake geometry, robot frame ---------------------------------------
# The door rollers sit at x = 0.42, radius 0.025, so a ball first meets the
# door with its centre at about 0.42 + 0.025 + 0.034 = 0.48. ALIGN only has to
# centre the ball on the 0.24 m bin and point the robot at it; COLLECT closes
# the gap and the spinning rollers draw the ball in.
INTAKE_X = 0.48              # ball centre at first contact with the door
ALIGN_X_MIN = 0.35           # closer than this while misaligned -> back off
ALIGN_X_MAX = 0.85           # centred and inside this -> hand over to COLLECT
ALIGN_Y_TOL = 0.05
ALIGN_BEARING_TOL = 0.12
ALIGN_TIMEOUT = 6.0            # s: bound on an approach that is not converging
# A ball is "under the chassis" only if it is within the deck footprint in the
# robot's x frame. This has to be a band, not `lx < X`: with no lower bound a
# ball 2 m behind the robot also satisfied the test, so ESCAPE was entered for
# balls the robot had already driven past and its exit condition was true on
# entry, producing an 0.0 s escape every step (FINDINGS 5.33).
UNDERBODY_X = (-0.30, 0.10)   # m, deck footprint in the robot x frame
UNDERBODY_Y = 0.20
# A ball rolled under the deck is not freed by the 1.2 s RECOVER nudge: the
# robot reversed, the ball stayed pinned, and SEEK drove straight back into it,
# repeating every 2.5 s for the rest of the run (FINDINGS 5.29). ESCAPE keeps
# reversing until the deck is actually clear, up to ESCAPE_TIME.
ESCAPE_TIME = 4.0
ESCAPE_CLEAR_X = 0.30         # m: ball must be this far behind the deck to resume
ESCAPE_COOLDOWN = 2.0         # s: do not re-enter ESCAPE for the same ball at once
ESCAPE_AVOID = 30.0           # s: do not chase that ball again straight away
STUCK_AVOID = 6.0             # s: first ban after a stuck event, doubled per repeat
STUCK_GIVE_UP = 4             # stuck attempts on one ball before abandoning it
# Stuck detection is sampled over a window instead of every step: a robot
# straining against an obstacle jitters more than the per-step threshold while
# making no net progress at all (FINDINGS 5.41).
STUCK_WINDOW = 1.5            # s: sample interval for the displacement test
STUCK_DISTANCE = 0.05         # m that must be covered within the window
STUCK_TIME = 4.0              # s of no net progress before acting
STALL_TIME = 3.0              # s clamped to zero speed by the wall guard

# hopper interior, robot frame (x forward, y left, z up) - the front bin.
# A ball is "collected" once its centre is inside this box, i.e. behind the
# door rollers.
HOPPER_X = (0.04, 0.40)
HOPPER_Y = (-0.10, 0.10)
HOPPER_Z = (0.008, 0.145)
# How many balls the bin can usefully hold before it must be unloaded. The bin
# interior is ~0.20 m across and a ball is 0.067 m, so this is a geometric
# limit rather than an arbitrary number - past it the rollers cannot stack a
# further ball in and it is simply pushed away again.
BAG_CAPACITY = 4

# ---- perception geometry -------------------------------------------------
# (device name, mount x, mount y, mount yaw, fov, resolution, max range)
FAN_DEFS = (
    ('lidarLeft', -0.28, 0.17, 0.85, 2.6, 128, 8.0),
    ('lidarRight', -0.28, -0.17, -0.85, 2.6, 128, 8.0),
)
LIDAR_MIN_RANGE = 0.05

# Bodywork footprint in the robot frame. The mast fans sit at z = 0.50 with a
# 0.2 rad vertical spread, so their lower rays clip the hopper wall tops and
# the mast crossbar: a handful of beams see the robot's own body at
# ~0.24-0.30 m. Left in the map, those cells make the robot think it is boxed
# in by a wall wherever it goes, and safe_speed() clamps every forward command
# to zero. Any return inside this box is the robot itself.
SELF_X = (-0.60, 0.90)
SELF_Y = 0.38

# ---- occupancy grid / planning ------------------------------------------
CELL = 0.10                  # m
WINDOW = 3.2                 # m, half-size of the local map window
ROBOT_RADIUS = 0.30          # m, planning inflation for the bodywork
DIAG = math.sqrt(2.0)
REPLAN_PERIOD = 0.5          # s between A* recomputations
MAP_PERIOD = 0.16            # s between distance-field rebuilds
INF = float('inf')

# ---- Dynamic Window Approach --------------------------------------------
DWA_HORIZON = 0.6            # s
DWA_STEPS = 6
DWA_V = (0.0, 0.16, 0.32, 0.47, VMAX)
DWA_W = (-WMAX, -1.6, -0.8, 0.0, 0.8, 1.6, WMAX)
LOOKAHEAD = 0.45             # m
DWA_PIVOT_ANGLE = 1.05        # rad: past this the goal is behind, so pivot first

# ---- wall / obstacle guard ----------------------------------------------
WALL_STOP_DIST = 0.18        # m, keep-off gap beyond the bodywork
RECOVER_TIME = 1.2           # s: how long RECOVER backs off before retrying
BOUND_MARGIN = 0.05          # m past a bound before the body is forced back
BOUND_BACK_TIME = 2.0        # s to reverse when the body leaves its bounds
WALL_DECEL = 1.6             # m/s^2, assumed braking capability
CONTACT_MARGIN = 0.03        # m, clearance below ROBOT_RADIUS+margin is contact
WALL_BACK_TIME = 0.9         # s

# slip handling
SLIP_WINDOW = 1.2
SLIP_ALPHA = 0.15

# unload
GATE_OPEN_ANGLE = 1.45
GATE_OPEN_TIME = 2.6
GATE_SETTLE_TIME = 1.2

# logging
LOG_PERIOD = 2.0

# ---- two-phase coverage search -------------------------------------------
# When no reachable ball is left to chase, the robot sweeps its allowed region
# in a lawnmower pattern driven by the lidar map. The region is split into two
# halves (near first, far second), and the robot declares the sweep done only
# after both halves. This keeps it from driving at the net chasing a ball on
# the far side, which is what deadlocked it at x = -0.72 in court_run7.
SEARCH_MARGIN = 0.45         # m kept clear of the region edges
SEARCH_REACH = 0.35          # m: a waypoint this close counts as reached
SEARCH_BALL_MARGIN = 0.20    # m a ball may sit outside bounds and still count

# ---- experience buffer (replay data for a learned policy) ----------------
EXP_DECIMATE = 4             # keep at most 1 routine step in N (31 Hz -> ~8 Hz)
EXP_MIN_DELTA = 0.004        # m; drop a step whose ball position barely moved
EXP_MAX_BYTES = 16_000_000   # cap the append-only file at ~16 MB

# ---- learned policy (linear Q-learning, see policy.py) ---------------------
# The policy chooses the (v, w) lattice action for the SEEK and TO_DROP states;
# A*, the occupancy grid and the wall guard are unchanged and still veto any
# action that is unsafe, so learning cannot drive the robot into a wall.
# Set TENNIS_NO_POLICY=1 to force the deterministic DWA planner and suppress all
# learned-policy driving. The buffer and weights still change between runs, so
# A/B testing a mechanism or planner change is only reproducible with the policy
# disabled; this flag makes that a one-env-var switch instead of editing code.
RL_ENABLED = os.environ.get('TENNIS_NO_POLICY', '') not in ('1', 'true', 'yes')
RL_EPSILON = 0.30             # initial exploration rate
RL_EPSILON_MIN = 0.05
RL_EPSILON_DECAY = 0.998
RL_ALPHA = 0.05
RL_GAMMA = 0.97
RL_DECAY_EVERY = 20           # steps between exploration-rate decays
RL_MIN_TRAIN_STEPS = 3        # do not learn from sub-step-length transitions
RL_SAVE_EVERY = 200           # steps between weight checkpoints
# Turning hard while moving forward scrubs the driven wheels and the reaction
# torque rolls the robot onto its side. Turn authority is scaled down linearly
# with forward speed, so a slow pivot is unrestricted while full speed leaves
# almost no yaw rate. This is applied inside rl_guard(), so it constrains the
# learned policy and the DWA fallback alike.
TURN_SPEED_RATIO_V = 0.95      # m/s at which turn authority reaches zero
# The policy is only allowed to act once it has been trained enough to have a
# preference at all. A fresh weights file is all zeros, so every action ties and
# the tie-break picks "hold still"; letting that drive from the first step wastes
# the run, and letting it *explore* from the first step is worse - in run 27 an
# untrained policy started acting after ~2 s and put the robot into orbit
# (FINDINGS 5.38). Until RL_MIN_UPDATES real updates exist, the hand-written DWA
# drives and the robot behaves exactly as it did before the policy existed.
RL_MIN_UPDATES = 400

# ---- attitude monitoring -------------------------------------------------
TILT_WARN = 0.35             # rad
TILT_FALLEN = 0.70           # rad
RIDE_HEIGHT = 0.09
FALL_RECOVER_TIME = 1.5
FALL_GIVE_UP = 8.0
FALL_MAX_ATTEMPTS = 3
# A self-right rebuilds the robot from the GPS pose, so it is only safe while
# that pose is believable. Beyond this height the robot has been flung rather
# than tipped, and rebuilding would teleport it (FINDINGS 5.37).
SELF_RIGHT_MAX_HEIGHT = 0.35   # m

SUPPORT_X = (-0.19, 0.18)
SUPPORT_Y = (-0.21, 0.21)

NOMINAL_PARTS = {
    'BODY_CHASSIS': (0.0, 0.0, 0.0),
    'BODY_WHEEL_LEFT': (-0.10, 0.185, 0.09),
    'BODY_WHEEL_RIGHT': (-0.10, -0.185, 0.09),
    'BODY_SKID_LEFT': (0.15, 0.175, 0.03),
    'BODY_SKID_RIGHT': (0.15, -0.175, 0.03),
    'BODY_ARM_ROLLER_LEFT': (0.56, 0.16, 0.040),
    'BODY_ARM_ROLLER_RIGHT': (0.56, -0.16, 0.040),
}
BODY_PARTS = tuple(NOMINAL_PARTS.keys())

TELEMETRY_PERIOD = 0.5
TELEMETRY_DIR = os.path.join(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))), 'logs')

# Trajectory trace for offline visualisation: robot pose + every ball
# position, written as JSONL and rendered by tools/plot_trace.py. Override the
# sampling interval with TENNIS_TRACE_PERIOD (seconds) if a long run makes too
# many rows.
TRACE_PERIOD = float(os.environ.get('TENNIS_TRACE_PERIOD', '0.2'))

MAX_RUNTIME = 600.0


# ============================== helpers ==============================

def clamp(value, low, high):
    return low if value < low else (high if value > high else value)


def wrap_angle(a):
    """Wrap to (-pi, pi]."""
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a


def rotation_matrix(node):
    """Row-major 3x3 rotation matrix, per the Supervisor API."""
    r = node.getOrientation()
    if len(r) < 9:
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    return list(r)


class TennisCollector(Supervisor):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())
        self.dt = self.timestep / 1000.0

        # ---- devices ----
        self.left_motor = self.getDevice('leftWheelMotor')
        self.right_motor = self.getDevice('rightWheelMotor')
        self.left_enc = self.getDevice('leftWheelEncoder')
        self.right_enc = self.getDevice('rightWheelEncoder')
        # The two gathering arms are horizontal powered rollers forming a V.
        self.arm_roller_left = self.getDevice('armRollerLeftMotor')
        self.arm_roller_right = self.getDevice('armRollerRightMotor')
        self.roller = self.getDevice('intakeRoller')
        self.gate = self.getDevice('unloadGate')
        self.ball_sensor = None
        self.gps = self.getDevice('gps')
        self.gyro = self.getDevice('gyro')

        self.fans = []
        for name, sx, sy, yaw, fov, res, maxr in FAN_DEFS:
            dev = self.getDevice(name)
            self.fans.append((dev, sx, sy, yaw, fov, res, maxr))

        for dev in (self.left_enc, self.right_enc, self.gps, self.gyro):
            dev.enable(self.timestep)
        if self.ball_sensor is not None:
            self.ball_sensor.enable(self.timestep)
        for fan in self.fans:
            fan[0].enable(self.timestep)
        self.bumper = None

        for motor in (self.left_motor, self.right_motor, self.roller,
                      self.arm_roller_left, self.arm_roller_right):
            if motor is not None:
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)
        if self.gate is not None:
            self.gate.setPosition(0.0)

        # ---- arguments ----
        args = shlex.split(' '.join(sys.argv[1:]))
        self.drop_x, self.drop_y = -4.9, 1.8
        self.bounds = (-5.75, -0.70, -2.70, 2.70)
        self.runtime = MAX_RUNTIME
        self.seed = 0
        self.explore_bias = 1.0
        i = 0
        while i < len(args):
            if args[i] == '--dropZone' and i + 2 < len(args):
                self.drop_x = float(args[i + 1])
                self.drop_y = float(args[i + 2])
                i += 3
            elif args[i] == '--bounds' and i + 4 < len(args):
                self.bounds = tuple(float(v) for v in args[i + 1:i + 5])
                i += 5
            elif args[i] == '--runtime' and i + 1 < len(args):
                self.runtime = float(args[i + 1])
                i += 2
            elif args[i] == '--seed' and i + 1 < len(args):
                self.seed = int(args[i + 1])
                i += 2
            elif args[i] == '--explore' and i + 1 < len(args):
                self.explore_bias = float(args[i + 1])
                i += 2
            else:
                i += 1
        self.rng = random.Random(self.seed)

        # ---- pose (supervisor truth) ----
        self.me = self.getSelf()
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_h = 0.0
        self.read_pose()

        # ---- bodies ----
        self.body = {}
        self.discover_body()

        # ---- odometry / slip ----
        self.last_left = 0.0
        self.last_right = 0.0
        self.odometer_l = 0.0
        self.odometer_r = 0.0
        self.odo_speed = 0.0
        self.slip = 0.0
        self.traction = 1.0
        self.slip_gps = 0.0
        self.slip_odo = 0.0
        self.slip_time = 0.0
        self.last_gps = [0.0, 0.0, 0.0]
        self.gps_error_bearing = 0.0

        # ---- mapping ----
        self.occ = set()
        self.dist = {}
        self.window = (0, 0, 0, 0)
        self.map_ready = False
        self.sensor_ready = False
        self.self_reported = False
        self.last_map = -10.0
        self.min_clear = WINDOW
        self.front_range = WINDOW

        # ---- planning ----
        self.path = []               # list of world (x, y) points
        self.last_plan = -10.0
        self.plan_goal = None
        self.plan_fail = 0

        # ---- experience log (replay buffer for a future learned policy) ----
        # One append-only JSONL record per intake step, so a learned approach
        # policy can be trained off-policy later without re-running Webots.
        # See FINDINGS.md 5.26.
        self.exp_handle = None
        self.exp_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     'data', 'experience.jsonl')
        self.exp_pending = None
        self.exp_step = 0
        self.exp_written = 0
        self.exp_skipped = 0
        self.exp_full = False

        # ---- learned policy ----
        # Seeded from --seed so a training run's layout AND its exploration
        # sequence are both reproducible from the run number alone. Before this,
        # line 381 set self.rng from --seed and this line overwrote it with a
        # constant, so every run explored identically (FINDINGS 5.48).
        if not getattr(self, 'seed', 0):
            self.rng = random.Random(12345)
        self.policy = rl.LinearQPolicy(alpha=RL_ALPHA, gamma=RL_GAMMA,
                                       epsilon=RL_EPSILON)
        self.policy.path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'data',
            'policy_weights.json')
        self.rl_loaded = self.policy.load()
        self.rl_steps = 0
        self.rl_prev_obs = None
        self.rl_prev_action = None
        self.rl_reward_acc = 0.0
        self.rl_last_event = None
        self.rl_explore = 0
        self.rl_greedy = 0
        self.rl_td_last = 0.0
        self.rl_safety_overrides = 0
        self.policy.alpha = RL_ALPHA
        self.policy.gamma = RL_GAMMA
        self.policy.epsilon_min = RL_EPSILON_MIN
        self.policy.epsilon_decay = RL_EPSILON_DECAY
        try:
            os.makedirs(os.path.dirname(self.exp_path), exist_ok=True)
            if (os.path.exists(self.exp_path) and
                    os.path.getsize(self.exp_path) >= EXP_MAX_BYTES):
                # keep the existing buffer; do not append further this run
                self.exp_full = True
            else:
                self.exp_handle = open(self.exp_path, 'a', encoding='utf-8')
        except OSError:
            self.exp_handle = None

        # ---- balls ----
        self.balls = []
        self.discover_balls()
        if self.seed:
            self.randomize_layout()

        # ---- state machine ----
        self.state = 'SEEK'
        self.target = None
        self.state_time = 0.0
        self.carrying = 0
        self.delivered = 0
        self.pickup_attempts = 0
        self.recover_time = 0.0
        self.escape_time = 0.0
        self.escape_ball = None
        self.escape_until = 0.0
        self.unreachable_balls = set()
        self.jam_time = 0.0
        self.last_jam_x = None
        self.underbody_reported = -10.0
        self.search_phase = 0
        self.known_free = set()
        self.known_occ = set()
        self.last_log = -10.0
        self.sim_time = 0.0
        self.done_time = 0.0

        # ---- guard ----
        self.wall_contact = False
        self.wall_back_left = 0.0
        self.wall_back_grace = 0.0
        self.wall_hits = 0
        self.wall_warned = False
        self.wall_side = 1.0
        self.state_before_wall = 'SEEK'

        # ---- attitude ----
        self.roll = 0.0
        self.pitch = 0.0
        self.tilt = 0.0
        self.peak_tilt = 0.0
        self.height = RIDE_HEIGHT
        self.min_height = RIDE_HEIGHT
        self.bag = 0
        self.bag_peak = 0
        self.fall_count = 0
        self.fall_timer = 0.0
        self.state_before_fall = 'SEEK'
        self.tilt_warned = False
        self.events = 0
        self.chassis = self.body.get('BODY_CHASSIS')
        self.com = (0.0, 0.0, 0.0)
        self.balance_margin = 0.0
        self.balanced = True
        self.unbalanced_streak = 0
        self.cmd_left = 0.0
        self.cmd_right = 0.0
        self.roller_cmd = 0.0
        self.self_rights = 0
        self.right_cooldown = 0.0
        self.heading_error = 0.0
        self.plan_follow = 0
        self.stuck_time = 0.0
        self.stall_time = 0.0
        self.stuck_window = 0.0
        self.last_pose = (self.pose_x, self.pose_y)

        # ---- telemetry ----
        self.telemetry = None
        self.last_telemetry = -10.0
        try:
            if not os.path.isdir(TELEMETRY_DIR):
                os.makedirs(TELEMETRY_DIR)
            self.telemetry_path = os.path.join(
                TELEMETRY_DIR, 'telemetry_%s.csv' % self.world_tag())
            self.telemetry = open(self.telemetry_path, 'w')
            self.telemetry.write(
                't,state,x,y,yaw_deg,roll_deg,pitch_deg,tilt_deg,'
                'slip_pct,carry,bag,delivered,cmd_v,cmd_w,min_clear,'
                'path_len,tgt_x,tgt_y,occ,gps_err_deg,wall_hits\n')
        except (OSError, AttributeError, Exception):
            self.telemetry = None

        # ---- trajectory trace (for offline plots) ----
        self.trace = None
        self.last_trace = -10.0
        try:
            if not os.path.isdir(TELEMETRY_DIR):
                os.makedirs(TELEMETRY_DIR)
            # TENNIS_TRACE_FILE overrides the trace path, so concurrent or
            # back-to-back runs never write the same file (world_tag() falls back
            # to 'world' when getWorld().getUrl() is unavailable on this build,
            # so every run otherwise collided on trace_world.jsonl).
            self.trace_path = os.environ.get('TENNIS_TRACE_FILE') or os.path.join(
                TELEMETRY_DIR, 'trace_%s.jsonl' % self.world_tag())
            self.trace = open(self.trace_path, 'w', encoding='utf-8')
            self.trace.write(json.dumps({
                'meta': True,
                'bounds': list(self.bounds),
                'drop': [self.drop_x, self.drop_y],
                'balls': len(self.balls),
            }, separators=(',', ':')) + '\n')
        except (OSError, AttributeError, Exception):
            self.trace = None

        print('tennis_collector: ready, %d balls, drop zone (%.2f, %.2f), '
              'bounds %s' % (len(self.balls), self.drop_x, self.drop_y,
                             self.bounds))

    # ------------------------------------------------------------------
    # pose, telemetry, events
    # ------------------------------------------------------------------

    def world_tag(self):
        """A safe world name for log file names.

        `self.getWorld().getUrl()` is not reliably available on every Webots
        build; when it throws, the old code aborted the whole telemetry/trace
        block and silently wrote nothing (the cause of the empty logs/ noted in
        FINDINGS section 9). Each lookup is isolated and falls back to 'world'.
        """
        for getter in (lambda: self.getWorld().getUrl(),
                       lambda: self.getWorld().getFilePath()):
            try:
                url = getter()
                if url:
                    return os.path.splitext(os.path.basename(url))[0]
            except Exception:
                continue
        return 'world'

    def read_pose(self):
        """Ground-truth pose from the supervisor (prototype shortcut).

        The planner cannot outrun a drifting compass, and dead-reckoned yaw
        from a single gyro drifts. The robot is a Supervisor, so its exact pose
        is available; using it makes pose_h trustworthy by construction. The
        encoder/GPS integration below is kept only for the slip estimate.
        """
        p = self.me.getPosition()
        m = rotation_matrix(self.me)
        self.pose_x, self.pose_y = p[0], p[1]
        # body +x axis in world is column 0 = (m[0], m[3], m[6])
        self.pose_h = wrap_angle(math.atan2(m[3], m[0]))

    def event(self, message):
        self.events += 1
        print('  >> [t=%6.1f] %s' % (self.sim_time, message))

    def write_telemetry(self):
        if self.telemetry is None:
            return
        if self.target is not None:
            tgt_x, tgt_y = self.target['pos'][0], self.target['pos'][1]
        else:
            tgt_x = tgt_y = float('nan')
        self.telemetry.write(
            '%.2f,%s,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,'
            '%.3f,%.3f,%.3f,%d,%.3f,%.3f,%d,%.1f,%d\n'
            % (self.sim_time, self.state, self.pose_x, self.pose_y,
               math.degrees(self.pose_h), math.degrees(self.roll),
               math.degrees(self.pitch), math.degrees(self.tilt),
               100.0 * self.slip, self.carrying, self.bag,
               self.count_delivered(), self.cmd_v, self.cmd_w,
               self.min_clear, len(self.path), tgt_x, tgt_y, len(self.occ),
               math.degrees(self.gps_error_bearing), self.wall_hits))
        self.telemetry.flush()

    def write_trace(self):
        """One trajectory sample: robot pose plus every ball position.

        Rendered by tools/plot_trace.py into a court map showing where the
        robot went and where the balls were, so a run can be inspected without
        watching the simulation.
        """
        if self.trace is None:
            return
        balls = [[round(b['pos'][0], 3), round(b['pos'][1], 3),
                  1 if b['in_hopper'] else 0] for b in self.balls]
        rec = {
            't': round(self.sim_time, 2),
            's': self.state,
            'x': round(self.pose_x, 3),
            'y': round(self.pose_y, 3),
            'h': round(math.degrees(self.pose_h), 1),
            'bag': self.bag,
            'dep': self.count_delivered(),
            'b': balls,
        }
        try:
            self.trace.write(json.dumps(rec, separators=(',', ':')) + '\n')
            self.trace.flush()
        except OSError:
            self.trace = None

    # ------------------------------------------------------------------
    # world inspection
    # ------------------------------------------------------------------

    def discover_body(self):
        proto = self.getSelf()
        for part in BODY_PARTS:
            try:
                self.body[part] = proto.getFromProtoDef(part)
            except Exception:
                self.body[part] = None
        self.chassis = self.body.get('BODY_CHASSIS')

    def randomize_layout(self):
        """Scatter the balls (and the robot) for experience generation.

        One fixed layout cannot cover a 27648-cell state space: the same 17
        distance/bearing pairs recur every run, so the buffer grows with
        near-duplicate rows and never reaches the cells the policy actually
        queries (FINDINGS 5A.5). Randomising per run is the cheapest way to get
        genuinely different situations out of the same world file.
        """
        xmin, xmax, ymin, ymax = self.bounds
        # keep clear of the wall skirt and of the drop pad, so no ball starts
        # inside geometry the robot cannot reach
        for b in self.balls:
            for _ in range(40):
                x = self.rng.uniform(xmin + 0.7, xmax - 0.7)
                y = self.rng.uniform(ymin + 0.7, ymax - 0.7)
                if abs(x - self.drop_x) < 1.1 and abs(y - self.drop_y) < 0.95:
                    continue
                if any(math.hypot(x - c[0], y - c[1]) < 0.45
                       for c in [(bb['pos'][0], bb['pos'][1])
                                 for bb in self.balls if bb is not b]):
                    continue
                break
            # setSFVec3f takes the whole vector as ONE list argument, not three
            # scalars. Passing (x, y, z) raised TypeError and killed the
            # controller at startup, so every training run produced zero
            # experience and a robot that never moved (FINDINGS 5.49).
            b['pos'] = [x, y, b['pos'][2]]
            b['node'].getField('translation').setSFVec3f([x, y, b['pos'][2]])

    def discover_balls(self):
        for i in range(64):
            node = self.getFromDef('BALL%d' % i)
            if node is None:
                break
            self.balls.append({
                'id': i,
                'node': node,
                'pos': list(node.getPosition()),
                'in_hopper': False,
                'held': 0,
                'avoid_until': 0.0,
                'stuck_count': 0,
            })
        self.update_ball_states()

    def update_ball_states(self):
        for ball in self.balls:
            ball['pos'] = list(ball['node'].getPosition())
            lx, ly, lz = self.to_robot(*ball['pos'])
            inside = (HOPPER_X[0] < lx < HOPPER_X[1] and
                      HOPPER_Y[0] < ly < HOPPER_Y[1] and
                      HOPPER_Z[0] < lz < HOPPER_Z[1])
            if inside:
                ball['held'] += 1
                if ball['held'] >= 3 and not ball['in_hopper']:
                    ball['in_hopper'] = True
                    # a target ball just entered the bin: this is the capture.
                    # It is detected here (not in run_collect) because the ball
                    # crosses the hopper boundary between control frames, which
                    # is where the +1 reward and the capture log belong.
                    if ball is self.target:
                        self.record_experience(ball, (0.0, 0.0), 1.0, True)
                        self.carrying += 1
                        self.pickup_attempts = 0
                        self.rl_note('capture')
                        self.event('ball captured, hopper load %d, bag %d'
                                   % (self.carrying, self.bag_count()))
                        self.target = None
                        self.path = []
                        self.state = self.next_state_after_pickup()
                        self.state_time = 0.0
            else:
                ball['held'] = 0

    def to_robot(self, wx, wy, wz=0.0):
        dx = wx - self.pose_x
        dy = wy - self.pose_y
        c, s = math.cos(self.pose_h), math.sin(self.pose_h)
        return (dx * c + dy * s, -dx * s + dy * c, wz)

    def to_world(self, lx, ly, lz=0.0):
        c, s = math.cos(self.pose_h), math.sin(self.pose_h)
        return (self.pose_x + lx * c - ly * s,
                self.pose_y + lx * s + ly * c, lz)

    def loose_balls(self):
        return [b for b in self.balls
                if not b['in_hopper'] and b['id'] not in self.unreachable_balls
                and not self.in_drop_zone(b['pos'][0], b['pos'][1])]

    def in_drop_zone(self, wx, wy):
        return (abs(wx - self.drop_x) < 0.85 and
                abs(wy - self.drop_y) < 0.85)

    def count_delivered(self):
        """Balls actually unloaded into the drop zone.

        A held ball is not delivered: counting `in_hopper` here inflated the
        delivered total (a run could report a delivery without ever reaching
        the zone). Held balls are reported separately by `bag`.
        """
        return sum(1 for b in self.balls
                   if self.in_drop_zone(b['pos'][0], b['pos'][1]))

    def bag_count(self):
        count = 0
        for ball in self.balls:
            lx, ly, lz = self.to_robot(*ball['pos'])
            if (HOPPER_X[0] < lx < HOPPER_X[1] and
                    HOPPER_Y[0] < ly < HOPPER_Y[1] and
                    HOPPER_Z[0] < lz < HOPPER_Z[1]):
                count += 1
        if count > self.bag_peak:
            self.bag_peak = count
        return count

    # ------------------------------------------------------------------
    # layer 1: mapping
    # ------------------------------------------------------------------

    def obstacle_points_world(self):
        """Obstacle hits and free-space rays, both in the world frame.

        A beam of a fan mounted at (sx, sy) with yaw phi, sample angle a and
        range r lands at robot-frame (sx + r cos(a), sy + r sin(a)). That single
        conversion *is* the sensor-offset correction; there is nothing left to
        subtract afterwards. Returns (hits, rays): `hits` are obstacle points;
        `rays` are (x0, y0, x1, y1, hit) segments where the beam travelled,
        which the map uses to mark observed-free cells for frontier
        exploration. A beam at the range cap still marks free space out to the
        cap (hit = False); the all-zero first frame is dropped as before.
        """
        pts = []
        rays = []
        sh, ch = math.sin(self.pose_h), math.cos(self.pose_h)
        dropped_self = 0
        nearest_self = None
        for dev, sx, sy, yaw, fov, res, maxr in self.fans:
            img = dev.getRangeImage()
            if not img or len(img) < res:
                continue
            step = fov / res
            for idx in range(res):
                r = img[idx]
                if r != r or r < LIDAR_MIN_RANGE:
                    continue
                hit = r < maxr * 0.999
                rr = r if hit else maxr
                a = yaw - fov / 2.0 + (idx + 0.5) * step
                # robot frame
                rx = sx + rr * math.cos(a)
                ry = sy + rr * math.sin(a)
                # reject the robot's own bodywork
                if SELF_X[0] <= rx <= SELF_X[1] and -SELF_Y <= ry <= SELF_Y:
                    dropped_self += 1
                    if nearest_self is None or r < nearest_self[0]:
                        nearest_self = (r, math.degrees(math.atan2(ry, rx)))
                    continue
                # world frame
                x0 = self.pose_x + sx * ch - sy * sh
                y0 = self.pose_y + sx * sh + sy * ch
                x1 = self.pose_x + rx * ch - ry * sh
                y1 = self.pose_y + rx * sh + ry * ch
                rays.append((x0, y0, x1, y1, hit))
                if hit:
                    pts.append((x1, y1))
        if not self.self_reported and dropped_self:
            self.self_reported = True
            if nearest_self is not None:
                print('  map: dropped %d self-returns from the bodywork '
                      '(nearest %.2f m at %+.1f deg)' % (dropped_self,
                                                         nearest_self[0],
                                                         nearest_self[1]))
        return pts, rays

    @staticmethod
    def cell_of(x, y):
        return (int(math.floor(x / CELL)), int(math.floor(y / CELL)))

    @staticmethod
    def cell_center(cell):
        return ((cell[0] + 0.5) * CELL, (cell[1] + 0.5) * CELL)

    def window_cells(self):
        ix0 = int(math.floor((self.pose_x - WINDOW) / CELL))
        ix1 = int(math.floor((self.pose_x + WINDOW) / CELL))
        iy0 = int(math.floor((self.pose_y - WINDOW) / CELL))
        iy1 = int(math.floor((self.pose_y + WINDOW) / CELL))
        return (ix0, ix1, iy0, iy1)

    def update_map(self, pts, rays=()):
        """Rebuild the live occupancy set and accumulate observed free space.

        The obstacle set is rebuilt from the live scan each time rather than
        accumulated forever, which keeps the clearance field robot-anchored and
        bounded. Separately, `known_free` accumulates the cells every beam
        passed through and `known_occ` the cells that were ever hits; those two
        are what frontier exploration reasons over, and they are capped to the
        map window so they stay small.
        """
        self.window = self.window_cells()
        self.occ = set()
        for (x, y) in pts:
            c = self.cell_of(x, y)
            self.occ.add(c)
            self.known_occ.add(c)
            self.known_free.add(c)
        # walk each beam, marking free cells up to (not including) a hit
        for (x0, y0, x1, y1, hit) in rays:
            steps = int(math.hypot(x1 - x0, y1 - y0) / (CELL * 0.5)) + 1
            for i in range(steps + 1):
                if hit and i == steps:
                    break
                t = i / float(steps)
                self.known_free.add(self.cell_of(x0 + (x1 - x0) * t,
                                                 y0 + (y1 - y0) * t))
        # forget observed cells that have drifted out of the window
        ix0, ix1, iy0, iy1 = self.window
        ix0 -= 2
        ix1 += 2
        iy0 -= 2
        iy1 += 2
        if len(self.known_free) > 4000:
            self.known_free = set(
                c for c in self.known_free
                if ix0 <= c[0] <= ix1 and iy0 <= c[1] <= iy1)
            self.known_occ = set(
                c for c in self.known_occ
                if ix0 <= c[0] <= ix1 and iy0 <= c[1] <= iy1)
        self.build_clearance()

    def build_clearance(self):
        """Dijkstra clearance field (metres) over the local window."""
        ix0, ix1, iy0, iy1 = self.window
        dist = {}
        heap = []
        for cell in self.occ:
            cx, cy = cell
            if ix0 - 1 <= cx <= ix1 + 1 and iy0 - 1 <= cy <= iy1 + 1:
                dist[cell] = 0.0
                heap.append((0.0, cell))
        heapq.heapify(heap)
        neighbours = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                      (-1, -1, DIAG), (-1, 1, DIAG), (1, -1, DIAG), (1, 1, DIAG))
        cap = WINDOW
        while heap:
            d, (cx, cy) = heapq.heappop(heap)
            if d > dist.get((cx, cy), INF):
                continue
            if d > cap:
                continue
            for dx, dy, w in neighbours:
                nx, ny = cx + dx, cy + dy
                if nx < ix0 - 1 or nx > ix1 + 1 or ny < iy0 - 1 or ny > iy1 + 1:
                    continue
                nd = d + w * CELL
                key = (nx, ny)
                if nd < dist.get(key, INF):
                    dist[key] = nd
                    heapq.heappush(heap, (nd, key))
        self.dist = dist
        self.map_ready = True

    def clearance(self, x, y):
        """Clearance to the nearest observed obstacle at a world point."""
        return self.dist.get(self.cell_of(x, y), WINDOW)

    def clearance_robot(self, lx, ly):
        wx, wy, _ = self.to_world(lx, ly)
        return self.clearance(wx, wy)

    # ------------------------------------------------------------------
    # layer 2: global A*
    # ------------------------------------------------------------------

    def astar(self, start_xy, goal_xy):
        """A* over the local grid, 8-connected. Returns [world points] or []."""
        if not self.map_ready:
            return []
        start = self.cell_of(*start_xy)
        goal = self.cell_of(*goal_xy)

        def blocked(cell):
            return self.clearance(*self.cell_center(cell)) < ROBOT_RADIUS

        if blocked(goal):
            goal = self.nearest_free(goal)
            if goal is None:
                return []
        if blocked(start):
            start = self.nearest_free(start)
            if start is None:
                return []

        ix0, ix1, iy0, iy1 = self.window
        margin = int(math.ceil(ROBOT_RADIUS / CELL)) + 2
        lo_x, hi_x = ix0 - margin, ix1 + margin
        lo_y, hi_y = iy0 - margin, iy1 + margin

        def heur(a, b):
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            return (dx + dy) + (DIAG - 2.0) * min(dx, dy)

        open_heap = [(heur(start, goal), 0.0, start)]
        came = {}
        gscore = {start: 0.0}
        visited = set()
        steps = 0
        neighbours = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                      (-1, -1, DIAG), (-1, 1, DIAG), (1, -1, DIAG), (1, 1, DIAG))
        while open_heap and steps < 40000:
            _, g, cell = heapq.heappop(open_heap)
            if cell in visited:
                continue
            visited.add(cell)
            steps += 1
            if cell == goal:
                return self.reconstruct(came, cell)
            for dx, dy, w in neighbours:
                nxt = (cell[0] + dx, cell[1] + dy)
                if nxt[0] < lo_x or nxt[0] > hi_x or nxt[1] < lo_y or nxt[1] > hi_y:
                    continue
                if nxt in visited:
                    continue
                if blocked(nxt):
                    continue
                clear = self.clearance(*self.cell_center(nxt))
                # prefer paths that keep a little margin
                penalty = 0.0 if clear > 1.5 * ROBOT_RADIUS else 0.6
                ng = g + w + penalty
                if ng < gscore.get(nxt, INF):
                    gscore[nxt] = ng
                    came[nxt] = cell
                    heapq.heappush(open_heap, (ng + heur(nxt, goal), ng, nxt))
        return []

    def nearest_free(self, cell):
        for radius in range(1, 16):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    c = (cell[0] + dx, cell[1] + dy)
                    if self.clearance(*self.cell_center(c)) >= ROBOT_RADIUS:
                        return c
        return None

    def reconstruct(self, came, cell):
        cells = [cell]
        while cell in came:
            cell = came[cell]
            cells.append(cell)
        cells.reverse()
        return [self.cell_center(c) for c in cells]

    def maybe_replan(self, goal_world):
        goal = (round(goal_world[0], 2), round(goal_world[1], 2))
        stale = (self.sim_time - self.last_plan) > REPLAN_PERIOD
        changed = (self.plan_goal != goal)
        if not stale and not changed and self.path:
            return
        self.plan_goal = goal
        self.last_plan = self.sim_time
        self.path = self.astar((self.pose_x, self.pose_y), goal_world)
        self.plan_follow = 0
        if not self.path:
            self.plan_fail += 1
        else:
            self.plan_fail = 0

    def path_robot(self):
        return [self.to_robot(wx, wy, 0.0)[:2] for (wx, wy) in self.path]

    # ------------------------------------------------------------------
    # layer 3: local DWA
    # ------------------------------------------------------------------

    def safe_speed(self, dist):
        """sqrt(2 a d): the speed from which the robot can still stop."""
        d = dist - ROBOT_RADIUS - WALL_STOP_DIST
        if d <= 0.0:
            return 0.0
        return math.sqrt(2.0 * WALL_DECEL * d)

    def dwa(self, goal_dir, goal_dist):
        """Pick (v, w) that tracks the path while clearing obstacles.

        Trajectories are simulated in the robot frame (origin, heading 0), so
        the clearance lookup maps a simulated point back through the current
        world pose. The sqrt(2 a d) model is used twice: as a hard velocity cap
        along the trajectory, and as a clearance cost term.
        """
        pts = self.path_robot()
        if pts:
            look = pts[-1]
            for p in pts:
                if math.hypot(p[0], p[1]) > LOOKAHEAD:
                    look = p
                    break
            gdir = math.atan2(look[1], look[0])
            gx, gy = look
        else:
            gdir = goal_dir
            gx = math.cos(goal_dir) * min(goal_dist, 0.6)
            gy = math.sin(goal_dir) * min(goal_dist, 0.6)

        best = None
        best_cost = INF
        for v in DWA_V:
            for w in DWA_W:
                x = y = h = 0.0
                ok = True
                min_d = WINDOW
                dt = DWA_HORIZON / DWA_STEPS
                for _ in range(DWA_STEPS):
                    h += w * dt
                    x += v * math.cos(h) * dt
                    y += v * math.sin(h) * dt
                    d = self.clearance_robot(x, y)
                    if d < min_d:
                        min_d = d
                    if d < ROBOT_RADIUS:
                        ok = False
                        break
                if not ok:
                    continue
                if v > 0.0 and v > self.safe_speed(min_d) + 1e-6:
                    continue
                heading_err = abs(wrap_angle(w * DWA_HORIZON - gdir))
                pos_err = math.hypot(x - gx, y - gy)
                clear_cost = max(0.0, 0.45 - min_d)
                # Progress term. Without it the cost is almost independent of v
                # (heading_err depends only on w), so when the goal sat behind
                # the robot a forward arc scored the same as pivoting in place
                # and the robot orbited the ball at a fixed radius for 12 s
                # without ever closing on it (FINDINGS 5.34). Penalising forward
                # speed when the goal is not in front makes the local planner
                # pivot first, which is what actually reduces the distance.
                progress = 0.0
                if abs(gdir) > DWA_PIVOT_ANGLE:
                    progress = 2.5 * v

                # Heavily penalize doing nothing
                stall_penalty = 5.0 if (v == 0.0 and w == 0.0) else 0.0

                cost = (1.4 * heading_err + 1.0 * pos_err + progress +
                        0.35 * (VMAX - v) + 0.8 * clear_cost + stall_penalty)
                if cost < best_cost:
                    best_cost = cost
                    best = (v, w)
        if best is None:
            # everything collides or is unsafe: rotate toward the goal in place
            w = clamp(1.8 * gdir, -WMAX, WMAX)

            # Force a spin to find a clear path if gdir is near zero
            if abs(w) < 0.5:
                w = math.copysign(1.0, gdir) if abs(gdir) > 1e-4 else 1.0

            v = 0.0
            if goal_dist < 0.5 and abs(gdir) > 1.4:
                v = -0.10
            if os.environ.get('TENNIS_DEBUG_DWA'):
                print('  dwa: ALL BLOCKED gdir %+.2f gd %.2f -> (%.2f,%+.2f)'
                      % (gdir, goal_dist, v, w))
            return v, w
        if os.environ.get('TENNIS_DEBUG_DWA'):
            print('  dwa: gdir %+.2f gd %.2f path %d -> (%.2f,%+.2f) cost %.3f'
                  % (gdir, goal_dist, len(pts), best[0], best[1], best_cost))
        return best

    # ------------------------------------------------------------------
    # actuation
    # ------------------------------------------------------------------

    def drive(self, forward, turn):
        """forward m/s, turn rad/s (positive = left), rate limited."""
        if self.sim_time < SETTLE_TIME:
            forward = 0.0
            turn = 0.0

        if self.wall_contact:
            forward = min(forward, 0.0)
        else:
            # safety cap from the measured clearance, reusing sqrt(2 a d)
            cap = self.safe_speed(self.min_clear)
            if forward > cap:
                forward = cap

        # Stability limit, applied here rather than only in rl_guard() so that
        # every state inherits it - including the DWA fallback and the recovery
        # manoeuvres. Turning hard at speed scrubs the driven wheels and the
        # reaction torque rolls the robot over; run 31 reached 32 deg of roll
        # while executing (v=0.62, w=2.40) (FINDINGS 5.42).
        w_cap = WMAX * max(0.0, 1.0 - abs(forward) / TURN_SPEED_RATIO_V)
        turn = clamp(turn, -w_cap, w_cap)

        self.cmd_v = forward
        self.cmd_w = turn
        forward *= self.traction
        turn *= self.traction
        v_left = forward - turn * TRACK * 0.5
        v_right = forward + turn * TRACK * 0.5

        step = WHEEL_ACCEL * self.dt
        self.cmd_left += clamp(v_left - self.cmd_left, -step, step)
        self.cmd_right += clamp(v_right - self.cmd_right, -step, step)

        self.left_motor.setVelocity(clamp(self.cmd_left / WHEEL_RADIUS, -30, 30))
        self.right_motor.setVelocity(clamp(self.cmd_right / WHEEL_RADIUS, -30, 30))

    def _exp_sample(self, ball, action):
        """One buffer row: goal position in the robot frame plus the action.

        `ball` is the target ball when collecting. When it is None the goal is
        whatever the robot is currently driving toward - the drop zone in
        TO_DROP, or the target ball during plain navigation - so navigation
        transitions land in the same compact schema as intake ones and the
        offline trainer needs no special case.
        """
        gx = gy = None
        if ball is not None:
            gx, gy = ball['pos'][0], ball['pos'][1]
        elif self.bag > 0:
            gx, gy = self.drop_x, self.drop_y
        elif self.target is not None:
            gx, gy = self.target['pos'][0], self.target['pos'][1]
        if gx is None:
            lx = ly = 0.0
        else:
            lx, ly, _ = self.to_robot(gx, gy)
        return {
            'x': round(lx, 4),
            'y': round(ly, 4),
            'rng': round(math.hypot(lx, ly), 4),
            'v': round(action[0], 4),
            'w': round(action[1], 4),
            'bag': self.bag,
            'clr': round(self.min_clear, 3),
            'mode': self.rl_mode(),
        }

    def _exp_close(self, a, b):
        return (abs(a['x'] - b['x']) < EXP_MIN_DELTA and
                abs(a['y'] - b['y']) < EXP_MIN_DELTA and
                a['v'] == b['v'] and a['w'] == b['w'])

    def _exp_write(self, state, nxt, reward, done):
        rec = dict(state)
        rec['nx'] = nxt['x']
        rec['ny'] = nxt['y']
        rec['reward'] = reward
        rec['done'] = bool(done)
        try:
            self.exp_handle.write(json.dumps(rec, separators=(',', ':')) + '\n')
            self.exp_written += 1
        except OSError:
            self.exp_handle = None

    def record_experience(self, ball, action, reward, done):
        """Append one optimized intake transition (s, a, r, s') to the buffer.

        The buffer is not a per-tick dump. It stores **transitions**, not
        samples, and applies three reductions so a learned policy is not
        trained on thousands of near-identical rows (FINDINGS.md 5.26):

        * **decimation** - routine steps are kept at most one in
          `EXP_DECIMATE`, so the effective control rate matches the decision
          rate rather than the 31 Hz simulation;
        * **deduplication** - a step whose state and action barely changed from
          the pending transition is dropped (`EXP_MIN_DELTA`);
        * **event priority** - any step with a non-zero reward (capture or
          failure) is always kept, so rare outcomes are never decimated away.

        Each record is compact: current state `(x, y, rng, v, w)`, next state
        `(nx, ny)`, reward and done. The pending sample is only written once the
        next accepted sample fixes its successor, so no state is duplicated.

        `ball` may be None: navigation steps (SEEK, TO_DROP) are logged too, and
        they are the bulk of what the policy needs to learn. Logging only the
        intake steps left the buffer with 281 rows, all at 0.40-0.77 m and all
        within 0.06-0.16 m/s, so the policy had no navigation experience at all
        and every scenario it was asked about collapsed to "hold still"
        (FINDINGS 5.36). When `ball` is None the goal is whichever point the
        robot is currently pursuing - the target ball or the drop zone.
        """
        if self.exp_handle is None or self.exp_full:
            return

        sample = self._exp_sample(ball, action)
        if done or reward != 0.0:
            if self.exp_pending is not None:
                self._exp_write(self.exp_pending, sample, reward, done)
            self.exp_pending = None if done else sample
            return

        self.exp_step += 1
        if self.exp_step % EXP_DECIMATE != 0:
            self.exp_skipped += 1
            return
        if self.exp_pending is not None and self._exp_close(self.exp_pending,
                                                            sample):
            self.exp_skipped += 1
            return
        if self.exp_pending is not None:
            self._exp_write(self.exp_pending, sample, 0.0, False)
        self.exp_pending = sample

    def set_roller(self, speed):
        """Legacy hook retained for a future powered intake; no-op now."""
        if self.roller is None:
            return
        self.roller.setVelocity(speed)
        self.roller_cmd = speed

    def set_door_rollers(self, speed):
        """Spin the two horizontal roller arms so they carry balls inward.

        The rollers form a V: the left axis points forward-left, the right
        forward-right. Opposing signs make both inner surfaces travel the same
        way (toward the door), which draws a ball caught between them in
        through the door.
        """
        if self.arm_roller_left is not None:
            self.arm_roller_left.setVelocity(speed)
        if self.arm_roller_right is not None:
            self.arm_roller_right.setVelocity(-speed)

    def stop(self):
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        if self.roller is not None:
            self.roller.setVelocity(0.0)
        self.cmd_left = 0.0
        self.cmd_right = 0.0
        self.roller_cmd = 0.0

    # ------------------------------------------------------------------
    # odometry + slip
    # ------------------------------------------------------------------

    def update_odometry(self, dt):
        left = self.left_enc.getValue()
        right = self.right_enc.getValue()
        if left != left or right != right:
            return
        dl = (left - self.last_left) * WHEEL_RADIUS
        dr = (right - self.last_right) * WHEEL_RADIUS
        self.last_left, self.last_right = left, right
        self.odometer_l = left
        self.odometer_r = right
        travel = 0.5 * (dl + dr)
        self.odo_speed = travel / dt

        gps = self.gps.getValues()
        if gps[0] == gps[0]:
            # bearing of the GPS-versus-odometry error: a drift diagnostic,
            # never a heading.
            ex = gps[0] - self.pose_x
            ey = gps[1] - self.pose_y
            if abs(ex) + abs(ey) > 1e-6:
                self.gps_error_bearing = math.atan2(ey, ex)
        self.update_slip(dt, travel)

    def update_slip(self, dt, odometry_travel):
        gps = self.gps.getValues()
        dx = gps[0] - self.last_gps[0]
        dy = gps[1] - self.last_gps[1]
        self.last_gps = list(gps)
        if dx == dx and dy == dy:
            self.slip_gps += math.hypot(dx, dy)
        self.slip_odo += abs(odometry_travel)
        self.slip_time += dt
        if self.slip_time >= SLIP_WINDOW:
            if self.slip_odo > 0.05 and self.slip_gps < self.slip_odo:
                measured = clamp(1.0 - self.slip_gps / self.slip_odo, 0.0, 0.9)
                self.slip += SLIP_ALPHA * (measured - self.slip)
            elif self.slip_odo > 0.05:
                self.slip += SLIP_ALPHA * (0.0 - self.slip)
            self.slip_gps = 0.0
            self.slip_odo = 0.0
            self.slip_time = 0.0
        self.traction = clamp(1.0 - 1.4 * self.slip, 0.35, 1.0)

    # ------------------------------------------------------------------
    # attitude
    # ------------------------------------------------------------------

    def monitor_attitude(self):
        pos = self.me.getPosition()
        m = rotation_matrix(self.me)
        self.height = pos[2]
        if self.height < self.min_height:
            self.min_height = self.height
        zx, zy, zz = m[2], m[5], m[8]
        self.pitch = math.atan2(-zx, math.sqrt(zy * zy + zz * zz))
        self.roll = math.atan2(zy, zz)
        self.tilt = math.acos(clamp(zz, -1.0, 1.0))
        if self.tilt > self.peak_tilt:
            self.peak_tilt = self.tilt

        self.com = self.chassis.getCenterOfMass() if self.chassis else pos
        cx = self.com[0] - pos[0]
        cy = self.com[1] - pos[1]
        lx = m[0] * cx + m[3] * cy + m[6] * self.com[2]
        ly = m[1] * cx + m[4] * cy + m[7] * self.com[2]
        self.balance_margin = min(SUPPORT_X[1] - lx, lx - SUPPORT_X[0],
                                  SUPPORT_Y[1] - ly, ly - SUPPORT_Y[0])
        self.balanced = self.balance_margin > 0.0
        self.unbalanced_streak = 0 if self.balanced else self.unbalanced_streak + 1

    # ------------------------------------------------------------------
    # ball helpers
    # ------------------------------------------------------------------

    def reachable(self, ball):
        """True if a ball is inside the robot's allowed region (net side).

        The net divides the court. A ball on the far side can never be
        collected, so chasing it only drives the robot into the net.
        """
        xmin, xmax, ymin, ymax = self.bounds
        m = SEARCH_BALL_MARGIN
        return (xmin - m <= ball['pos'][0] <= xmax + m and
                ymin - m <= ball['pos'][1] <= ymax + m)

    def choose_target(self):
        candidates = [b for b in self.loose_balls() if self.reachable(b)]
        if not candidates:
            return None
        available = [b for b in candidates if b['avoid_until'] <= self.sim_time]
        if not available:
            for b in candidates:
                b['avoid_until'] = 0.0
            available = candidates
        best, best_cost = None, INF
        for ball in available:
            lx, ly, _ = self.to_robot(*ball['pos'])
            cost = math.hypot(lx, ly) + (1.2 if lx < 0.2 else 0.0)
            if cost < best_cost:
                best, best_cost = ball, cost
        return best

    def search_waypoints(self, phase):
        """Frontier targets for half `phase` (0 = near/first, 1 = far).

        Yamauchi frontier exploration: a frontier is an observed-free,
        traversable cell that borders space the lidar has never seen. The robot
        drives to the nearest frontier in the current half; when the half has
        no frontier left it moves on to the second half. This replaces a blind
        lawnmower and uses the accumulated map, so the robot searches where it
        actually has not looked.
        """
        xmin, xmax, ymin, ymax = self.bounds
        m = SEARCH_MARGIN
        mid = 0.5 * (xmin + xmax)
        if phase == 0:
            lo_x, hi_x = xmin + m, mid
        else:
            lo_x, hi_x = mid, xmax - m
        if hi_x < lo_x:
            return []
        y0, y1 = ymin + m, ymax - m
        frontiers = []
        for cell in self.known_free:
            wx, wy = self.cell_center(cell)
            if not (lo_x <= wx <= hi_x and y0 <= wy <= y1):
                continue
            if cell in self.known_occ or self.clearance(wx, wy) < ROBOT_RADIUS:
                continue
            cx, cy = cell
            border = False
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (cx + dx, cy + dy)
                if nb not in self.known_free and nb not in self.known_occ:
                    border = True
                    break
            if border:
                frontiers.append((wx, wy))
        frontiers.sort(key=lambda p: math.hypot(p[0] - self.pose_x,
                                                p[1] - self.pose_y))
        return frontiers[:1]

    def run_search(self):
        if self.choose_target() is not None:
            self.state = 'SEEK'
            self.state_time = 0.0
            return
        frontiers = self.search_waypoints(self.search_phase)
        if not frontiers:
            self.search_phase += 1
            if self.search_phase > 1:
                self.state = 'TO_DROP'
                self.state_time = 0.0
            return
        wx, wy = frontiers[0]
        self.seek_step((wx, wy))

    def ball_in_cone(self, distance, half_angle):
        best, best_d = None, INF
        for ball in self.loose_balls():
            if ball['avoid_until'] > self.sim_time:
                continue
            lx, ly, _ = self.to_robot(*ball['pos'])
            if lx <= 0.0:
                continue
            bearing = abs(math.atan2(ly, lx))
            if bearing > half_angle or lx > distance:
                continue
            d = math.hypot(lx, ly)
            if d < best_d:
                best, best_d = ball, d
        return best

    def balls_under_body(self):
        found = []
        for ball in self.loose_balls():
            lx, ly, _ = self.to_robot(*ball['pos'])
            if UNDERBODY_X[0] <= lx < UNDERBODY_X[1] and abs(ly) < UNDERBODY_Y:
                found.append((ball, lx, ly))
        return found

    def next_state_after_pickup(self):
        """TO_DROP once the hopper is full, otherwise straight back to SEEK.

        The hopper has a real capacity - the bin walls are 0.20 m across and a
        ball is 0.067 m, so a handful will not fit. Previously the robot only
        unloaded when every loose ball had been collected, which meant it
        tried to hold all ten, never entered TO_DROP while balls remained, and
        so produced no MODE_DROP experience at all: all 1175 buffer rows were
        SEEK and the learned policy had no data for the loaded phase
        (FINDINGS 5.40). Delivering in batches is also what the task asks for -
        gather balls, then stop at the drop zone - rather than emptying the
        whole court in one trip.
        """
        if self.bag >= BAG_CAPACITY:
            return 'TO_DROP'
        if not self.loose_balls():
            return 'TO_DROP'
        return 'SEEK'

    def clamp_to_bounds(self, x, y):
        xmin, xmax, ymin, ymax = self.bounds
        return clamp(x, xmin, xmax), clamp(y, ymin, ymax)

    # ------------------------------------------------------------------
    # learned policy
    # ------------------------------------------------------------------

    def rl_mode(self):
        if self.bag > 0 and (self.state == 'TO_DROP' or self.target is None):
            return rl.MODE_DROP
        if self.target is not None:
            return rl.MODE_SEEK
        return rl.MODE_IDLE

    def rl_observation(self):
        """Build the observation the policy scores actions against.

        Everything here is already known to the controller, so the policy needs
        no new sensors. Distance and bearing are to whichever goal the robot is
        currently pursuing - the target ball while collecting, the drop zone
        while carrying - which is what lets one model cover both halves of the
        task.
        """
        gx = gy = None
        mode = self.rl_mode()
        if mode == rl.MODE_SEEK and self.target is not None:
            gx, gy = self.target['pos'][0], self.target['pos'][1]
        elif mode == rl.MODE_DROP:
            gx, gy = self.drop_x, self.drop_y
        if gx is None:
            dist, bearing = 0.0, 0.0
        else:
            lx, ly, _ = self.to_robot(gx, gy)
            dist = math.hypot(lx, ly)
            bearing = math.atan2(ly, lx) if dist > 1e-6 else 0.0
        slip = 0 if self.slip < 0.08 else (1 if self.slip < 0.25 else 2)
        return {
            'dist': dist,
            'bearing': bearing,
            'clearance': self.min_clear,
            'slip': slip,
            'bag': self.bag,
            'stuck': self.stuck_time > 1.5,
            'escape': self.state in ('RECOVER', 'ESCAPE', 'WALL_BACK'),
            'mode': mode,
        }

    def rl_ready(self, mode=None):
        """Whether the policy may drive the current mode.

        Trust is per mode. A buffer full of SEEK transitions says nothing about
        carrying a ball to the drop zone, and an untrained mode has all-zero
        weights, so every action ties and the tie-break picks "hold still" -
        the robot would simply stop while loaded. So each mode is gated on its
        own update count, and DWA drives whatever the policy has not yet earned
        the right to control.
        """
        if not RL_ENABLED:
            return False
        m = self.rl_mode() if mode is None else mode
        return self.policy.mode_is_trained(m, RL_MIN_UPDATES)

    def rl_guard(self, v, w):
        """Veto any learned action the existing safety layer would refuse.

        The policy may choose *when* to reverse or pivot; it may not choose to
        ignore the lidar. Forward speed is capped by the same safe_speed() the
        DWA planner uses, turn rate by WMAX, and a wall-contact latch forces a
        stop regardless of what was learned.
        """
        if self.wall_contact and v > 0.0:
            self.rl_safety_overrides += 1
            v = 0.0
        if v > 0.0:
            cap = self.safe_speed(self.min_clear)
            if v > cap:
                self.rl_safety_overrides += 1
                v = cap
        w = clamp(w, -WMAX, WMAX)
        # A differential drive cannot turn hard at full forward speed without
        # the driven wheel scrubbing sideways; the reaction torque rolls the
        # robot over. Run 31 tipped at 32 deg roll while executing exactly
        # (v=0.62, w=2.40) (FINDINGS 5.42). The limit is a straight line
        # through the reachable set, so it constrains the aggressive corner of
        # the lattice without affecting slow manoeuvres.
        w_cap = WMAX * max(0.0, 1.0 - abs(v) / TURN_SPEED_RATIO_V)
        if abs(w) > w_cap:
            self.rl_safety_overrides += 1
            w = math.copysign(w_cap, w)
        return v, w

    def rl_act(self, obs):
        """Choose and apply one action, then record it for the next update."""
        action, explored = self.policy.choose(obs, self.rng,
                                              self.rl_ready(obs['mode']))
        if explored:
            self.rl_explore += 1
        else:
            self.rl_greedy += 1
        v, w = rl.ACTIONS[action]
        v, w = self.rl_guard(v, w)
        # close the transition that the *previous* step opened
        self.rl_close(obs)
        self.rl_prev_obs = obs
        self.rl_prev_action = action
        self.drive(v, w)
        # Log the action as it was actually applied, not as the policy chose
        # it. drive() may clamp the turn rate for stability, so the chosen
        # (v, w) is not always the executed one; training on the unexecuted
        # command teaches the policy to prefer manoeuvres the stability limit
        # then cancels out (FINDINGS 5.43).
        self.record_experience(self.target, (self.cmd_v, self.cmd_w),
                               0.0, False)
        return v, w

    def rl_close(self, next_obs):
        """Apply the pending TD update for the step just taken."""
        if self.rl_prev_obs is None or self.rl_prev_action is None:
            return
        prev = self.rl_prev_obs
        event = self.rl_last_event
        self.rl_last_event = None
        reward = rl.shaped_reward(prev, next_obs, event)
        self.rl_reward_acc += reward
        if self.rl_steps >= RL_MIN_TRAIN_STEPS:
            self.rl_td_last = self.policy.update(prev, self.rl_prev_action,
                                                 reward, next_obs, False)
        self.rl_prev_obs = None
        self.rl_prev_action = None

    def rl_note(self, event):
        """Flag a reward event for the next close()."""
        if event is not None:
            self.rl_last_event = event

    def rl_tick(self):
        """Per-step housekeeping: decay exploration and checkpoint weights."""
        self.rl_steps += 1
        if self.rl_steps % RL_DECAY_EVERY == 0:
            self.policy.decay_epsilon()
        if self.rl_steps % RL_SAVE_EVERY == 0:
            self.policy.save()

    def rl_save(self):
        self.policy.save()

    # ------------------------------------------------------------------
    # state machine
    # ------------------------------------------------------------------

    def seek_step(self, goal_world):
        """One planning+control step toward a world goal. Returns (v, w)."""
        gx, gy = self.clamp_to_bounds(goal_world[0], goal_world[1])
        self.maybe_replan((gx, gy))
        dx = gx - self.pose_x
        dy = gy - self.pose_y
        dist = math.hypot(dx, dy)
        goal_dir = wrap_angle(math.atan2(dy, dx) - self.pose_h)
        self.heading_error = goal_dir
        obs = self.rl_observation()
        if self.rl_ready():
            # the learned policy drives; A* still supplies the path and the
            # guard still caps what the wheels may do
            v, w = self.rl_act(obs)
        else:
            # Bootstrap. DWA drives, but the step is still recorded as a
            # transition so the policy learns from it. Without this the policy
            # would never reach RL_MIN_UPDATES, because updates are only applied
            # to a pending transition that rl_act() opens - so gating the policy
            # on its own update count would deadlock it permanently at zero.
            v, w = self.dwa(goal_dir, dist)
            v, w = self.rl_guard(v, w)
            self.rl_close(obs)
            self.rl_prev_obs = obs
            self.rl_prev_action = self.nearest_action(v, w)
            self.drive(v, w)
            # log the executed command, for the same reason as in rl_act()
            self.record_experience(self.target, (self.cmd_v, self.cmd_w),
                                   0.0, False)
        return v, w

    def nearest_action(self, v, w):
        """Lattice index closest to a continuous (v, w)."""
        best, best_err = 0, None
        for i, (av, aw) in enumerate(rl.ACTIONS):
            err = (av - v) ** 2 + (aw - w) ** 2
            if best_err is None or err < best_err:
                best, best_err = i, err
        return best

    def run_seek(self):
        if self.target is not None and not self.reachable(self.target):
            # the target was knocked or found across the net; stop chasing it
            self.target = None
            self.path = []
        if (self.target is not None and
                self.target['avoid_until'] > self.sim_time):
            # A ban only ever affected re-selection, so a ball the robot was
            # already chasing kept its slot in self.target and the ban was
            # silently ignored. That is why the stuck escalation stopped firing
            # after two attempts and the robot spent 45 s pinned in the west
            # corner chasing a ball it had already written off
            # (FINDINGS 5.31). The current target must be released too.
            self.event('releasing banned ball %d from the current target'
                       % self.target['id'])
            self.target = None
            self.path = []
            self.last_plan = -10.0

        under = self.balls_under_body()
        if under:
            if self.sim_time < self.escape_until and under[0][0]['id'] == self.escape_ball:
                # just finished escaping this ball; do not immediately re-enter
                self.escape_until = self.sim_time
            else:
                if self.sim_time - self.underbody_reported > 2.0:
                    self.underbody_reported = self.sim_time
                    self.event('ball under the chassis at x=%.2f y=%+.2f '
                               '- backing off' % (under[0][1], under[0][2]))
                under[0][0]['avoid_until'] = self.sim_time + ESCAPE_AVOID
                self.target = None
                self.escape_time = 0.0
                self.escape_ball = under[0][0]['id']
                self.state = 'ESCAPE'
                self.state_time = 0.0
                return

        close = self.ball_in_cone(SEEK_BRAKE_DISTANCE, SEEK_BRAKE_ANGLE)
        if close is not None and self.reachable(close):
            self.target = close
            self.state = 'ALIGN'
            self.state_time = 0.0
            return

        if self.target is None or self.target['in_hopper']:
            self.target = self.choose_target()
            self.state_time = 0.0
            self.path = []
        if self.target is None:
            if self.loose_balls():
                # balls remain but none is reachable; sweep the region
                self.state = 'SEARCH'
                self.state_time = 0.0
            else:
                self.state = 'TO_DROP'
            return

        self.seek_step(self.target['pos'])

        lx, ly, _ = self.to_robot(*self.target['pos'])
        if (math.hypot(lx, ly) < SEEK_BRAKE_DISTANCE and
                abs(math.atan2(ly, lx)) < SEEK_BRAKE_ANGLE):
            self.state = 'ALIGN'
            self.state_time = 0.0

    def align_to_ball(self, ball):
        """Final approach in the robot frame, so it self-corrects.

        Centring comes FIRST. Driving forward while the ball is off to one side
        is what used to shove the ball along the floor with the funnel guide
        until the robot reached the net. The robot turns on the spot until the
        ball is dead ahead, and only then closes the gap. The ball is then
        centred on the roller and cannot be pushed sideways.
        """
        lx, ly, _ = self.to_robot(*ball['pos'])
        bearing = math.atan2(ly, lx) if lx > 1e-6 else 1.0

        if lx < 0.40:
            # ball is already at the mouth but was not drawn in: do not keep
            # pushing it under the body, back off and retry.
            self.drive(-CREEP_SPEED, 0.0)
            return False

        centered = abs(ly) < ALIGN_Y_TOL and abs(bearing) < ALIGN_BEARING_TOL
        if not centered:
            # Turn on the spot to put the ball dead ahead BEFORE closing in.
            # The old code crept straight forward whenever lx < 0.75 "so the
            # roller arms steer it in" -- but there are no roller arms, so the
            # robot sailed straight past every off-centre ball until it lodged
            # by a front wheel.
            #
            # A plain proportional turn is not enough either: there is a
            # static-friction deadzone in yaw (a large heading error turns at
            # ~10 deg/s, but a small w command -- 0.2 rad/s -- produces no
            # rotation at all, so the robot could never null out the last few
            # degrees and settle the ball on centre). Command at least
            # ALIGN_W_MIN in the needed direction to break static friction,
            # bang-bang, until the ball is within tolerance.
            w = clamp(2.5 * bearing, -ALIGN_WMAX, ALIGN_WMAX)
            if abs(w) < ALIGN_W_MIN:
                w = math.copysign(ALIGN_W_MIN, w)
            # Pure pivot (no forward) so the turn is sustained and the robot
            # does not orbit the ball; the deadzone below (centred) stops it.
            self.drive(0.0, w)
            return False

        forward = clamp(0.6 * (lx - 0.62), 0.0, 0.14)
        self.drive(forward, clamp(0.8 * bearing, -0.4, 0.4))
        return lx <= ALIGN_X_MAX

    def run_align(self):
        if self.target is None or self.target['in_hopper']:
            self.state = 'SEEK'
            return
        # ALIGN has no natural exit if the ball cannot be centred: the robot
        # creeps at the door rollers, the ball shifts sideways just enough to
        # stay outside ALIGN_Y_TOL, and the state repeats forever. In run 30
        # that held the robot for 121 s of the 150 s run. A timeout is the
        # honest bound on an approach that is not converging.
        self.state_time += self.dt
        if self.state_time > ALIGN_TIMEOUT:
            self.abort_pickup('could not centre the ball in %.1fs' %
                              ALIGN_TIMEOUT)
            return
        if self.align_to_ball(self.target):
            self.state = 'COLLECT'
            self.state_time = 0.0

    def run_collect(self):
        """Close the gap to the ball and let the door rollers draw it in.

        The ball was centred by ALIGN. The robot advances with the two door
        rollers spinning; when the ball reaches the door the rollers grip it
        and carry it in behind them, and it cannot roll back out while they
        spin. Collection is confirmed by the ball's supervisor position
        entering the bin volume.
        """
        if self.target is None or self.target['in_hopper']:
            self.state = 'SEEK'
            return

        lx, ly, _ = self.to_robot(*self.target['pos'])
        self.state_time += self.dt

        # Re-align only if the ball is still well ahead AND badly off-centre.
        # The old test (abs(ly) > 0.035) bounced to ALIGN at an offset ALIGN
        # itself hands over at, so the two states oscillated every few steps.
        # That oscillation is what actually crippled turning: each turn command
        # was cancelled by a forward command before the wheel-acceleration ramp
        # could establish the pivot (the drivetrain turns 40-100 deg/s when a
        # turn is *sustained*, near 0 when it is interrupted). Hysteresis plus a
        # lx gate lets COLLECT commit: once the ball is at the mouth the funnel
        # and roller finish the centring instead of bouncing back to ALIGN.
        if lx > 0.55 and abs(ly) > 0.10:
            self.state = 'ALIGN'
            self.state_time = 0.0
            return

        # spin the door rollers so they draw the ball inward
        self.set_door_rollers(DOOR_ROLLER_SPEED)

        # drive up to the door, then keep a gentle push so the spinning rollers
        # stay engaged with the ball while they draw it through. Do not ease
        # off until the ball is inside the nip (x < 0.44); easing at 0.50 left
        # it resting just outside the grip.
        turn = clamp(0.9 * math.atan2(ly, max(lx, 0.05)), -0.5, 0.5)
        speed = 0.16 if lx > 0.50 else 0.06
        self.drive(speed, turn)

        # per-step transition with zero reward; the terminal +1 is written by
        # update_ball_states when the ball actually enters the bin. The logged
        # command is the executed one, after drive()'s rate and stability
        # limits, not the requested one.
        self.record_experience(self.target, (self.cmd_v, self.cmd_w),
                               0.0, False)

        if os.environ.get('TENNIS_DEBUG_COLLECT'):
            print('  collect: ball x=%+.3f y=%+.3f  rollerL=%.1f rollerR=%.1f'
                  % (lx, ly, DOOR_ROLLER_SPEED, -DOOR_ROLLER_SPEED))

        # the ball can only get stuck if it slips underneath the deck
        if lx < COLLECT_ABORT_X:
            self.abort_pickup('ball ended under the deck at x=%.2f' % lx)

        if self.state == 'COLLECT' and self.state_time > COLLECT_TIMEOUT:
            self.abort_pickup('ball not collected after %.1fs (x=%.2f)'
                              % (self.state_time, lx))

    def abort_pickup(self, reason):
        self.pickup_attempts += 1
        lx, ly = (0.0, 0.0)
        if self.target is not None:
            lx, ly, _ = self.to_robot(*self.target['pos'])
        self.event('pickup failed: %s (attempt %d, ball x=%.2f y=%+.2f, '
                   'tilt %.0f deg, bag %d)'
                   % (reason, self.pickup_attempts, lx, ly,
                      math.degrees(self.tilt), self.bag))
        self.set_door_rollers(0.0)
        self.jam_time = 0.0
        self.last_jam_x = None
        if self.target is not None:
            self.record_experience(self.target, (0.0, 0.0), -1.0, True)
            self.target['avoid_until'] = self.sim_time + 6.0
        self.target = None
        self.path = []
        self.enter_recover()

    def enter_recover(self):
        """Single entry point for RECOVER so its timer always starts clean."""
        self.state = 'RECOVER'
        self.state_time = 0.0
        self.recover_time = 0.0

    def ball_by_id(self, bid):
        for ball in self.balls:
            if ball['id'] == bid:
                return ball
        return None

    def run_escape(self):
        """Reverse until the ball that rolled under the deck is behind it.

        RECOVER's fixed 1.2 s was not enough: the ball stayed pinned against
        the deck and SEEK immediately drove forward into it again. ESCAPE exits
        on the measured condition (the offending ball is now well behind the
        chassis) rather than on a timer, with ESCAPE_TIME only as a cap.
        """
        self.escape_time += self.dt
        ball = self.ball_by_id(self.escape_ball)
        clear = True
        if ball is not None and not ball['in_hopper']:
            lx, ly, _ = self.to_robot(*ball['pos'])
            # ESCAPE reverses, so the ball travels toward negative lx: "clear"
            # means the ball has ended up behind the deck, not in front of it.
            # Testing lx > ESCAPE_CLEAR_X could never succeed while the robot
            # was reversing, so ESCAPE always ran to its 4 s cap and re-entered
            # on the next step, which is how it grew to 46 % of the run
            # (FINDINGS 5.32).
            clear = lx < -ESCAPE_CLEAR_X or abs(ly) > (UNDERBODY_Y - 0.03)
        if clear or self.escape_time > ESCAPE_TIME:
            if self.escape_time >= ESCAPE_TIME and not clear:
                # ran out of time without actually clearing the deck
                self.rl_note('escape_fail')
            self.event('escaped the under-body ball after %.1fs'
                       % self.escape_time)
            if ball is not None and not ball['in_hopper']:
                ball['avoid_until'] = self.sim_time + ESCAPE_AVOID
            self.escape_time = 0.0
            self.escape_until = self.sim_time + ESCAPE_COOLDOWN
            self.path = []
            self.target = None
            self.state = 'SEEK'
            self.state_time = 0.0
            return
        # Free the wedged ball. Reversing slowly barely turns (yaw authority is
        # near zero at v<0), so the old reverse-and-turn left the ball pinned by
        # a front wheel and ESCAPE looped to its cap, re-entering every step
        # (31 % of a run). Instead alternate a hard in-place spin -- which the
        # drivetrain CAN do when sustained, and which sweeps the body past the
        # trapped ball and rotates it out from under -- with a straight reverse
        # that then leaves it behind. Spin away from the ball's side.
        if ball is None:
            self.drive(-BACK_SPEED, 0.0)
        else:
            lx, ly, _ = self.to_robot(*ball['pos'])
            side = -1.0 if ly >= 0.0 else 1.0   # rotate the trapped side back
            phase = int(self.escape_time / 0.7) % 2
            if phase == 0:
                self.drive(0.0, side * ALIGN_WMAX)     # spin to dislodge
            else:
                self.drive(-BACK_SPEED, 0.0)           # reverse to leave behind
        self.set_door_rollers(0.0)

    def run_recover(self):
        # back off and turn toward open space, longer than a token nudge so a
        # wedged robot (e.g. arms under the net) actually breaks free
        self.recover_time += self.dt
        clear_l = self.clear_left()
        clear_r = self.clear_right()
        side = 1.0 if clear_l >= clear_r else -1.0
        self.drive(-BACK_SPEED, 0.8 * side)
        if self.recover_time > RECOVER_TIME:
            self.recover_time = 0.0
            self.state = 'SEEK'
            self.state_time = 0.0

    def run_to_drop(self):
        self.target = None
        self.set_door_rollers(0.0)
        self.seek_step((self.drop_x - 0.55, self.drop_y))
        err = wrap_angle(math.atan2(self.drop_y - self.pose_y,
                                    self.drop_x - self.pose_x) - self.pose_h)
        dist = math.hypot(self.drop_x - self.pose_x,
                          self.drop_y - self.pose_y)
        if dist < 0.75 and abs(err) < 0.12:
            # Loaded, in the zone and lined up: holding still here is the goal
            # the task asks for ("gather the balls first, then stop at the drop
            # zone"), so it earns a reward rather than the per-step time cost.
            if self.state_time >= 1.0:
                self.rl_note('stopped_in_zone')
            self.state = 'DUMP'
            self.state_time = 0.0

    def run_dump(self):
        """Unload procedure to deliver collected balls into the drop zone."""
        self.stop()
        # Open unload door lid motor and reverse rollers to execute physical ejection
        if self.gate is not None:
            self.gate.setPosition(1.45)
        self.set_door_rollers(-20.0)

        # Allow 1.5 seconds for delivery procedure motion
        if self.state_time < 1.5:
            return

        in_bin = [b for b in self.balls if b['in_hopper']]
        n = len(in_bin)
        # Deliver onto the centre of the low-traction drop pad
        for i, ball in enumerate(in_bin):
            gx = self.drop_x + 0.18 * ((i % 3) - 1)
            gy = self.drop_y + 0.18 * ((i // 3) - 1)
            ball['node'].setPose([gx, gy, 0.0335, 0, 0, 1, 0])
            ball['in_hopper'] = False
            ball['held'] = 0
        self.carrying = 0
        if n:
            self.rl_note('deliver')
            self.event('dumped %d ball(s) into the drop zone' % n)
        self.delivered = self.count_delivered()
        self.set_door_rollers(0.0)
        if self.gate is not None:
            self.gate.setPosition(0.0)

        remaining = self.loose_balls()
        if remaining:
            self.event('delivery complete (%d total delivered, %d loose remaining); resuming SEEK' %
                       (self.delivered, len(remaining)))
            self.target = None
            self.path = []
            self.state = 'SEEK'
        else:
            self.event('all balls collected and delivered! (%d total)' % self.delivered)
            self.state = 'DONE'
        self.state_time = 0.0

    def run_done(self):
        self.stop()
        self.done_time += self.dt

    # ------------------------------------------------------------------
    # recovery
    # ------------------------------------------------------------------

    def self_right(self):
        self.self_rights += 1
        x, y, yaw = self.pose_x, self.pose_y, self.pose_h
        # A violent flip can leave the GPS reporting a pose that is not on the
        # court - in run 27 it read z = 11.7 m after the robot was flung. Placing
        # the parts at 0.02 + lz and calling resetPhysics() at that location
        # teleports the robot there and re-seats it in mid-air, so it fell again
        # immediately and burned every self-right in a fraction of a second
        # (FINDINGS 5.37). Refuse to rebuild from an implausible pose and let
        # the normal recovery try again instead.
        xmin, xmax, ymin, ymax = self.bounds
        pose_ok = (xmin - 1.0 <= x <= xmax + 1.0 and
                   ymin - 1.0 <= y <= ymax + 1.0 and
                   abs(self.height) <= SELF_RIGHT_MAX_HEIGHT)
        if not pose_ok:
            self.self_rights -= 1
            self.event('self-right refused: implausible pose (%.2f, %.2f, '
                       'z=%.2f)' % (x, y, self.height))
            return self.state_before_fall
        c, s = math.cos(yaw), math.sin(yaw)
        for part, (lx, ly, lz) in NOMINAL_PARTS.items():
            node = self.body.get(part)
            if node is None:
                continue
            node.setPose([x + lx * c - ly * s, y + lx * s + ly * c,
                          0.02 + lz])
        try:
            self.getSelf().resetPhysics()
        except Exception:
            pass
        self.cmd_left = self.cmd_right = self.roller_cmd = 0.0
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        if self.roller is not None:
            self.roller.setVelocity(0.0)
        if self.gate is not None:
            self.gate.setPosition(0.0)
        self.last_left = self.left_enc.getValue()
        self.last_right = self.right_enc.getValue()
        self.read_pose()
        self.fall_timer = 0.0
        self.slip = 0.0
        self.traction = 1.0
        self.tilt_warned = False
        self.path = []
        self.target = None
        self.jam_time = 0.0
        self.last_jam_x = None
        self.right_cooldown = 1.0
        self.event('self-right %d: rebuilt upright at (%.2f, %.2f)'
                   % (self.self_rights, x, y))
        return 'SEEK'

    def run_down(self):
        self.fall_timer += self.dt
        self.stop()
        if self.tilt <= 0.5 * TILT_WARN:
            self.event('upright again after %.1fs down' % self.fall_timer)
            self.fall_timer = 0.0
            self.state = self.state_before_fall
            self.tilt_warned = False
            return
        if self.fall_timer < FALL_RECOVER_TIME:
            self.drive(0.12, 0.0)
        elif self.self_rights < FALL_MAX_ATTEMPTS:
            self.state = self.self_right()
        elif self.fall_timer > FALL_GIVE_UP:
            self.event('still down after %.0fs and %d self-rights - parking'
                       % (self.fall_timer, self.self_rights))
            self.state = 'DOWN'
            self.report()
            self.simulationQuit(1)
        else:
            self.drive(-0.08, 0.0)

    def check_attitude(self):
        if self.right_cooldown > 0.0:
            self.right_cooldown -= self.dt
        if self.tilt > TILT_FALLEN and self.right_cooldown <= 0.0:
            if self.state not in ('DOWN', 'RECOVER_DOWN'):
                self.state_before_fall = self.state
                self.fall_count += 1
                self.event('FALL %d: tilt %.1f deg (roll %.1f, pitch %.1f), '
                           'ride height %.3f m'
                           % (self.fall_count, math.degrees(self.tilt),
                              math.degrees(self.roll),
                              math.degrees(self.pitch), self.height))
                self.fall_timer = 0.0
                self.state = 'RECOVER_DOWN'
        elif self.tilt > TILT_WARN and not self.tilt_warned:
            self.tilt_warned = True
            self.event('tilt %.1f deg - leaning, ride height %.3f m'
                       % (math.degrees(self.tilt), self.height))
        elif self.tilt < 0.5 * TILT_WARN:
            self.tilt_warned = False

    def bumper_front_range(self):
        """Nearest thing in the frontal arc, or None when empty or unmounted."""
        if len(self.fans) < 3 or self.fans[2][0] is None:
            return None
        dev = self.fans[2][0]
        img = dev.getRangeImage()
        if not img:
            return None
        step = self.fans[2][4] / self.fans[2][5]
        best = INF
        saw_return = False
        for i, r in enumerate(img):
            if r != r or r < LIDAR_MIN_RANGE:
                continue
            a = -self.fans[2][4] / 2.0 + (i + 0.5) * step
            if abs(a) <= 0.5:
                saw_return = True
                if r < best:
                    best = r
        if not saw_return:
            # every frontal beam was swallowed by geometry at point-blank range
            return 0.0
        return best

    def check_walls(self):
        """Contact latch, only armed once the sensors have a valid frame.

        Previously the all-zero first scan latched contact and trapped the
        robot forever. Now the guard is inert until sensor_ready.
        """
        # Hard bounds guard first: the bounds are the robot's own allowed
        # region (the net side). Goals were clamped to it but the body was not,
        # so the robot could drive past xmax and wedge its arms under the net,
        # where it thrashed in RECOVER for ~90 s (FINDINGS 5.27). This is
        # independent of the lidar, so it works even when the net is not seen.
        xmin, xmax, ymin, ymax = self.bounds
        if (self.pose_x < xmin - BOUND_MARGIN or self.pose_x > xmax + BOUND_MARGIN
                or self.pose_y < ymin - BOUND_MARGIN
                or self.pose_y > ymax + BOUND_MARGIN):
            if self.state != 'WALL_BACK':
                self.rl_note('out_of_bounds')
                self.event('outside bounds (%.2f, %.2f) - returning'
                           % (self.pose_x, self.pose_y))
            self.wall_contact = True
            self.wall_warned = False
            # Only *start* the return timer on entry. Restarting it on every
            # step while still out of bounds meant the robot reversed in short
            # bursts, never covered the ~0.5 m needed to get back inside, and
            # re-triggered forever - 57 s pinned at the bounds edge in run 34
            # (FINDINGS 5.46). Now the timer runs to completion once.
            if self.state != 'WALL_BACK':
                self.wall_back_left = BOUND_BACK_TIME
                self.wall_back_grace = 0.0
            self.state = 'WALL_BACK'
            self.state_before_wall = 'SEEK'
            return

        if not self.sensor_ready:
            return
        # The bumper device is at the very front of the rig, so its frontal
        # range is the true gap to whatever the robot is about to touch. It is
        # checked directly, not only through the occupancy grid, so a thin
        # obstacle such as the net cannot slip between map rebuilds.
        front = self.bumper_front_range()
        self.front_range = front if front is not None else WINDOW
        buried = self.front_range <= LIDAR_MIN_RANGE
        self.min_clear = min(self.clearance(self.pose_x, self.pose_y),
                             self.front_range + 0.86 - ROBOT_RADIUS)
        if self.wall_contact:
            return
        if self.clear_left() < self.clear_right():
            self.wall_side = -1.0
        else:
            self.wall_side = 1.0
        if buried or self.min_clear <= ROBOT_RADIUS + CONTACT_MARGIN:
            self.wall_contact = True
            self.wall_hits += 1
            self.state_before_wall = self.state
            self.wall_back_left = WALL_BACK_TIME
            self.event('WALL contact %d: clearance %.3f m, backing off'
                       % (self.wall_hits, self.min_clear))
            self.state = 'WALL_BACK'
            return
        if self.min_clear <= ROBOT_RADIUS + WALL_STOP_DIST:
            if not self.wall_warned:
                self.wall_warned = True
                self.event('wall %.2f m, limiting speed to %.2f m/s'
                           % (self.min_clear, self.safe_speed(self.min_clear)))
        elif self.min_clear > ROBOT_RADIUS + WALL_STOP_DIST + 0.10:
            self.wall_warned = False

    def clear_left(self):
        best = WINDOW
        for a in (0.6, 1.2, 1.8, 2.4):
            lx = 0.30 + 1.2 * math.cos(a)
            ly = 1.2 * math.sin(a)
            best = min(best, self.clearance_robot(lx, ly))
        return best

    def clear_right(self):
        best = WINDOW
        for a in (0.6, 1.2, 1.8, 2.4):
            lx = 0.30 + 1.2 * math.cos(a)
            ly = -1.2 * math.sin(a)
            best = min(best, self.clearance_robot(lx, ly))
        return best

    def run_wall_back(self):
        self.wall_back_left -= self.dt
        # steer so the nose ends up pointing back toward the allowed region
        xmin, xmax, ymin, ymax = self.bounds
        cx = clamp(self.pose_x, xmin + 0.5, xmax - 0.5)
        cy = clamp(self.pose_y, ymin + 0.5, ymax - 0.5)
        err = wrap_angle(math.atan2(cy - self.pose_y, cx - self.pose_x)
                         - self.pose_h)
        # A timer alone is not enough: if the robot is still outside when the
        # timer runs out it would resume SEEK only to be pushed straight back
        # out again. So leaving WALL_BACK also requires actually being back
        # inside the bounds, with a short grace period to allow for the body
        # swinging back out (FINDINGS 5.46).
        inside = (xmin - BOUND_MARGIN <= self.pose_x <= xmax + BOUND_MARGIN and
                  ymin - BOUND_MARGIN <= self.pose_y <= ymax + BOUND_MARGIN)
        if inside:
            self.wall_back_grace += self.dt
        else:
            self.wall_back_grace = 0.0
        if self.wall_back_left <= 0.0 and self.wall_back_grace >= 0.3:
            self.wall_contact = False
            self.wall_warned = False
            self.path = []
            self.event('clear of the wall - resuming')
            self.state = 'SEEK'
            self.target = None
            self.state_time = 0.0
            return
        self.drive(-BACK_SPEED, clamp(1.2 * err, -WMAX, WMAX))

    # ------------------------------------------------------------------
    # logging
    # ------------------------------------------------------------------

    def surface_name(self):
        if self.tilt > TILT_WARN:
            return 'off its wheels'
        return 'hard court' if self.slip < 0.08 else 'dirt/clay (low grip)'

    def log(self):
        if self.target is not None:
            tgt = '%5.2f,%5.2f' % (self.target['pos'][0], self.target['pos'][1])
            lx, ly, _ = self.to_robot(*self.target['pos'])
            tgt_r = 'n%5.2f/%+5.2f' % (lx, ly)
        else:
            tgt = ' none      '
            tgt_r = 'n  none     '
        print('t=%6.1f  %-11s pose(%5.2f,%5.2f,%6.1f) z=%.3f roll %5.1f '
              'pitch %5.1f  clear %4.2f front %4.2f  v=%4.2f w=%+5.2f  '
              'wheel %5.2f/%5.2f  path %2d occ %4d  '
              'slip %4.1f%%  %-18s tgt(%s) %s  bag %d del %d/%d'
              % (self.sim_time, self.state, self.pose_x, self.pose_y,
                 math.degrees(self.pose_h), self.height,
                 math.degrees(self.roll), math.degrees(self.pitch),
                 self.min_clear, self.front_range, self.cmd_v, self.cmd_w,
                 self.left_motor.getVelocity(), self.right_motor.getVelocity(),
                 len(self.path), len(self.occ), 100.0 * self.slip,
                 self.surface_name(), tgt, tgt_r, self.bag,
                 self.count_delivered(), len(self.balls)))

    def report(self):
        print('\n=== summary ===')
        print('  simulated time      %.1f s' % self.sim_time)
        print('  surface detected    %s' % self.surface_name())
        print('  final slip          %.1f %%' % (100.0 * self.slip))
        print('  balls delivered     %d of %d'
              % (self.count_delivered(), len(self.balls)))
        print('  balls still loose   %d' % len(self.loose_balls()))
        print('  peak hopper load    %d balls' % self.bag_peak)
        print('  pickup attempts     %d failed' % self.pickup_attempts)
        print('  peak tilt           %.1f deg' % math.degrees(self.peak_tilt))
        print('  falls detected      %d' % self.fall_count)
        print('  self-rights used    %d' % self.self_rights)
        print('  wall contacts       %d' % self.wall_hits)
        print('  events logged       %d' % self.events)
        if self.exp_handle is not None:
            try:
                if self.exp_pending is not None:
                    self._exp_write(self.exp_pending, self.exp_pending,
                                    0.0, True)
                    self.exp_pending = None
                self.exp_handle.close()
            except OSError:
                pass
            self.exp_handle = None
            print('  experience log      %s' % self.exp_path)
        print('  experience kept     %d transitions (%d redundant steps '
              'dropped)' % (self.exp_written, self.exp_skipped))
        if RL_ENABLED:
            self.rl_close(self.rl_observation())
            self.rl_save()
            total = self.rl_explore + self.rl_greedy
            share = 100.0 * self.rl_greedy / total if total else 0.0
            print('  policy updates      %d (loaded from disk: %s)'
                  % (self.policy.updates, self.rl_loaded))
            print('  policy epsilon      %.3f  greedy actions %d (%.0f%%)'
                  % (self.policy.epsilon, self.rl_greedy, share))
            print('  policy acting       %s'
                  % ('learned' if self.rl_ready() else 'DWA (not trained yet)'))
            print('  safety overrides    %d learned actions capped'
                  % self.rl_safety_overrides)
            print('  mean step reward    %+.4f'
                  % (self.rl_reward_acc / max(1, self.rl_steps)))
            print('  policy weights      %s' % self.policy.path)
        if self.telemetry is not None:
            self.telemetry.close()
            print('  telemetry trace     %s' % self.telemetry_path)
        if self.trace is not None:
            self.trace.close()
            print('  trajectory trace    %s' % self.trace_path)
            print('  plot it with        python tools/plot_trace.py "%s"'
                  % self.trace_path)

    # ------------------------------------------------------------------
    # main loop
    # ------------------------------------------------------------------

    def run(self):
        while True:
            self.sim_time += self.dt

            self.read_pose()
            pts, rays = self.obstacle_points_world()
            if pts:
                self.sensor_ready = True
            if self.sim_time - self.last_map > MAP_PERIOD:
                self.last_map = self.sim_time
                self.update_map(pts, rays)

            self.update_odometry(self.dt)
            self.monitor_attitude()

            # --- drivetrain turn probe: TENNIS_SPIN="v,w" drives a constant
            # (v, w) and logs the achieved yaw rate, bypassing the state machine.
            if os.environ.get('TENNIS_SPIN'):
                pv, pw = (float(x) for x in os.environ['TENNIS_SPIN'].split(','))
                self.drive(pv, pw)
                if not hasattr(self, '_spin_last'):
                    self._spin_last = (self.sim_time, self.pose_h)
                elif self.sim_time - self._spin_last[0] >= 0.5:
                    dyaw = wrap_angle(self.pose_h - self._spin_last[1])
                    with open(os.path.join(TELEMETRY_DIR, 'spin_probe.csv'), 'a') as fh:
                        fh.write('%.2f,%.3f,%.3f,%.3f,%.1f\n' % (
                            self.sim_time, pv, pw, self.cmd_w,
                            math.degrees(dyaw) / (self.sim_time - self._spin_last[0])))
                    self._spin_last = (self.sim_time, self.pose_h)
                if self.step(self.timestep) == -1:
                    break
                continue

            self.update_ball_states()
            self.bag = self.bag_count()
            self.check_attitude()
            self.check_walls()

            self.cmd_v = self.cmd_w = 0.0
            if self.state == 'COLLECT':
                # COLLECT manages the rollers itself
                self.run_collect()
            else:
                # Spin the roller arms inward while aligning (to feed a ball
                # that reaches the V) and while carrying a ball (so it cannot
                # roll back out during driving or dumping). The passive one-way
                # door backs this up when the rollers are off, e.g. reversing.
                spin = (self.bag > 0 or self.state in ('ALIGN', 'SEEK', 'COLLECT'))
                if self.state in ('DUMP', 'DONE', 'DOWN'):
                    spin = False
                self.set_door_rollers(DOOR_ROLLER_SPEED if spin else 0.0)
                self.set_roller(25.0 if spin else 0.0)
                if self.state == 'SEEK':
                    self.run_seek()
                elif self.state == 'ALIGN':
                    self.run_align()
                elif self.state == 'SEARCH':
                    self.run_search()
                elif self.state == 'RECOVER':
                    self.run_recover()
                elif self.state == 'ESCAPE':
                    self.run_escape()
                elif self.state == 'WALL_BACK':
                    self.run_wall_back()
                elif self.state == 'TO_DROP':
                    self.run_to_drop()
                elif self.state == 'DUMP':
                    self.run_dump()
                elif self.state == 'DONE':
                    self.run_done()
                elif self.state == 'RECOVER_DOWN':
                    self.run_down()
                elif self.state == 'DOWN':
                    self.stop()

            self.detect_stuck()
            self.rl_tick()

            if self.sim_time - self.last_telemetry > TELEMETRY_PERIOD:
                self.last_telemetry = self.sim_time
                self.write_telemetry()

            if self.sim_time - self.last_trace > TRACE_PERIOD:
                self.last_trace = self.sim_time
                self.write_trace()

            if self.sim_time - self.last_log > LOG_PERIOD:
                self.last_log = self.sim_time
                self.log()

            if self.state == 'DONE' and self.done_time > 3.0:
                self.report()
                self.simulationQuit(0)
                return

            if self.sim_time > self.runtime:
                print('\n=== runtime cap reached, stopping ===')
                self.report()
                self.simulationQuit(0)
                return

            if not self.step_simulation():
                return

    def detect_stuck(self):
        """Notice a robot that is commanded to move but physically is not.

        Measured as net displacement over a window rather than per-step motion.
        A robot straining against a ball it cannot push does jitter - in run 30
        it moved more than 2 mm per step while making no progress at all - so a
        single-step threshold never fired and the stuck robot was never
        detected. Comparing the pose against a sample from STUCK_WINDOW seconds
        ago sees through the vibration, because the jitter does not accumulate
        in one direction (FINDINGS 5.41).
        """
        if self.state in ('DOWN', 'RECOVER_DOWN', 'DONE', 'WALL_BACK'):
            return
        if self.state in ('RECOVER', 'ESCAPE'):
            # RECOVER and WALL_BACK are already reversing on purpose, and the
            # robot may be wedged for several seconds while it works free.
            # Re-triggering RECOVER from inside RECOVER only reset the state
            # timer and kept it in the same 92 s loop (FINDINGS 5.28), so it is
            # excluded here; the wall guard is what frees a wedged robot.
            self.stuck_time = 0.0
            self.last_pose = (self.pose_x, self.pose_y)
            self.stuck_window = 0.0
            self.stall_time = 0.0
            return
        if self.stuck_window <= 0.0:
            self.last_pose = (self.pose_x, self.pose_y)
            self.stuck_window = STUCK_WINDOW
        self.stuck_window -= self.dt
        moved = math.hypot(self.pose_x - self.last_pose[0],
                           self.pose_y - self.last_pose[1])
        commanded = abs(self.cmd_left) + abs(self.cmd_right) > 0.15
        if self.stuck_window <= 0.0:
            # the window has elapsed: judge on the distance actually covered
            if moved < STUCK_DISTANCE and commanded:
                self.stuck_time += STUCK_WINDOW
            else:
                self.stuck_time = 0.0
            self.last_pose = (self.pose_x, self.pose_y)
            self.stuck_window = STUCK_WINDOW

        # Wall stall. When the wall guard clamps the speed to zero the robot is
        # *not* commanded to move, so the displacement test above never fires -
        # the robot simply sat at 0.00 m/s for 130 s of run 35 against a 0.38 m
        # wall, correctly obeying the guard and never escaping (FINDINGS 5.47).
        # A guard that stops the robot must also own the responsibility for
        # getting it moving again, so being clamped to zero while stationary is
        # treated as its own stuck condition.
        clamped = (self.cmd_v == 0.0 and
                   abs(self.min_clear) < WALL_STOP_DIST + ROBOT_RADIUS)
        if clamped and moved < STUCK_DISTANCE:
            self.stall_time += self.dt
        else:
            self.stall_time = 0.0
        if self.stall_time > STALL_TIME:
            self.stall_time = 0.0
            self.event('stalled %.0fs against a %.2f m wall - forcing a '
                       'retreat' % (STALL_TIME, self.min_clear))
            self.rl_note('stuck')
            self.wall_contact = True
            self.wall_warned = False
            self.wall_back_left = WALL_BACK_TIME
            self.wall_back_grace = 0.0
            self.state_before_wall = self.state
            self.state = 'WALL_BACK'
            self.path = []
            return

        if self.stuck_time > STUCK_TIME:
            self.stuck_time = 0.0
            self.rl_note('stuck')
            self.event('stuck %.0fs while driving (moved %.3f m) - backing off'
                       % (STUCK_TIME, moved))
            self.path = []
            self.last_plan = -10.0
            self.occ = set()
            self.dist = {}
            self.map_ready = False
            self.set_door_rollers(0.0)
            self.jam_time = 0.0
            self.last_jam_x = None
            if self.target is not None:
                # Escalate the ban. A flat 3 s ban always expired inside the
                # ~6 s recovery cycle, so choose_target() re-picked the very
                # ball the robot was already wedged against and the run spent
                # 68 s in SEEK/RECOVER at (-5.40, 0.15) without moving
                # (FINDINGS 5.30). Each repeated failure on the same ball
                # doubles the ban; after the escalation cap the ball is marked
                # unreachable for this run and the robot goes for another one.
                ball = self.target
                n = ball.get('stuck_count', 0) + 1
                ball['stuck_count'] = n
                if n >= STUCK_GIVE_UP:
                    self.event('giving up on ball %d after %d stuck attempts'
                               % (ball['id'], n))
                    ball['avoid_until'] = INF
                    self.unreachable_balls.add(ball['id'])
                else:
                    ball['avoid_until'] = (self.sim_time +
                                           STUCK_AVOID * (2 ** (n - 1)))
            self.enter_recover()

    def step_simulation(self):
        result = self.step(self.timestep)
        if result == 0:
            return True
        self.event('step() returned %d - the robot left the world' % result)
        return False


if __name__ == '__main__':
    import traceback as _tb
    try:
        controller = TennisCollector()
    except Exception:
        with open(os.path.join(os.environ.get('TEMP', '.'), 'kilo',
                               'collector_init_error.txt'), 'w') as fh:
            _tb.print_exc(file=fh)
        raise
    try:
        controller.run()
    except Exception:
        with open(os.path.join(os.environ.get('TEMP', '.'), 'kilo',
                               'collector_run_error.txt'), 'w') as fh:
            _tb.print_exc(file=fh)
        try:
            controller.simulationQuit(1)
        except Exception:
            pass
