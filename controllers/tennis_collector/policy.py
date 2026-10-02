"""Linear Q-learning policy for the tennis-ball collector.

Why linear Q-learning and not DDPG/SAC
--------------------------------------
The controller has no numpy, no torch and no GPU inside Webots, and each
simulated step costs real wall-clock time. What the collector needs is a
policy that (a) can be trained incrementally from the replay buffer that
already exists on disk, (b) survives across runs, and (c) is safe enough to
explore with on a machine that can fall over. A factored linear function
approximator over a handful of interpretable features gives all three with
pure-Python arithmetic and no dependencies at all.

Why factored, and why interactions
----------------------------------
The first version used ``Q(s,a) = w[state] + w[action] + w[bias]``. That is
purely additive, so the action term is global: the model can express "prefer
action 3 everywhere" but **not** "go forward when the ball is ahead, reverse
when wedged". Trained on the buffer it collapsed to a single action for every
situation (FINDINGS 5.35). The fix is to give the model state-action
*interaction* terms, but only for the dimensions where the choice genuinely
depends on the state:

    Q(s,a) = bias
           + w_dist[d]  + w_bear[b] + w_clear[c] + w_bag[g] + w_mode[m]
           + w_act[a]
           + w_dist_act[d][a] + w_bear_act[b][a] + w_clear_act[c][a]

The three interaction blocks are what carry the task: ``dist_act`` learns the
final approach (creep when close, cruise when far), ``bear_act`` learns to
pivot or reverse when the goal is behind, and ``clear_act`` learns to back off
when something is close. Sticking, surface and escape flags only shift the
value of a state and are not worth interacting with every action - they are
covered by the additive terms.

Sizing: the action dimensions give 6 + 8 + 4 + 15 = 33 additive cells plus
(6 + 8 + 4) * 15 = 270 interaction cells, so 303 weights in total. That is
small enough to learn from the few thousand transitions a handful of 150 s
runs produce, and each weight still receives gradient from many distinct
situations.

Safety
------
Exploration happens only through the *action* distribution; the learned value
is never allowed to command motion the existing wall guard would refuse. The
controller passes the guard-limited speed through as an action cap, so a policy
that has learned "drive fast into the wall" cannot override the lidar.
"""

import json
import math
import os

# --------------------------------------------------------------------------
# action set
# --------------------------------------------------------------------------
# (v, w) pairs. Every entry must be *executable*: the controller clamps turn
# rate as a function of forward speed so the robot cannot scrub a wheel and
# roll over (see TURN_SPEED_RATIO_V in the controller). An action outside that
# envelope - (0.62, 2.40) was the obvious one - would be chosen, clamped into
# something else, and then credited in the buffer with progress it did not
# actually cause, so the policy would keep reaching for a manoeuvre that is
# never performed (FINDINGS 5.43). The envelope at v = 0.62 allows about
# 0.8 rad/s, which is what the high-speed entries below use.
#
# Forward speeds match the DWA lattice so every learned action is one the
# planner already treats as reachable; reverse and spin entries are what give
# the policy the option of backing out of a wedge instead of only ever pushing
# forward into it.
ACTIONS = (
    (0.00, 0.00),    # 0  hold still
    (0.16, 0.00),    # 1  creep forward
    (0.32, 0.00),    # 2  cruise forward
    (0.62, 0.00),    # 3  full forward, straight
    (0.16, 0.80),    # 4  forward, gentle left
    (0.32, 0.80),    # 5  forward, moderate left
    (0.62, 0.80),    # 6  full forward, max allowed left
    (0.16, -0.80),   # 7  forward, gentle right
    (0.32, -0.80),   # 8  forward, moderate right
    (0.62, -0.80),   # 9  full forward, max allowed right
    (0.00, 1.20),    # 10 pivot left in place
    (0.00, -1.20),   # 11 pivot right in place
    (-0.12, 0.00),   # 12 reverse straight
    (-0.22, 0.80),   # 13 reverse, turning left (the classic wedge escape)
    (-0.22, -0.80),  # 14 reverse, turning right
)
N_ACTIONS = len(ACTIONS)

# --------------------------------------------------------------------------
# state abstraction
# --------------------------------------------------------------------------
# Distances are binned coarsely. A fine grid would need orders of magnitude
# more samples than a 150 s run can produce, and the point of the policy is to
# generalise the *shape* of the situation (behind / close / blocked), not to
# memorise exact poses.
DIST_EDGES = (0.35, 0.70, 1.20, 2.00, 3.50)
N_DIST = len(DIST_EDGES) + 1
N_BEAR = 8                     # 45 deg buckets over the full circle
CLEAR_EDGES = (0.45, 0.75, 1.20)
N_CLEAR = len(CLEAR_EDGES) + 1
N_SLIP = 3                     # gripping / slipping / unknown surface
N_BAG = 4                      # 0, 1, 2, 3+ balls carried
N_MODE = 3                     # seeking a ball / heading to the drop zone / idle

# The one-hot mode flag lets a single model express three different behaviours
# from the same features, which is what encodes "gather first, then go to the
# drop zone" without needing separate networks.
MODE_SEEK = 0
MODE_DROP = 1
MODE_IDLE = 2

# ---- weight layout -------------------------------------------------------
# Fixed offsets so the layout is stable across runs and the file stays readable.
OFF_BIAS = 0
OFF_DIST = OFF_BIAS + 1
OFF_BEAR = OFF_DIST + N_DIST
OFF_CLEAR = OFF_BEAR + N_BEAR
OFF_BAG = OFF_CLEAR + N_CLEAR
OFF_SLIP = OFF_BAG + N_BAG
OFF_MODE = OFF_SLIP + N_SLIP
OFF_ACT = OFF_MODE + N_MODE
OFF_STUCK = OFF_ACT + N_ACTIONS
OFF_ESCAPE = OFF_STUCK + 1
OFF_DIST_ACT = OFF_ESCAPE + 1
OFF_BEAR_ACT = OFF_DIST_ACT + N_DIST * N_ACTIONS
OFF_CLEAR_ACT = OFF_BEAR_ACT + N_BEAR * N_ACTIONS
N_WEIGHTS = OFF_CLEAR_ACT + N_CLEAR * N_ACTIONS


def bin_index(value, edges):
    """Index of the bucket `value` falls into for the given ascending edges."""
    for i, edge in enumerate(edges):
        if value < edge:
            return i
    return len(edges)


def dist_bin(d):
    return bin_index(abs(d), DIST_EDGES)


def clear_bin(c):
    if c is None:
        return N_CLEAR - 1
    return bin_index(c, CLEAR_EDGES)


def bear_bin(bearing):
    """Bucket a bearing in radians into N_BEAR sectors, centred on straight ahead."""
    b = (bearing + math.pi) % (2.0 * math.pi) - math.pi
    return int((b + math.pi) / (2.0 * math.pi) * N_BEAR) % N_BEAR


def indices(obs):
    """The handful of integer cells this observation activates."""
    return (dist_bin(obs['dist']),
            bear_bin(obs['bearing']),
            clear_bin(obs['clearance']),
            min(obs['bag'], N_BAG - 1),
            min(obs['slip'], N_SLIP - 1),
            obs['mode'],
            1 if obs.get('stuck') else 0,
            1 if obs.get('escape') else 0)


def q_value(weights, obs, action):
    """Q(s, a) for the factored model. Pure function, used by both paths."""
    d, b, c, g, sl, m, stuck, esc = indices(obs)
    return (weights[OFF_BIAS]
            + weights[OFF_DIST + d]
            + weights[OFF_BEAR + b]
            + weights[OFF_CLEAR + c]
            + weights[OFF_BAG + g]
            + weights[OFF_SLIP + sl]
            + weights[OFF_MODE + m]
            + weights[OFF_ACT + action]
            + weights[OFF_STUCK + stuck]
            + weights[OFF_ESCAPE + esc]
            + weights[OFF_DIST_ACT + d * N_ACTIONS + action]
            + weights[OFF_BEAR_ACT + b * N_ACTIONS + action]
            + weights[OFF_CLEAR_ACT + c * N_ACTIONS + action])


def active_weights(obs, action):
    """Indices the TD update should move for this (state, action).

    Every cell listed receives the same TD error, which is exactly the gradient
    of the factored model above.
    """
    d, b, c, g, sl, m, stuck, esc = indices(obs)
    return (OFF_BIAS,
            OFF_DIST + d, OFF_BEAR + b, OFF_CLEAR + c, OFF_BAG + g,
            OFF_SLIP + sl, OFF_MODE + m, OFF_ACT + action,
            OFF_STUCK + stuck, OFF_ESCAPE + esc,
            OFF_DIST_ACT + d * N_ACTIONS + action,
            OFF_BEAR_ACT + b * N_ACTIONS + action,
            OFF_CLEAR_ACT + c * N_ACTIONS + action)


# --------------------------------------------------------------------------
# reward shaping
# --------------------------------------------------------------------------
# The task, in reward terms:
#   * collect a ball          -> strong positive, this is the main objective
#   * deliver at the drop zone-> larger positive, and only once per ball
#   * getting stuck           -> penalty, so the policy learns to escape
#   * closing on the ball     -> positive, and must dominate the step cost
#   * leaving the court       -> penalty
#
# Scale discipline matters more than usual here. The first version used
# R_CLOSER = 0.02 per metre with R_STEP = -0.01 per step, so closing a full
# metre earned +0.02 while the 50 steps it took to travel that metre cost
# -0.50. Step cost swamped shaping by 25x, every action looked almost equally
# bad, and the discounted return was dominated by *step count* - which the
# policy minimises by standing still. It duly learned to freeze, and the run
# after training was far worse than the DWA baseline it replaced (FINDINGS
# 5.39). The invariant to hold is:
#
#     shaping earned over a full approach  >>  step cost over that approach
#
# so R_CLOSER is now per *metre* and is sized against the step cost over the
# ~2 m a typical approach covers.
R_CAPTURE = 10.0
R_DELIVER = 25.0
R_STUCK = -6.0
R_ESCAPE_FAIL = -4.0
R_OUT_OF_BOUNDS = -5.0
R_STEP = -0.01
R_CLOSER = 0.6                # per metre of goal distance closed
R_FARTHER = -0.6
R_FAR_CAP = 0.10              # per-step shaping cap, m: ignore teleports
R_STOP_IN_ZONE = 5.0          # stopping when loaded and in the zone


def shaped_reward(prev, cur, event):
    """Reward for one transition.

    `prev` and `cur` are observation dicts; `event` is one of
    ``'capture'``, ``'deliver'``, ``'stuck'``, ``'escape_fail'``,
    ``'out_of_bounds'``, ``'stopped_in_zone'`` or None.
    """
    r = R_STEP
    if event == 'capture':
        r += R_CAPTURE
    elif event == 'deliver':
        r += R_DELIVER
    elif event == 'stuck':
        r += R_STUCK
    elif event == 'escape_fail':
        r += R_ESCAPE_FAIL
    elif event == 'out_of_bounds':
        r += R_OUT_OF_BOUNDS
    elif event == 'stopped_in_zone':
        r += R_STOP_IN_ZONE

    # distance shaping: only while there is a ball to chase
    if prev.get('mode') == MODE_SEEK and cur.get('mode') == MODE_SEEK:
        pd, cd = prev.get('dist'), cur.get('dist')
        if pd is not None and cd is not None:
            delta = pd - cd
            if abs(delta) <= R_FAR_CAP:
                r += R_CLOSER * delta if delta > 0 else R_FARTHER * -delta
    return r


def reward_from_row(prev_xy, next_xy, terminal, terminal_reward):
    """Recompute a shaped reward from a replay-buffer row.

    The buffer stores only the robot-frame ball position, the commanded action
    and a terminal reward, so the shaping has to be reconstructed offline. This
    keeps the online and offline objectives identical instead of training on
    the raw sparse `+1 / 0` signal the controller happens to log.
    """
    if terminal:
        if terminal_reward >= 1.0:
            return R_CAPTURE + R_STEP
        return R_ESCAPE_FAIL + R_STEP
    r = R_STEP
    if prev_xy is not None and next_xy is not None:
        d0 = math.hypot(*prev_xy)
        d1 = math.hypot(*next_xy)
        delta = d0 - d1
        if abs(delta) <= R_FAR_CAP:
            r += R_CLOSER * delta if delta > 0 else R_FARTHER * -delta
    return r


# --------------------------------------------------------------------------
# the policy
# --------------------------------------------------------------------------
DEFAULT_ALPHA = 0.05
DEFAULT_GAMMA = 0.97
# Directed-exploration thresholds. A cell with fewer than EXPLORE_MIN_VISITS
# observations is treated as undecided, and within it any action whose own
# visit count is below EXPLORE_PAIR_VISITS counts as untried.
EXPLORE_MIN_VISITS = 12
EXPLORE_PAIR_VISITS = 2


class LinearQPolicy(object):
    """Factored linear Q-function with weights persisted to disk."""

    def __init__(self, alpha=DEFAULT_ALPHA, gamma=DEFAULT_GAMMA,
                 epsilon=0.30, path=None):
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_min = 0.05
        self.epsilon_decay = 0.995
        self.path = path
        self.weights = [0.0] * N_WEIGHTS
        self.updates = 0
        self.mode_updates = [0] * N_MODE
        self.cell_visits = {}
        self.pair_visits = {}
        self.max_abs_td = 0.0

    # -- persistence -------------------------------------------------------
    def load(self):
        if not self.path or not os.path.exists(self.path):
            return False
        try:
            with open(self.path, 'r', encoding='utf-8') as fh:
                data = json.load(fh)
        except (OSError, ValueError):
            return False
        w = data.get('weights')
        if not isinstance(w, list) or len(w) != N_WEIGHTS:
            return False
        try:
            self.weights = [float(x) for x in w]
            self.updates = int(data.get('updates', 0))
            self.epsilon = float(data.get('epsilon', self.epsilon))
            seen = data.get('mode_updates')
            if isinstance(seen, list) and len(seen) == N_MODE:
                self.mode_updates = [int(x) for x in seen]
            return True
        except (TypeError, ValueError):
            return False

    def save(self):
        if not self.path:
            return False
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            tmp = self.path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fh:
                json.dump({'weights': self.weights,
                           'updates': self.updates,
                           'mode_updates': self.mode_updates,
                           'epsilon': self.epsilon,
                           'alpha': self.alpha,
                           'gamma': self.gamma,
                           'n_weights': N_WEIGHTS}, fh)
            os.replace(tmp, self.path)
            return True
        except OSError:
            return False

    def mode_is_trained(self, mode, minimum):
        """Whether this mode has enough experience to act on.

        Trust has to be granted per mode, not globally. Carrying a ball to the
        drop zone is a different behaviour from chasing one, and a buffer full
        of SEEK transitions leaves MODE_DROP completely untrained - its weights
        are still zero, so every action ties and the tie-break returns "hold
        still". A global update count would have declared the policy ready and
        let it drive blind into the one phase it had never seen (FINDINGS 5.40).
        """
        return self.mode_updates[mode] >= minimum

    # -- evaluation --------------------------------------------------------
    def q_values(self, obs):
        """Q for every action in one observation."""
        return [q_value(self.weights, obs, a) for a in range(N_ACTIONS)]

    def best_action(self, obs):
        qs = self.q_values(obs)
        return max(range(N_ACTIONS), key=lambda a: qs[a]), qs

    def choose(self, obs, rng, explore=True):
        """Epsilon-greedy, with directed exploration when evidence is thin.

        Plain epsilon-greedy under-explores here. The buffer shows why: at
        "ball ahead, open ground" the DWA bootstrap almost never emitted a fast
        forward action, so the policy had no evidence that one works and
        epsilon kept picking pivots instead (FINDINGS 5.45). Repeating more
        epochs cannot fix absent data. So when the greedy action's advantage
        over the runner-up is small - meaning this state is not really decided
        yet - the choice is biased toward actions this state has never tried,
        which is what actually acquires the missing evidence.

        The bias decays as a cell accumulates visits, so exploration is
        strongest exactly where the model knows least and vanishes where it is
        confident.
        """
        if not explore:
            a, _ = self.best_action(obs)
            return a, False
        if rng.random() < self.epsilon:
            return rng.randrange(N_ACTIONS), True

        d, b, c, g, sl, m, stuck, esc = indices(obs)
        # count how often this state cell has been visited at all
        visits = self.cell_visits.get((d, b, c, m), 0)
        if visits < EXPLORE_MIN_VISITS:
            # thin evidence: among the actions this cell has barely used, prefer
            # the one with the best value, breaking ties at random
            candidates = self.untried_actions((d, b, c, m), obs)
            if candidates:
                best = max(candidates,
                           key=lambda a: (self._q(obs, a), rng.random()))
                return best, True
        a, _ = self.best_action(obs)
        return a, False

    def _q(self, obs, action):
        return q_value(self.weights, obs, action)

    def untried_actions(self, cell, obs):
        """Actions whose (cell, action) pair has seen little or no data."""
        out = []
        for a in range(N_ACTIONS):
            if self.pair_visits.get((cell, a), 0) < EXPLORE_PAIR_VISITS:
                out.append(a)
        return out

    def note_visit(self, obs, action):
        """Record that (state, action) was exercised, for the exploration bias."""
        d, b, c, g, sl, m, stuck, esc = indices(obs)
        self.cell_visits[(d, b, c, m)] = self.cell_visits.get((d, b, c, m), 0) + 1
        key = ((d, b, c, m), action)
        self.pair_visits[key] = self.pair_visits.get(key, 0) + 1

    # -- learning ----------------------------------------------------------
    def update(self, obs, action, reward, next_obs, done):
        """One TD(0) step of semi-gradient linear Q-learning."""
        target = reward
        if not done and next_obs is not None:
            target = reward + self.gamma * max(self.q_values(next_obs))
        td = target - q_value(self.weights, obs, action)
        step = self.alpha * td
        for i in active_weights(obs, action):
            self.weights[i] += step
        self.updates += 1
        self.mode_updates[obs['mode']] += 1
        self.note_visit(obs, action)
        if abs(td) > self.max_abs_td:
            self.max_abs_td = abs(td)
        return td

    def decay_epsilon(self):
        if self.epsilon > self.epsilon_min:
            self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
