"""Offline trainer for the collector's linear Q policy.

The controller learns online while it drives, but Webots is slow and a single
150 s run yields only a few thousand transitions. This script replays the
persisted experience buffer many times so the weights keep improving without
paying for more simulation, and it is the cheap way to compare hyper-parameters
or check that a policy file actually encodes a sensible policy.

Usage
-----
    python tools/train_policy.py                     # train from the default buffer
    python tools/train_policy.py path/to/buffer.jsonl
    python tools/train_policy.py --epochs 40 --alpha 0.02 --out data/policy_weights.json

The buffer format is the compact one the controller writes:
``{x, y, rng, v, w, nx, ny, reward, done}`` - robot-frame ball x, ball y,
distance, commanded v and w, the next observation's x and y, the reward and
the done flag.
"""

import argparse
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PROJ, 'controllers', 'tennis_collector'))

import policy as rl  # noqa: E402


def load_buffer(path):
    rows = []
    with open(path, 'r', encoding='utf-8') as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except ValueError:
                continue          # tolerate a truncated final record
    return rows


def row_to_obs(row, is_next=False):
    """Rebuild a policy observation from one experience row.

    The controller logs the goal position in the robot frame plus bag, local
    clearance and mode, so the observation is recovered exactly. Only `slip`,
    `stuck` and `escape` are absent from the buffer; those stay at their
    neutral values rather than being invented, because the online controller
    supplies them properly and a fabricated value would teach relationships the
    data does not support.

    `clr` must be read from the row rather than defaulted. Filling it with a
    fixed neutral value put every training sample in one clearance bin while
    the controller fed the policy a different bin at run time, so the trained
    cells were never the ones being queried and the policy came out frozen in
    open space (FINDINGS 5.44).
    """
    if is_next:
        x, y = row.get('nx'), row.get('ny')
    else:
        x, y = row.get('x'), row.get('y')
    if x is None or y is None:
        return None
    dist = math.hypot(x, y)
    bearing = math.atan2(y, x) if dist > 1e-6 else 0.0
    done = bool(row.get('done'))
    failed = done and float(row.get('reward', 0.0)) < 0.0
    clearance = row.get('clr')
    if clearance is None:
        clearance = 1.5
    return {
        'dist': dist,
        'bearing': bearing,
        'clearance': clearance,
        'slip': 0,
        'bag': row.get('bag', 0),
        'stuck': bool(failed),
        'escape': bool(failed),
        'mode': row.get('mode', rl.MODE_SEEK),
    }


def row_to_action(row):
    """Nearest lattice action to the (v, w) the controller actually drove."""
    v = float(row.get('v', 0.0))
    w = float(row.get('w', 0.0))
    best, best_err = 0, None
    for i, (av, aw) in enumerate(rl.ACTIONS):
        err = (av - v) ** 2 + (aw - w) ** 2
        if best_err is None or err < best_err:
            best, best_err = i, err
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('buffer', nargs='?',
                    default=os.path.join(PROJ, 'controllers', 'tennis_collector',
                                         'data', 'experience.jsonl'))
    ap.add_argument('--epochs', type=int, default=20)
    ap.add_argument('--alpha', type=float, default=0.05)
    ap.add_argument('--gamma', type=float, default=0.97)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--out', default=os.path.join(PROJ, 'controllers',
                                                  'tennis_collector', 'data',
                                                  'policy_weights.json'))
    args = ap.parse_args()

    if not os.path.exists(args.buffer):
        print('no buffer at %s - run the simulator first' % args.buffer)
        return 1
    rows = load_buffer(args.buffer)
    print('buffer    %s' % args.buffer)
    print('rows      %d' % len(rows))
    if not rows:
        print('buffer is empty; nothing to train on')
        return 1

    pol = rl.LinearQPolicy(alpha=args.alpha, gamma=args.gamma, epsilon=0.0)
    pol.path = args.out
    rng = random.Random(args.seed)

    # Build (obs, action, reward, next_obs, done) tuples once, so the epochs
    # loop is pure arithmetic. The reward is recomputed with the same shaping
    # the controller uses online, so offline training optimises the same
    # objective rather than the raw sparse signal in the file.
    samples = []
    for row in rows:
        obs = row_to_obs(row)
        nxt = row_to_obs(row, is_next=True)
        if obs is None or nxt is None:
            continue
        done = bool(row.get('done'))
        reward = rl.reward_from_row((row.get('x'), row.get('y')),
                                    (row.get('nx'), row.get('ny')),
                                    done,
                                    float(row.get('reward', 0.0)))
        samples.append((obs, row_to_action(row), reward, nxt, done))
    print('usable    %d transitions' % len(samples))
    if not samples:
        print('no usable transitions')
        return 1

    rewards = [s[2] for s in samples]
    print('reward    mean %+.4f  min %+.3f  max %+.3f'
          % (sum(rewards) / len(rewards), min(rewards), max(rewards)))
    print('terminals %d' % sum(1 for s in samples if s[4]))

    for epoch in range(args.epochs):
        rng.shuffle(samples)
        total_td = 0.0
        for obs, action, reward, nxt, done in samples:
            total_td += pol.update(obs, action, reward, nxt, done)
        pol.decay_epsilon()
        if epoch % 5 == 0 or epoch == args.epochs - 1:
            print('epoch %3d  mean |td| %.5f  epsilon %.3f'
                  % (epoch, abs(total_td) / len(samples), pol.epsilon))

    pol.save()
    print('saved     %s (%d updates)' % (args.out, pol.updates))

    # Report what the learned policy actually prefers in a few situations, so a
    # training run that silently learned nothing is visible immediately.
    print('\nlearned behaviour (greedy action per situation):')
    scenarios = [
        ('ball ahead 1.0 m, open',
         {'dist': 1.0, 'bearing': 0.0, 'clearance': 2.0, 'slip': 0, 'bag': 0,
          'stuck': False, 'escape': False, 'mode': rl.MODE_SEEK}),
        ('ball ahead 0.4 m, open',
         {'dist': 0.4, 'bearing': 0.0, 'clearance': 2.0, 'slip': 0, 'bag': 0,
          'stuck': False, 'escape': False, 'mode': rl.MODE_SEEK}),
        ('ball behind 1.0 m',
         {'dist': 1.0, 'bearing': 3.14, 'clearance': 2.0, 'slip': 0, 'bag': 0,
          'stuck': False, 'escape': False, 'mode': rl.MODE_SEEK}),
        ('ball ahead, wall 0.4 m',
         {'dist': 1.0, 'bearing': 0.0, 'clearance': 0.4, 'slip': 0, 'bag': 0,
          'stuck': False, 'escape': False, 'mode': rl.MODE_SEEK}),
        ('stuck, blocked',
         {'dist': 1.0, 'bearing': 0.0, 'clearance': 0.3, 'slip': 0, 'bag': 0,
          'stuck': True, 'escape': True, 'mode': rl.MODE_SEEK}),
        ('carrying, zone 1.5 m away',
         {'dist': 1.5, 'bearing': 0.0, 'clearance': 2.0, 'slip': 0, 'bag': 2,
          'stuck': False, 'escape': False, 'mode': rl.MODE_DROP}),
    ]
    for name, obs in scenarios:
        a, qs = pol.best_action(obs)
        v, w = rl.ACTIONS[a]
        print('  %-32s -> a=%2d (v=%+.2f w=%+.2f) q=%+.3f'
              % (name, a, v, w, qs[a]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
