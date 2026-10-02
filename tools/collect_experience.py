#!/usr/bin/env python
# ---------------------------------------------------------------------------
# Bulk experience generation for the learned policy.
#
#   python tools/collect_experience.py --runs 20
#   python tools/collect_experience.py --runs 20 --runtime 90 --explore 1.8
#
# Why this exists
# ---------------
# FINDINGS 5A.5: the learned policy has not beaten the hand-written DWA planner,
# and the reason is data volume, not tuning. A single 150 s run yields a few
# thousand transitions against a 27648-cell state space (12 distance x 12 bearing
# x 4 clearance x 3 mode x 15 action); MODE_DROP saw 29. Tuning weights on that
# is noise-fitting.
#
# Worse, one fixed ball layout means the same ~17 distance/bearing pairs recur
# every run, so most new rows are near-duplicates of rows already in the buffer.
# This script therefore varies the ball layout per run via the controller's
# --seed, and varies the exploration bias to deliberately visit situations the
# greedy policy avoids (wedged against a ball, goal behind, low clearance).
#
# Each run is a separate Webots process on its own port, run sequentially.
# Parallel runs would fight over the shared experience.jsonl append handle and
# the Webots preference file, so this stays serial.
# ---------------------------------------------------------------------------
import argparse
import glob
import io
import json
import os
import re
import shutil
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(ROOT, 'worlds', 'tennis_court_train.wbt')
BUFFER = os.path.join(ROOT, 'controllers', 'tennis_collector', 'data',
                      'experience.jsonl')
TEMPLATE = os.path.join(ROOT, 'worlds', 'tennis_court_train.wbt')
# Not a dotfile: Webots refuses to open a world whose name begins with '.'
RUN_WORLD = os.path.join(ROOT, 'worlds', 'tennis_court_train_run.wbt')

WEBOTS = r'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe'


def buffer_rows():
    if not os.path.exists(BUFFER):
        return 0
    with io.open(BUFFER, encoding='utf-8', errors='replace') as fh:
        return sum(1 for line in fh if line.strip())


def write_world(seed, explore, runtime):
    """Rewrite the controllerArgs of the training world for this run.

    The world file is edited in place rather than passed as CLI overrides
    because Webots' Supervisor parses controllerArgs from the world, and this
    keeps the per-run settings visible in the file that was actually simulated.
    """
    with io.open(TEMPLATE, encoding='utf-8') as fh:
        src = fh.read()
    out = re.sub(
        r'controllerArgs \[ "[^"]*" \]',
        'controllerArgs [ "--dropZone -4.9 1.8 '
        '--bounds -5.75 -0.70 -2.70 2.70 --runtime %g --seed %d '
        '--explore %g" ]' % (runtime, seed, explore),
        src)
    if out == src:
        sys.exit('could not rewrite controllerArgs in %s' % TEMPLATE)
    with io.open(RUN_WORLD, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(out)
    return RUN_WORLD


def run_one(seed, explore, runtime, port, log):
    world = write_world(seed, explore, runtime)
    # --minimize keeps the batch runs from stealing focus across 20 launches.
    # --stdout is required or the controller's report never reaches the log.
    cmd = [
        WEBOTS, '--batch', '--mode=fast', '--no-rendering',
        '--stdout', '--stderr', '--minimize',
        '--port=%d' % port, world,
    ]
    with io.open(log, 'w', encoding='utf-8') as fh:
        proc = subprocess.run(cmd, stdout=fh, stderr=subprocess.STDOUT,
                              cwd=ROOT, timeout=900)
    return proc.returncode


def archive_reports(seed, artdir):
    """Move the run's trace out of logs/ and render its per-iteration report.

    The controller writes its trace to a fixed name derived from the world, so
    every run overwrites the previous one. Without archiving, iteration N's plot
    and CSV describe iteration N+1 - and since these files are what makes two
    runs comparable, a silently mismatched plot is worse than no plot. The
    controller crashing is also detectable here: no trace means no robot.
    """
    logs = os.path.join(ROOT, 'logs')
    traces = sorted(glob.glob(os.path.join(logs, 'trace_*.jsonl')),
                    key=os.path.getmtime)
    if not traces:
        return None, 'no trace written - the controller probably crashed'
    src = traces[-1]
    if not os.path.isdir(artdir):
        os.makedirs(artdir)

    stamp = time.time()

    # Iteration artifacts are ACCUMULATED, never overwritten. A run's plot and
    # CSV are the only record of what that run actually did; silently replacing
    # them destroys the history that makes runs comparable at all. Seeds are
    # required to be unique, and the label is made unique regardless, so a
    # repeated seed cannot clobber an earlier run.
    label = 'run%03d' % seed
    png = os.path.join(artdir, label + '.png')
    csv_path = os.path.join(artdir, label + '.csv')
    summary = os.path.join(artdir, label + '_summary.csv')
    clash = [p for p in (png, csv_path, summary)
             if os.path.exists(p)]
    if clash:
        base = label
        k = 2
        while True:
            label = '%s_%d' % (base, k)
            clash = [p for p in (os.path.join(artdir, label + '.png'),
                                 os.path.join(artdir, label + '.csv'),
                                 os.path.join(artdir,
                                              label + '_summary.csv'))
                     if os.path.exists(p)]
            if not clash:
                break
            k += 1
        print('    note: %s already archived; writing as %s instead'
              % (base, label))

    kept = os.path.join(artdir, 'trace_%s_%d.jsonl' % (label, int(stamp)))
    shutil.copyfile(src, kept)

    cmd = [sys.executable,
           os.path.join(ROOT, 'tools', 'iteration_report.py'),
           kept, artdir, '--label', label]
    proc = subprocess.run(cmd, cwd=ROOT, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT)
    report = proc.stdout.decode('utf-8', 'replace').strip()
    if proc.returncode != 0:
        report = 'report failed: %s' % report
    return kept, report


def summarise(log):
    """Pull the run's own report out of the stdout log."""
    events = []
    cap = delivered = None
    with io.open(log, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            if '>> [' in line:
                events.append(line.strip())
            if 'delivered' in line and 'bag' in line:
                delivered = line.strip()
    for ev in events:
        if 'captured' in ev:
            cap += 1 if cap is not None else 0
    return events, delivered


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--runs', type=int, default=20,
                    help='how many simulation runs to perform')
    ap.add_argument('--seed0', type=int, default=100,
                    help='first seed; each run uses seed0 + i')
    ap.add_argument('--runtime', type=float, default=150.0,
                    help='simulated seconds per run')
    ap.add_argument('--explore', type=float, default=1.6,
                    help='exploration bias multiplier passed to the policy')
    ap.add_argument('--explore-jitter', type=float, default=0.4,
                    help='per-run uniform jitter on --explore')
    ap.add_argument('--port0', type=int, default=13000)
    ap.add_argument('--logdir',
                    default=os.path.join(ROOT, 'logs', 'train'))
    ap.add_argument('--artdir',
                    default=os.path.join(ROOT, 'logs', 'iterations'),
                    help='where each run\'s png/csv/trace are archived')
    ap.add_argument('--no-reports', action='store_true',
                    help='skip the per-iteration png/csv (faster)')
    args = ap.parse_args()

    if not os.path.exists(WEBOTS):
        sys.exit('webots not found at %s' % WEBOTS)
    if not os.path.exists(TEMPLATE):
        sys.exit('missing %s' % TEMPLATE)

    if not os.path.isdir(args.logdir):
        os.makedirs(args.logdir)

    import random
    rng = random.Random(args.seed0)

    start_rows = buffer_rows()
    print('buffer starts at %d rows' % start_rows)
    rows = [start_rows]

    for i in range(args.runs):
        seed = args.seed0 + i
        explore = max(1.0, args.explore + rng.uniform(-args.explore_jitter,
                                                      args.explore_jitter))
        log = os.path.join(args.logdir, 'run%03d.log' % seed)
        t0 = time.time()
        try:
            rc = run_one(seed, explore, args.runtime,
                         args.port0 + (i % 400), log)
        except subprocess.TimeoutExpired:
            print('run %d: TIMEOUT' % seed)
            continue
        wall = time.time() - t0
        events, delivered = summarise(log)
        n = buffer_rows()
        rows.append(n)
        print('run %2d seed %-4d explore %.2f  rc %d  %5.1fs  '
              'rows %6d (+%d)' % (i, seed, explore, rc, wall, n,
                                   n - rows[-2]))
        for ev in events:
            print('    ' + ev.split('] ', 1)[-1])
        if delivered:
            print('    ' + delivered)

        if args.no_reports:
            continue

        trace, report = archive_reports(seed, args.artdir)
        for line in (report or '').splitlines():
            print('    ' + line.strip())
        if not report:
            print('    WARNING: no report for this run')

    if os.path.exists(RUN_WORLD):
        os.remove(RUN_WORLD)

    end_rows = buffer_rows()
    print()
    print('=' * 64)
    print('buffer %d -> %d rows (+%d) over %d runs'
          % (start_rows, end_rows, end_rows - start_rows, args.runs))
    if end_rows:
        print('mean new rows per run: %.0f'
              % ((end_rows - start_rows) / float(args.runs)))
    print('now: python tools/train_policy.py --epochs 40')


if __name__ == '__main__':
    main()