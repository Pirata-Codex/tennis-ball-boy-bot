"""Per-iteration report: CSV metrics + a plot for one tennis_bot run.

    python tools/iteration_report.py <trace.jsonl> <outdir> [--label run007]

Writes, into <outdir>:

    <label>.png       the court map (robot path, state colouring, ball tracks)
    <label>.csv       one row per trace sample: t, state, pose, bag, delivered
    <label>_summary.csv  one row of run-level metrics

The per-sample CSV is what makes a *series* of runs comparable, which the PNG is
not: state-time fractions and stationary stretches are easy to diff across
iterations, whereas each court map has to be read by eye.

The PNG reuses the drawing conventions of tools/plot_trace.py, plus two panels
that were needed to compare iterations (see `state timeline` below): the state
timeline makes an oscillation between two states - the failure mode most of
FINDINGS 5.27-5.47 was - visible as a stripe pattern that a path plot hides,
and the distance-to-goal curve shows a robot that is busy but not converging.

Usage:
    python tools/iteration_report.py logs/trace_world.jsonl logs/iterations
"""

import argparse
import csv
import glob
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow
import numpy as np

STATE_ORDER = ['SEEK', 'SEARCH', 'ALIGN', 'COLLECT', 'RECOVER', 'WALL_BACK',
               'TO_DROP', 'DUMP', 'DONE', 'ESCAPE']
STATE_COLOR = {
    'SEEK': '#1f77b4', 'SEARCH': '#17becf', 'ALIGN': '#ff7f0e',
    'COLLECT': '#d62728', 'RECOVER': '#9467bd', 'WALL_BACK': '#8c564b',
    'TO_DROP': '#2ca02c', 'DUMP': '#e377c2', 'DONE': '#7f7f7f',
    'ESCAPE': '#bcbd22',
}

STATIONARY_DIST = 0.05   # m; below this a sample counts as not moving
STATIONARY_TIME = 1.0    # s; minimum window for reporting a stall


def newest_trace():
    logs = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'logs')
    files = glob.glob(os.path.join(logs, 'trace_*.jsonl'))
    if not files:
        raise SystemExit('no trace_*.jsonl found in %s' % logs)
    return max(files, key=os.path.getmtime)


def load(path):
    meta, rows = None, []
    with open(path, 'r', encoding='utf-8') as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except ValueError:
                continue
            if obj.get('meta'):
                meta = obj
            elif 'x' in obj:
                rows.append(obj)
    if meta is None:
        raise SystemExit('trace has no meta line: %s' % path)
    if not rows:
        raise SystemExit('trace has no samples: %s' % path)
    return meta, rows


def state_times(rows):
    """Seconds spent in each state.

    Samples are irregularly spaced, so the duration attributed to a state is
    the gap to the *next* sample rather than a nominal sample period. Using a
    nominal period would misreport the breakdown whenever the write rate
    changes.
    """
    times = {}
    for i, r in enumerate(rows):
        dt = (rows[i + 1]['t'] - r['t']) if i + 1 < len(rows) else 0.0
        times[r['s']] = times.get(r['s'], 0.0) + dt
    return times


def distance_m(rows):
    total = 0.0
    for i in range(1, len(rows)):
        total += np.hypot(rows[i]['x'] - rows[i - 1]['x'],
                          rows[i]['y'] - rows[i - 1]['y'])
    return total


def stationary_spans(rows, dist=STATIONARY_DIST, window=STATIONARY_TIME):
    """Longest runs during which the robot covered almost no ground.

    This is the metric that actually tracked the bugs in FINDINGS 5.27-5.47:
    each one showed up first as a long stationary stretch, long before it was
    diagnosed. Reported with the position and state so the stall is locatable.
    """
    spans = []
    start_i = None
    anchor = None
    for i, r in enumerate(rows):
        if start_i is None:
            start_i, anchor = i, (r['x'], r['y'])
            continue
        if np.hypot(r['x'] - anchor[0], r['y'] - anchor[1]) >= dist:
            # moved: close any open span
            if start_i is not None:
                dur = rows[i - 1]['t'] - rows[start_i]['t']
                if dur >= window:
                    spans.append(span(rows, start_i, i - 1, dur))
            start_i, anchor = i, (r['x'], r['y'])
    if start_i is not None and len(rows) - 1 > start_i:
        dur = rows[-1]['t'] - rows[start_i]['t']
        if dur >= window:
            spans.append(span(rows, start_i, len(rows) - 1, dur))
    spans.sort(key=lambda s: -s['duration'])
    return spans


def span(rows, i, j, dur):
    xs = [r['x'] for r in rows[i:j + 1]]
    ys = [r['y'] for r in rows[i:j + 1]]
    return {
        'duration': dur,
        't_start': rows[i]['t'],
        'x': float(np.mean(xs)),
        'y': float(np.mean(ys)),
        'states': sorted({r['s'] for r in rows[i:j + 1]}),
    }


def transits(rows):
    """Count state changes. High values mean oscillation, not progress."""
    n = 0
    for i in range(1, len(rows)):
        if rows[i]['s'] != rows[i - 1]['s']:
            n += 1
    return n


def write_samples_csv(path, rows, meta):
    fields = ['t', 'state', 'x', 'y', 'heading_deg', 'bag', 'delivered',
              'depth', 'n_balls_in_hopper', 'ball0_x', 'ball0_y']
    with open(path, 'w', newline='', encoding='utf-8') as fh:
        wr = csv.writer(fh)
        wr.writerow(fields)
        for r in rows:
            balls = r.get('b') or []
            inside = sum(1 for b in balls if len(b) > 2 and b[2])
            b0 = balls[0] if balls else ['', '']
            wr.writerow([
                '%.2f' % r['t'], r['s'], '%.3f' % r['x'], '%.3f' % r['y'],
                '%.1f' % r['h'], r.get('bag', ''), r.get('dep', ''),
                r.get('dep', ''), inside,
                ('%.3f' % b0[0]) if b0[0] != '' else '',
                ('%.3f' % b0[1]) if b0[1] != '' else '',
            ])


def summary_row(rows, meta, label, trace):
    ts = np.array([r['t'] for r in rows])
    xs = np.array([r['x'] for r in rows])
    ys = np.array([r['y'] for r in rows])
    times = state_times(rows)
    spans = stationary_spans(rows)
    dur = float(ts[-1] - ts[0])
    row = {
        'label': label,
        'trace': os.path.basename(trace),
        'duration_s': '%.1f' % dur,
        'samples': len(rows),
        'distance_m': '%.2f' % distance_m(rows),
        'x_min': '%.2f' % xs.min(), 'x_max': '%.2f' % xs.max(),
        'y_min': '%.2f' % ys.min(), 'y_max': '%.2f' % ys.max(),
        'max_bag': max(r.get('bag', 0) for r in rows),
        'delivered': rows[-1].get('dep', 0),
        'balls': meta.get('balls', 0),
        'state_transits': transits(rows),
        'longest_stall_s': '%.1f' % (spans[0]['duration'] if spans else 0.0),
        'stall_count': len(spans),
    }
    for st in STATE_ORDER:
        row['t_%s_s' % st] = '%.1f' % times.get(st, 0.0)
    if spans:
        worst = spans[0]
        row['worst_stall_at'] = '%.2f,%.2f' % (worst['x'], worst['y'])
        row['worst_stall_state'] = '|'.join(worst['states'])
    else:
        row['worst_stall_at'] = ''
        row['worst_stall_state'] = ''
    return row


def write_summary_csv(path, row):
    with open(path, 'w', newline='', encoding='utf-8') as fh:
        wr = csv.DictWriter(fh, fieldnames=list(row.keys()))
        wr.writeheader()
        wr.writerow(row)


def write_plot(path, rows, meta, label):
    xmin, xmax, ymin, ymax = meta['bounds']
    dx, dy = meta['drop']
    nballs = meta.get('balls', max(len(r.get('b') or []) for r in rows))
    xs = np.array([r['x'] for r in rows])
    ys = np.array([r['y'] for r in rows])
    hs = np.array([r['h'] for r in rows])
    ts = np.array([r['t'] for r in rows])
    states = [r['s'] for r in rows]
    bags = np.array([r.get('bag', 0) for r in rows])

    fig = plt.figure(figsize=(15, 9))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.35, 1.0], height_ratios=[1, 1],
                          hspace=0.28, wspace=0.22)
    ax = fig.add_subplot(gs[:, 0])
    axt = fig.add_subplot(gs[0, 1])
    axb = fig.add_subplot(gs[1, 1])

    times = state_times(rows)
    dur = max(float(ts[-1] - ts[0]), 1e-6)

    # ---------------- court map ----------------
    ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                           fill=False, ec='#333333', lw=1.5))
    ax.plot([0, 0], [ymin, ymax], color='#888888', lw=2.5)
    ax.text(0.06, ymax - 0.20, 'net', color='#666666', fontsize=9)
    ax.add_patch(Rectangle((dx - 0.85, dy - 0.85), 1.7, 1.7, facecolor='#d9f0d3',
                           ec='#2ca02c', lw=1.2, alpha=0.7))
    ax.text(dx, dy, 'drop', color='#2ca02c', fontsize=9, ha='center',
            va='center')

    first = {}
    last = {}
    for i, r in enumerate(rows):
        for bi, b in enumerate(r.get('b') or []):
            first.setdefault(bi, b[:2])
            last[bi] = b[:2]
    for bi in sorted(first):
        ax.scatter([first[bi][0]], [first[bi][1]], marker='o', s=55,
                   facecolor='none', edgecolor='#888888', lw=1.1, zorder=2)
        ax.scatter([last[bi][0]], [last[bi][1]], marker='x', s=45,
                   color='#555555', zorder=3)

    ax.plot(xs, ys, color='#bbbbbb', lw=1.0, zorder=2)
    for st in STATE_ORDER:
        idx = [i for i, s in enumerate(states) if s == st]
        if not idx:
            continue
        ax.scatter(xs[idx], ys[idx], s=9,
                   color=STATE_COLOR.get(st, '#333333'), label=st, zorder=3)

    step = max(1, len(rows) // 25)
    for i in range(0, len(rows), step):
        rad = np.radians(hs[i])
        ax.add_patch(FancyArrow(xs[i], ys[i], 0.18 * np.cos(rad),
                                0.18 * np.sin(rad), width=0.0,
                                head_width=0.09, head_length=0.08,
                                color='#444444', zorder=4))
    ax.scatter([xs[0]], [ys[0]], marker='s', s=60, color='black', zorder=6,
               label='start')
    ax.scatter([xs[-1]], [ys[-1]], marker='D', s=55, color='#d62728', zorder=6,
               label='end')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(xmin - 0.2, max(xmax, 0.0) + 0.2)
    ax.set_ylim(ymin - 0.2, ymax + 0.2)
    ax.grid(True, ls=':', lw=0.4, alpha=0.5)
    ax.legend(loc='upper right', fontsize=7, ncol=2, framealpha=0.9)

    # ---------------- state timeline ----------------
    # A path plot averages over time, so a robot oscillating between SEEK and
    # WALL_BACK looks like one loop. A per-state band over time does not.
    lanes, lane_of = [], {}
    for st in STATE_ORDER:
        if times.get(st):
            lane_of[st] = len(lanes)
            lanes.append(st)
    if lanes:
        for st in lanes:
            idx = [i for i, s in enumerate(states) if s == st]
            axt.scatter(ts[idx], [lane_of[st]] * len(idx), marker='s', s=4,
                        color=STATE_COLOR.get(st, '#333333'))
        axt.set_yticks(range(len(lanes)))
        axt.set_yticklabels(lanes, fontsize=8)
        axt.set_ylim(-0.7, len(lanes) - 0.3)
    axt.set_xlabel('t (s)')
    axt.set_title('state timeline', fontsize=10)
    axt.grid(True, axis='x', ls=':', lw=0.4, alpha=0.5)
    for st, sec in sorted(times.items(), key=lambda kv: -kv[1])[:4]:
        pct = 100.0 * sec / dur
        axt.text(1.002, lane_of.get(st, 0),
                 ' %.0f%%' % pct, transform=axt.get_yaxis_transform(),
                 va='center', fontsize=7, color=STATE_COLOR.get(st, '#333'))

    # ---------------- speed / bag over time ----------------
    # Speed by central difference on the trace: the trace does not always
    # carry the commanded v (and where it does it is the *command*, not what
    # the robot achieved), so measuring the pose is both simpler and more
    # honest. A stationary robot jitters, so this still shows a near-zero
    # floor rather than exactly zero.
    with np.errstate(divide='ignore', invalid='ignore'):
        v = np.abs(np.gradient(np.hypot(xs, ys), ts))
    v = np.nan_to_num(v)
    axb.plot(ts, v, color='#1f77b4', lw=1.0, label='speed (m/s)')
    axb.set_xlabel('t (s)')
    axb.set_ylabel('speed (m/s)', color='#1f77b4', fontsize=9)
    axb.tick_params(axis='y', labelcolor='#1f77b4', labelsize=8)
    axb.grid(True, ls=':', lw=0.4, alpha=0.5)

    axc = axb.twinx()
    axc.step(ts, bags, where='post', color='#d62728', lw=1.3,
             label='bag')
    axc.set_ylabel('bag', color='#d62728', fontsize=9)
    axc.tick_params(axis='y', labelcolor='#d62728', labelsize=8)
    axb.set_title('speed and hopper load', fontsize=10)

    fig.suptitle('tennis_bot %s: %d balls, %.1f s, delivered %d, '
                 'max bag %d, %.1f m travelled'
                 % (label, nballs, ts[-1] - ts[0], rows[-1].get('dep', 0),
                    int(bags.max()) if len(bags) else 0, distance_m(rows)),
                 fontsize=12)

    fig.savefig(path, dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('trace', nargs='?', default=None,
                    help='trace jsonl (default: newest in logs/)')
    ap.add_argument('outdir', nargs='?', default=None,
                    help='output directory (default: alongside the trace)')
    ap.add_argument('--label', default=None,
                    help='basename for the generated files')
    args = ap.parse_args()

    trace = args.trace or newest_trace()
    if not os.path.exists(trace):
        raise SystemExit('no such trace: %s' % trace)
    outdir = args.outdir or os.path.dirname(os.path.abspath(trace))
    if not os.path.isdir(outdir):
        os.makedirs(outdir)
    label = args.label or os.path.splitext(os.path.basename(trace))[0]

    meta, rows = load(trace)

    csv_path = os.path.join(outdir, label + '.csv')
    png_path = os.path.join(outdir, label + '.png')
    sum_path = os.path.join(outdir, label + '_summary.csv')

    row = summary_row(rows, meta, label, trace)
    write_samples_csv(csv_path, rows, meta)
    write_summary_csv(sum_path, row)
    write_plot(png_path, rows, meta, label)

    spans = stationary_spans(rows)
    print('wrote %s' % png_path)
    print('wrote %s' % csv_path)
    print('wrote %s' % sum_path)
    print('  %.1f s, %.1f m, delivered %d, max bag %d, %d state transits'
          % (float(row['duration_s']), float(row['distance_m']),
             int(row['delivered']), int(row['max_bag']),
             int(row['state_transits'])))
    if spans:
        top = spans[:3]
        print('  longest stalls: ' + ', '.join(
            '%.1fs at (%.2f, %.2f) %s' % (s['duration'], s['x'], s['y'],
                                          '/'.join(s['states']))
            for s in top))
    else:
        print('  no stationary stretch longer than %.1f s'
              % STATIONARY_TIME)


if __name__ == '__main__':
    main()