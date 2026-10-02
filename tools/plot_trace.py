"""Plot a tennis_bot run: the court, the robot's path and the ball tracks.

The controller writes one JSONL sample every `TRACE_PERIOD` seconds to
`logs/trace_<world>.jsonl`:

    {"meta":true,"bounds":[xmin,xmax,ymin,ymax],"drop":[x,y],"balls":N}
    {"t":..,"s":"SEEK","x":..,"y":..,"h":..,"bag":..,"dep":..,
     "b":[[x,y,inside], ...]}

This script renders that trace to a PNG so a headless run can be inspected:
where the robot went, which states it was in, and where the balls were and
ended up.

Usage:
    python tools/plot_trace.py                  # newest trace in logs/
    python tools/plot_trace.py <trace.jsonl>
    python tools/plot_trace.py <trace.jsonl> out.png
"""

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
               'TO_DROP', 'DUMP', 'DONE', 'RECOVER_DOWN', 'DOWN']
STATE_COLOR = {
    'SEEK': '#1f77b4', 'SEARCH': '#17becf', 'ALIGN': '#ff7f0e',
    'COLLECT': '#d62728', 'RECOVER': '#9467bd', 'WALL_BACK': '#8c564b',
    'TO_DROP': '#2ca02c', 'DUMP': '#e377c2', 'DONE': '#7f7f7f',
    'RECOVER_DOWN': '#bcbd22', 'DOWN': '#000000',
}


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


def main():
    args = [a for a in sys.argv[1:]]
    trace = args[0] if args else newest_trace()
    out = args[1] if len(args) > 1 else os.path.splitext(trace)[0] + '.png'

    meta, rows = load(trace)
    xmin, xmax, ymin, ymax = meta['bounds']
    dx, dy = meta['drop']
    nballs = meta.get('balls', max(len(r['b']) for r in rows))
    xs = np.array([r['x'] for r in rows])
    ys = np.array([r['y'] for r in rows])
    hs = np.array([r['h'] for r in rows])
    states = [r['s'] for r in rows]
    ts = np.array([r['t'] for r in rows])

    fig, ax = plt.subplots(figsize=(11, 8))
    ax.set_title('tennis_bot run: %s\n%d balls, %.1f s, delivered %d'
                 % (os.path.basename(trace), nballs, ts[-1], rows[-1]['dep']))

    # court and net (the net is at x = 0, usually just outside the robot limits)
    ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                           fill=False, ec='#333333', lw=1.5))
    ax.plot([0, 0], [ymin, ymax], color='#888888', lw=2.5, ls='-')
    ax.text(0.06, ymax - 0.20, 'net', color='#666666', fontsize=9)
    ax.add_patch(Rectangle((dx - 0.85, dy - 0.85), 1.7, 1.7, fill=True,
                           fc='#d9f0d3', ec='#2ca02c', lw=1.2, alpha=0.7))
    ax.text(dx, dy, 'drop', color='#2ca02c', fontsize=9, ha='center',
            va='center')

    # ball tracks: start, track, end (collected balls highlighted)
    for bi in range(nballs):
        bx = [r['b'][bi][0] for r in rows if bi < len(r['b'])]
        by = [r['b'][bi][1] for r in rows if bi < len(r['b'])]
        if not bx:
            continue
        collected = any(r['b'][bi][2] for r in rows if bi < len(r['b']))
        ax.plot(bx, by, color='#f0c419' if collected else '#bbbbbb',
                lw=1.0, alpha=0.8, zorder=1)
        ax.scatter(bx[0], by[0], marker='o', s=40, facecolors='none',
                   edgecolors='#7a5c00', zorder=2)
        if collected:
            ax.scatter(bx[-1], by[-1], marker='*', s=170, color='#f0a500',
                       edgecolors='black', linewidths=0.5, zorder=5)
        else:
            ax.scatter(bx[-1], by[-1], marker='x', s=45, color='#e15759',
                       zorder=4)
            ax.text(bx[-1] + 0.06, by[-1], 'b%d' % bi, fontsize=7,
                    color='#e15759')

    # robot path, coloured by state
    ax.plot(xs, ys, color='#cccccc', lw=1.0, zorder=2)
    for st in STATE_ORDER:
        idx = [i for i, s in enumerate(states) if s == st]
        if idx:
            ax.scatter(xs[idx], ys[idx], s=8, color=STATE_COLOR.get(st, '#333'),
                       label=st, zorder=3)

    # heading arrows at a few points
    step = max(1, len(rows) // 25)
    for i in range(0, len(rows), step):
        rad = np.radians(hs[i])
        ax.add_patch(FancyArrow(xs[i], ys[i], 0.18 * np.cos(rad),
                                0.18 * np.sin(rad), width=0.0,
                                head_width=0.09, head_length=0.08,
                                color='#444444', zorder=4))
    ax.scatter(xs[0], ys[0], marker='s', s=60, color='black', zorder=6,
               label='start')
    ax.scatter(xs[-1], ys[-1], marker='D', s=55, color='#d62728', zorder=6,
               label='end')

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(xmin - 0.2, max(xmax, 0.0) + 0.2)
    ax.set_ylim(ymin - 0.2, ymax + 0.2)
    ax.grid(True, ls=':', lw=0.4, alpha=0.5)
    ax.legend(loc='upper right', fontsize=7, ncol=2, framealpha=0.9)

    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print('wrote %s' % out)


if __name__ == '__main__':
    main()
