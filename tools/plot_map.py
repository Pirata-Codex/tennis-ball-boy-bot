"""Plot the map the robot built for itself (logs/map_<world>.json).

    python tools/plot_map.py                     # every logs/map_*.json
    python tools/plot_map.py logs/map_tennis_court.json [out.png]

Shows the floor cells the ball sensor has looked at, the obstacle cells, the
remembered balls (filled = collected, hollow = still on the floor).
"""
import glob
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot(path, out=None):
    with open(path, encoding='utf-8') as fh:
        m = json.load(fh)
    cell = m['cell']
    xmin, xmax, ymin, ymax = m['bounds']
    pad = 0.6
    x0, x1, y0, y1 = xmin - pad, xmax + pad, ymin - pad, ymax + pad
    ix0, iy0 = int(np.floor(x0 / cell)), int(np.floor(y0 / cell))
    nx = int(np.ceil((x1 - x0) / cell)) + 1
    ny = int(np.ceil((y1 - y0) / cell)) + 1
    grid = np.zeros((ny, nx))              # 0 unknown, 1 looked at, 2 occupied
    for key, val in (('looked_at', 1), ('occupied', 2)):
        for ix, iy in m.get(key, []):
            gx, gy = ix - ix0, iy - iy0
            if 0 <= gx < nx and 0 <= gy < ny:
                grid[gy, gx] = val

    fig, ax = plt.subplots(figsize=(7, 7 * (y1 - y0) / (x1 - x0) + 0.6))
    cmap = matplotlib.colors.ListedColormap(['#d9d9d9', '#cfe8cf', '#333333'])
    ax.imshow(grid, origin='lower', cmap=cmap, vmin=0, vmax=2,
              extent=(ix0 * cell, (ix0 + nx) * cell,
                      iy0 * cell, (iy0 + ny) * cell), interpolation='nearest')
    ax.add_patch(plt.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                               fill=False, ls='--', ec='#666'))

    balls = m.get('balls', [])
    for b in balls:
        ax.plot(b['x'], b['y'], 'o', ms=9, mec='#b8860b',
                mfc='#e6d600' if b['collected'] else 'none', mew=2)
        ax.annotate('%d (%dx)' % (b['id'], b['sightings']), (b['x'], b['y']),
                    xytext=(5, 5), textcoords='offset points', fontsize=7)

    ax.plot([], [], 's', color='#cfe8cf', label='looked at')
    ax.plot([], [], 's', color='#333333', label='obstacle')
    ax.plot([], [], 'o', mfc='#e6d600', mec='#b8860b', label='remembered, collected')
    ax.plot([], [], 'o', mfc='none', mec='#b8860b', label='remembered, on floor')
    ax.legend(loc='upper right', fontsize=7, framealpha=0.85)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('%s  t=%.0f s  coverage %.0f%%  remembered %d  in bin %s'
                 % (os.path.basename(path), m.get('t', 0),
                    100 * m.get('coverage', 0), len(balls), m.get('in_bin', '?')),
                 fontsize=9)
    out = out or os.path.splitext(path)[0] + '.png'
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print('wrote', out)


if __name__ == '__main__':
    if len(sys.argv) > 1:
        plot(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
    else:
        here = os.path.dirname(os.path.abspath(__file__))
        for p in sorted(glob.glob(os.path.join(here, '..', 'logs', 'map_*.json'))):
            plot(p)
