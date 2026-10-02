"""Per-ball approach diagnostics from a trace.

    python tools/approach_debug.py [trace.jsonl]

For each ball: closest approach in the robot frame, whether it ever reached the
mouth (0.30<lx<0.60, |ly|<0.08), and whether it entered the hopper. Then dumps
the robot-frame (lx,ly) of the nearest ball over time so you can see whether the
robot centres the ball or drives past it.
"""
import json, math, sys, glob, os

path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'logs', 'trace_world.jsonl')
rows, meta = [], None
for line in open(path, encoding='utf-8'):
    line = line.strip()
    if not line:
        continue
    try:
        o = json.loads(line)
    except ValueError:
        continue
    if o.get('meta'): meta = o
    elif 'x' in o: rows.append(o)
nb = meta['balls']

def rf(o, i):
    rx, ry, h = o['x'], o['y'], o['h']; hr = math.radians(h)
    bx, by, fl = o['b'][i]
    dx, dy = bx - rx, by - ry
    lx = math.cos(-hr) * dx - math.sin(-hr) * dy
    ly = math.sin(-hr) * dx + math.cos(-hr) * dy
    return lx, ly, fl

best = {i: (9, 0, 0, 0, '') for i in range(nb)}
mouth = {i: 0 for i in range(nb)}
inh = {i: False for i in range(nb)}
for o in rows:
    for i in range(nb):
        if i >= len(o['b']):
            continue
        lx, ly, fl = rf(o, i)
        if fl: inh[i] = True
        if 0.30 < lx < 0.60 and abs(ly) < 0.08: mouth[i] += 1
        d = math.hypot(lx, ly)
        if d < best[i][0]: best[i] = (d, lx, ly, o['t'], o['s'])
for i in range(nb):
    d, lx, ly, t, s = best[i]
    print("ball%d closest d=%.3f (lx=%+.3f ly=%+.3f) @t=%.1f %-7s mouth_frames=%d in_hopper=%s"
          % (i, d, lx, ly, t, s, mouth[i], inh[i]))
print("final balls:", [[round(v, 2) for v in b] for b in rows[-1]['b']], "t=%.1f" % rows[-1]['t'])
