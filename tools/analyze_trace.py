"""Summarise a tennis_bot trace: where time went and where the robot got stuck.

    python tools/analyze_trace.py                 # newest trace in logs/
    python tools/analyze_trace.py <trace.jsonl>

Prints the state-time breakdown, distance travelled, the visited area, the
longest stationary stretches (with position) and how many balls were delivered.
"""

import glob
import json
import os
import sys


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
    if meta is None or not rows:
        raise SystemExit('trace %s is missing meta or samples' % path)
    return meta, rows


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else newest_trace()
    meta, rows = load(path)
    n = len(rows)

    print('trace        %s' % path)
    print('samples      %d over %.1f s' % (n, rows[-1]['t']))
    if n > 1:
        dt = (rows[-1]['t'] - rows[0]['t']) / (n - 1)
        print('sample rate  %.2f s  (%.1f Hz)' % (dt, 1.0 / dt if dt else 0))

    # state time
    time_in = {}
    for a, b in zip(rows, rows[1:]):
        time_in[a['s']] = time_in.get(a['s'], 0.0) + (b['t'] - a['t'])
    print('\nstate time:')
    for st, secs in sorted(time_in.items(), key=lambda kv: -kv[1]):
        print('  %-12s %6.1f s  %4.1f%%' % (st, secs, 100.0 * secs / rows[-1]['t']))

    # distance and area
    dist = 0.0
    for a, b in zip(rows, rows[1:]):
        dist += ((b['x'] - a['x']) ** 2 + (b['y'] - a['y']) ** 2) ** 0.5
    xs = [r['x'] for r in rows]
    ys = [r['y'] for r in rows]
    print('\ndistance     %.1f m' % dist)
    print('x range      %.2f .. %.2f' % (min(xs), max(xs)))
    print('y range      %.2f .. %.2f' % (min(ys), max(ys)))
    print('delivered    %d of %d' % (rows[-1]['dep'], meta.get('balls', '?')))
    print('max bag      %d' % max(r['bag'] for r in rows))

    # longest stationary stretches (> 1 s in a 0.25 m box)
    print('\nlongest stationary stretches:')
    stretches = []
    i = 0
    while i < n - 1:
        j = i + 1
        while j < n:
            d = ((rows[j]['x'] - rows[i]['x']) ** 2 +
                 (rows[j]['y'] - rows[i]['y']) ** 2) ** 0.5
            if d > 0.25:
                break
            j += 1
        span = rows[min(j, n - 1)]['t'] - rows[i]['t']
        if span > 1.0:
            stretches.append((span, rows[i]['x'], rows[i]['y'], rows[i]['s']))
        i = j if j > i else i + 1
    stretches.sort(reverse=True)
    for span, x, y, st in stretches[:6]:
        print('  %5.1f s at (%+.2f, %+.2f)  state %s' % (span, x, y, st))

    # time spent with a dead-end state log
    print('\nstate sequence (compressed):')
    seq = []
    for r in rows:
        if not seq or seq[-1][0] != r['s']:
            seq.append([r['s'], r['t'], r['t']])
        else:
            seq[-1][2] = r['t']
    for st, t0, t1 in seq[:40]:
        print('  %-12s %6.1f .. %6.1f s' % (st, t0, t1))


if __name__ == '__main__':
    main()
