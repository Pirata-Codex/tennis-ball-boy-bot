"""Compare per-iteration runs: collect every *_summary.csv into one table.

    python tools/compare_iterations.py                       # logs/iterations
    python tools/compare_iterations.py logs/iterations --csv logs/iterations/compare.csv

Each row is one run. The columns worth watching across a series are
`distance_m`, `delivered`, `max_bag`, `state_transits` and especially
`longest_stall_s` - across FINDINGS 5.27-5.47 every deadlock fix moved
`longest_stall_s` and `state_transits` first, before it moved anything else.

Prints the table plus a matplotlib trend plot when a series has >= 3 runs, since
a number per line is easy to misread across twenty lines.
"""

import argparse
import csv
import glob
import os
import sys

# columns shown by default, in reading order
SHOW = ['label', 'duration_s', 'distance_m', 'delivered', 'max_bag',
        'state_transits', 'longest_stall_s', 'stall_count', 'worst_stall_state']


def newest(path):
    rows = []
    for f in sorted(glob.glob(os.path.join(path, '*_summary.csv'))):
        with open(f, 'r', encoding='utf-8', newline='') as fh:
            got = list(csv.DictReader(fh))
        if got:
            rows.extend(got)
    return rows


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def order_key(r):
    """Sort by the run number in the label, so seed100 comes before seed99's
    later series and a merged directory stays readable."""
    import re
    m = re.search(r'(\d+)', r.get('label', ''))
    return (int(m.group(1)) if m else 0, r.get('label', ''))


def print_table(rows, cols):
    cols = [c for c in cols if any(c in r for r in rows)]
    widths = []
    for c in cols:
        w = max([len(c)] + [len(str(r.get(c, ''))) for r in rows])
        widths.append(min(w, 18))
    print('  '.join(c[:w].ljust(w) for c, w in zip(cols, widths)))
    print('  '.join('-' * w for w in widths))
    for r in rows:
        cells = []
        for c, w in zip(cols, widths):
            v = str(r.get(c, ''))
            cells.append(v[:w].ljust(w))
        print('  '.join(cells))


def trend_plot(rows, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    labels = [r.get('label', '?') for r in rows]
    series = [
        ('distance_m', '#1f77b4'),
        ('delivered', '#2ca02c'),
        ('max_bag', '#17becf'),
        ('longest_stall_s', '#d62728'),
        ('state_transits', '#9467bd'),
    ]
    have = [(c, col) for c, col in series
            if any(num(r.get(c)) is not None for r in rows)]
    if len(rows) < 3 or not have:
        print('need >= 3 runs to plot a trend')
        return

    fig, ax = plt.subplots(figsize=(max(6, len(rows) * 0.7), 5))
    x = range(len(rows))
    for name, col in have:
        ys = [num(r.get(name)) for r in rows]
        pts = [(i, y) for i, y in enumerate(ys) if y is not None]
        if not pts:
            continue
        ax.plot([p[0] for p in pts], [p[1] for p in pts], marker='o',
                lw=1.4, ms=4, color=col, label=name)
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, rotation=60, ha='right', fontsize=7)
    ax.set_ylabel('value (series have different units)')
    ax.set_title('per-iteration metrics across runs')
    ax.grid(True, ls=':', lw=0.4, alpha=0.6)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print('wrote %s' % out)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('path', nargs='?', default=None,
                    help='directory of *_summary.csv (default logs/iterations)')
    ap.add_argument('--csv', default=None,
                    help='write the merged table here')
    ap.add_argument('--plot', default=None,
                    help='write the trend plot here')
    args = ap.parse_args()

    path = args.path
    if path is None:
        path = os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), 'logs', 'iterations')
    if not os.path.isdir(path):
        raise SystemExit('no such directory: %s' % path)

    rows = newest(path)
    if not rows:
        raise SystemExit('no *_summary.csv in %s' % path)
    rows.sort(key=order_key)

    print_table(rows, SHOW)

    if args.csv:
        cols = list(rows[0].keys())
        for r in rows:
            for c in cols:
                r.setdefault(c, '')
        with open(args.csv, 'w', newline='', encoding='utf-8') as fh:
            wr = csv.DictWriter(fh, fieldnames=cols)
            wr.writeheader()
            wr.writerows(rows)
        print('wrote %s' % args.csv)

    if args.plot:
        trend_plot(rows, args.plot)
    else:
        trend_plot(rows, os.path.join(path, 'compare_trend.png'))


if __name__ == '__main__':
    main()