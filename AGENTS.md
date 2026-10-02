# tennis_bot — working rules for this repository

## Never delete iteration artifacts

`logs/iterations/` is an **append-only record**. Every run writes
`runNNN.png`, `runNNN.csv`, `runNNN_summary.csv` and `trace_*.jsonl` there, and
those files are the only record of what a given run actually did.

* **Do not** `Remove-Item logs/iterations/*`, `rm -rf`, or clean them up.
* Do not "start fresh" by clearing the directory.
* Regenerating them is fine; deleting them is not. Several plots from earlier
  batches were lost this way and could not be recovered — the experience rows
  survived in `experience.jsonl`, but the plots and per-sample CSVs are gone.

`tools/collect_experience.py` enforces this: it never overwrites an existing
label, and picks `runNNN_2`, `runNNN_3`, … instead, so even reusing a seed
cannot clobber history.

`compare.csv` and `compare_trend.png` are the exception. They are *derived* from
every `*_summary.csv` in the directory, so overwriting them is correct — they
always reflect the full set.

To inspect or reduce the data, filter at read time instead:

    python tools/compare_iterations.py logs/iterations

## World-file edits must be validated with rendering enabled

All the physics test runs use `--no-rendering`. A world-file syntax error is a
hard load failure that `--no-rendering` can hide, and one such error
(`Track` as a `Viewpoint` child, which R2025a rejects) shipped unnoticed until a
deliberately rendered run caught it. After editing any `.wbt`:

    webots --batch --mode=fast --stdout --stderr worlds/<world>.wbt

and confirm zero `error` lines. See FINDINGS.md §5.52.

## A training run that writes nothing is a failure

`experience.jsonl` is append-only and shared. If a run adds 0 rows, treat it as
a crash, not a successful run — the controller can die in `__init__` and Webots
still exits 0. `collect_experience.py` reports
`WARNING: no report for this run` in that case. See FINDINGS.md §5.49.

## Current state

See the "Still unproven" notes at the end of FINDINGS.md. In short: the robot
collects and delivers (4/10 in a 150 s run), but the **learned policy has not
yet beaten the hand-written DWA planner**, and the buffer is still far smaller
than the state space. Do not let the policy drive on its own judgement yet.