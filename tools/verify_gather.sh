#!/usr/bin/env bash
# Deterministic verification sweep for the gathering/turning fix.
# Runs each world with the learned policy disabled (TENNIS_NO_POLICY=1) so the
# DWA planner drives and results are reproducible, and prints capture/delivery.
# Requires an active GPU/display (Webots R2025a needs it, lidars especially).
#
# Usage:  bash tools/verify_gather.sh
set -u
WB="/e/Apps/Webots/msys64/mingw64/bin/webots.exe"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
run() {
  local world="$1" port="$2"
  rm -f logs/trace_world.jsonl
  TENNIS_NO_POLICY=1 timeout 180 "$WB" --batch --mode=fast --no-rendering \
      --stdout --stderr --port="$port" "worlds/$world" > "/tmp/verify_${port}.log" 2>&1
  printf '%-26s ' "$world"
  python tools/analyze_trace.py logs/trace_world.jsonl 2>/dev/null \
      | grep -E "delivered|max bag" | tr '\n' ' '
  echo
}
echo "=== gather verification (TENNIS_NO_POLICY=1) ==="
run mech_test.wbt 1401          # must stay: max bag 1  (no regression)
run mech_test_offset.wbt 1402   # target:    max bag > 0 (the fix)
echo "--- full court (goal: delivered > 0) ---"
run tennis_court.wbt 1403
