#!/usr/bin/env bash
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)/RigidWorldModel"   # clone: see third_party/UPSTREAM.md
PY="${PY:-$HOME/miniconda3/envs/rigidworldmodel/bin/python}"
cd "$REPO"; unset PYTHONPATH
export WANDB_MODE=offline WANDB_SILENT=true
for t in m05_a m05_b m09_a m09_b m13_a m13_b vlm_a vlm_b; do
  echo "########## SWEEP $t ##########"
  "$PY" main_stage_2.py --config=configs/sweep_$t.yaml --use_wandb 2>&1 | tail -3
  echo "--- $t done ---"
done
echo "########## SWEEP COMPLETE ##########"
