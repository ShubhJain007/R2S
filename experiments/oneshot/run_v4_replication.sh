#!/usr/bin/env bash
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)/RigidWorldModel"   # clone: see third_party/UPSTREAM.md
PY="${PY:-$HOME/miniconda3/envs/rigidworldmodel/bin/python}"
cd "$REPO"; unset PYTHONPATH
export WANDB_MODE=offline WANDB_SILENT=true
for s in 1 2 3; do
  echo "########## V4 REPLICATION RUN $s ##########"
  time "$PY" main_stage_2.py --config=configs/drill_v4run$s.yaml --use_wandb
  echo "--- run $s done ---"
done
echo "########## ALL V4 RUNS DONE ##########"
