#!/usr/bin/env bash
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)/RigidWorldModel"   # clone: see third_party/UPSTREAM.md
PY="${PY:-$HOME/miniconda3/envs/rigidworldmodel/bin/python}"
cd "$REPO"; unset PYTHONPATH
export WANDB_MODE=offline WANDB_SILENT=true
for i in 1 2 3; do
  echo "########## PAPER TRIAL $i ##########"
  time "$PY" main_stage_2.py --config=configs/drill_s2_paper$i.yaml --use_wandb
  echo "--- trial $i done ---"
done
echo "########## ALL TRIALS DONE ##########"
