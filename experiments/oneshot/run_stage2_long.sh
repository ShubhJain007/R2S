#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)/RigidWorldModel"   # clone: see third_party/UPSTREAM.md
PY="${PY:-$HOME/miniconda3/envs/rigidworldmodel/bin/python}"
cd "$REPO"; unset PYTHONPATH
export WANDB_MODE=offline WANDB_SILENT=true
echo "=== STAGE 2 LONG: 60 epochs (LR anneals to ~0.007) ==="
time "$PY" main_stage_2.py --config=configs/drill_stage_2_long.yaml --use_wandb
echo; echo "=== PHASE 0 REPORT (60-epoch run) ==="
"$PY" "$HERE/phase0_report.py" "$REPO/checkpoints/drill2_long"
