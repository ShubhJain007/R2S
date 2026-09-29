#!/usr/bin/env bash
set -euo pipefail
REPO=/home/kneepolean/shubhj/R2S/RigidWorldModel
PY=/home/kneepolean/miniconda3/envs/rigidworldmodel/bin/python
cd "$REPO"; unset PYTHONPATH
export WANDB_MODE=offline WANDB_SILENT=true
echo "=== STAGE 2 LONG: 60 epochs (LR anneals to ~0.007) ==="
time "$PY" main_stage_2.py --config=configs/drill_stage_2_long.yaml --use_wandb
echo; echo "=== PHASE 0 REPORT (60-epoch run) ==="
"$PY" /home/kneepolean/shubhj/R2S/phase0_report.py "$REPO/checkpoints/drill2_long"
