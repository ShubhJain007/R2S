#!/usr/bin/env bash
# Phase 0 - One-Shot Real-to-Sim sim round-trip on the Drill object.
# Runs stage 1 (geometry+appearance) -> stage 2 (physics identification) -> report.
#
# Usage:  bash experiments/oneshot/run_phase0.sh
# Logs:   RigidWorldModel/checkpoints/phase0_{stage1,stage2}.log
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)/RigidWorldModel"   # clone: see third_party/UPSTREAM.md
PY="${PY:-$HOME/miniconda3/envs/rigidworldmodel/bin/python}"
cd "$REPO"

# The shell exports a ROS Humble python3.10 PYTHONPATH that shadows this 3.9 env.
unset PYTHONPATH
# wandb offline: gives a local per-epoch mass/friction/com trace with no account.
export WANDB_MODE=offline
export WANDB_SILENT=true

echo "=== GPU precheck ==="
if ! "$PY" -c "import torch,sys; sys.exit(0 if torch.cuda.is_available() else 1)"; then
  echo "FAIL: CUDA not available."
  nvidia-smi 2>&1 | head -3 || true
  echo
  echo "Known cause on this machine: the loaded NVIDIA kernel module (580.173.02) does not"
  echo "match the installed userspace libraries (580.178.04). The correct module is already"
  echo "built by DKMS for kernel 6.8.0-138 - it just is not the one currently loaded."
  echo "Fix: sudo reboot     then re-run this script."
  exit 1
fi
"$PY" -c "import torch; print('GPU:', torch.cuda.get_device_name(0), '| torch', torch.__version__)"

mkdir -p checkpoints

echo
echo "=== STAGE 1: geometry + appearance (30 epochs) ==="
time "$PY" main_stage_1.py --config=configs/drill_stage_1.yaml --use_wandb \
  2>&1 | tee checkpoints/phase0_stage1.log

echo
echo "=== STAGE 2: physics identification - mass / friction / com (25 epochs) ==="
time "$PY" main_stage_2.py --config=configs/drill_stage_2.yaml --use_wandb \
  2>&1 | tee checkpoints/phase0_stage2.log

echo
echo "=== PHASE 0 REPORT ==="
"$PY" "$HERE/phase0_report.py" "$REPO/checkpoints/drill2"
