# Experiments

Each folder answers one question. Scripts write into [`../results/`](../results/); the figures in the top-level README are
built from those files by [`../docs/figures/make_figures.py`](../docs/figures/make_figures.py).

| # | Folder | Question | Answer | Results |
|---|---|---|---|---|
| 1 | [`oneshot/`](oneshot/) | Does One-Shot Real-to-Sim recover mass from a push? | No. The estimate follows its starting value (20% error from the shipped seed). | [`oneshot_traces.json`](../results/oneshot_traces.json), [`oneshot/logs/`](oneshot/logs/) |
| 2 | [`scalable_real2sim/`](scalable_real2sim/) | Does Scalable Real2Sim's torque identification hold up? | Mass yes, down to the gentlest half of the data. Inertia no: the shipped value is a robot-model artefact. | [`s2s_tests.json`](../results/s2s_tests.json) |
| 3 | [`ground_truth/`](ground_truth/) | What is identifiable from gentle motion, against CAD ground truth? | Mass under 1%. Inertia 300–800% wrong from motion, 4.4% from geometry. | [`utias_gentle_motion.json`](../results/utias_gentle_motion.json), [`utias_geometry_inertia.json`](../results/utias_geometry_inertia.json), [`utias_signal_budget.json`](../results/utias_signal_budget.json) |
| 4 | [`scalable_real2sim/`](scalable_real2sim/) | Does taking inertia from geometry cost held-out accuracy? | No. Same torque error, inertia spread 50% → 4%, gyration ratio 3.9 → 0.68. | [`hybrid/`](../results/hybrid/) |
| 5 | [`articulation/`](articulation/) | Can joint axes be read off part geometry? | 4 of 5 synthetic joints; barrel hinges need interaction or a prior. | [`axis_recovery.log`](../results/axis_recovery.log), [`axis_test/`](../results/axis_test/) |
|   | [`materials/`](materials/) | How would deformable materials be identified? | A Warp pattern demo on synthetic data: stiffness recovered within 0.5% for three constitutive laws, with and without the force channel. | printed to the terminal |

Experiment 6 (RH20T) lives in [`../rh20t_splat/`](../rh20t_splat/).

## Running them

All commands are run from the repository root. The scripts locate the third-party clones and datasets relative to the root
(see [`../third_party/UPSTREAM.md`](../third_party/UPSTREAM.md)); `PY` below is the interpreter of the named conda environment.

```bash
# 1 · One-Shot Real-to-Sim (env: rigidworldmodel, GPU; about 30 min per stage-2 run)
bash experiments/oneshot/run_phase0.sh              # stage 1 -> stage 2 -> report
bash experiments/oneshot/run_v4_replication.sh      # 3 runs of the shipped configuration
bash experiments/oneshot/run_sweep.sh               # 8-run initialisation sweep
$PY experiments/oneshot/sweep_report.py             # table from the wandb offline runs
$PY experiments/oneshot/export_traces.py            # -> results/oneshot_traces.json

# 2, 4 · Scalable Real2Sim (env: s2s; needs the 38 GB benchmark in s2s-data/)
$PY experiments/scalable_real2sim/s2s_tests.py spam sugar lego
$PY experiments/scalable_real2sim/hybrid_pipeline.py spam sugar lego
$PY experiments/scalable_real2sim/eval_holdout.py 5 spam sugar lego

# 3 · ground truth (env: s2s; needs only the utias-inertial clone, a few seconds)
$PY experiments/ground_truth/utias_gentle_motion_test.py
$PY experiments/ground_truth/signal_budget.py

# 5 · articulation (env: s2s, under a second)
$PY experiments/articulation/axis_recovery_test.py

# materials (env: s2s with warp, GPU; about two minutes)
$PY experiments/materials/warp_material_demo.py
```

On a machine with ROS installed, prefix with `env -u PYTHONPATH PYTHONNOUSERSITE=1`.
