# Results

Everything the experiments measured. Nothing here is hand-edited; each file is written by the script named next to it.

| File | Written by | Contents |
|---|---|---|
| `oneshot_traces.json` | `experiments/oneshot/export_traces.py` | per-epoch mass and friction for the 11 One-Shot stage-2 runs (3 replication + 8 sweep) |
| `T1_spam_default.{json,log,json.npy}` | upstream `identify_grasped_object_payload.py` | Scalable Real2Sim's own output for spam, the reference for test T1 |
| `s2s_tests.json`, `s2s_tests_{all,spam}.log` | `experiments/scalable_real2sim/s2s_tests.py` | tests T1–T5 on spam / sugar / lego |
| `utias_gentle_motion.{json,log}` | `experiments/ground_truth/utias_gentle_motion_test.py` | mass / CoM / inertia from gentle motion on 20 ground-truth tools, clean and noisy, LS and SDP |
| `utias_geometry_inertia.{json,log}` | lab-notebook run (2026-09-17) | uniform-density geometry inertia against ground truth, CAD and vision-reconstructed meshes |
| `utias_signal_budget.{json,log}` | `experiments/ground_truth/signal_budget.py` | RMS wrench contributed by mass, CoM and inertia per tool, against F/T noise |
| `hybrid/holdout_K5.{json,log}` | `experiments/scalable_real2sim/eval_holdout.py` | 5-fold held-out torque reconstruction, torque-only vs hybrid |
| `hybrid/<obj>_hybrid_inertial_params.json`, `hybrid/<obj>_hybrid.sdf` | `experiments/scalable_real2sim/hybrid_pipeline.py` | identified parameters and the shipped SDF with its `<inertial>` block replaced |
| `hybrid/spam_recording.npz`, `hybrid/baseline_gripper0.05.npz` | converted from the Scalable Real2Sim benchmark | a 10 s joint-torque recording with and without the object: the demo input in the top-level README |
| `hybrid/spam_r2s_pipeline.json` | `python -m r2s_pipeline identify` | the pipeline's output for that recording (0.3658 kg against a 0.3780 kg reference) |
| `axis_recovery.log`, `axis_test/<object>/` | `experiments/articulation/axis_recovery_test.py` | joint-axis recovery table, and the five synthetic objects as URDF + OBJ |
| `rh20t_splat/` | `rh20t_splat/*.py` | RH20T calibration checks, splat renders, object mesh, fused cloud, demo media (CC BY-SA 4.0, see its README) |
