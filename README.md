# R2S — real-to-sim physics estimation from robot interaction logs

Can a robot's own interaction logs (joint torques or a wrist force/torque sensor, plus camera images) be turned into a
physically faithful simulation of the objects it handled — with mass, centre of mass, inertia and friction estimated rather
than guessed? This repository validates that core claim against two published methods and ground-truth data, and ships the
resulting pipeline, `r2s_pipeline`.

**Status (Sept 2026): research code.** Headline findings, each measured against ground truth (details below):

| Quantity | From robot motion / torque | From reconstructed geometry | Verdict |
|---|---|---|---|
| Mass | **0.23–0.98% median error** even from gentle motion (≤ 0.39 m/s²) | — | identify from torque / wrench |
| Centre of mass | 3.6–8 mm median | 1.0 mm (CAD mesh) | either; geometry when the mesh is good |
| Inertia | **300–800% error, 0/20 within 10%** — not identifiable at payload scale | **4.4% median** (CAD mesh), 12% (vision mesh) | take from geometry + a density prior |

## What was tested

### 1. One-Shot Real-to-Sim (RigidWorldModel, arXiv:2412.00259 v4, RA-L 2025)

- Environment rebuilt from scratch (conda, Python 3.9, torch 2.1 + CUDA 12.1, pytorch3d from source); seven fixes the upstream
  README does not mention are listed in `STATUS.md`.
- 3-seed replication of the shipped (paper v4) configuration on the Drill: **mass error 0.162 kg (20.2%), s.d. 0.087**.
- 8-run initialisation sweep: the final mass tracks the *initial seed* (0.5 → 0.81, 0.9 → 0.93, 1.3 → 1.27 kg). The optimiser
  anchors on initialisation; the data exerts almost no pull. Inertia is a bounding-box formula, not the reconstructed mesh.
- Scripts: `run_phase0.sh`, `run_v4_replication.sh`, `run_sweep.sh`, `phase0_report.py`, `v4_report.py`, `sweep_report.py`.

### 2. Scalable Real2Sim (arXiv:2503.00370) — tests T1–T5 on spam / sugar / lego (`s2s_tests.py`, `results/s2s_tests.json`)

| Test | Result |
|---|---|
| T1 reproduce shipped mass | **PASS** to 6 decimals on all 3 objects |
| T2 mass with less excitation | stable down to the gentlest 50% of samples (≤ 3.7% drift); collapses below 25%. `cond(W)` is a ground-truth-free identifiability check (~5 healthy, > 20 failing) |
| T3 inertia source | geometry-derived inertia beats torque-identified inertia on held-out torque, 3/3 objects; the shipped inertia is ~0.025 kg·m² on two axes for *every* object — a robot-model artefact |
| T4 bounding-ellipsoid option | no effect (0.0%), and its code path is broken on current Drake |
| T5 held-out torque | good on joints 0–4; no signal on wrist joints 5–6 |

The released benchmark contains **no object with ground-truth physics** (the paper's 3D-printed calibration object is not in
it), so inertia accuracy cannot be validated from its public data.

### 3. Ground truth: 20 workshop tools (utiasSTARS dataset; `utias_gentle_motion_test.py`, `results/utias_*.json`)

Identification from gentle motion (wrist-wrench Newton–Euler regressor, fit on the first 70%, held-out 30%):

| Setting | Mass median | Mass ≤ 5% | CoM median | Inertia median | Inertia ≤ 10% |
|---|---|---|---|---|---|
| clean, least squares | 0.23% | 17/20 | 8.0 mm | 786% | 0/20 |
| clean, physically-consistent SDP | 0.27% | 16/20 | 8.3 mm | 303% | 0/20 |
| + 0.3 N / 0.005 N·m noise | 0.78% | 14/20 | 15.9 mm | 852% | 0/20 |
| gentlest 10% of motion (≤ 0.39 m/s²), noisy | 0.98% | — | 3.6 mm | 4272% | — |

Uniform-density geometry inertia vs ground truth: Frobenius error median **4.4%** (CAD mesh; 0.0% for single-material tools,
18.4% median for multi-material ones), 12% on coarse vision-reconstructed meshes. The multi-material residual is where a
material / part prior (part segmentation or a VLM) helps.

### 4. The hybrid pipeline (`hybrid_pipeline.py`, `eval_holdout.py`, `results/hybrid/`)

Mass + CoM from torque (SDP), inertia from the voxelised mesh. 5-fold held-out torque reconstruction, mean over spam / sugar /
lego:

| Method | rel. RMSE joints 0–4 | abs. RMSE (N·m) | mass CV | inertia CV across folds | gyration ratio |
|---|---|---|---|---|---|
| torque only | 0.1111 | 0.436 | 3.6% | 50% | 3.87 (physically impossible) |
| hybrid | 0.1102 | 0.439 | 3.6% | **4%** | **0.68** |
| hybrid + geometric CoM | **0.1065** | **0.422** | 3.6% | 4% | 0.68 |

Torque prediction is insensitive to the inertia source for objects under 0.5 kg — which is exactly why torque-identified
inertia is meaningless there, and why inertia must come from geometry.

### 5. Articulation axes from geometry (`axis_recovery_test.py`, `results/axis_test/`)

Synthetic stand-in for PartNet-Mobility (flush hinge, 6 mm gap, barrel hinge, face contact, prismatic drawer), written out as
URDF + OBJ and reloaded through the same loader: **4/5 pass** (direction < 2°, position < 5 mm), identical with noise and dropped
faces. The barrel-hinge laptop fails structurally (the axis lies on no contact surface) and needs interaction or a learned prior.
A semantic hint (e.g. "lid") is necessary for face-like contacts.

### 6. RH20T real-robot episodes (`rh20t_splat/`)

Gaussian-splat reconstruction of RH20T workspaces from the dataset's calibrated cameras (no pose estimation), object mass from
the wrist F/T segmented by gripper width, and the `r2s_pipeline` Newton–Euler fit on teleoperated episodes (which have no
static plateau). Reconstruction outputs are kept locally.

## The deliverable: `r2s_pipeline/`

A package (see `r2s_pipeline/README.md`) that turns a recording into inertial parameters:

- two sensing modes: wrist wrench, or joint torque through Drake with any URDF;
- automatic wrench-convention detection and empty-gripper baseline subtraction;
- mass + CoM from motion, inertia from geometry, friction from slip events;
- diagnostics: held-out torque error, conditioning, gyration ratio, symmetry warning;
- JSON / SDF / URDF output, and a hardware-free self-test on ground truth (**PASS, mass 0.06–1.8% on 5 objects**).

On the Scalable Real2Sim spam recording in joint-torque mode with the nominal iiwa model: **0.3658 kg vs 0.3780 kg reference
(3.2%)**. Without the empty-gripper baseline the 2.53 kg gripper lands in the payload estimate, so the baseline recording is
the most important item in the recording spec.

Recording on a KUKA LBR Med7 (joint torque sensors, no wrist F/T needed): `RECORDING_SPEC.md`, `MED7_RECORDING.md`,
`record_lbr.py`, `med7.urdf`.

## Repository layout

```
r2s_pipeline/          the pipeline package (CLI: python -m r2s_pipeline)
hybrid_pipeline.py     mass/CoM from torque + inertia from geometry, SDF output
eval_holdout.py        K-fold held-out torque reconstruction
s2s_tests.py           Scalable Real2Sim tests T1–T5
utias_gentle_motion_test.py, axis_recovery_test.py, warp_material_demo.py
run_*.sh, *_report.py  One-Shot (RigidWorldModel) replication, sweeps and reports
rh20t_splat/           RH20T reconstruction and identification scripts
record_lbr.py, med7.urdf, RECORDING_SPEC.md, MED7_RECORDING.md   recording on a KUKA LBR Med7
results/               all measured results (JSON / logs / SDF)
third_party/           upstream repos (URLs, pinned commits, licences) and our patches to them
STATUS.md              full lab notebook, including environment fixes and every finding
Setup_guide.md         the original plan and success criteria
```

## Setup

Third-party code is not vendored: follow `third_party/UPSTREAM.md` to clone it at the pinned commits and apply our patches.
Two conda environments were used: `rigidworldmodel` (Python 3.9, torch 2.1 + CUDA 12.1, pytorch3d 0.7.5 from source) for
One-Shot, and `s2s` (Python 3.10, Drake 1.51.1) for Scalable Real2Sim and this pipeline. Run with `PYTHONNOUSERSITE=1` and an
empty `PYTHONPATH` if ROS is installed. See `STATUS.md` for the exact fixes.

## Data (not in this repository)

| Dataset | Size | Source |
|---|---|---|
| Scalable Real2Sim benchmark | 38 GB | Hugging Face `nepfaff/scalable-real2sim` |
| RH20T (cfg5, cfg7, depth, low-dim, calibration) | 126 GB | https://rh20t.github.io |
| RigidWorldModel Drill (`train1`) | 2 GB | link in the RigidWorldModel README |
| utiasSTARS workshop tools | 85 MB | included in the `utias-inertial` upstream repo |

## Licence

No licence has been chosen yet; until one is added, the default copyright applies. Upstream code and datasets keep their own
licences (all upstream repositories above are MIT; `robot_payload_id` has no licence file).
