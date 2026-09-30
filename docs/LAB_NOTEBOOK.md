# Lab notebook

> Chronological record, kept as it was written. The first section describes the state **before** the first run (a GPU
> driver mismatch that a reboot fixed); the dated updates below it are the findings. For the current summary read the
> [top-level README](../README.md).
>
> Paths are as they were at the time. The files have since moved: `run_*.sh` and `*_report.py` are in
> [`experiments/oneshot/`](../experiments/oneshot/), `s2s_tests.py`, `hybrid_pipeline.py` and `eval_holdout.py` in
> [`experiments/scalable_real2sim/`](../experiments/scalable_real2sim/), `utias_gentle_motion_test.py` in
> [`experiments/ground_truth/`](../experiments/ground_truth/), `axis_recovery_test.py` in
> [`experiments/articulation/`](../experiments/articulation/), `RECORDING_SPEC.md` in [`recording/`](../recording/), and
> this file was `STATUS.md`.

## Phase 0: status at handoff (2026-09-16)

**Bottom line:** the environment is complete and pre-flighted. Phase 0 has *not* run,
because the GPU is unusable until the machine is rebooted. That reboot is the only
outstanding manual step.

## Run it

```bash
sudo reboot                                  # required, see "GPU blocker" below
bash /home/kneepolean/shubhj/R2S/run_phase0.sh
```

`run_phase0.sh` does stage 1 (30 epochs) → stage 2 (25 epochs) → `phase0_report.py`.
It prechecks the GPU and refuses to start with a clear message if the driver is still
mismatched. Logs: `RigidWorldModel/checkpoints/phase0_stage{1,2}.log`.
Expect roughly 15 min/object on a 4090-class card, per the paper.

## GPU blocker

`nvidia-smi` fails with `Driver/library version mismatch`; torch reports CUDA error 804
(`forward compatibility was attempted on non supported HW`).

| | version |
|---|---|
| loaded kernel module (`/proc/driver/nvidia/version`) | 580.173.02 |
| userspace libs (`libcuda.so`, `libnvidia-ml.so`) | 580.178.04 |
| DKMS module built on disk for kernel 6.8.0-138 | 580.178.04 |

The driver was upgraded without a reboot. The correct module is already built — nothing
needs installing. I checked for a userspace-only workaround and there is none: no
`/usr/local/cuda/compat` directory exists and no older `libcuda` is present anywhere on
disk, so there is no library matching the running module. Reloading the module needs
root and the GPU is held by the display server, so in practice: reboot.

## What was set up

| Component | Version / licence |
|---|---|
| `RigidWorldModel` (One-Shot Real-to-Sim, RA-L 2025) | **MIT** |
| `shape_as_points` (author's fork, required, not a submodule) | **MIT** |
| vendored `diffsdfsim` + `lcp_physics` | **Apache-2.0** |
| conda env `rigidworldmodel` | Python 3.9.25 |
| torch / torchvision | 2.1.0+cu121 / 0.16.0+cu121 |
| pytorch3d | 0.7.5, built from source with CUDA (`sm_89`) |
| CUDA toolkit (in-env, for building) | 12.1 |

All licences are commercially permissive. Nothing non-commercial was pulled in.

Repo URL is **`github.com/yifanzhu95/RigidWorldModel`** — the guide only gave a project
page. It has no submodules, so `--recursive` is a no-op; the `shape_as_points` fork must
be cloned manually into `diffworld/shape_as_points`, as the README says.

## Fixes the README does not mention

1. `setuptools==69.5.1` — torch 2.1's `cpp_extension` does `from pkg_resources import packaging`, removed in setuptools ≥71.
2. **CUDA 12.1 is a hard requirement, not a suggestion.** pytorch3d's pulsar host code fails to compile against CUDA 12.4+ headers (`make_float3` ambiguous). The system toolkit here is 12.6. Upgrading pytorch3d does *not* help — the offending source is byte-identical in v0.7.5/0.7.7/0.7.9. Fix was installing CUDA 12.1 into the env and building with `CUDA_HOME` pointed at it.
3. `FORCE_CUDA=1` and `TORCH_CUDA_ARCH_LIST=8.9` are required to build pytorch3d here, because its setup.py gates CUDA on `torch.cuda.is_available()`, which is false while the driver is broken.
4. `opencv-python==4.10.0.84` — 5.x requires numpy≥2, which torch 2.1 cannot use.
5. `pip install --no-build-isolation py3ode` — its setup.py imports `pip`, absent in the isolated build env.
6. `pyrender` is needed (listed in requirements.txt but easy to miss; it is listed twice).
7. **numpy must stay at 1.26.4.** open3d and opencv both try to pull 2.x back in. Re-check after adding any package.

## Environment hazards on this machine

- `pip` on PATH is `~/.local/bin/pip` (python3.10 user-site) and **shadows the conda env's pip even after `conda activate`**. Always use `$ENV/bin/python -m pip`. Installing via bare `pip` will silently modify the system python instead.
- The shell exports a ROS Humble `PYTHONPATH` (python3.10) that leaks into the 3.9 env. `run_phase0.sh` unsets it; do the same for any manual run.
- `~/.config/pip/pip.conf` sets `no-cache-dir = true` and an NVIDIA extra index, which makes installs slow and re-download every time.

## Data

`data/drill/train1/` (from the README's Dropbox link, 443 MB → 1.8 GB). The tarball
extracts to `Drill/`, but the configs expect `drill/` — symlinked.

Dataloaders for both stages were instantiated successfully against the real configs and
data, so paths, zarr layout and formats are all verified. Contents:

| array | shape |
|---|---|
| `applied_forces` | (31, 3) |
| `obj_poses` / `robot_poses` | (31, 12) |
| `rgb_images` | (31, 720, 1024, 3) |
| `depth_images` / `masks` | (31, 720, 1024) |
| `timestamps` | (31, 1), dt = 0.01 |

The zarr's own `description` says *"a single box on flat ground"* — stale metadata, the
asset is the drill.

## Two corrections to Setup_guide.md

**§3 — only one object ships, not nine.** The repo provides the Drill only. It is
non-cylindrical, so it satisfies the §8 warning about rotationally-symmetric objects.

**§3 — inertia is not independently identifiable.** The guide asks for "mass within a
few % and inertia within ~10%", but the simulator does not treat inertia as a free
parameter. It derives it (`world.py:_custom_get_ang_inertia`):

```
I_ii = mass * inertia_scale_i * (sum of squared side lengths) / 12
```

`inertia_scale` is clamped to [0.5, 1.5]. No ground-truth inertia tensor ships. The only
ground truth available is in the comments of `configs/drill_stage_2.yaml`:
**mass 0.8 kg** (initialised at 0.2) and **friction 0.5** (initialised at 0.2).

So `phase0_report.py` gates on mass, and reports inertia as an explicitly-labelled proxy
(the relative deviation of the `mass · inertia_scale` factor, valid only if the geometry's
side lengths were recovered correctly). Do not quote that number as the paper's inertia
result.

## Head start on Phase 2 (§5)

The tactile→torque substitution is **much smaller than §5 assumes**. One-Shot does not
consume a tactile map. In `main_stage_2.py`, `force_func` builds a 6-D wrench and then
zeroes everything except indices 3 and 4 — i.e. only the **horizontal x/y components of a
single 3-vector force** ever reach the simulator. The "robot" is a sphere pusher
(`radius 0.025, mass 0.1, friction 1.0`); there is no arm, no URDF and no Jacobian
anywhere in this codebase.

So the graft is: compute `Jᵀτ → end-effector wrench` on the *Scalable Real2Sim* side, take
the linear part, and write it into that 3-vector slot. That is data preparation, not
surgery on the optimisation loop.

**Risk to check early:** only one trajectory (`train1`, 31 frames ≈ 0.3 s) ships here, so
held-out validation (Rung 2) depends entirely on the Scalable Real2Sim data having several
interactions per asset. §5's success criterion — the one the guide says the company rests
on — is unverifiable if it does not. Worth confirming before investing in the graft.

## Files added (all outside the cloned repo)

- `run_phase0.sh` — one-command Phase 0
- `phase0_report.py` — scores recovered parameters against ground truth
- `STATUS.md` — this file

---

# Update 2026-09-17 — Phase 0 complete on both methods; strategic findings

## One-Shot (arXiv 2412.00259 v4) — does NOT identify mass from a push
- 3-seed replication of the shipped (= paper v4) config, Drill: mass err 0.162 kg (20.2%), sd 0.087.
- 8-run initialization sweep: final mass tracks the SEED (0.5->0.81, 0.9->0.93, 1.3->1.27). The optimizer
  anchors on initialization; the data exerts almost no pull. VLM seeding therefore does not help here.
- Its inertia is a bounding-box formula, ignoring the watertight mesh it already reconstructs (up to 85% error).
- Note: paper v1 said mass was assumed known / ill-conditioned; v4 (RA-L) optimizes it. Quote v4 only.

## Scalable Real2Sim (arXiv 2503.00370) — mass works, inertia output is an artifact
- Env: conda `s2s` (py3.10), Drake 1.51.1, run with PYTHONNOUSERSITE=1 and PYTHONPATH unset.
- Driver: `s2s_tests.py <object...>`; results in `results/s2s_tests.json`. T1-T5 on spam/sugar/lego.
- T1 PASS: shipped mass reproduced to 6 decimals on all 3 objects.
- T2: mass survives the gentlest 50% of samples (<=3.7% drift); collapses below 25%. cond(W) is a
  GT-free identifiability metric (~5 healthy, >20 failing). Inertia never stable (45-1000% dev).
- T3: geometry-derived (uniform-density mesh) inertia beats torque-identified inertia on held-out torque
  prediction, 3/3 objects. Shipped inertia is ~0.025 kg m^2 on two axes for EVERY object regardless of
  mass (lego 0.169 kg -> impossible). It is a constant robot-model artifact, not the object.
- T4: --use_bounding_ellipsoid changes nothing (0.0%, 3/3). Dead. Also the code path is broken on current
  Drake (feeds 14k verts to an SDP); convex hull fixes it, identically.
- T5: held-out torque prediction good on joints 0-4, ~1.0 (no signal) on wrist joints 5-6.
- T6 not runnable: identification pipeline has no simulator/policy loop.
- Benchmark objects have NO ground-truth physics; paper accuracy numbers are from a calibration rig.
- Inconsistencies: numpy pin vs README; parent/submodule Drake pins incompatible; submodule pyproject
  unsatisfiable; CLI writes JSON to a hardcoded path in the input dir; robot_payload_id has NO LICENSE.

## Decision
Stay on physics estimation. Architecture: mass+CoM from torque (static holds / gentle motion suffice),
inertia from reconstructed geometry (+ density prior), friction from slip events. Do not promise inertia
from operational logs. World models: only as a density/material prior, not for geometry.

---

# Update 2026-09-17 (later) — Hybrid pipeline built and evaluated

Files: `hybrid_pipeline.py` (mass+CoM from torque SDP, inertia from voxelised mesh; emits
`results/hybrid/<obj>_hybrid_inertial_params.json` + `<obj>_hybrid.sdf` = shipped SDF with the
<inertial> block replaced), `eval_holdout.py` (K-fold leave-one-block-out torque reconstruction).
Run from R2S/ with the `s2s` env, PYTHONNOUSERSITE=1, PYTHONPATH unset.

5-fold held-out torque reconstruction, spam/sugar/lego (mean):
  method          relRMSE j0-4  absRMSE Nm  mass CV%  inertia fold CV%  gyration ratio
  torque-only        0.1111       0.436        3.6          50              3.87  (impossible)
  hybrid             0.1102       0.439        3.6           4              0.68
  hybrid_geomcom     0.1065       0.422        3.6           4              0.68
- Torque prediction is INSENSITIVE to the inertia source (all within fold std): payload rotational
  inertia contributes negligibly to joint torque for <0.5 kg objects; wrist joints are noise. This is
  WHY the SDP inertia is garbage (flat objective), and why inertia must come from geometry.
- Hybrid keeps mass exactly, makes inertia stable (CV 50% -> 4%) and physical (gyration 3.9 -> 0.68).
- Held-out torque validates mass/CoM/friction, NOT inertia accuracy. Inertia accuracy needs GT
  (3D-printed mustard) or an inertia-sensitive downstream sim test (fast rotation / tipping).

New pipeline issues found:
- #8 `get_object_pose_in_link_frame` (RANSAC+FPFH) is unseeded and non-deterministic: 20 mm / 180-deg
  spread on symmetric objects. Object-frame CoM in the shipped JSONs is one random draw. Fixed in our
  pipeline by seeding (o3d.utility.random.seed(0)); reproducible, but for symmetric shapes the CoM
  sign vs the mesh is still only as good as a geometry-only registration -> `symmetry_flag` warns.
- BundleSDF meshes are not watertight (7k-50k boundary edges); surface-integral inertia is off by
  0.5-6% vs voxel. Pipeline uses voxel inertia.

## GT check (2026-09-17): the 3D-printed mustard is NOT in the benchmark
Extracted all three mustards. small_mustard (0.451 kg) and mustard_large (0.635 kg) are real French's
bottles; organic_mustard (0.326 kg) is a real "365 Organic Honey Mustard". None is the paper's 427.1 g
printed object. The released benchmark therefore has ZERO objects with ground-truth physics; inertia
accuracy cannot be validated from public data. To get GT: weigh the objects (mass GT for all 20 in an
afternoon) and/or 3D-print one uniform-density calibration object (inertia GT that also directly tests
the geometry-inertia assumption).

## Open data found (2026-09-17): utiasSTARS workshop-tools dataset gives inertia GROUND TRUTH
`utias-inertial/` = github.com/utiasSTARS/inertial-identification-with-part-segmentation (MIT, 85 MB).
20 workshop tools: watertight CAD mesh (mm!), coarse vision-reconstructed mesh (unit-normalised, ~130-250
verts), Materials.txt, and CAD-derived GT mass/CoM/inertia with realistic multi-material assignments
(e.g. hammer = steel head + Al shaft + rubber handle). Also synthetic stop-and-go manipulation .pkl.
Test run: `results/utias_geometry_inertia.{log,json}` -- uniform-density geometry inertia vs GT:
  CAD mesh:   I Frobenius err median 4.4% / mean 13.5% / worst 50.3%; CoM median 1.0 mm
              single-material (n=6): 0.0%;  multi-material (n=14): median 18.4%, worst 50% (plastic+steel)
  recon mesh: principal-moment err median 12% / mean 21% (mesh quality adds ~as much as density)
  gyration ratios 0.48-0.74 for all 20 (physical) vs 2.8-4.9 for torque-identified.
=> Geometry inertia beats torque-identified inertia (42%+/-16% on a rig; unphysical on real objects)
   by 3-10x ON GROUND TRUTH. The density residual is now measured; a material/part prior (their
   part-segmentation method, or a VLM) is the lever for the 14 multi-material cases.
Other datasets seen: FMB (Franka, 22k trajectories, F/T logged, no mass GT), DAM-VLA (joint-torque
estimates @100 Hz). YCB objects have published masses (unverified this session; ycbbenchmarks.com 500).

## GT test of identification from GENTLE motion (2026-09-17): utias synthetic, 20 objects
`utias_gentle_motion_test.py` -> `results/utias_gentle_motion.{log,json}`. Wrist-wrench Newton-Euler
regressor, fit first 70%, hold out 30%. Data conventions (found empirically; GT residual 1.0 -> 0.08):
kinematics [v;w] in sensor frame; wrench in WORLD frame as the reaction -> rotate by R^T and negate.
Motion <= 2.5 m/s^2 (production-like). Their data already carries some noise (gtres 0.02-0.5).
                     mass (median)  mass<=5%   CoM (median)   inertia (median)   I<=10%
  clean, LS             0.23%        17/20       8.0 mm           786%             0/20
  clean, SDP            0.27%        16/20       8.3 mm           303%             0/20
  +0.3N/0.005Nm noise   0.78%        14/20      15.9 mm           852%             0/20
  gentlest 10% (<=0.39 m/s^2), noisy: mass 0.98%  CoM 3.6 mm  inertia 4272%
=> MASS FROM GENTLE MOTION: VALIDATED against GT (sub-1% median, even at 0.39 m/s^2). Gravity loading
   carries mass; excitation is irrelevant for it. Failures under noise are the <50 g objects (file
   20 g, ruler 9 g) where m*g ~ sensor noise -> product note: light objects need averaging/better sensor.
=> INERTIA FROM MOTION: FALSIFIED against GT (300-800% error, 0/20 within 10%, even noise-free with a
   perfect model). Physical-consistency SDP helps (786->303%) but is nowhere near sufficient.
=> The architecture is now GT-validated at both ends: mass from motion, inertia from geometry (4.4%).
Note: 4 objects (C_Clamp, Electronic_Caliper, Machinist_Hammer, Measuring_Tape) have GT residual
0.4-0.5 on their OWN data -> their sim/GT is inconsistent for those; they are the mass outliers.

## r2s_pipeline (2026-09-17) — the deliverable
Package `r2s_pipeline/` (README inside) + `RECORDING_SPEC.md`. Two modes (wrist wrench, joint torque
via Drake with any URDF), automatic wrench-convention detection, baseline (empty-gripper) subtraction,
geometry inertia, held-out + conditioning + gyration + symmetry diagnostics, JSON/SDF/URDF output,
hardware-free self-test on ground truth.
  self-test:  PASS, mass 0.06-1.8 % on 5 utias objects
  SR2S spam via joint-torque mode with the NOMINAL iiwa model + empty-gripper baseline: 0.3658 kg vs
  0.3780 reference (3.2 %); without the baseline the gripper (2.53 kg) lands in the payload -> the
  baseline recording is the single most important item in the recording spec.

## M2 de-risk (2026-09-18): joint-axis recovery from geometry alone -- `axis_recovery_test.py`
PartNet-Mobility is gated (sapien-sim/PartNetMobility, manual approval); all open HF mirrors are point
clouds only. Ran a synthetic stand-in with exact GT axes, written out as URDF+OBJ and reloaded through
the PartNet-format loader (same code path runs on real data: `--partnet DIR`). Five objects covering
the failure modes: flush hinge, 6 mm hinge gap, axis offset 8 mm inside a hinge barrel (laptop),
face contact (box lid, needs the edge hint), prismatic drawer. Clean / 0.7 mm noise + 2% dropped
faces / URDF-reloaded all give the SAME result:
  PASS 4/5 (dir < 2 deg, pos < 5 mm). cabinet 0.00deg/0.7mm, gate 0.00/4.5, drawer 0.00/-,
  box_lid 0.00/0.2.  FAIL laptop: 0.00deg / 7.1 mm -- the axis is not on any contact surface, so
  contact geometry cannot locate it. Structural, not noise. Needs interaction (open it, fit the arc)
  or a learned hinge prior.
  Without the semantic hint: box_lid 87 deg (wrong edge), drawer 9.5 deg -> the LLM hint is cheap
  AND necessary for face-like contacts.
=> Roadmap holds for hinges at contact edges (doors, gates, lids, drawers). Barrel/offset hinges are
   the branch that needs the learned predictor or measurement.
