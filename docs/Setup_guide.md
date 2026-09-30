# Real2Sim Physics-Estimation — Claude Code Setup & Run Guide

> **Purpose of this file:** hand this to a Claude Code session as context. It contains the
> project goal, the exact open-source repos to set up, the experiments to run, and the
> success criteria — so Claude Code can stand everything up and run the validation
> experiments as autonomously as possible.
>
> **How Claude Code should use this doc:** work top to bottom. Each PHASE has a goal, setup
> steps, a run command, and an explicit **SUCCESS CRITERION**. Do not proceed to the next
> phase until the current one's criterion is met. Where a step says **[VERIFY LIVE]**, the
> exact command/behaviour may have changed since this doc was written — check the repo's
> current README before running, and adapt.

---

## 0. North star (why we're doing this)

We are validating the technical core of a product: **turn a robot's real interaction logs
into a physically faithful, re-runnable simulation**, where an object's physical properties
(mass, inertia, and — approximately — friction) are **estimated from the robot's own
recorded interactions**, not hand-authored.

The single most important thing to prove before building anything else:

1. **The physics-estimation method works in our hands** (recover known parameters in sim).
2. **It generalises** (fitted parameters predict *held-out*, unseen interactions on real data).
3. **It works with our sensor budget** — joint **torque/current** instead of **tactile**,
   because mid-scale robot fleets have torque logs but rarely tactile sensors.

Everything in this guide serves those three proofs. We do NOT need a robot or a capture rig
for Phase 0–2 — we run on open datasets that ship raw sensor data.

**Method we are testing as the primary approach:** *One-Shot Real-to-Sim via End-to-End
Differentiable Simulation and Rendering* (arXiv 2412.00259) — joint optimisation of
geometry + appearance + physics. We adapt its **tactile force input → joint torque**, using
*Scalable Real2Sim* (arXiv 2503.00370, IROS 2025) as the reference for the torque-based
identification.

---

## 1. Hardware / environment assumptions

- **GPU:** NVIDIA GPU required. 3DGS reconstruction + differentiable sim want a modern card
  (RTX 3090/4090-class, ≥16–24 GB VRAM ideal). Isaac Sim specifically wants RTX + ~32 GB RAM.
- **OS:** Linux (Ubuntu 22.04/24.04). Several of these (Isaac Lab Mimic, Neuralangelo) are
  Linux-only.
- **CUDA:** 12.x (WorldComposer pins CUDA 12.8; check each repo).
- **Conda** (or mamba) available. **Poetry** available (Scalable Real2Sim uses it).
- **git-lfs** installed (large asset/data files).
- **huggingface-cli** installed and logged in (`pip install -U huggingface_hub`,
  `huggingface-cli login`) — needed to pull benchmark datasets.

Claude Code: verify each of these up front and install/report anything missing before
cloning repos.

---

## 2. Repos, in priority order

| Priority | Repo | What it gives us | Use it for |
|---|---|---|---|
| 1 | **One-Shot Real-to-Sim** (`tianyi20.github.io/rigid-world-model.github.io/`) | The actual method we're testing: SaP geometry + grid appearance + differentiable LCP contact sim. PyBullet ground-truth setup. | Primary method. Phase 0 (sim round-trip) + Phase 2 (real data). |
| 2 | **Scalable Real2Sim** (`github.com/Chung-I/scalable-real2sim`, orig author `nepfaff`) | Torque+camera → geometry + inertial params. **20-asset benchmark + raw torque sensor data** on HF (`nepfaff/scalable-real2sim`). | The torque-substitution reference + real torque data to stress-test One-Shot. |
| 3 | **Real2Render2Real (R2R2R)** (`github.com/uynitsuj/real2render2real`) | Full extraction→3DGS-to-mesh→data-gen pipeline. **Ships `data.zip` + `outputs.zip`** so you can run data-gen with zero capture. | Orientation: understand the whole real→asset→data flow end to end. |
| 4 | **WorldComposer** (`github.com/jaber628/WorldComposer`) | Real-scene→Isaac-Sim-ready asset pipeline on NVIDIA 3dgrut; installs `isaacsim`. Auto-data-gen may be "release in progress". | Template for delivering assets in Isaac Sim/USD (our customers' format). |

Simulator options for the differentiable physics loop (pick per phase — see §6):
- **PyBullet** (what One-Shot ships with) — use first, no migration risk.
- **Genesis** (differentiable, fast) — Phase 4 migration target.
- **MuJoCo / MJX**, **Isaac Lab** (BSD-3) — alternatives.

**BEFORE cloning any repo, Claude Code must check its LICENSE** and report it. Academic
repos may be research-only or CC-BY-NC on data. Flag anything non-commercial: fine for our
Phase 0–2 validation, NOT usable in a shipped product. Do not skip this.

---

## 3. PHASE 0 — Method sanity check in simulation (cheapest, do first)

**Goal:** confirm One-Shot's method recovers *known* physical parameters when the
observations come from simulation (Rung 1 / synthetic round-trip). This is the "does it work
at all in our hands" gate.

**Setup**
1. `git clone --recursive <One-Shot repo>` (get the URL from the project page linked above).
   **[VERIFY LIVE]** confirm the actual GitHub URL and that `--recursive` is needed.
2. Create the conda env per its README; install deps (expect PyBullet, PyTorch, a
   Poisson/marching-cubes lib, a differentiable renderer).
3. Confirm the repo ships its **PyBullet synthetic objects** (the paper uses 9 sim objects:
   bleach, mustard, sugar, etc.). **[VERIFY LIVE]**

**Run**
- Run their simulation-experiment / world-model-identification entrypoint on one sim object.
  **[VERIFY LIVE]** find the exact script (likely something like a `run_*.py` or a config in
  an `experiments/`/`configs/` dir).

**SUCCESS CRITERION (Phase 0):**
For at least one simulated object, the optimiser recovers mass within a few % of the value
that was set, and inertia within ~10%. If it can't recover parameters from its own
simulator's data, STOP and debug the install/loop before doing anything else.

---

## 4. PHASE 1 — Orientation on the full pipeline (parallel, optional but recommended)

**Goal:** understand the end-to-end real→asset→training-data flow without capturing anything,
using R2R2R's bundled data.

**Setup**
1. `git clone --recursive https://github.com/uynitsuj/real2render2real.git`
2. R2R2R has **three conda envs** (real-to-sim extraction, 3DGS-to-mesh, data generation).
   Set up per its README (`env_real_to_sim.sh` etc.). **[VERIFY LIVE]**
3. **To skip straight to data-gen:** unzip the bundled `outputs.zip` and `data.zip` in the
   repo root, then go to the Data Generation section. **[VERIFY LIVE]** that these zips are
   still shipped and downloadable.

**Run**
- Run the data-generation stage on the bundled data.

**SUCCESS CRITERION (Phase 1):**
Data-generation produces manipulation training data from the bundled assets without needing a
robot or capture. You can inspect generated trajectories/renders. (This is orientation — no
accuracy bar, just "the flow runs and I understand each stage.")

---

## 5. PHASE 2 — The real test: One-Shot's method on real torque data

**Goal:** run the One-Shot approach on **real** sensor data, using **joint torque in place of
tactile**, and validate via **held-out trajectory prediction** (Rung 2). This is the
experiment that de-risks the whole product thesis.

**Setup**
1. Pull Scalable Real2Sim's benchmark data:
   ```
   huggingface-cli download nepfaff/scalable-real2sim --repo-type dataset --local-dir ./s2s-data
   ```
   This gives 20 assets **plus the raw sensor observations** (RGB-D + joint torque) used to
   create them. **[VERIFY LIVE]** the repo id and that raw observations are included.
2. Clone Scalable Real2Sim (`github.com/Chung-I/scalable-real2sim`) and read
   `run_asset_generation.py` — this is where they turn torque + camera into inertial params.
   Note: `run_data_collection.py` is robot-specific and won't run out of the box; the
   **asset-generation entrypoint is the runnable one**. **[VERIFY LIVE]**

**The core adaptation (tactile → torque)**
- One-Shot's optimisation loss expects **contact force** (from tactile). We feed **joint
  torque** instead.
- **The bridge:** torque and tactile measure the *same quantity* — contact force — at
  different points. Convert joint torques to the end-effector force/wrench via the arm's
  **inverse dynamics + manipulator Jacobian** (Jacobian-transpose maps joint torque ↔
  end-effector force). The robot's URDF gives the kinematics/dynamics needed.
- **Reference implementation for the torque handling:** Scalable Real2Sim's
  `run_asset_generation.py`. Mirror how they read/use the torque signal; splice that into
  the point in One-Shot's loop where it currently consumes tactile force.
- **Expectation:** mass and inertia recover well from torque (Scalable Real2Sim proves this).
  **Friction degrades** (tactile senses shear/slip more directly) — treat friction as
  approximate for now; do NOT gate success on friction accuracy.

**Validation (Rung 2 — held-out prediction, needs no ground-truth parameters)**
1. For a given real object with multiple recorded interactions, fit physics on a subset
   (e.g. 7 of 10 interactions).
2. Take the fitted parameters, simulate the **held-out** interactions (the other 3).
3. Compare simulated vs. actually-recorded object trajectories.

**SUCCESS CRITERION (Phase 2):**
Parameters fit on a subset of an object's real interactions predict the object's motion on
**held-out** interactions of that object with low trajectory error. Mass/inertia should
transfer; friction may be loose. If held-out prediction works, the method + the torque
substitution are both validated on real data — this is the green light for the product.

---

## 6. Simulator choice per phase (don't over-engineer early)

- **Phase 0–2:** use **One-Shot's shipped PyBullet/LCP simulator as-is.** It's a known-good
  differentiable contact solver the authors already validated. Do NOT swap simulators yet —
  it adds risk with no v1 benefit.
- **Phase 4 (later, for throughput/scale):** migrate the physics loop to **Genesis**
  (differentiable, GPU-fast) or **MuJoCo-MJX**. Migration test: take a Phase-0 object with
  known parameters, confirm the new simulator recovers the *same* values PyBullet did before
  trusting it. If Genesis's **gradients through contact** are unstable for friction/inertia,
  fall back to Genesis-as-forward-sim + a sampling optimiser (CMA-ES), or the GNN-surrogate
  approach from *Few-Shot Neural Differentiable Simulator* (arXiv 2603.06218).

---

## 7. Anti-exploit / negative test (add early, cheap, catches the worst bug)

A sim can look "correct" while its physics is too loose — a bad policy/grasp "succeeds"
because contact was too forgiving. Guard against it:

- Build a small set of **deliberately-wrong** interactions/policies (wrong grasp angle,
  insufficient force) and confirm they **FAIL** in the reconstructed sim.
- A good digital twin makes *incorrect* behaviour fail the way reality would. If both correct
  and incorrect behaviours pass, the physics is too loose regardless of Rung-1/2 results.

---

## 8. Known caveats to expect (so Claude Code doesn't get stuck)

- **One-Shot fails on rotationally-symmetric objects (cylinders/bottles/cans)** — a surface
  point cloud gives no rotational cue. For first tests, **pick a non-cylindrical object**
  (boxy carton, asymmetric tool) so a method limitation doesn't masquerade as a setup bug.
- **One-Shot needs clean object masks.** Budget a segmentation front-end (SAM / lang-segment-
  anything) if the data isn't pre-masked.
- **One-Shot is ~15 min/object on a 4090** and is not real-time — fine for our offline/batch
  use, but don't expect interactive speed.
- **Neuralangelo** (a Scalable Real2Sim dependency) has its own `.venv` and heavy build —
  follow its setup exactly.
- **WorldComposer's auto-collection (data-gen) may be "release in progress"** — its real2sim
  (real scene → Isaac asset) part is released; verify which stages actually run today.
- **Dataset drift:** bundled `data.zip`/`outputs.zip` and HF datasets can move, shrink, or
  change license after publication. **[VERIFY LIVE]** downloadability and license each time.

---

## 9. Suggested run order (TL;DR for Claude Code)

1. **Verify environment** (§1): GPU, CUDA, conda, poetry, git-lfs, huggingface-cli. Fix gaps.
2. **Clone One-Shot**, check LICENSE, set up env, run **PHASE 0** sim round-trip → hit
   Phase-0 success criterion.
3. *(Parallel/optional)* **Clone R2R2R**, run **PHASE 1** on bundled data for orientation.
4. **Pull Scalable Real2Sim data + repo**, study its torque handling.
5. **PHASE 2:** graft torque→force (inverse-dynamics/Jacobian) into One-Shot's loop, run on
   Scalable Real2Sim's real data, validate with **held-out prediction** → hit Phase-2
   success criterion.
6. Add the **anti-exploit negative test** (§7).
7. Only after Phase 2 passes: consider **Genesis migration** (§6) and Isaac Sim delivery via
   **WorldComposer** (§2).

**The whole company rests on the Phase-2 success criterion.** Spend effort there. Everything
before it is setup; everything after it is scaling.

---

## 10. Key references (papers)

- **One-Shot Real-to-Sim** — arXiv **2412.00259** (Zhu, Xiang, Dollar, Pan; Yale). Primary method.
- **Scalable Real2Sim** — arXiv **2503.00370**, IROS 2025 (Pfaff, Fu, Binagia, Isola, Tedrake; MIT/Amazon). Torque-based identification reference; ships code + data.
- **RigPI** — arXiv (VLM-seeded differentiable sim; ~1.5% mass error). The precision-upgrade / VLM-seed pattern for later.
- **Few-Shot Neural Differentiable Simulator** — arXiv **2603.06218**. GNN-surrogate for stable contact gradients (Genesis fallback).
- **R2R2R** — CoRL 2025 (`uynitsuj/real2render2real`). Full data-gen pipeline with bundled data.
- **SimplerEnv** — reference for sim-vs-real *policy* correlation evaluation (our later Rung-4 proof).

---

## 11. What NOT to do (guardrails)

- Don't rebuild the data layer, the simulator, or the reconstruction from scratch — stand on
  these repos. Our differentiator is fleet-scale physics estimation + correlation proof, not
  the algorithm.
- Don't swap PyBullet for Genesis before Phase 2 passes.
- Don't gate success on friction accuracy (torque substitution is weak there by design).
- Don't test One-Shot first on a cylinder.
- Don't ship anything trained on a non-commercially-licensed benchmark dataset — validation
  only.
- Don't skip the LICENSE check on any repo or dataset.