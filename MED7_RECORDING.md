# Recording on your KUKA LBR Med7 — what to capture

You do **not** need a wrist F/T sensor. The LBR Med7 has joint torque sensors on all 7 axes, which is
the same sensing Scalable Real2Sim validated on (iiwa 7 R800). You use **joint-torque mode**.

Everything needed is already on this machine:
- driver: `lbr_fri_ros2_stack` in `~/AUTOKnee-server` (topic `/lbr/state`, msg `lbr_fri_idl/LBRState`)
- model: `med7.urdf` generated from `lbr_description` — verified to load in Drake, 7 joints, EE frame `lbr_link_7`
- recorder: `record_lbr.py`
- pipeline: `r2s_pipeline`

---

## 1. Signals to record (all from `/lbr/state`)

| field | what it is | use |
|---|---|---|
| `measured_joint_position[7]` | joint angles | **required** — velocities/accelerations derived from it |
| `measured_torque[7]` | raw joint torque | **required** — the primary signal |
| `external_torque[7]` | KUKA's own estimate after subtracting arm dynamics | **record it too** (see §2) |
| `time_stamp_sec` + `_nano_sec` | FRI clock | **required** |
| `sample_time`, `session_state` | cycle time, FRI state | sanity checks |

`record_lbr.py` captures all of these and drops samples where `session_state < MONITORING_READY`
(torque is meaningless when the FRI session isn't live).

```bash
source ~/AUTOKnee-server/install/setup.bash          # with the lbr driver running
python3 record_lbr.py baseline_empty.npz --seconds 30     # FIRST, empty gripper
python3 record_lbr.py object_A.npz       --seconds 30     # then each object
```

## 2. Record `measured_torque` AND `external_torque` — they are two different bets

- `measured_torque` is raw. We subtract the arm's dynamics ourselves using `med7.urdf` and fit
  per-joint friction/offset terms to absorb model error. This is the validated path (3.2 % on SR2S).
- `external_torque` is KUKA's own residual, computed with their factory-calibrated internal model —
  potentially **better than ours**, because KUKA knows the real link masses and joint friction.

Record both and we compare on your data. If `external_torque` wins, the pipeline gets simpler and more
accurate on your robot.

**Important caveat:** `external_torque` depends on the **tool/load configured in your Sunrise
application**. If a tool mass is configured, KUKA has already subtracted part (or all) of what you are
trying to measure. For identification runs either configure **no tool/load**, or keep the configuration
identical between the baseline and object recordings so it cancels.

## 3. The baseline — the single most important recording

Record the same setup with **nothing in the gripper**, once per gripper configuration. On the SR2S iiwa
the gripper+fingers were **2.53 kg**; without subtracting it a 0.378 kg object came out as 2.90 kg.
With the baseline it came out at 0.366 kg (3.2 %).

Redo the baseline if you change the gripper, the finger set, or the gripper opening.

## 4. Motion protocol

**For mass — nothing special.** Mass comes from gravity loading; we measured <1 % error from motion as
gentle as 0.4 m/s². Ordinary handling speed is fine; even a static hold works.

**For CoM — hold the object in 3–4 distinct orientations,** a few seconds each, ideally with the wrist
rotated so gravity acts along different object axes. One orientation only constrains two of the three
CoM components. Move between poses at normal speed; no need for anything aggressive.

A good 30 s recording: 4 static holds (≈3 s each) in clearly different wrist orientations, with smooth
motion between them. Do the same motion for the baseline.

**Do NOT design excitation trajectories for inertia.** Inertia is not identifiable from motion at any
realistic excitation (300–800 % error against ground truth, 0/20 objects within 10 %). It comes from the
object mesh. Don't spend robot time or risk on shaking.

## 5. Geometry — for inertia

You need an object mesh (metres) and `T_so`, the mesh frame in the `lbr_link_7` frame at grasp.
You already have RealSense cameras and the `scan_and_merge` / `ir_tracking` packages, so:
- mesh: scan the object (or use CAD if you have it — CAD is better)
- `T_so`: from your tracker at the moment of grasp, or a fixed known grasp fixture

Without `T_so` you still get mass and CoM in the robot's EE frame; inertia is then placed at the mesh
centroid. For symmetric objects prefer a **visual** pose over geometric point-cloud registration —
geometry alone can't distinguish equivalent poses, and the pipeline will raise a `symmetry flag`.

## 6. Med7-specific gotchas

- **Mounting orientation.** The URDF assumes the base is upright with gravity along world −Z. If your
  Med7 is on a tilted or mobile base, gravity in the model must match reality or mass will be wrong by
  `cos(tilt)`. Tell me the mounting and I'll set it.
- **FRI rate.** Set the cycle time as low as your setup allows (1–2 ms → 500–1000 Hz). `record_lbr.py`
  prints what it actually received.
- **Mass floor.** On a 7-DoF arm expect roughly a **100–200 g** resolution floor without averaging.
  Below that, `m·g` sinks into torque noise. Your Med7 is a 7 kg-payload arm, so for anything under
  ~100 g use a scale instead.
- **Medical/safety mode.** Make sure the robot is in a mode where FRI actually streams torque; in some
  safety states the session drops to MONITORING_WAIT and the values are stale.
- **Keep the object rigidly grasped,** no contact with anything else (table, cables, drapes) during a
  recording. Slip shows up as a train-vs-all mass mismatch, which the pipeline flags.

## 7. Run it

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 ~/miniconda3/envs/s2s/bin/python -m r2s_pipeline identify \
    object_A.npz --mode joint_torque \
    --robot med7.urdf --ee lbr_link_7 \
    --baseline baseline_empty.npz \
    --mesh object_A.obj --out object_A.json
```

Then read two lines of the output: the **held-out RMSE** and the **mass train-vs-all** warning. If the
second appears, the recording drifted or the grasp slipped — redo it.

## 8. Checklist

- [ ] baseline recorded, empty gripper, same motion, same tool config
- [ ] object rigidly grasped, no environment contact
- [ ] ≥30 s, ≥100 Hz (prefer 500–1000)
- [ ] 3–4 distinct wrist orientations held
- [ ] both `measured_torque` and `external_torque` captured (the recorder does this)
- [ ] mesh in metres + `T_so` at grasp
- [ ] object ≥100–200 g
- [ ] mounting orientation confirmed upright (or told to me)

## 9. What to send me for the comparison

Per object: `object_X.npz`, `baseline_empty.npz`, the mesh, `T_so`, and — if you have it — a **scale
measurement of the true mass**. That last one turns this from a reproduction into a real validation:
the public benchmarks have no ground-truth mass, so your kitchen scale would give us the first
ground-truth mass numbers in the whole project.
