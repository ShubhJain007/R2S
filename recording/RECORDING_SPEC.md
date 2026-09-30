# What to record — data spec for `r2s_pipeline`

The pipeline estimates **mass + centre of mass from force/torque data** and **inertia from the
object's reconstructed geometry**. Both halves were validated against ground truth (mass <1 %
median under gentle motion; geometry inertia 4.4 % median). This document says exactly what to
record so that works on your robot.

There are two recording modes. **Pick the one your robot supports.** Wrist F/T is simpler and
cleaner; joint torque is what most fleets already log.

---

## 0. The one thing that matters most: record a BASELINE

Record the **same setup with no object in the gripper**, once per gripper configuration. The gripper,
fingers, adapter plate and F/T sensor plate all show up as "payload" otherwise. On a Kuka iiwa with a
Schunk WSG-50 that is **2.5 kg of phantom mass** on a 0.38 kg object. With the baseline subtracted the
generic pipeline recovered the object to within 3 % using only the robot's nominal URDF.

The baseline is a normal recording (same format, same motion protocol below), just empty-handed.
Reuse it for every object grasped with that gripper/opening.

---

## 1. Mode A — wrist force/torque sensor (recommended)

| signal | shape | units | notes |
|---|---|---|---|
| `t` | (N,) | s | monotonic timestamps |
| `wrench` | (N,6) | N, N·m | `[fx fy fz nx ny nz]` from the F/T sensor. **Any frame/sign convention** — the pipeline detects sensor-vs-world frame and contact-vs-reaction sign from the static samples and prints what it found. Check that line. |
| `T_ws` | (N,4,4) | m | pose of the **F/T sensor frame** in the world/robot base frame (from forward kinematics) |
| `vel`, `acc` | (N,6) | m/s, m/s² | optional `[v; ω]`, `[a; α]` of the sensor frame. If absent they are derived from `T_ws` (low-pass 5 Hz). |

**Sensor hygiene**
- **Zero / bias the F/T sensor** with the gripper empty, in the same pose you will start from. Bias drift is the largest error source on real sensors; re-zero between objects.
- The object must be **rigidly grasped** for the whole recording — no slip, no compliance. A slipping grasp shows up as a train/all-data mass mismatch, which the pipeline flags.
- **No contact with anything else** during the recording (table, other objects, cables).

## 2. Mode B — joint torque (fleet logs)

| signal | shape | units | notes |
|---|---|---|---|
| `t` | (N,) | s | |
| `q` | (N,nj) | rad | joint positions |
| `tau` | (N,nj) | N·m | **measured** joint torque (joint torque sensors) or motor torque. Not commanded torque. |
| robot model | file | — | URDF / SDF / Drake `.dmd.yaml` of the arm **without gripper**, and the name of the flange/EE frame |

The pipeline computes the arm's own dynamics from the model and fits per-joint viscous / Coulomb /
offset terms alongside the object, which absorbs most nominal-model error. **The baseline recording
is not optional in this mode** — it is what removes the gripper.

Joints far from the payload (base joints) carry the signal. On the iiwa the two wrist joints are
noise-dominated; that is expected, not a fault.

## 3. Geometry — for inertia

| item | notes |
|---|---|
| object mesh | any reconstruction (3DGS→mesh, BundleSDF, NeRF, photogrammetry, CAD). Watertight preferred but **not required** — inertia is computed on a filled voxelisation. Units: metres (use `--mesh-scale` otherwise). |
| `T_so` (4,4) | pose of the mesh frame in the F/T-sensor / EE frame **at grasp**. From your perception stack (object tracker) or a fixed known grasp. Without it, CoM is reported in the sensor frame and inertia is placed at the mesh centroid. |

Mesh quality is a first-order input: on coarse (~200-vertex) meshes the inertia error roughly triples
versus a good reconstruction. Reconstruct properly.

**For symmetric objects** (cans, boxes, bricks) a geometry-only pose registration cannot tell
equivalent poses apart, so the sign of the CoM offset relative to the mesh is only as good as your
`T_so`. The pipeline raises a `symmetry flag` for these. Prefer `T_so` from a *visual* tracker over
geometric point-cloud registration.

## 4. Motion protocol — what the robot has to do

**For mass: nothing special.** Mass comes from gravity loading and was recovered to <1 % even from
the gentlest 10 % of samples (≤0.4 m/s²). Normal handling motion is fine. A static hold works.

**For CoM: hold the object in ≥2, ideally 3-4, different orientations.** CoM is identified from how
the gravity torque changes with orientation; one orientation only constrains two of its three
components. Ordinary pick-and-place with some reorientation supplies this incidentally.

**Duration:** ≥10 s at ≥100 Hz. More is better; the pipeline holds out the last 30 % to check itself.

**Do NOT** design excitation trajectories for inertia. Inertia is **not** identifiable from motion
at any excitation level a production robot will use (300-800 % error on ground truth, 0/20 objects
within 10 %, even noise-free). It comes from geometry. Don't spend robot time on it.

## 5. Object size vs sensor noise

Mass is identifiable when `m·g ≫ force noise`. With a Robotiq FT-300-class sensor (~0.3 N noise)
objects under ~50 g fail (a 9 g ruler, a 20 g file). Options for light objects: average many holds,
use a better sensor, or accept mass from a scale. Joint-torque sensing on a 7-DoF arm is noisier
still; expect the floor around 100-200 g unless you average.

## 6. Rates and filtering

| | minimum | recommended |
|---|---|---|
| F/T / torque | 100 Hz | 500-1000 Hz |
| poses / joints | 100 Hz | same clock as the wrench |

Everything is low-pass filtered at 5 Hz before differentiation (`--fc`). If your timestamps are not
on a common clock, resample first; the pipeline validates monotonic `t` and equal lengths.

## 7. File format

One `.npz` per recording, keys exactly as in the tables above (`np.savez("rec.npz", t=..., wrench=..., T_ws=..., T_so=...)`).
Baseline: same format, no object.

```
python -m r2s_pipeline identify rec.npz --mesh obj.obj --baseline empty.npz --out obj.json
python -m r2s_pipeline identify rec.npz --mode joint_torque --robot arm.urdf --ee flange --baseline empty.npz --mesh obj.obj
python -m r2s_pipeline selftest        # no hardware; verifies the install on ground-truth data
```

## 8. Checklist per object

- [ ] baseline recorded for this gripper opening (empty)
- [ ] F/T zeroed with empty gripper (mode A)
- [ ] object rigidly grasped, no environment contact
- [ ] ≥10 s, ≥100 Hz, ≥2 orientations held
- [ ] mesh in metres, `T_so` from the tracker at grasp
- [ ] read the printed *wrench convention* line and the *mass train-vs-all* line; both must look sane

## 9. What you get

`obj.json` with `mass`, `com_sensor_frame`, `com_object_frame`, `inertia_object_frame_about_com`,
and `diagnostics` (conditioning, held-out error, gyration ratio, symmetry flag, mesh info); plus
ready-to-paste SDF and URDF `<inertial>` blocks, and optionally a patched copy of an existing SDF.
Gyration ratio should be ~0.5-0.8; >1 is physically impossible and means something upstream is wrong.
