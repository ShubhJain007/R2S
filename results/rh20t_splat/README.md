# RH20T reconstruction figures

Outputs of the scripts in [`rh20t_splat/`](../../rh20t_splat/) on RH20T configuration 7 data. The splat models (`splat.pt`,
up to 529 MB each) are not included.

| Folder | Produced by | What it shows |
|---|---|---|
| `out_verify/` | `verify_calib.py`, `verify_calib2.py` | extrinsics convention check (world→camera): reprojected world origin per camera, fused multi-camera point cloud `fused_world.ply` |
| `out_base/` | `verify_base.py` | the robot base frame drawn into every camera |
| `out_scale/` | exploratory (no single script) | locating the weighing scale in each camera and cropping its LCD display, to read a ground-truth weight for the force/torque mass estimate (`mass_from_ft.py`) |
| `out_ep/` | `verify_episode.py`, `episode_views.py` | the same checks on episode (not calibration) data, sampled frames |
| `out_wrist/` | `wrist_poses.py` | in-hand camera poses from forward kinematics |
| `out_splat/`, `out_splat_ep/`, `out_splat_ep2/`, `out_splat_sh/` | `train_splat.py`, `eval_splat.py` | Gaussian-splat renders vs ground-truth views (`cmp_*`): workspace splat, per-episode splats, and a spherical-harmonics variant; `showcase.png` summarises the last |
| `out_object/` | `object_segment.py`, `object_mesh.py` | segmented object and its watertight mesh `object.obj` from multi-view depth |

## Licence and attribution

Derived from **RH20T** (Fang et al., *RH20T: A Comprehensive Robotic Dataset for Learning Diverse Skills in One-Shot*,
ICRA 2024, https://rh20t.github.io), episode `task_0004_user_0014_scene_0001_cfg_0007` of the RH20T-C subset (scenes
0001–0005), which is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License (CC BY-SA 4.0).
These figures, the mesh and the point cloud are shared under the same licence, **CC BY-SA 4.0**
(https://creativecommons.org/licenses/by-sa/4.0/).
