#!/usr/bin/env python
"""Watertight mesh of a rigid object from noisy multi-view depth.

Per view, the TABLE is the calibration target: it is in every view and its plane is known.
  scale s  = median(z_plane / z_measured) over table pixels     -> removes per-camera depth bias
  sigma    = robust std of the table's height residual after that -> that view's noise
Fusion on a voxel grid around the seed cluster:
  weighted TSDF (w = 1/sigma^2) defines the surface where it was observed;
  a voxel is carved free only if some view sees THROUGH it by more than 3 sigma;
  never-observed, never-carved voxels stay solid -> the unseen bottom closes on the table plane.
"""
import os, sys, numpy as np, cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rh20t_api"))
from wrist_poses import setup, load_tcp_series, wrist_extrinsics, derive_tc_mat, frames as color_frames
from depth_io import read_depth_video
from object_segment import static_depth
from episode_views import gripper_mask

def view_stats(d, K, E, n, dd, cen, half):
    """depth-scale correction and noise for one view, from table pixels around the object."""
    H, W = d.shape; v, u = np.mgrid[0:H, 0:W]; m = (d > 0.15) & (d < 2.0)
    if m.sum() < 500: return None
    z = d[m]; ray = np.stack([(u[m] - K[0, 2]) / K[0, 0], (v[m] - K[1, 2]) / K[1, 1], np.ones(m.sum())])
    Twc = np.linalg.inv(E); R, C = Twc[:3, :3], Twc[:3, 3]; rw = R @ ray; X = C[:, None] + rw * z
    h = n @ X + dd
    near = (np.linalg.norm(X[:2] - cen[:2, None], axis=0) < 0.30) & (np.abs(h) < 0.04)
    near &= ~((np.abs(X[0] - cen[0]) < half[0] + 0.03) & (np.abs(X[1] - cen[1]) < half[1] + 0.03))
    if near.sum() < 300: return None
    zt = -(n @ C + dd) / (n @ rw[:, near]); s = float(np.median(zt / z[near]))
    h2 = n @ (C[:, None] + rw[:, near] * z[near] * s) + dd
    return s, float(1.4826 * np.median(np.abs(h2 - np.median(h2)))), float(np.median(h[near]))

if __name__ == "__main__":
    ep, calib, out = sys.argv[1], sys.argv[2], sys.argv[3]
    seed = np.load(os.path.join(out, "object_seed.npz")); Q = seed["points"]; n, dd = seed["plane"][:3], seed["plane"][3]
    conf, K, E, T_world_base = setup(calib)
    lo, hi = np.percentile(Q, 2, axis=0), np.percentile(Q, 98, axis=0); cen = (lo + hi) / 2; half = (hi - lo) / 2
    # ---- views: fixed cameras (static median depth) + wrist frames before the grasp
    views = []
    for s in [d[4:] for d in sorted(os.listdir(ep)) if d.startswith("cam_") and d[4:] not in conf.in_hand_serial and not d[4:].startswith("f")]:
        if not os.path.exists(os.path.join(ep, f"cam_{s}", "depth.mp4")): continue
        d = static_depth(ep, s, int(seed["t0"]) + 3000)
        if d is None: continue
        Ks = K[s].copy(); Ks[:2] *= d.shape[1] / 1280.0; views.append(dict(name=s, d=d, K=Ks, E=E[s]))
    tt, pp = load_tcp_series(ep, conf.in_hand_serial[0])
    for s in conf.in_hand_serial:
        dp = os.path.join(ep, f"cam_{s}", "depth.mp4")
        if not os.path.exists(dp): continue
        ts = np.load(os.path.join(ep, f"cam_{s}", "timestamps.npy"), allow_pickle=True).item(); td = np.asarray(ts.get("depth", ts["color"]))
        fr = read_depth_video(dp, s); _, cf = color_frames(ep, s); gm = gripper_mask(cf); tc = derive_tc_mat(E, T_world_base, calib, conf, s)
        Ks = K[s].copy(); Ks[:2] *= fr[0].shape[1] / 1280.0
        for i in range(0, min(len(fr), len(td)), 3):
            if td[i] > int(seed["t_grasp"]) - 1500: break
            d = fr[i].astype(np.float32) / 1000.0; d[~gm] = 0
            j = int(np.argmin(np.abs(tt - td[i]))); views.append(dict(name=f"{s}_f{i}", d=d, K=Ks, E=wrist_extrinsics(conf, T_world_base, pp[j], tc)))
    keep = []
    print(f"{'view':22s} {'scale':>7s} {'sigma mm':>9s} {'raw bias mm':>11s}")
    for v in views:
        st = view_stats(v["d"], v["K"], v["E"], n, dd, cen, half)
        if st is None or st[1] > 0.012: continue
        v["s"], v["sig"] = st[0], max(st[1], 0.0015); keep.append(v)
        if "_f" not in v["name"] or v["name"].endswith("_f0") or v["name"].endswith("_f60"):
            print(f"{v['name']:22s} {st[0]:7.4f} {st[1]*1000:9.1f} {st[2]*1000:11.1f}")
    nw = sum("_f" in v["name"] for v in keep); print(f"views used: {len(keep)-nw} fixed + {nw} wrist  (of {len(views)})")
    # ---- voxel grid above the table
    vs = 0.0025; pad = 0.03
    ax = [np.arange(lo[i] - pad, hi[i] + pad, vs) for i in range(2)] + [np.arange(min(lo[2], -dd / n[2]) - 0.01, hi[2] + pad, vs)]
    G = np.stack(np.meshgrid(*ax, indexing="ij"), -1).reshape(-1, 3); hG = G @ n + dd
    num = np.zeros(len(G)); den = np.zeros(len(G))
    USE_WRIST = "--no-wrist" not in sys.argv; ALPHA = 0.3
    for v in keep:
        if "_f" in v["name"] and not USE_WRIST: continue
        Xc = (v["E"][:3, :3] @ G.T + v["E"][:3, 3:4]); zc = Xc[2]
        uu = np.round(v["K"][0, 0] * Xc[0] / zc + v["K"][0, 2]).astype(int); vv = np.round(v["K"][1, 1] * Xc[1] / zc + v["K"][1, 2]).astype(int)
        H, W = v["d"].shape; ok = (zc > 0.1) & (uu >= 0) & (uu < W) & (vv >= 0) & (vv < H)
        dm = np.zeros(len(G)); dm[ok] = v["d"][vv[ok], uu[ok]] * v["s"]; ok &= dm > 0.15
        sdf = dm - zc; tr = max(0.008, 3 * v["sig"])
        # soft votes instead of hard carving: carving is an OR over views, so ONE view with a
        # slightly wrong pose deletes the interior. A view that sees a surface IN FRONT of a voxel
        # is evidence the voxel is solid (down-weighted: it could also be free space behind the object,
        # but then the cameras on that side out-vote it).
        w = 1.0 / v["sig"] ** 2
        upd = ok & (sdf > -tr); num[upd] += w * np.clip(sdf[upd] / tr, -1, 1); den[upd] += w
        beh = ok & (sdf <= -tr);  num[beh] -= ALPHA * w;                        den[beh] += ALPHA * w
    F = np.where(den > 0, num / np.maximum(den, 1e-9), -1.0)      # unobserved -> solid
    F[hG < 0] = 1.0                                               # below the table is never object
    shp = tuple(len(a) for a in ax); F3 = F.reshape(shp)
    from skimage import measure
    import trimesh
    F3 = cv2.GaussianBlur(F3, (0, 0), 0.8) if False else F3
    verts, faces, _, _ = measure.marching_cubes(np.pad(F3, 1, constant_values=1.0), level=0.0, spacing=(vs, vs, vs))
    verts += np.array([a[0] for a in ax]) - vs
    mesh = trimesh.Trimesh(verts, faces); parts = mesh.split(only_watertight=False)
    mesh = max(parts, key=lambda m: len(m.faces)); trimesh.repair.fix_normals(mesh)
    mesh.export(os.path.join(out, "object.obj")); np.savez(os.path.join(out, "object_field.npz"), F=F3, origin=[a[0] for a in ax], vs=vs)
    # ---- report, against the one free ground truth: the width the jaws stopped at
    T = seed["T_world_tcp"]; V = mesh.vertices
    print(f"\nmesh: {len(mesh.faces)} faces, watertight={mesh.is_watertight}, volume {mesh.volume*1e6:.0f} cm^3")
    print(f"extent along world x,y,z (mm): {np.round((V.max(0)-V.min(0))*1000,0)}   height above table {((V@n+dd).max())*1000:.0f} mm")
    for nm, a in [("tool x", T[:3, 0]), ("tool y", T[:3, 1])]:
        p = V @ a; print(f"extent along {nm}: {(p.max()-p.min())*1000:6.1f} mm      [gripper stopped at {float(seed['width_mm']):.0f} mm]")
