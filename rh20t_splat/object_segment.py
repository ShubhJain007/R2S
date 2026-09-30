#!/usr/bin/env python
"""Rigid-object segmentation from calibrated multi-view depth -- no neural segmenter.

  1. static window: the first seconds of the episode, arm parked, object at rest
  2. per fixed camera: temporal-median depth (kills depth noise and transient pixels)
  3. fuse to the world (marker) frame, RANSAC the table plane, keep what sticks up from it
  4. cluster; the OBJECT is the cluster nearest the TCP at the moment the jaws stop closing
     -- an arm-agnostic selector: no labels, no prompts, just "what did the gripper take?"
"""
import os, sys, numpy as np, cv2, open3d as o3d
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rh20t_api"))
from wrist_poses import setup, load_tcp_series
from depth_io import read_depth_video
from rh20t_api.transforms import pose_array_quat_2_matrix

def grasp_event(ep, conf, T_world_base):
    """time the jaws stop on the object, the width they stopped at, and the TCP pose then (world)."""
    gr = np.load(os.path.join(ep, "transformed", "gripper.npy"), allow_pickle=True).item()
    s = conf.in_hand_serial[0]; gk = np.array(sorted(gr[s])); gw = np.array([gr[s][k]["gripper_info"][0] for k in gk], float)
    o, c = gw.max(), gw.min(); held = (gw > c + 5) & (gw < o - 5)
    # longest run of 'held' with near-constant width = the real grasp
    best, i = (0, 0, 0), 0
    while i < len(held):
        if held[i]:
            j = i
            while j + 1 < len(held) and held[j + 1] and abs(gw[j + 1] - gw[i]) < 4: j += 1
            if j - i > best[0]: best = (j - i, i, j)
            i = j + 1
        else: i += 1
    _, i0, i1 = best
    tt, pp = load_tcp_series(ep, s)
    T = T_world_base @ pose_array_quat_2_matrix(pp[int(np.argmin(np.abs(tt - gk[i0])))].astype(np.float64))
    return dict(t_grasp=int(gk[i0]), t_release=int(gk[i1]), width_mm=float(np.median(gw[i0:i1 + 1])), T_world_tcp=T, t0=int(gk[0]))

def static_depth(ep, serial, t_end, n_max=40):
    ts = np.load(os.path.join(ep, f"cam_{serial}", "timestamps.npy"), allow_pickle=True).item()
    td = np.asarray(ts.get("depth", ts["color"]))
    fr = read_depth_video(os.path.join(ep, f"cam_{serial}", "depth.mp4"), serial, max_frames=n_max)
    k = [i for i in range(min(len(fr), len(td))) if td[i] < t_end]
    if len(k) < 5: return None
    a = np.stack([fr[i] for i in k]).astype(np.float32) / 1000.0; a[a <= 0] = np.nan
    with np.errstate(all="ignore"): d = np.nanmedian(a, axis=0)
    d[~np.isfinite(d)] = 0
    return d

def backproject(d, K, E, stride=1, zmax=2.0):
    H, W = d.shape; v, u = np.mgrid[0:H:stride, 0:W:stride]; z = d[::stride, ::stride]
    m = (z > 0.15) & (z < zmax); u, v, z = u[m], v[m], z[m]
    P = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)])
    return (np.linalg.inv(E) @ P)[:3].T

if __name__ == "__main__":
    ep, calib, out = sys.argv[1], sys.argv[2], sys.argv[3]; os.makedirs(out, exist_ok=True)
    conf, K, E, T_world_base = setup(calib)
    g = grasp_event(ep, conf, T_world_base)
    tcp = g["T_world_tcp"][:3, 3]
    print(f"grasp at +{(g['t_grasp']-g['t0'])/1000:.1f}s, release +{(g['t_release']-g['t0'])/1000:.1f}s, jaws stopped at {g['width_mm']:.0f} mm")
    print("TCP at grasp (world, m):", np.round(tcp, 3), " tool z-axis:", np.round(g["T_world_tcp"][:3, 2], 2))
    fixed = [d[4:] for d in sorted(os.listdir(ep)) if d.startswith("cam_") and d[4:] not in conf.in_hand_serial
             and not d[4:].startswith("f") and os.path.exists(os.path.join(ep, d, "depth.mp4"))]
    clouds = {}
    for s in fixed:
        d = static_depth(ep, s, g["t0"] + 3000)
        if d is None: continue
        Ks = K[s].copy(); Ks[:2] *= d.shape[1] / 1280.0
        clouds[s] = backproject(d, Ks, E[s], stride=2)
    P = np.concatenate(list(clouds.values()))
    roi = np.linalg.norm(P[:, :2] - tcp[:2], axis=1) < 0.6           # workspace around the grasp
    P = P[roi]; print(f"{len(clouds)} fixed cameras, {len(P)} points in the workspace")
    pc = o3d.geometry.PointCloud(); pc.points = o3d.utility.Vector3dVector(P)
    plane, inl = pc.segment_plane(0.008, 3, 1000); n = np.array(plane[:3]); dd = plane[3]
    if n[2] < 0: n, dd = -n, -dd
    h = P @ n + dd
    print(f"table plane: normal {np.round(n,3)}, height offset {-dd/n[2]*1000:.0f} mm in marker z, inliers {len(inl)/len(P):.0%}")
    up = P[(h > 0.012) & (h < 0.20)]
    pu = o3d.geometry.PointCloud(); pu.points = o3d.utility.Vector3dVector(up); pu = pu.voxel_down_sample(0.004)
    U = np.asarray(pu.points); lab = np.array(pu.cluster_dbscan(eps=0.015, min_points=12))
    print(f"above-table points: {len(U)}, clusters: {lab.max()+1}")
    rows = []
    for c in range(lab.max() + 1):
        Q = U[lab == c]
        if len(Q) < 40: continue
        ext = Q.max(0) - Q.min(0); cen = Q.mean(0)
        rows.append((np.linalg.norm(cen[:2] - tcp[:2]), c, len(Q), cen, ext))
    rows.sort(key=lambda r: r[0])
    print(f"\n{'dist to TCP':>11s} {'cluster':>7s} {'pts':>6s} {'centre (m)':>24s} {'extent (mm)':>20s}")
    for dist, c, npts, cen, ext in rows[:6]:
        print(f"{dist*1000:9.0f}mm {c:7d} {npts:6d} {str(np.round(cen,3)):>24s} {str(np.round(ext*1000,0)):>20s}")
    dist, c, npts, cen, ext = rows[0]
    np.savez(os.path.join(out, "object_seed.npz"), points=U[lab == c], plane=np.r_[n, dd], tcp=tcp,
             T_world_tcp=g["T_world_tcp"], width_mm=g["width_mm"], t_grasp=g["t_grasp"], t0=g["t0"])
    # top-down debug picture
    S = 900; img = np.full((S, S, 3), 255, np.uint8); sc = S / 1.2; o = tcp[:2] - 0.6
    def px(p): return tuple(np.round((p[:2] - o) * sc).astype(int))
    for p in U[::2]: cv2.circle(img, px(p), 1, (190, 190, 190), -1)
    for p in U[lab == c]: cv2.circle(img, px(p), 1, (0, 140, 255), -1)
    cv2.drawMarker(img, px(tcp), (0, 0, 255), cv2.MARKER_CROSS, 24, 2); cv2.putText(img, "TCP at grasp", (px(tcp)[0] + 10, px(tcp)[1] - 10), 0, 0.6, (0, 0, 255), 2)
    cv2.imwrite(os.path.join(out, "topdown.png"), img)
    print(f"\nselected cluster {c}: extent {np.round(ext*1000,0)} mm, {dist*1000:.0f} mm from TCP (gripper measured {g['width_mm']:.0f} mm)")
