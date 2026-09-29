#!/usr/bin/env python
"""Verify the RH20T extrinsics convention by cross-camera depth reprojection.

The convention is READ from the API source, not guessed:
    rh20t_api/transforms.py:154  calc_tcp_camera_mat -> `camera_extrinsics @ tcp_world_mat`
    => extrinsics[serial] maps WORLD (= Aruco marker frame) -> CAMERA.

So a point measured in camera A lands in camera B via   E_B @ inv(E_A).

The test: unproject A's depth, push it into B, and compare the depth B actually
measured at that pixel.  If the convention is right the two agree to a few mm.
Every plausible-but-wrong variant is also scored, so the check DISCRIMINATES --
a single number that only the true convention can produce.  Same discipline as
kinematics.detect_wrench_convention(): never trust a frame, measure it.
"""
import os, sys, glob, itertools
import numpy as np, cv2

def load(calib_dir):
    I = np.load(os.path.join(calib_dir, "intrinsics.npy"), allow_pickle=True).item()
    E = np.load(os.path.join(calib_dir, "extrinsics.npy"), allow_pickle=True).item()
    E = {k: np.asarray(v)[0] if np.asarray(v).ndim == 3 else np.asarray(v) for k, v in E.items()}
    K = {k: np.asarray(v)[:3, :3] for k, v in I.items()}
    return K, E

def rgbd(calib_dir, s):
    c = cv2.imread(os.path.join(calib_dir, "imgs", f"cam_{s}_c.png"))
    d = np.load(os.path.join(calib_dir, "imgs", f"cam_{s}_d.npy")).astype(np.float64) / 1000.0  # mm -> m
    return c, d

def unproject(d, K, stride=4, zmin=0.2, zmax=2.5):
    H, W = d.shape
    v, u = np.mgrid[0:H:stride, 0:W:stride]
    z = d[::stride, ::stride]
    m = (z > zmin) & (z < zmax)
    u, v, z = u[m], v[m], z[m]
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    return np.stack([x, y, z, np.ones_like(z)]), (u, v)

def score(calib_dir, A, B, K, E, variant):
    """Median |projected z - measured z| in mm over points that land in B's frame."""
    _, dA = rgbd(calib_dir, A)
    _, dB = rgbd(calib_dir, B)
    P, _ = unproject(dA, K[A])
    if P.shape[1] < 500: return None, 0
    EA, EB = E[A], E[B]
    T = {"correct  E_B @ inv(E_A)": EB @ np.linalg.inv(EA),
         "flipped  inv(E_B) @ E_A": np.linalg.inv(EB) @ EA,
         "no-inv   E_B @ E_A":      EB @ EA,
         "both-inv inv(E_B@E_A)":   np.linalg.inv(EB @ EA)}[variant]
    Q = T @ P
    z = Q[2]
    ok = z > 0.05
    u = K[B][0, 0] * Q[0][ok] / z[ok] + K[B][0, 2]
    v = K[B][1, 1] * Q[1][ok] / z[ok] + K[B][1, 2]
    zc = z[ok]
    H, W = dB.shape
    inb = (u >= 0) & (u < W - 1) & (v >= 0) & (v < H - 1)
    if inb.sum() < 200: return None, int(inb.sum())
    meas = dB[np.round(v[inb]).astype(int), np.round(u[inb]).astype(int)]
    val = meas > 0.2
    if val.sum() < 200: return None, int(val.sum())
    err = np.abs(zc[inb][val] - meas[val]) * 1000.0
    return float(np.median(err)), int(val.sum())

if __name__ == "__main__":
    calib_dir = sys.argv[1]
    K, E = load(calib_dir)
    serials = [s for s in E if os.path.exists(os.path.join(calib_dir, "imgs", f"cam_{s}_d.npy"))]
    print(f"{len(serials)} cameras with depth\n")
    variants = ["correct  E_B @ inv(E_A)", "flipped  inv(E_B) @ E_A",
                "no-inv   E_B @ E_A", "both-inv inv(E_B@E_A)"]
    pairs = list(itertools.combinations(serials, 2))[:15]
    agg = {v: [] for v in variants}
    for var in variants:
        for A, B in pairs:
            e, n = score(calib_dir, A, B, K, E, var)
            if e is not None: agg[var].append(e)
    print(f"{'variant':28s} {'median reproj depth err':>24s}  {'pairs':>6s}")
    print("-" * 64)
    for var in variants:
        v = agg[var]
        s = f"{np.median(v):8.1f} mm" if v else "      n/a"
        print(f"{var:28s} {s:>24s}  {len(v):6d}")
