#!/usr/bin/env python
"""Render a camera sweep through a trained splat -> numbered PNG frames (novel views, no input camera).

The world origin is the calibration marker on the table with +z up (verify_calib2.py), so the camera
orbits a point just beside it and looks down at the work surface. Works on both checkpoint layouts
written by train_splat.py (plain `colors`, or `sh0`/`shN`).

Usage:
    python rh20t_splat/render_flythrough.py rh20t_splat/out_splat/splat.pt OUT_DIR [--az 55 170] [--frames 48]
    python rh20t_splat/make_demo_media.py --frames OUT_DIR      # frames -> results/rh20t_splat/demo/workspace_flythrough.gif
"""
import argparse, math, os
import numpy as np, cv2, torch
from gsplat import rasterization

DEV = "cuda"


def look_at(eye, target, up=np.array([0, 0, 1.0])):
    """world->camera (OpenCV: x right, y down, z forward) -- the convention gsplat's viewmats use."""
    f = target - eye; f /= np.linalg.norm(f); r = np.cross(f, up); r /= np.linalg.norm(r); d = np.cross(f, r)
    R = np.stack([r, d, f]); E = np.eye(4); E[:3, :3] = R; E[:3, 3] = -R @ eye
    return E


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ckpt"); ap.add_argument("out")
    ap.add_argument("--az", type=float, nargs=2, default=[55, 170], help="start / end azimuth, degrees")
    ap.add_argument("--radius", type=float, default=0.85); ap.add_argument("--height", type=float, default=0.60)
    ap.add_argument("--target", type=float, nargs=3, default=[0.0, 0.10, 0.0])
    ap.add_argument("--frames", type=int, default=48); ap.add_argument("--size", type=int, nargs=2, default=[960, 540])
    ap.add_argument("--focal", type=float, default=560.0)
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)

    P = {k: v.to(DEV) for k, v in torch.load(a.ckpt, map_location="cpu").items()}
    if "colors" in P: col, shd = torch.sigmoid(P["colors"]), None
    else: col = torch.cat([P["sh0"], P["shN"]], 1); shd = int(math.isqrt(col.shape[1])) - 1
    W, H = a.size; K = torch.tensor([[a.focal, 0, W / 2], [0, a.focal, H / 2], [0, 0, 1.0]], device=DEV)[None]
    tgt = np.array(a.target)
    # ease in/out so the turn-around of a ping-pong loop does not jerk
    s = 0.5 - 0.5 * np.cos(np.linspace(0, np.pi, a.frames))
    with torch.no_grad():
        for i, az in enumerate(np.radians(a.az[0] + s * (a.az[1] - a.az[0]))):
            eye = tgt + np.array([a.radius * math.cos(az), a.radius * math.sin(az), a.height])
            vm = torch.tensor(look_at(eye, tgt), dtype=torch.float32, device=DEV)[None]
            out, _, _ = rasterization(P["means"], torch.nn.functional.normalize(P["quats"], dim=-1), torch.exp(P["scales"]),
                                      torch.sigmoid(P["opacities"]), col, vm, K, W, H, sh_degree=shd, render_mode="RGB")
            cv2.imwrite(os.path.join(a.out, f"{i:03d}.png"), (out[0].clamp(0, 1).cpu().numpy() * 255).astype(np.uint8)[:, :, ::-1].copy())
    print(f"{a.frames} frames -> {a.out}")


if __name__ == "__main__": main()
