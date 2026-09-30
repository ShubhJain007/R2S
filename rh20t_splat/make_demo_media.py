#!/usr/bin/env python
"""Assemble the README media for the RH20T reconstruction from figures already in results/rh20t_splat/.

    demo/real_vs_splat.jpg       camera image next to the splat render from the same calibrated pose (3 cameras)
    demo/calibration_checks.jpg  the frame checks that make pose-free splatting possible
    demo/workspace_flythrough.gif  (only with --frames DIR: PNG frames from render_flythrough.py, played there and back)

Usage:
    python rh20t_splat/make_demo_media.py [--frames DIR]
"""
import argparse, glob, os, subprocess
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES = os.path.join(ROOT, "results", "rh20t_splat"); OUT = os.path.join(RES, "demo"); os.makedirs(OUT, exist_ok=True)
FONT = next((p for p in ("/usr/share/fonts/truetype/lato/Lato-Semibold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf") if os.path.exists(p)), None)
font = lambda s: ImageFont.truetype(FONT, s) if FONT else ImageFont.load_default()


def tag(im, text, size=22, pad=8):
    """Caption pill in the top-left corner."""
    d = ImageDraw.Draw(im, "RGBA"); f = font(size); w = d.textlength(text, font=f)
    d.rounded_rectangle([12, 12, 12 + w + 2 * pad, 12 + size + 2 * pad], radius=6, fill=(13, 17, 23, 200))
    d.text((12 + pad, 12 + pad - 2), text, font=f, fill=(240, 246, 252, 255))
    return im


def real_vs_splat():
    """out_splat/cmp_*.png are [camera | render] pairs at 2560x720: two held-out cameras and one training camera."""
    W, gap = 800, 8; rows = []
    for p in sorted(glob.glob(os.path.join(RES, "out_splat", "cmp_*.png"))):
        im = Image.open(p).convert("RGB"); w, h = im.size
        a = tag(im.crop((0, 0, w // 2, h)).resize((W, W * h // (w // 2)), Image.LANCZOS), "camera")
        b = tag(im.crop((w // 2, 0, w, h)).resize((W, W * h // (w // 2)), Image.LANCZOS), "splat render, same pose")
        row = Image.new("RGB", (2 * W + gap, a.height), (13, 17, 23)); row.paste(a, (0, 0)); row.paste(b, (W + gap, 0)); rows.append(row)
    S = Image.new("RGB", (rows[0].width, sum(r.height for r in rows) + gap * (len(rows) - 1)), (13, 17, 23))
    y = 0
    for r in rows: S.paste(r, (0, y)); y += r.height + gap
    S.save(os.path.join(OUT, "real_vs_splat.jpg"), quality=86, optimize=True); print("wrote demo/real_vs_splat.jpg", S.size)


def calibration_checks():
    tiles = [("out_verify/origin_104122064161.png", "world origin reprojected"),
             ("out_base/base_104122064161.png", "robot base, TCP and marker frames"),
             ("out_ep/ep_104122064161_0094.png", "TCP tracked through an episode")]
    W, gap = 640, 8; ims = []
    for rel, text in tiles:
        im = Image.open(os.path.join(RES, rel)).convert("RGB"); im = im.resize((W, W * im.height // im.width), Image.LANCZOS)
        ims.append(tag(im.crop((0, 40, W, im.height)), text, size=20))                     # crop the debug text burnt into the source
    S = Image.new("RGB", (len(ims) * W + gap * (len(ims) - 1), ims[0].height), (13, 17, 23))
    for k, im in enumerate(ims): S.paste(im, (k * (W + gap), 0))
    S.save(os.path.join(OUT, "calibration_checks.jpg"), quality=86, optimize=True); print("wrote demo/calibration_checks.jpg", S.size)


def flythrough(frames, width=480, fps=12):
    fs = sorted(glob.glob(os.path.join(frames, "*.png"))); seq = fs + fs[-2:0:-1]            # there and back
    lst = os.path.join(frames, "frames.txt")
    open(lst, "w").write("".join(f"file '{f}'\nduration {1 / fps}\n" for f in seq))
    vf = f"scale={width}:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle"
    out = os.path.join(OUT, "workspace_flythrough.gif")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", lst, "-vf", vf, "-r", str(fps), out], check=True)
    print(f"wrote demo/workspace_flythrough.gif  {len(seq)} frames, {os.path.getsize(out) / 1e6:.1f} MB")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--frames"); a = ap.parse_args()
    real_vs_splat(); calibration_checks()
    if a.frames: flythrough(a.frames)
