#!/usr/bin/env python
"""The README's opening animation: the five self-test tools turning, with the true and the estimated centre
of mass drawn on each and the measured errors underneath.

Two steps, because the pipeline and the renderer live in different conda environments:

    python docs/figures/tool_demo.py estimate     # env with the pipeline (s2s)   -> docs/figures/tools/selftest.json
    PYOPENGL_PLATFORM=egl python docs/figures/tool_demo.py render   # env with pyrender -> docs/figures/tools_demo.gif

`estimate` runs exactly what `python -m r2s_pipeline selftest` runs and stores its numbers; nothing is
hand-entered in `render`.
"""
import json, os, subprocess, sys, tempfile
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
DATA = os.path.join(HERE, "tools", "selftest.json")
TOOLS = ["Hammer", "Pliers", "Screwdriver", "Vise_Grip", "Box_Wrench"]          # the self-test's default set


def estimate():
    sys.path.insert(0, ROOT)
    from r2s_pipeline import identify
    from r2s_pipeline.cli import utias_to_recording
    out = []
    for obj in TOOLS:
        rec, gt, mesh = utias_to_recording(obj)
        res = identify(rec, mesh_path=mesh, T_so=rec["T_so"], mode="wrench", mesh_scale=0.001, verbose=False)
        I = np.asarray(res["inertia_object_frame_about_com"])
        out.append(dict(obj=obj, mesh=os.path.relpath(mesh, ROOT), mass=res["mass"], mass_gt=gt["mass"], com=res["com_object_frame"], com_gt=gt["com"].tolist(),
                        mass_err=abs(res["mass"] - gt["mass"]) / gt["mass"] * 100, com_err_mm=float(np.linalg.norm(np.asarray(res["com_object_frame"]) - gt["com"]) * 1000),
                        inertia_err=float(np.linalg.norm(I - gt["I"]) / np.linalg.norm(gt["I"]) * 100)))
        print(f"{obj:<12} mass {out[-1]['mass_err']:.2f}%  CoM {out[-1]['com_err_mm']:.1f} mm  geometry-inertia {out[-1]['inertia_err']:.1f}%")
    json.dump(out, open(DATA, "w"), indent=1); print("saved ->", os.path.relpath(DATA, ROOT))


def render(frames=40, fps=12, col=172, top=250, height=384, ss=3):
    import trimesh, pyrender
    from PIL import Image, ImageDraw, ImageFont
    tools = json.load(open(DATA)); W = col * len(tools)
    BG, INK, SUB, EST = (13, 17, 23), (240, 246, 252), (145, 152, 161), (57, 135, 229)
    fp = lambda n: next((p for p in (f"/usr/share/fonts/truetype/lato/Lato-{n}.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf") if os.path.exists(p)), None)
    font = lambda n, s: ImageFont.truetype(fp(n), s) if fp(n) else ImageFont.load_default()
    r = pyrender.OffscreenRenderer(col * ss, top * ss); cam_pose = np.eye(4); cam_pose[2, 3] = 2000.0
    prep = []
    for t in tools:
        m = trimesh.load(os.path.join(ROOT, t["mesh"]), process=False); m = trimesh.Trimesh(m.vertices, m.faces, process=False)   # millimetres
        V = m.vertices - m.vertices.mean(0); _, U = np.linalg.eigh(V.T @ V); R = U[:, ::-1].T              # rows: long, mid, thin axis
        if np.linalg.det(R) < 0: R[2] *= -1
        A = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]]) @ R                                           # long axis -> image up
        c = (A @ m.vertices.T).T; off = (c.max(0) + c.min(0)) / 2
        T = np.eye(4); T[:3, :3] = A; T[:3, 3] = -off; m.apply_transform(T)
        half_h = m.extents[1] / 2 * 1.12; half_w = half_h * col / top
        if max(m.extents[0], m.extents[2]) / 2 * 1.1 > half_w: half_w = max(m.extents[0], m.extents[2]) / 2 * 1.1; half_h = half_w * top / col
        pts = [A @ (np.asarray(t[k]) * 1000) - off for k in ("com_gt", "com")]
        prep.append((m, half_w, half_h, pts))
    tmp = tempfile.mkdtemp()
    for f in range(frames):
        th = 2 * np.pi * f / frames; Ry = trimesh.transformations.rotation_matrix(th, [0, 1, 0])
        img = Image.new("RGB", (W, height), BG); d = ImageDraw.Draw(img)
        for k, (t, (m, hw, hh, pts)) in enumerate(zip(tools, prep)):
            sc = pyrender.Scene(bg_color=[*BG, 255], ambient_light=[0.18, 0.18, 0.2])
            sc.add(pyrender.Mesh.from_trimesh(m, material=pyrender.MetallicRoughnessMaterial(baseColorFactor=[0.55, 0.61, 0.70, 1.0], metallicFactor=0.35, roughnessFactor=0.55), smooth=False), pose=Ry)
            sc.add(pyrender.OrthographicCamera(xmag=hw, ymag=hh, znear=10, zfar=4000), pose=cam_pose)
            L = np.eye(4); L[:3, :3] = trimesh.transformations.euler_matrix(-0.6, -0.5, 0)[:3, :3]; sc.add(pyrender.DirectionalLight(intensity=2.6), pose=L)
            rgb, _ = r.render(sc); img.paste(Image.fromarray(rgb).resize((col, top), Image.LANCZOS), (k * col, 0))
            for p, est in zip(pts, (False, True)):                       # true CoM: ring.  estimated: dot.  Drawn on top, as an X-ray.
                q = Ry[:3, :3] @ p; x = k * col + col / 2 + q[0] / hw * col / 2; y = top / 2 - q[1] / hh * top / 2
                if est: d.ellipse([x - 4, y - 4, x + 4, y + 4], fill=EST, outline=BG, width=1)
                else: d.ellipse([x - 8, y - 8, x + 8, y + 8], outline=INK, width=2)
            cx = k * col + col / 2
            d.text((cx, top + 12), t["obj"].replace("_", " "), font=font("Bold", 15), fill=INK, anchor="ma")
            d.text((cx, top + 34), f"{t['mass_gt'] * 1000:.0f} g true, {t['mass'] * 1000:.0f} g estimated", font=font("Regular", 12), fill=SUB, anchor="ma")
            d.text((cx, top + 52), f"mass error {t['mass_err']:.2f}%", font=font("Bold", 13), fill=INK, anchor="ma")
            d.text((cx, top + 72), f"centre of mass {t['com_err_mm']:.1f} mm off", font=font("Regular", 12), fill=SUB, anchor="ma")
        y = height - 20; x = W / 2 - 236
        d.ellipse([x, y - 7, x + 14, y + 7], outline=INK, width=2); d.text((x + 22, y), "true centre of mass", font=font("Regular", 12), fill=SUB, anchor="lm")
        d.ellipse([x + 160, y - 4, x + 168, y + 4], fill=EST); d.text((x + 176, y), "estimated from the wrist wrench during gentle motion", font=font("Regular", 12), fill=SUB, anchor="lm")
        img.save(os.path.join(tmp, f"{f:03d}.png"))
    out = os.path.join(HERE, "tools_demo.gif")
    vf = "split[a][b];[a]palettegen=max_colors=64:stats_mode=full[p];[b][p]paletteuse=dither=none"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i", os.path.join(tmp, "%03d.png"), "-vf", vf, out], check=True)
    print(f"wrote {os.path.relpath(out, ROOT)}  {frames} frames, {os.path.getsize(out) / 1e6:.1f} MB")


if __name__ == "__main__":
    {"estimate": estimate, "render": render}[sys.argv[1] if len(sys.argv) > 1 else "render"]()
