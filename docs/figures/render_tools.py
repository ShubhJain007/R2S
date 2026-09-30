#!/usr/bin/env python
"""Render a thumbnail of each of the 20 utiasSTARS workshop tools -> docs/figures/tools/<Name>.png (RGBA).

The thumbnails are used by make_figures.py. Meshes come from the `utias-inertial` clone (MIT, see
third_party/UPSTREAM.md); each tool is viewed along its thinnest principal axis with its longest axis
horizontal.

Usage (env with pyrender, e.g. `rigidworldmodel`):
    PYOPENGL_PLATFORM=egl python docs/figures/render_tools.py
"""
import glob, os
import numpy as np, trimesh, pyrender
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, "utias-inertial", "data", "Workshop Tools Dataset")
OUT = os.path.join(ROOT, "docs", "figures", "tools"); os.makedirs(OUT, exist_ok=True)
W, H, SS = 480, 200, 3                                   # output size, supersampling


def principal_frame(mesh):
    """Rotation whose rows are the principal axes of the vertex cloud, longest first."""
    V = mesh.vertices - mesh.vertices.mean(0)
    w, U = np.linalg.eigh(V.T @ V)
    R = U[:, ::-1].T
    if np.linalg.det(R) < 0: R[2] *= -1
    return R


def look(eye, target=np.zeros(3), up=np.array([0, 1.0, 0])):
    z = eye - target; z /= np.linalg.norm(z); x = np.cross(up, z); x /= np.linalg.norm(x); y = np.cross(z, x)
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, eye; return T


r = pyrender.OffscreenRenderer(W * SS, H * SS)
for d in sorted(os.path.dirname(p) for p in glob.glob(os.path.join(SRC, "*", "mesh.ply"))):
    name = os.path.basename(d)
    m = trimesh.load(os.path.join(d, "mesh.ply"), process=False)
    m = trimesh.Trimesh(m.vertices, m.faces, process=False)          # drop stored colours
    T = np.eye(4); T[:3, :3] = principal_frame(m); m.apply_transform(T)
    m.apply_translation(-m.bounding_box.centroid)
    m.apply_transform(trimesh.transformations.rotation_matrix(np.radians(-28), [1, 0, 0]))   # tip towards the viewer
    ex, ey = m.extents[0], m.extents[1]
    half_w = max(ex / 2, ey / 2 * W / H) * 1.06
    scene = pyrender.Scene(bg_color=[0, 0, 0, 0], ambient_light=[0.16, 0.16, 0.18])
    mat = pyrender.MetallicRoughnessMaterial(baseColorFactor=[0.50, 0.56, 0.66, 1.0], metallicFactor=0.35, roughnessFactor=0.55)
    scene.add(pyrender.Mesh.from_trimesh(m, material=mat, smooth=False))
    far = m.extents.max() * 4
    scene.add(pyrender.OrthographicCamera(xmag=half_w, ymag=half_w * H / W, znear=far * 0.01, zfar=far * 3), pose=look(np.array([0, 0, far])))
    scene.add(pyrender.DirectionalLight(intensity=2.4), pose=look(np.array([-0.6, 0.9, 1.0]) * far))
    scene.add(pyrender.DirectionalLight(intensity=0.7), pose=look(np.array([0.9, -0.2, 0.6]) * far))
    rgba, _ = r.render(scene, flags=pyrender.RenderFlags.RGBA)
    Image.fromarray(rgba).resize((W, H), Image.LANCZOS).save(os.path.join(OUT, name.replace(" ", "_") + ".png"), optimize=True)
    print(f"{name:<20} {m.extents.round(0)} mm")
