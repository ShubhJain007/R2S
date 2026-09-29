#!/usr/bin/env python
"""M2 de-risk: recover joint axes from GEOMETRY ALONE, given parent/child meshes in assembled pose.

Two paths through the same loader:
  --synthetic   five articulated objects mimicking PartNet-Mobility categories, built with exact
                ground-truth axes and written out as URDF+OBJ (so the loader is exercised), then
                recovered. Includes the hard cases: hinge gap, axis offset from contact (laptop
                barrel), face contact instead of line contact (box lid), prismatic, mesh noise.
  --partnet DIR real PartNet-Mobility object dirs (mobility.urdf + textured_objs/). Same code.

Recovery (the M2 algorithm):
  1. sample child surface, keep points within tau of the parent  -> contact region
  2. revolute: fit the axis line to the contact region, gated by a semantic hint
       hint = direction ("vertical"/"horizontal-x"/...) + optional edge side ("-x","+y",...)
       - if the contact region is line-like (PCA anisotropic): axis = principal direction through centroid
       - if it is face-like (box lid): axis = the edge of the contact region on the hinted side
  3. prismatic: sliding direction = principal direction of the contact region, gated by hint
Reported: axis direction error (deg), axis position error (mm, min distance between lines),
with and without the semantic hint.
"""
import argparse, os, sys, json, xml.etree.ElementTree as ET
import numpy as np, trimesh

rng = np.random.default_rng(0)
UNIT = {"x": np.array([1., 0, 0]), "y": np.array([0, 1., 0]), "z": np.array([0, 0, 1.])}


# --------------------------------------------------------------------------- synthetic objects
def box(ext, center):
    m = trimesh.creation.box(extents=ext); m.apply_translation(center); return m


def make_objects():
    """Each: parent mesh, child mesh (assembled pose, world frame), truth {type, axis_dir, axis_pt}, hint."""
    objs = {}
    # 1 cabinet door: flush hinge on the door's -x vertical edge (easy)
    body = box([0.6, 0.4, 0.8], [0, 0, 0.4])
    door = box([0.01, 0.38, 0.76], [-0.305, 0, 0.4])
    objs["cabinet_door"] = (body, door, dict(type="revolute", d=UNIT["z"], p=np.array([-0.30, 0.19, 0.4])),
                            dict(dir="vertical", side="+y"))
    # 2 gate on post: 6 mm hinge gap (medium)
    post = box([0.08, 0.08, 1.2], [0, 0, 0.6])
    gate = box([0.03, 0.9, 1.0], [0, 0.04 + 0.006 + 0.45, 0.55])
    objs["gate_post"] = (post, gate, dict(type="revolute", d=UNIT["z"], p=np.array([0, 0.04 + 0.003, 0.55])),
                         dict(dir="vertical", side="-y"))
    # 3 laptop: lid open 90 deg, axis inside a hinge barrel OFFSET 8 mm from the contact plane (hard)
    base = box([0.30, 0.22, 0.015], [0, 0, 0.0075])
    lid = box([0.30, 0.012, 0.20], [0, -0.11 - 0.006, 0.015 + 0.10])
    objs["laptop"] = (base, lid, dict(type="revolute", d=UNIT["x"], p=np.array([0, -0.11, 0.015 + 0.008])),
                      dict(dir="horizontal-x", side=None))
    # 4 drawer: prismatic along -y, sliding inside a cavity (contact = whole sliding surfaces)
    cab = box([0.5, 0.5, 0.3], [0, 0, 0.15])
    cavity = box([0.42, 0.46, 0.22], [0, 0.0, 0.15])
    cab = trimesh.boolean.difference([cab, cavity])
    drawer = box([0.40, 0.44, 0.20], [0, 0.0, 0.15])
    objs["drawer"] = (cab, drawer, dict(type="prismatic", d=-UNIT["y"], p=np.array([0, 0, 0.15])),
                      dict(dir="horizontal-y", side=None))
    # 5 box lid: closed, contact is the ENTIRE top face; hinge is the -x edge (needs the side hint)
    bx = box([0.3, 0.2, 0.15], [0, 0, 0.075])
    lid = box([0.3, 0.2, 0.01], [0, 0, 0.155])
    objs["box_lid"] = (bx, lid, dict(type="revolute", d=UNIT["y"], p=np.array([-0.15, 0, 0.15])),
                       dict(dir="horizontal-y", side="-x"))
    return objs


def perturb(m, noise=0.0007, drop_faces=0.02):
    """Generated-mesh realism: vertex jitter + a few missing faces (non-watertight)."""
    m = m.copy()
    m = m.subdivide_to_size(0.02)              # dense enough to sample
    m.vertices += rng.normal(0, noise, m.vertices.shape)
    keep = rng.random(len(m.faces)) > drop_faces
    m.update_faces(keep)
    return m


# --------------------------------------------------------------------------- URDF I/O (real-data path)
def write_urdf(name, parent, child, truth, out_dir):
    d = os.path.join(out_dir, name); os.makedirs(d, exist_ok=True)
    parent.export(os.path.join(d, "parent.obj"))
    # child mesh expressed in the JOINT frame (origin at axis point, identity rotation)
    c = child.copy(); c.apply_translation(-truth["p"]); c.export(os.path.join(d, "child.obj"))
    p = truth["p"]; ax = truth["d"]
    urdf = f"""<robot name="{name}">
  <link name="parent"><visual><geometry><mesh filename="parent.obj"/></geometry></visual></link>
  <link name="child"><visual><geometry><mesh filename="child.obj"/></geometry></visual></link>
  <joint name="j" type="{truth['type']}">
    <parent link="parent"/><child link="child"/>
    <origin xyz="{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}" rpy="0 0 0"/>
    <axis xyz="{ax[0]:.6f} {ax[1]:.6f} {ax[2]:.6f}"/>
  </joint>
</robot>"""
    open(os.path.join(d, "mobility.urdf"), "w").write(urdf)
    return d


def _rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp,   cp*sr,            cp*cr]])


def load_urdf_joints(obj_dir):
    """Yield (joint_name, type, parent_mesh_world, child_mesh_world, truth) for every movable joint.
    Handles PartNet-Mobility mobility.urdf: links with multiple visual meshes and joint chains."""
    root = ET.parse(os.path.join(obj_dir, "mobility.urdf")).getroot()
    links = {}
    for L in root.findall("link"):
        parts = []
        for v in L.findall("visual"):
            g = v.find("geometry/mesh")
            if g is None: continue
            fn = os.path.join(obj_dir, g.get("filename"))
            if not os.path.exists(fn): continue
            m = trimesh.load(fn, force="mesh", process=False)
            o = v.find("origin")
            if o is not None:
                xyz = np.array([float(x) for x in (o.get("xyz") or "0 0 0").split()])
                rpy = [float(x) for x in (o.get("rpy") or "0 0 0").split()]
                T = np.eye(4); T[:3, :3] = _rpy_to_R(*rpy); T[:3, 3] = xyz; m.apply_transform(T)
            parts.append(m)
        links[L.get("name")] = trimesh.util.concatenate(parts) if parts else None
    joints = []
    for J in root.findall("joint"):
        o = J.find("origin"); xyz = np.array([float(x) for x in (o.get("xyz") if o is not None else "0 0 0").split()])
        rpy = [float(x) for x in ((o.get("rpy") if o is not None else None) or "0 0 0").split()]
        a = J.find("axis"); ax = np.array([float(x) for x in (a.get("xyz") if a is not None else "1 0 0").split()])
        joints.append(dict(name=J.get("name"), type=J.get("type"), parent=J.find("parent").get("link"),
                           child=J.find("child").get("link"), xyz=xyz, R=_rpy_to_R(*rpy), axis=ax))
    # world transform of each link = chain of joint origins from root
    parent_of = {j["child"]: j for j in joints}
    def T_world(link):
        T = np.eye(4); l = link
        while l in parent_of:
            j = parent_of[l]; Tj = np.eye(4); Tj[:3, :3] = j["R"]; Tj[:3, 3] = j["xyz"]
            T = Tj @ T; l = j["parent"]
        return T
    for j in joints:
        if j["type"] not in ("revolute", "prismatic", "continuous"): continue
        pm, cm = links.get(j["parent"]), links.get(j["child"])
        if pm is None or cm is None: continue
        Tp, Tc = T_world(j["parent"]), T_world(j["child"])
        P = pm.copy(); P.apply_transform(Tp); C = cm.copy(); C.apply_transform(Tc)
        d = Tc[:3, :3] @ j["axis"]; d = d / np.linalg.norm(d); p = Tc[:3, 3]
        yield j["name"], ("revolute" if j["type"] == "continuous" else j["type"]), P, C, dict(type=j["type"], d=d, p=p)


# --------------------------------------------------------------------------- M2: axis recovery
HINT_DIR = {"vertical": UNIT["z"], "horizontal-x": UNIT["x"], "horizontal-y": UNIT["y"], None: None}


def contact_region(parent, child, n=6000, tau=None):
    pts, _ = trimesh.sample.sample_surface(child, n, seed=0)
    _, d, _ = trimesh.proximity.closest_point(parent, pts)
    if tau is None:
        diag = np.linalg.norm(child.extents); tau = max(d.min() + 0.004, 0.01 * diag)   # adaptive
    C = pts[d <= tau]
    return C, tau, d.min()


def recover(parent, child, jtype, hint, use_hint=True):
    C, tau, dmin = contact_region(parent, child)
    if len(C) < 20:
        return None, dict(reason=f"only {len(C)} contact points (min dist {dmin*1000:.1f} mm)")
    cen = C.mean(0); U, S, Vt = np.linalg.svd(C - cen, full_matrices=False)
    ratio = S[1] / max(S[0], 1e-12)             # <0.35 -> line-like, else face-like
    hd = HINT_DIR.get(hint["dir"]) if use_hint else None
    if jtype == "prismatic":
        if hd is not None and ratio > 0.35:        # face-like patch: PCA direction is ill-conditioned, trust the hint
            d = hd * (np.sign(Vt[0] @ hd) or 1.0)
        else:
            d = Vt[0] if hd is None else (hd if abs(Vt[0] @ hd) < 0.7 else Vt[0] * np.sign(Vt[0] @ hd))
        return dict(d=d / np.linalg.norm(d), p=cen), dict(n=len(C), ratio=ratio, mode="prismatic" + ("+hint" if hd is not None and ratio > 0.35 else ""))
    # revolute
    if ratio < 0.35:                            # line-like contact (hinge edge)
        d = Vt[0]
        if hd is not None and abs(d @ hd) > 0.7: d = hd
        return dict(d=d / np.linalg.norm(d), p=cen), dict(n=len(C), ratio=ratio, mode="line")
    # face-like contact: need the edge on the hinted side
    if hd is None or not use_hint or hint.get("side") is None:
        # no hint: best guess = longest in-plane direction through centroid (will be wrong on position)
        return dict(d=Vt[0], p=cen), dict(n=len(C), ratio=ratio, mode="face-nohint")
    side = hint["side"]; sgn = -1 if side[0] == "-" else 1; k = "xyz".index(side[1])
    e = C[:, k]; edge = C[(e - e.min() if sgn < 0 else e.max() - e) < 0.15 * (e.max() - e.min() + 1e-9)]
    p = edge.mean(0); p[k] = e.min() if sgn < 0 else e.max()
    return dict(d=hd, p=p), dict(n=len(C), ratio=ratio, mode="face+side-hint")


def axis_errors(rec, truth):
    d1, d2 = rec["d"], truth["d"]
    ang = np.degrees(np.arccos(np.clip(abs(d1 @ d2), 0, 1)))
    # min distance between the two lines (for prismatic only direction matters -> report distance anyway)
    w = rec["p"] - truth["p"]; n = np.cross(d1, d2)
    dist = abs(w @ n) / np.linalg.norm(n) if np.linalg.norm(n) > 1e-6 else np.linalg.norm(np.cross(w, d2))
    return ang, dist * 1000


# --------------------------------------------------------------------------- run
def run_set(items, label):
    print(f"\n{'='*100}\n {label}\n{'='*100}")
    print(f"{'object':<16}{'joint':>10} | {'with hint: dir° / pos mm':>26} {'mode':>14} | {'no hint: dir° / pos mm':>24} {'mode':>12}")
    print("-" * 100)
    ok = 0; tot = 0
    for name, jtype, P, C, truth, hint in items:
        tot += 1
        r1, i1 = recover(P, C, jtype, hint, use_hint=True)
        r0, i0 = recover(P, C, jtype, hint, use_hint=False)
        s1 = f"{'FAIL: '+i1.get('reason','')[:20]:>26}" if r1 is None else (lambda a, d: f"{a:>10.2f} / {d:>8.1f}  {'PASS' if (a < 2 and (d < 5 or jtype=='prismatic')) else 'fail'}")(*axis_errors(r1, truth))
        s0 = f"{'FAIL':>24}" if r0 is None else (lambda a, d: f"{a:>10.2f} / {d:>8.1f}")(*axis_errors(r0, truth))
        if r1 is not None:
            a, d = axis_errors(r1, truth); ok += int(a < 2 and (d < 5 or jtype == "prismatic"))
        print(f"{name:<16}{jtype:>10} | {s1} {i1.get('mode',''):>14} | {s0} {i0.get('mode',''):>12}")
    print("-" * 100); print(f"  PASS {ok}/{tot}  (criteria: direction < 2 deg, position < 5 mm; prismatic: direction only)")
    return ok, tot


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--partnet", help="dir of PartNet-Mobility object dirs")
    ap.add_argument("--out", default="/home/kneepolean/shubhj/R2S/results/axis_test"); a = ap.parse_args()
    if a.partnet:
        items = []
        for od in sorted(os.listdir(a.partnet)):
            p = os.path.join(a.partnet, od)
            if not os.path.exists(os.path.join(p, "mobility.urdf")): continue
            for jn, jt, P, C, truth in load_urdf_joints(p):
                hint = dict(dir=None, side=None)      # real data: hint must come from the LLM stage
                items.append((f"{od}/{jn}"[:16], jt, P, C, truth, hint))
        run_set(items, f"PartNet-Mobility ({len(items)} joints, NO hints)")
        sys.exit()
    objs = make_objects(); os.makedirs(a.out, exist_ok=True)
    clean, noisy, reloaded = [], [], []
    for name, (P, C, truth, hint) in objs.items():
        clean.append((name, truth["type"], P, C, truth, hint))
        noisy.append((name, truth["type"], perturb(P), perturb(C), truth, hint))
        d = write_urdf(name, P, C, truth, a.out)
        for jn, jt, Pl, Cl, tl in load_urdf_joints(d):
            reloaded.append((name, jt, Pl, Cl, tl, hint))
    run_set(clean, "SYNTHETIC, clean CAD meshes")
    run_set(noisy, "SYNTHETIC, perturbed (0.7 mm vertex noise, 2% faces dropped -> non-watertight)")
    run_set(reloaded, "SYNTHETIC written to URDF+OBJ and RELOADED through the PartNet loader (loader check)")
    print(f"\nURDFs written to {a.out}/<object>/mobility.urdf  -- run --partnet on real PartNet-Mobility dirs with the same code.")
