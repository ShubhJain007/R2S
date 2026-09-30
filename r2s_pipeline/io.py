"""Recording loader and asset writers.

Recording: a single .npz (see recording/RECORDING_SPEC.md).  Required keys depend on mode:
  wrench mode:        t (N,), T_ws (N,4,4), wrench (N,6)            optional: vel (N,6), acc (N,6), gripper (N,)
  joint_torque mode:  t (N,), q (N,nj), tau (N,nj)                   optional: gripper (N,)
Optional for both: T_so (4,4) object-mesh frame in the sensor/EE frame.
"""
import json
import numpy as np


def load_recording(path):
    d = dict(np.load(path, allow_pickle=False))
    for k in ("t", "wrench", "q", "tau", "vel", "acc"):
        if k in d: d[k] = np.asarray(d[k], float)
    if "t" in d: d["t"] = d["t"].ravel()
    if "T_ws" in d: d["T_ws"] = np.asarray(d["T_ws"], float).reshape(-1, 4, 4)
    if "T_so" in d: d["T_so"] = np.asarray(d["T_so"], float).reshape(4, 4)
    return d


def validate(rec, mode):
    need = ("t", "T_ws", "wrench") if mode == "wrench" else ("t", "q", "tau")
    missing = [k for k in need if k not in rec]
    if missing: raise ValueError(f"recording is missing {missing} for mode '{mode}'")
    N = len(rec["t"])
    for k in need[1:]:
        if len(rec[k]) != N: raise ValueError(f"{k} has {len(rec[k])} samples, t has {N}")
    dt = np.diff(rec["t"])
    if np.any(dt <= 0): raise ValueError("timestamps must be strictly increasing")
    return dict(n=N, duration_s=float(rec["t"][-1] - rec["t"][0]), rate_hz=float(1 / np.median(dt)))


def write_json(path, result):
    clean = {k: v for k, v in result.items() if k != "log"}
    with open(path, "w") as f: json.dump(clean, f, indent=2, default=float)


def sdf_inertial_block(result, indent="      "):
    m = result["mass"]; c = result.get("com_object_frame") or result["geometry_centroid_object_frame"]
    I = np.asarray(result["inertia_object_frame_about_com"])
    L = [f"<inertial>", f"  <mass>{m:.6g}</mass>", f"  <pose>{c[0]:.5f} {c[1]:.5f} {c[2]:.5f} 0 0 0</pose>", "  <inertia>"]
    for tag, (i, j) in {"ixx": (0, 0), "ixy": (0, 1), "ixz": (0, 2), "iyy": (1, 1), "iyz": (1, 2), "izz": (2, 2)}.items():
        L.append(f"    <{tag}>{I[i, j]:.6e}</{tag}>")
    L += ["  </inertia>", "</inertial>"]
    return "\n".join(indent + l for l in L)


def urdf_inertial_block(result, indent="    "):
    m = result["mass"]; c = result.get("com_object_frame") or result["geometry_centroid_object_frame"]
    I = np.asarray(result["inertia_object_frame_about_com"])
    L = ["<inertial>", f'  <origin xyz="{c[0]:.5f} {c[1]:.5f} {c[2]:.5f}" rpy="0 0 0"/>', f'  <mass value="{m:.6g}"/>',
         f'  <inertia ixx="{I[0,0]:.6e}" ixy="{I[0,1]:.6e}" ixz="{I[0,2]:.6e}" iyy="{I[1,1]:.6e}" iyz="{I[1,2]:.6e}" izz="{I[2,2]:.6e}"/>',
         "</inertial>"]
    return "\n".join(indent + l for l in L)


def patch_sdf(src, dst, result):
    """Replace the first <inertial> block of an existing SDF with the identified one."""
    from lxml import etree as ET
    tree = ET.parse(str(src)); inert = tree.getroot().find(".//inertial")
    if inert is None: raise ValueError("no <inertial> block in source SDF")
    new = ET.fromstring(sdf_inertial_block(result, indent=""))
    inert.getparent().replace(inert, new)
    tree.write(str(dst), pretty_print=True, xml_declaration=True, encoding="utf-8")
