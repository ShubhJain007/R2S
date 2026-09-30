#!/usr/bin/env python
"""Hybrid physical-parameter pipeline:  mass + CoM from joint torque,  inertia from geometry.

Built on the Scalable Real2Sim identification recipe (see s2s_tests.Problem), replacing only the
rotational-inertia source. Rationale (Phase 0, 2026-09-17): torque-identified inertia on these
objects is a ~0.025 kg m^2 constant artifact independent of the object, while uniform-density mesh
inertia is physically plausible and predicts held-out torques better.

Methods (all share the same fitted friction / reflected-inertia terms for link 7):
  torque         mass, CoM, inertia all from the SDP            (= Scalable Real2Sim baseline)
  hybrid         mass, CoM from SDP; inertia from geometry       (proposed)
  hybrid_geomcom mass from SDP; CoM and inertia from geometry    (ablation)

Usage:
  python experiments/scalable_real2sim/hybrid_pipeline.py spam [sugar lego ...]
    -> results/hybrid/<obj>_hybrid_inertial_params.json   (their JSON schema, object frame, about CoM)
    -> results/hybrid/<obj>_hybrid.sdf                     (shipped SDF with <inertial> replaced)
"""
import json, sys
from pathlib import Path
import numpy as np
import trimesh
from lxml import etree as ET
from pydrake.all import RigidTransform, RotationalInertia, SpatialInertia

from s2s_tests import Problem, NJ, DATA, ROOT

METHODS = ("torque", "hybrid", "hybrid_geomcom")


class HybridIdentifier(Problem):
    def __init__(self, obj):
        super().__init__(obj)
        self.mesh = trimesh.load(self.mesh_path, process=False)
        if not self.mesh.is_watertight:
            trimesh.repair.fill_holes(self.mesh)
        self.mesh_watertight = self.mesh.is_watertight

    # ---- geometry-derived quantities (object frame) ----
    def geometry_inertia(self, mass, pitch=0.002):
        """Uniform-density rotational inertia about the centroid, scaled to `mass`.
        Computed on a filled VOXELISATION of the mesh rather than a surface integral: the BundleSDF
        meshes are not watertight (thousands of boundary edges), and the divergence-theorem volume
        is unreliable on open surfaces. Voxel vs surface differ by only 0.5-6% in I/m here, but the
        voxel route does not depend on closure."""
        if not hasattr(self, "_vox_pts"):
            self._vox_pts = np.asarray(self.mesh.voxelized(pitch=pitch).fill().points)
        P = self._vox_pts; com = P.mean(0); r = P - com
        w = mass / len(P)
        I = np.zeros((3, 3))
        for k in range(3):
            for l in range(3):
                I[k, l] = w * (((r * r).sum(1) if k == l else 0) - r[:, k] * r[:, l]).sum()
        return I, com

    # ---- inject object-frame params into an SDP solution vector (for torque prediction) ----
    def inject(self, fit, mass, com_O, I_cm_O):
        """Replace link-7 lumped (h, I) in the solution with those implied by (mass, com_O, I_cm_O).
        mass is left as fitted (it is the lumped link+payload mass either way)."""
        M_O = SpatialInertia.MakeFromCentralInertia(mass=mass, p_PScm_E=com_O, I_SScm_E=RotationalInertia(
            Ixx=I_cm_O[0, 0], Iyy=I_cm_O[1, 1], Izz=I_cm_O[2, 2],
            Ixy=I_cm_O[0, 1], Ixz=I_cm_O[0, 2], Iyz=I_cm_O[1, 2]))
        X = RigidTransform(self.X_LO)
        M_L = M_O.ReExpress(X.rotation()).Shift(-X.translation())       # about link-7 origin, link frame
        h_L = mass * np.asarray(M_L.get_com()) + self.init_last.m * self.init_last.get_com()
        I_L = np.asarray(M_L.CalcRotationalInertia().CopyToFullMatrix3()) + np.asarray(self.init_last.get_inertia_matrix())
        sol = fit["sol"].copy(); names = fit["names"]; L = NJ - 1
        for k, v in zip(("hx", "hy", "hz"), h_L):
            sol[names.index(f"{k}{L}(0)")] = v
        for key, (i, j) in {"Ixx": (0, 0), "Iyy": (1, 1), "Izz": (2, 2), "Ixy": (0, 1), "Ixz": (0, 2), "Iyz": (1, 2)}.items():
            sol[names.index(f"{key}{L}(0)")] = I_L[i, j]
        return sol

    # ---- the pipeline ----
    def identify(self, rows):
        """Fit on the given data rows; return {method: (params_dict, solution_vector)}."""
        fit = self.fit(rows)
        if fit is None:
            return None
        p_t = self.payload_params(fit)                                   # torque-only
        I_g, com_g = self.geometry_inertia(p_t["mass"])
        out = {
            "torque": (dict(mass=p_t["mass"], com=p_t["com"], I=p_t["I"]), fit["sol"]),
            "hybrid": (dict(mass=p_t["mass"], com=p_t["com"], I=I_g),
                       self.inject(fit, p_t["mass"], p_t["com"], I_g)),
            "hybrid_geomcom": (dict(mass=p_t["mass"], com=com_g, I=I_g),
                               self.inject(fit, p_t["mass"], com_g, I_g)),
        }
        return out, fit

    # ---- registration-ambiguity flag: near-degenerate principal moments => a symmetry axis the
    #      geometry-only registration cannot resolve; object-frame CoM sign is then uncertain ----
    def symmetry_flag(self, I, tol=0.10):
        ev = np.sort(np.linalg.eigvalsh(I))
        pairs = [(ev[0], ev[1]), (ev[1], ev[2])]
        return any(abs(a - b) / max(abs(b), 1e-12) < tol for a, b in pairs)

    # ---- plausibility: radius of gyration vs object size ----
    def gyration_ratio(self, mass, I):
        r_g = np.sqrt(np.max(np.linalg.eigvalsh(I)) / mass)
        return float(r_g / (0.5 * np.max(self.mesh.extents)))          # > ~1 is physically impossible


def to_json(p):
    return {"mass": float(p["mass"]), "center_of_mass": [float(x) for x in p["com"]],
            "inertia_matrix": [[float(x) for x in row] for row in np.asarray(p["I"])]}


def write_sdf(obj, p, out_path):
    """Copy the shipped SDF, replacing only the <inertial> block."""
    src = DATA / "object_data" / obj / f"{obj}_bundle_sdf.sdf"
    tree = ET.parse(str(src)); root = tree.getroot()
    inert = root.find(".//inertial")
    inert.find("mass").text = f"{p['mass']:.10g}"
    c = p["com"]; inert.find("pose").text = f"{c[0]:.5f} {c[1]:.5f} {c[2]:.5f} 0 0 0"
    I = np.asarray(p["I"]); ie = inert.find("inertia")
    for tag, (i, j) in {"ixx": (0, 0), "ixy": (0, 1), "ixz": (0, 2), "iyy": (1, 1), "iyz": (1, 2), "izz": (2, 2)}.items():
        ie.find(tag).text = f"{I[i, j]:.5e}"
    tree.write(str(out_path), pretty_print=True, xml_declaration=True, encoding="utf-8")


if __name__ == "__main__":
    out_dir = ROOT / "results" / "hybrid"; out_dir.mkdir(parents=True, exist_ok=True)
    for obj in sys.argv[1:] or ["spam"]:
        H = HybridIdentifier(obj)
        res, fit = H.identify(H.rows(np.ones(H.T, bool)))               # deployable asset: fit on all data
        p = res["hybrid"][0]
        (out_dir / f"{obj}_hybrid_inertial_params.json").write_text(json.dumps(to_json(p), indent=2))
        write_sdf(obj, p, out_dir / f"{obj}_hybrid.sdf")
        pt = res["torque"][0]
        print(f"{obj:<8} mass {p['mass']:.4f} kg | com {np.round(p['com'],4)} | mesh watertight={H.mesh_watertight}")
        print(f"         inertia diag  torque-only {np.round(np.diag(pt['I']),5)}  gyration ratio {H.gyration_ratio(pt['mass'], pt['I']):.2f}")
        print(f"                       hybrid      {np.round(np.diag(p['I']),5)}  gyration ratio {H.gyration_ratio(p['mass'], p['I']):.2f}")
        if H.symmetry_flag(p["I"]):
            print(f"         WARNING: near-degenerate principal moments -> object has a symmetry axis; the sign of the"
                  f" object-frame CoM offset ({np.round(p['com']-H.geometry_inertia(p['mass'])[1],4)} from centroid)"
                  f" is only as reliable as the geometric registration.")
        print(f"         -> {out_dir/(obj+'_hybrid_inertial_params.json')}\n         -> {out_dir/(obj+'_hybrid.sdf')}")
