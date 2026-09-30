#!/usr/bin/env python
"""Scalable Real2Sim (arXiv 2503.00370) test suite T2-T5 on the released benchmark data.

Replicates the exact recipe of robot_payload_id/scripts/identify_grasped_object_payload.py
(plant setup, robot-parameter loading, data processing, data-matrix construction, SDP), but
builds the data matrix ONCE and re-fits the SDP on row subsets.

The benchmark objects ship NO ground-truth physical parameters (the paper's accuracy numbers
come from a separate calibration rig), so every test here is scored ground-truth-free by
HELD-OUT TORQUE PREDICTION: fit on the first (T - HOLDOUT_S) seconds, predict joint torques
on the final HOLDOUT_S seconds. Lower is better. The data-matrix condition number is reported
as an identifiability measure.

Model:  tau - w0 = W @ theta   (rows are time-major, 7 joint rows per timestep)
"""
import copy, json, logging, sys, time
from pathlib import Path
import numpy as np
import trimesh

from pydrake.all import (FixedOffsetFrame, MultibodyPlant, RigidTransform, RotationMatrix,
                         RotationalInertia, SpatialInertia, UnitInertia)
from robot_payload_id.data import extract_numeric_data_matrix_autodiff
from robot_payload_id.optimization import solve_inertial_param_sdp
from robot_payload_id.utils import (ArmComponents, JointData, get_parser, get_plant_joint_params,
                                    process_joint_data, write_parameters_to_plant)
from pydrake.all import AffineBall
def compute_min_ellipsoid(mesh_file, transform=None):
    """Same as robot_payload_id.utils.utils.compute_min_ellipsoid, but on the CONVEX HULL
    vertices. The MVEE depends only on the hull, so this is mathematically identical; passing
    all ~14k mesh vertices makes Drake 1.51's Clarabel fail with InsufficientProgress."""
    mesh = trimesh.load(mesh_file, process=False)
    if transform is not None:
        mesh = mesh.apply_transform(transform)
    return AffineBall.MinimumVolumeCircumscribedEllipsoid(np.asarray(mesh.convex_hull.vertices).T)
import importlib.util as _ilu
_spec = _ilu.spec_from_file_location("idpay", str(Path(__file__).resolve().parents[2] / "scalable-real2sim/scalable_real2sim/robot_payload_id/scripts/identify_grasped_object_payload.py"))
_idpay = _ilu.module_from_spec(_spec); _spec.loader.exec_module(_idpay)
get_object_pose_in_link_frame = _idpay.get_object_pose_in_link_frame

logging.basicConfig(level=logging.WARNING)
ROOT = Path(__file__).resolve().parents[2]   # repo root
DATA = ROOT / "s2s-data/scalable_real2sim_benchmark_dataset"
ROBOT = DATA / "robot_system_id_data"
NJ = 7
HOLDOUT_S = 2.0


class Problem:
    """Everything needed to fit/predict for one object, built once."""

    def __init__(self, obj: str):
        self.obj = obj
        odir = DATA / "object_data" / obj
        self.jd_path = odir / "system_id_data"
        self.mesh_path = odir / "bundle_sdf_mesh" / "textured_mesh.obj"
        self.shipped = json.loads((odir / "bundle_sdf_inertial_params.json").read_text())

        # --- object frame in link-7 frame (identical to the script) ---
        cloud = np.load(self.jd_path / "manipuland_cloud_link7_frame.npy")
        # The upstream registration is RANSAC+FPFH with no seed and is NON-deterministic (20 mm /
        # 180-degree spread on symmetric objects). Seed it so assets are reproducible.
        import open3d as _o3d; _o3d.utility.random.seed(0); np.random.seed(0)
        self.X_LO = get_object_pose_in_link_frame(self.mesh_path, cloud, visualize=False)

        # --- arm plant with payload frame (identical) ---
        model_path = str(ROOT / "scalable-real2sim/scalable_real2sim/robot_payload_id/models/iiwa.dmd.yaml")
        plant = MultibodyPlant(0.0)
        get_parser(plant).AddModels(model_path)
        self.last_link = plant.GetBodyByName("iiwa_link_7")
        plant.AddFrame(FixedOffsetFrame(name="payload_frame", P=self.last_link.body_frame(),
                                        X_PF=RigidTransform(self.X_LO)))
        plant.Finalize()
        arm = ArmComponents(num_joints=NJ, diagram=None, plant=plant, trajectory_source=None,
                            state_logger=None, commanded_torque_logger=None, meshcat=None,
                            meshcat_visualizer=None)

        # --- robot params at closest gripper opening (identical) ---
        avail = [float(f.name.split("_")[-1]) for f in ROBOT.iterdir()
                 if f.is_dir() and f.name.startswith("gripper_position_")]
        g = float(np.mean(np.load(self.jd_path / "wsg_positions.npy")))
        self.gripper = min(avail, key=lambda x: abs(x - g))
        robot_sol = np.load(ROBOT / f"gripper_position_{self.gripper:.2f}" / "identified_robot_params.npy",
                            allow_pickle=True).item()
        self.arm = write_parameters_to_plant(arm, robot_sol)

        # --- joint data processing (identical defaults: vel/acc/torque filters) ---
        raw = JointData.load_from_disk_allow_missing(self.jd_path)
        raw = JointData.cut_off_at_beginning(raw, 0.0)
        self.jd = process_joint_data(joint_data=raw, num_endpoints_to_remove=0, compute_velocities=True,
                                     filter_positions=False, pos_filter_order=0, pos_cutoff_freq_hz=0,
                                     vel_filter_order=10, vel_cutoff_freq_hz=5.6,
                                     acc_filter_order=10, acc_cutoff_freq_hz=4.2,
                                     torque_filter_order=10, torque_cutoff_freq_hz=4.0)
        self.t = np.asarray(self.jd.sample_times_s).ravel()
        self.T = len(self.t)

        # --- data matrix (identical), built once ---
        t0 = time.time()
        self.W, self.w0, _ = extract_numeric_data_matrix_autodiff(
            plant_components=self.arm, joint_data=self.jd, add_rotor_inertia=False,
            add_reflected_inertia=True, add_viscous_friction=True, add_dynamic_dry_friction=True,
            payload_only=True)
        self.tau = self.jd.joint_torques.flatten() - self.w0
        self.build_s = time.time() - t0
        self.init_last = get_plant_joint_params(self.arm.plant, self.arm.plant_context, add_rotor_inertia=False,
                                                add_reflected_inertia=True, add_viscous_friction=True,
                                                add_dynamic_dry_friction=True, payload_only=True)[-1]
        self.ellipsoid = None  # built lazily

        # excitation per timestep = ||qdd||
        self.exc = np.linalg.norm(np.asarray(self.jd.joint_accelerations), axis=1)

    # ---------- row selection helpers ----------
    def rows(self, tmask):
        """time-mask (T,) -> row indices into W/tau (7 rows per timestep)."""
        idx = np.nonzero(tmask)[0]
        return (idx[:, None] * NJ + np.arange(NJ)[None, :]).ravel()

    def train_mask(self):
        return self.t < self.t[-1] - HOLDOUT_S

    def test_mask(self):
        return ~self.train_mask()

    # ---------- fit / predict ----------
    def fit(self, rows, use_ellipsoid=False):
        if use_ellipsoid and self.ellipsoid is None:
            self.ellipsoid = compute_min_ellipsoid(self.mesh_path, transform=self.X_LO)
        _, result, names, var_vec, _ = solve_inertial_param_sdp(
            num_links=NJ, W_data=self.W[rows], tau_data=self.tau[rows], base_param_mapping=None,
            regularization_weight=0.0, params_guess=None, use_euclidean_regularization=False,
            identify_rotor_inertia=False, identify_reflected_inertia=True, identify_viscous_friction=True,
            identify_dynamic_dry_friction=True, payload_only=True, initial_last_link_params=self.init_last,
            payload_bounding_ellipsoid=self.ellipsoid if use_ellipsoid else None)
        if not result.is_success():
            return None
        sol = result.GetSolution(var_vec)
        return dict(names=list(names), sol=np.asarray(sol, dtype=float),
                    cost=result.get_optimal_cost(), cond=np.linalg.cond(self.W[rows]),
                    n_rows=len(rows))

    def predict_err(self, sol_vec, rows):
        """Relative RMS torque prediction error on the given rows (per-joint normalised)."""
        pred = self.W[rows] @ sol_vec
        res = (pred - self.tau[rows]).reshape(-1, NJ)
        tru = self.tau[rows].reshape(-1, NJ)
        rel = np.sqrt((res ** 2).mean(0)) / (tru.std(0) + 1e-9)
        return float(rel.mean()), rel

    # ---------- physical params in object frame (identical extraction) ----------
    def payload_params(self, fit):
        d = dict(zip(fit["names"], fit["sol"]))
        L = NJ - 1
        m = d[f"m{L}(0)"] - self.init_last.m
        h = np.array([d[f"hx{L}(0)"], d[f"hy{L}(0)"], d[f"hz{L}(0)"]]) - self.init_last.m * self.init_last.get_com()
        com = h / m
        I = np.array([[d[f"Ixx{L}(0)"], d[f"Ixy{L}(0)"], d[f"Ixz{L}(0)"]],
                      [d[f"Ixy{L}(0)"], d[f"Iyy{L}(0)"], d[f"Iyz{L}(0)"]],
                      [d[f"Ixz{L}(0)"], d[f"Iyz{L}(0)"], d[f"Izz{L}(0)"]]]) - self.init_last.get_inertia_matrix()
        ctx = copy.deepcopy(self.arm.plant_context)
        self.last_link.SetSpatialInertiaInBodyFrame(ctx, SpatialInertia(
            mass=m, p_PScm_E=com, G_SP_E=UnitInertia(Ixx=I[0, 0] / m, Iyy=I[1, 1] / m, Izz=I[2, 2] / m,
                                                      Ixy=I[0, 1] / m, Ixz=I[0, 2] / m, Iyz=I[1, 2] / m)))
        M_P = self.arm.plant.CalcSpatialInertia(context=ctx, frame_F=self.arm.plant.GetFrameByName("payload_frame"),
                                                body_indexes=[self.last_link.index()])
        M_Pcm = M_P.Shift(M_P.get_com())
        return dict(mass=float(M_Pcm.get_mass()), com=np.asarray(M_P.get_com()),
                    I=np.asarray(M_Pcm.CalcRotationalInertia().CopyToFullMatrix3()))

    # ---------- T3: geometry-derived inertia injected into a solution vector ----------
    def geometry_inertia_solution(self, fit):
        """Keep the fitted mass/CoM/friction terms; replace the last link's rotational inertia
        with the uniform-density mesh inertia (scaled to the fitted mass, placed at fitted CoM)."""
        p = self.payload_params(fit)
        mesh = trimesh.load(self.mesh_path, process=False)
        if not mesh.is_watertight:
            trimesh.repair.fill_holes(mesh)
        mesh.density = p["mass"] / mesh.volume
        I_geom_cm_O = np.asarray(mesh.moment_inertia)            # about mesh CoM, object frame
        # payload spatial inertia in payload(object) frame, about its CoM, then to link-7 frame/origin
        m = p["mass"]
        # I_geom is about the mesh CoM -> use the central-inertia constructor
        M_O = SpatialInertia.MakeFromCentralInertia(mass=m, p_PScm_E=p["com"],
                                                    I_SScm_E=RotationalInertia(
            Ixx=I_geom_cm_O[0, 0], Iyy=I_geom_cm_O[1, 1], Izz=I_geom_cm_O[2, 2],
            Ixy=I_geom_cm_O[0, 1], Ixz=I_geom_cm_O[0, 2], Iyz=I_geom_cm_O[1, 2]))
        X = RigidTransform(self.X_LO)
        M_L = M_O.ReExpress(X.rotation()).Shift(-X.translation())   # about link-7 origin, link frame
        # lumped params of (payload + link7) about the link-7 origin, in the link frame:
        # rotational inertias about the same point simply add.
        I_pay_L = np.asarray(M_L.CalcRotationalInertia().CopyToFullMatrix3())
        I_L = I_pay_L + np.asarray(self.init_last.get_inertia_matrix())
        sol = fit["sol"].copy(); names = fit["names"]; L = NJ - 1
        for key, (i, j) in {"Ixx": (0, 0), "Iyy": (1, 1), "Izz": (2, 2), "Ixy": (0, 1), "Ixz": (0, 2), "Iyz": (1, 2)}.items():
            sol[names.index(f"{key}{L}(0)")] = I_L[i, j]
        return sol, I_geom_cm_O


def fmt_I(I):
    return f"diag=({I[0,0]:.5f},{I[1,1]:.5f},{I[2,2]:.5f}) off=({I[0,1]:+.5f},{I[0,2]:+.5f},{I[1,2]:+.5f})"


def run(obj):
    out = {}
    print(f"\n{'='*78}\n OBJECT: {obj}\n{'='*78}")
    P = Problem(obj)
    tr, te = P.rows(P.train_mask()), P.rows(P.test_mask())
    print(f"data: {P.T} samples over {P.t[-1]-P.t[0]:.2f}s | train {P.train_mask().sum()} / holdout {P.test_mask().sum()} samples "
          f"| gripper {P.gripper:.2f} | W built in {P.build_s:.0f}s | cond(W_full)={np.linalg.cond(P.W):.2f}")

    # ---------------- T1 check: full-data fit reproduces shipped ----------------
    full = P.fit(P.rows(np.ones(P.T, bool)))
    pf = P.payload_params(full)
    sm, sI = P.shipped["mass"], np.array(P.shipped["inertia_matrix"])
    print(f"\n[T1] full-data fit vs shipped:  mass {pf['mass']:.6f} vs {sm:.6f} ({abs(pf['mass']-sm)/sm*100:.2f}%)  "
          f"| I Frobenius diff {np.linalg.norm(pf['I']-sI)/np.linalg.norm(sI)*100:.1f}%")
    out["T1"] = dict(mass=pf["mass"], shipped_mass=sm)

    # ---------------- T5: held-out prediction, default config ----------------
    base = P.fit(tr)
    e_base, per_j = P.predict_err(base["sol"], te)
    pb = P.payload_params(base)
    print(f"\n[T5] fit first {P.t[-1]-HOLDOUT_S:.0f}s -> predict last {HOLDOUT_S:.0f}s | rel torque RMSE = {e_base:.4f}  "
          f"(per joint: {' '.join(f'{x:.3f}' for x in per_j)})")
    print(f"     params from train split: mass {pb['mass']:.5f}  com {np.round(pb['com'],4)}  {fmt_I(pb['I'])}")
    out["T5"] = dict(rel_rmse=e_base, mass=pb["mass"])

    # ---------------- T4: bounding ellipsoid on/off ----------------
    ell = P.fit(tr, use_ellipsoid=True)
    if ell is None:
        print("\n[T4] ellipsoid fit FAILED (solver)")
    else:
        e_ell, _ = P.predict_err(ell["sol"], te); pe = P.payload_params(ell)
        print(f"\n[T4] bounding ellipsoid constraint:")
        print(f"     {'':<10}{'mass':>10}{'held-out rel RMSE':>20}   inertia (object frame, about CoM)")
        print(f"     {'OFF':<10}{pb['mass']:>10.5f}{e_base:>20.4f}   {fmt_I(pb['I'])}")
        print(f"     {'ON':<10}{pe['mass']:>10.5f}{e_ell:>20.4f}   {fmt_I(pe['I'])}")
        print(f"     inertia change ON vs OFF: Frobenius {np.linalg.norm(pe['I']-pb['I'])/np.linalg.norm(pb['I'])*100:.1f}%")
        out["T4"] = dict(off=e_base, on=e_ell, mass_on=pe["mass"])

    # ---------------- T3: torque-identified vs geometry-derived inertia ----------------
    sol_geom, I_geom = P.geometry_inertia_solution(base)
    e_geom, _ = P.predict_err(sol_geom, te)
    print(f"\n[T3] inertia source -> held-out torque prediction (mass/CoM/friction identical, only inertia differs):")
    print(f"     torque-identified (SDP):     rel RMSE {e_base:.4f}   {fmt_I(pb['I'])}")
    print(f"     geometry (uniform density):  rel RMSE {e_geom:.4f}   {fmt_I(I_geom)}")
    print(f"     Frobenius diff between the two inertias: {np.linalg.norm(I_geom-pb['I'])/np.linalg.norm(pb['I'])*100:.1f}%")
    out["T3"] = dict(sdp=e_base, geom=e_geom)

    # ---------------- T2: excitation degradation ----------------
    print(f"\n[T2] excitation degradation (fit on subsets of the train window; always predict the same held-out 2s)")
    print(f"     {'subset':<34}{'n':>6}{'cond(W)':>10}{'mass':>9}{'|dm|%':>7}{'holdout RMSE':>14}{'I Frob dev%':>12}")
    trm = P.train_mask(); ref = pb
    def report(label, tmask):
        r = P.rows(tmask & trm)
        f = P.fit(r)
        if f is None:
            print(f"     {label:<34}{'--- solver failed ---':>40}"); return None
        e, _ = P.predict_err(f["sol"], te); p = P.payload_params(f)
        dm = abs(p["mass"] - ref["mass"]) / ref["mass"] * 100
        dI = np.linalg.norm(p["I"] - ref["I"]) / np.linalg.norm(ref["I"]) * 100
        print(f"     {label:<34}{(tmask & trm).sum():>6}{f['cond']:>10.1f}{p['mass']:>9.4f}{dm:>7.1f}{e:>14.4f}{dI:>12.1f}")
        return dict(mass=p["mass"], dm=dm, rmse=e, cond=f["cond"], dI=dI)
    rows = {}
    rows["full train window"] = report("full train window", np.ones(P.T, bool))
    for s in (0.5, 1, 2, 4):
        rows[f"first {s}s"] = report(f"first {s}s only", P.t < P.t[0] + s)
    exc_tr = P.exc[trm]
    for q in (50, 25, 10, 5):
        thr = np.percentile(exc_tr, q)
        rows[f"gentlest {q}%"] = report(f"gentlest {q}% of samples (|qdd|<{thr:.0f})", P.exc <= thr)
    for q in (50, 25):
        thr = np.percentile(exc_tr, 100 - q)
        rows[f"most excited {q}%"] = report(f"most excited {q}% (|qdd|>{thr:.0f})", P.exc >= thr)
    out["T2"] = rows
    return out


if __name__ == "__main__":
    objs = sys.argv[1:] or ["spam"]
    results = {o: run(o) for o in objs}
    (ROOT / "results" / "s2s_tests.json").write_text(json.dumps(results, indent=1, default=float))
    print(f"\nsaved -> {ROOT/'results'/'s2s_tests.json'}")
