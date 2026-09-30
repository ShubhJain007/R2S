"""Core identification: mass + CoM from torque/wrench, inertia from geometry.

Two recording modes
  wrench        wrist F/T sensor + sensor pose trajectory            (validated path)
  joint_torque  joint torques + joint positions + robot URDF/SDF     (needs Drake; arm dynamics from
                the nominal model are subtracted, with per-joint friction/offset terms fitted jointly
                to absorb model error -- the approach Scalable Real2Sim uses)

Output for the object: mass, CoM (sensor frame and object frame), inertia about CoM (object frame,
from geometry), diagnostics (conditioning, static-mass cross-check, held-out wrench/torque error,
gyration ratio, symmetry flag).
"""
import numpy as np
from . import regressor as R
from . import kinematics as K
from . import geometry as Gm

COND_OK = 100.0   # above this the full [m,h,I] fit is ill-conditioned and mass/CoM come from the [m,h]-only fit


# --------------------------------------------------------------------------- wrench mode
def build_wrench_problem(rec, fc_hz=5.0, wrench_frame="auto", wrench_sign=0):
    t, T_ws, wrench = rec["t"], rec["T_ws"], rec["wrench"]
    if "vel" in rec and "acc" in rec:
        kin = K.from_given(T_ws, rec["vel"], rec["acc"], rec.get("kin_frame", "sensor"))
    else:
        kin = K.from_poses(t, T_ws, fc_hz)
    if wrench_frame == "auto" or wrench_sign == 0:
        frame, sign, rep = K.detect_wrench_convention(wrench, T_ws, kin)
        if wrench_frame != "auto": frame = wrench_frame
        if wrench_sign != 0: sign = wrench_sign
    else:
        frame, sign = wrench_frame, wrench_sign; rep = f"wrench convention: forced {frame}/{sign:+d}"
    w = K.wrench_to_sensor_contact(wrench, T_ws, frame, sign).reshape(-1)
    Phi = R.stack(kin["a_O"], kin["alpha"], kin["w"], kin["g_S"])
    return Phi, w, kin, rep, 6


# --------------------------------------------------------------------------- joint-torque mode
def build_torque_problem(rec, robot_file, ee_frame, fc_hz=5.0, joint_friction=True, torque_sign=+1):
    """tau_meas - tau_arm(q,qd,qdd) = J^T W_contact = J^T M(R) Phi_S theta  (+ friction/offset terms)."""
    from pydrake.all import (MultibodyPlant, Parser, JacobianWrtVariable, MultibodyForces, RigidTransform)
    from scipy.signal import butter, filtfilt
    t, q, tau = rec["t"], rec["q"], rec["tau"]
    N, nj = q.shape
    fs = 1.0 / np.median(np.diff(t)); b, a = butter(4, fc_hz / (fs / 2))
    qf = filtfilt(b, a, q, axis=0); qd = np.gradient(qf, t, axis=0); qd = filtfilt(b, a, qd, axis=0)
    qdd = np.gradient(qd, t, axis=0); qdd = filtfilt(b, a, qdd, axis=0)
    tauf = filtfilt(b, a, tau, axis=0) * torque_sign

    plant = MultibodyPlant(0.0); Parser(plant).AddModels(robot_file); plant.Finalize()
    ctx = plant.CreateDefaultContext(); ee = plant.GetFrameByName(ee_frame); W = plant.world_frame()
    assert plant.num_positions() == nj, f"robot model has {plant.num_positions()} joints, recording has {nj}"
    T_ws = np.zeros((N, 4, 4)); rows = []; resid = np.zeros((N, nj))
    for i in range(N):
        plant.SetPositions(ctx, qf[i]); plant.SetVelocities(ctx, qd[i])
        f_el = MultibodyForces(plant); plant.CalcForceElementsContribution(ctx, f_el)      # gravity etc.
        tau_arm = plant.CalcInverseDynamics(ctx, qdd[i], f_el)
        resid[i] = tauf[i] - tau_arm
        X = ee.CalcPoseInWorld(ctx); T_ws[i, :3, :3] = X.rotation().matrix(); T_ws[i, :3, 3] = X.translation(); T_ws[i, 3, 3] = 1
        J = plant.CalcJacobianSpatialVelocity(ctx, JacobianWrtVariable.kV, ee, np.zeros(3), W, W)   # 6 x nj, [w; v] world
        Rm = T_ws[i, :3, :3]
        M = np.zeros((6, 6)); M[:3, 3:] = Rm; M[3:, :3] = Rm                # [f_S; n_S] -> [n_w; f_w]
        rows.append(J.T @ M)
    kin = K.from_poses(t, T_ws, fc_hz)
    Phi_S = R.stack(kin["a_O"], kin["alpha"], kin["w"], kin["g_S"])          # (6N,10)
    Phi = np.vstack([rows[i] @ Phi_S[6 * i:6 * i + 6] for i in range(N)])     # (nj*N,10)
    if joint_friction:  # per-joint viscous, Coulomb, constant offset -> absorbs nominal-model error
        F = np.zeros((nj * N, 3 * nj))
        for i in range(N):
            for j in range(nj):
                F[i * nj + j, 3 * j:3 * j + 3] = [qd[i, j], np.sign(qd[i, j]), 1.0]
        Phi = np.hstack([Phi, F])
    return Phi, resid.reshape(-1), kin, "joint-torque mode: arm dynamics from nominal model" + (" + per-joint friction/offset terms" if joint_friction else ""), nj, T_ws


# --------------------------------------------------------------------------- solve + diagnostics
def solve(Phi, w, rows_per_sample, holdout_frac=0.3, use_sdp=False):
    N = len(w) // rows_per_sample; ntr = int((1 - holdout_frac) * N)
    tr = np.arange(rows_per_sample * ntr); te = np.arange(rows_per_sample * ntr, rows_per_sample * N)
    fit = (R.fit_sdp if use_sdp else R.fit_ls)
    th = fit(Phi[tr], w[tr])
    if th is None: th = R.fit_ls(Phi[tr], w[tr])
    cond_full = np.linalg.cond(Phi[tr][:, :10]); cond_mh = np.linalg.cond(Phi[tr][:, :4])
    th_mh = R.fit_ls(Phi[tr][:, :4], w[tr]) if cond_full > COND_OK else None
    m, c, I_torque = R.unpack(th[:10])
    if th_mh is not None:
        m, c = th_mh[0], th_mh[1:4] / th_mh[0]
    pred = Phi[te] @ th; res = (pred - w[te]).reshape(-1, rows_per_sample); tru = w[te].reshape(-1, rows_per_sample)
    # relative to the signal RMS (not std: under gentle motion the wrench is nearly constant and std -> 0)
    rel = np.sqrt((res ** 2).mean(0)) / (np.sqrt((tru ** 2).mean(0)) + 1e-12)
    abs_rmse = np.sqrt((res ** 2).mean(0))
    th_full = R.fit_ls(Phi, w)                                  # deployable estimate on ALL data
    m_all, c_all, _ = R.unpack(th_full[:10])
    if cond_full > COND_OK:
        t4 = R.fit_ls(Phi[:, :4], w); m_all, c_all = t4[0], t4[1:4] / t4[0]
    return dict(mass=float(m_all), com_S=c_all, mass_train=float(m), com_S_train=c,
                I_torque_cm_S=I_torque, cond_full=float(cond_full), cond_mass_com=float(cond_mh),
                used_reduced_fit=th_mh is not None, holdout_rel_rmse=float(rel.mean()),
                holdout_rel_per_row=rel.tolist(), holdout_abs_rmse_per_row=abs_rmse.tolist(), n_samples=int(N), n_train=int(ntr))


def _build(rec, mode, robot_file, ee_frame, fc_hz, wrench_frame, wrench_sign, torque_sign):
    if mode == "wrench":
        Phi, w, kin, rep, rps = build_wrench_problem(rec, fc_hz, wrench_frame, wrench_sign); T_ws = rec["T_ws"]
    else:
        Phi, w, kin, rep, rps, T_ws = build_torque_problem(rec, robot_file, ee_frame, fc_hz, True, torque_sign)
    return Phi, w, kin, rep, rps, T_ws


def identify(rec, mesh_path=None, T_so=None, mode="wrench", robot_file=None, ee_frame=None,
             mesh_scale=1.0, fc_hz=5.0, use_sdp=False, wrench_frame="auto", wrench_sign=0,
             torque_sign=+1, density_fn=None, baseline=None, verbose=True):
    """Main entry.  rec: dict of arrays (see recording/RECORDING_SPEC.md).  T_so: (4,4) object-mesh frame in the
    sensor/EE frame (if None, results are reported in the sensor frame only).
    baseline: a recording of the SAME setup with NO object (empty gripper / bare sensor). Its inertial
    parameters are identified the same way and subtracted -- this removes the gripper/fingers/sensor
    plate, which otherwise land in the object's mass (2.5 kg on an iiwa+WSG-50)."""
    log = []
    Phi, w, kin, rep, rps, T_ws = _build(rec, mode, robot_file, ee_frame, fc_hz, wrench_frame, wrench_sign, torque_sign)
    log.append(rep)
    s = solve(Phi, w, rps, use_sdp=use_sdp)
    if baseline is not None:
        Phi_b, w_b, kin_b, rep_b, rps_b, _ = _build(baseline, mode, robot_file, ee_frame, fc_hz, wrench_frame, wrench_sign, torque_sign)
        sb = solve(Phi_b, w_b, rps_b, use_sdp=use_sdp)
        th_loaded = R.pack(s["mass"], s["com_S"], s["I_torque_cm_S"]); th_base = R.pack(sb["mass"], sb["com_S"], sb["I_torque_cm_S"])
        m_o, c_o, I_o = R.unpack(th_loaded - th_base)
        log.append(f"baseline (no object): mass {sb['mass']:.4f} kg at {np.round(sb['com_S'],4)}  ->  subtracted")
        log.append(f"loaded {s['mass']:.4f} kg - baseline {sb['mass']:.4f} kg = object {m_o:.4f} kg")
        s = dict(s, mass=float(m_o), com_S=c_o, I_torque_cm_S=I_o, mass_train=float(s["mass_train"] - sb["mass_train"]))
    ex = np.linalg.norm(kin["a_O"], axis=1)
    log.append(f"samples {s['n_samples']} | excitation |a| max {ex.max():.2f} m/s^2, median {np.median(ex):.2f} | "
               f"cond(full)={s['cond_full']:.1f} cond(mass,CoM)={s['cond_mass_com']:.1f}"
               + ("  -> full fit ill-conditioned, mass/CoM from [m,h]-only fit" if s["used_reduced_fit"] else ""))
    ab = s['holdout_abs_rmse_per_row']
    log.append(f"held-out ({100*(1-s['n_train']/s['n_samples']):.0f}%) RMSE / signal RMS = {s['holdout_rel_rmse']:.3f}   "
               f"abs RMSE {np.round(ab[:3],3)} N  {np.round(ab[3:6],4)} Nm" if rps == 6 else
               f"held-out ({100*(1-s['n_train']/s['n_samples']):.0f}%) RMSE / signal RMS = {s['holdout_rel_rmse']:.3f}   abs RMSE per joint {np.round(ab,3)} Nm")
    out = dict(mass=s["mass"], com_sensor_frame=s["com_S"].tolist(), diagnostics=dict(
        cond_full=s["cond_full"], cond_mass_com=s["cond_mass_com"], holdout_rel_rmse=s["holdout_rel_rmse"],
        holdout_rel_per_row=s["holdout_rel_per_row"], n_samples=s["n_samples"], excitation_max=float(ex.max()),
        used_reduced_fit=s["used_reduced_fit"], wrench_convention=rep,
        mass_train_split=s["mass_train"], mass_train_vs_all_pct=float(abs(s["mass_train"] - s["mass"]) / s["mass"] * 100)))
    if out["diagnostics"]["mass_train_vs_all_pct"] > 5:
        log.append(f"** WARNING: mass changes {out['diagnostics']['mass_train_vs_all_pct']:.1f}% between train split and all data -- "
                   "recording may be too short or non-stationary (sensor drift, grasp slip) **")
    # ---- inertia from geometry ----
    if mesh_path is not None:
        mesh = Gm.load_mesh(mesh_path, mesh_scale)
        I_geom, cen, ginfo = Gm.inertia_from_geometry(mesh, s["mass"], density_fn=density_fn)
        out["inertia_object_frame_about_com"] = I_geom.tolist()
        out["geometry_centroid_object_frame"] = cen.tolist()
        gyr = Gm.gyration_ratio(mesh, s["mass"], I_geom); sym = Gm.symmetry_flag(I_geom)
        out["diagnostics"].update(gyration_ratio=gyr, symmetry_flag=sym, mesh=ginfo)
        log.append(f"geometry: {ginfo['n_voxels']} voxels @ {ginfo['pitch']*1000:.1f} mm, watertight={ginfo['watertight']}, "
                   f"volume {ginfo['volume_m3']*1e6:.0f} cm^3 -> mean density {s['mass']/ginfo['volume_m3']:.0f} kg/m^3 | gyration {gyr:.2f}")
        if sym: log.append("symmetry flag: near-degenerate principal moments -> object-frame CoM sign depends on the pose registration")
        if T_so is not None:
            T_so = np.asarray(T_so); R_so, p_so = T_so[:3, :3], T_so[:3, 3]
            com_O = R_so.T @ (s["com_S"] - p_so)
            out["com_object_frame"] = com_O.tolist()
            out["com_offset_from_centroid_mm"] = float(np.linalg.norm(com_O - cen) * 1000)
            log.append(f"CoM (object frame) {np.round(com_O, 4)} | {out['com_offset_from_centroid_mm']:.1f} mm from geometric centroid")
        else:
            out["com_object_frame"] = None
            log.append("no T_so given: CoM reported in sensor frame; inertia uses the geometric centroid")
        # torque-derived inertia only as a diagnostic, never as output
        I_t = s["I_torque_cm_S"]
        out["diagnostics"]["torque_inertia_gyration_ratio"] = Gm.gyration_ratio(mesh, s["mass"], I_t) if np.all(np.isfinite(I_t)) else None
    if verbose:
        for l in log: print("  " + l)
    out["log"] = log
    return out
