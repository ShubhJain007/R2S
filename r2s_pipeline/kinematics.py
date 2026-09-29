"""Sensor-frame kinematics (a_O, alpha, w, g_S) from a pose trajectory, plus wrench-convention detection.

If the recording provides velocities/accelerations they are used; otherwise they are derived from the
pose trajectory by finite differences after zero-phase Butterworth low-pass filtering (the same
approach Scalable Real2Sim uses on joint data).
"""
import numpy as np
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation as Rot
from .regressor import G_WORLD


def _lowpass(x, fs, fc, order=4):
    if fc is None or fc <= 0 or fc >= fs / 2:
        return x
    b, a = butter(order, fc / (fs / 2))
    return filtfilt(b, a, x, axis=0)


def from_poses(t, T_ws, fc_hz=5.0):
    """t (N,), T_ws (N,4,4) world<-sensor.  Returns dict of sensor-frame a_O, alpha, w, g_S (N,3 each)."""
    t = np.asarray(t, float); N = len(t)
    fs = 1.0 / np.median(np.diff(t))
    R = T_ws[:, :3, :3]; p = T_ws[:, :3, 3]
    p_f = _lowpass(p, fs, fc_hz)
    v_w = np.gradient(p_f, t, axis=0)
    a_w = np.gradient(_lowpass(v_w, fs, fc_hz), t, axis=0)
    # angular velocity in WORLD from consecutive rotations, then filter
    rv = np.zeros((N, 3))
    for i in range(1, N):
        rv[i] = Rot.from_matrix(R[i] @ R[i - 1].T).as_rotvec() / (t[i] - t[i - 1])
    rv[0] = rv[1]
    w_w = _lowpass(rv, fs, fc_hz)
    al_w = np.gradient(_lowpass(w_w, fs, fc_hz), t, axis=0)
    # express in sensor frame
    a_O = np.einsum("nji,nj->ni", R, a_w)
    w_S = np.einsum("nji,nj->ni", R, w_w)
    al_S = np.einsum("nji,nj->ni", R, al_w)
    g_S = np.einsum("nji,j->ni", R, G_WORLD)
    return dict(a_O=a_O, alpha=al_S, w=w_S, g_S=g_S)


def from_given(T_ws, vel, acc, kin_frame="sensor"):
    """Use recorded (N,6) [v; w] velocity and [a; alpha] acceleration. kin_frame: 'sensor' or 'world'."""
    R = T_ws[:, :3, :3]
    a, al, w = acc[:, :3], acc[:, 3:], vel[:, 3:]
    if kin_frame == "world":
        a = np.einsum("nji,nj->ni", R, a); al = np.einsum("nji,nj->ni", R, al); w = np.einsum("nji,nj->ni", R, w)
    g_S = np.einsum("nji,j->ni", R, G_WORLD)
    return dict(a_O=a, alpha=al, w=w, g_S=g_S)


def detect_wrench_convention(wrench, T_ws, kin, mass_hint=None):
    """Decide whether the recorded wrench is in the sensor or world frame, and whether it is the contact
    wrench (sensor ON object) or the reaction (object ON sensor).  Uses the near-static samples: the
    contact force must equal -m*g_S.  Returns (frame, sign, report).  This exact ambiguity produced a
    45 % mass error before it was caught, so it is checked every time and printed."""
    R = T_ws[:, :3, :3]
    excitation = np.linalg.norm(kin["a_O"], axis=1) + np.linalg.norm(kin["w"], axis=1)
    idx = np.argsort(excitation)[: max(20, len(excitation) // 10)]
    best = None
    for frame in ("sensor", "world"):
        f = wrench[idx, :3] if frame == "sensor" else np.einsum("nji,nj->ni", R[idx], wrench[idx, :3])
        gs = kin["g_S"][idx]
        for sign in (+1, -1):
            fc = sign * f
            # contact force should be anti-parallel to g_S with |f| = m*g; score by direction agreement
            cos = np.sum(fc * (-gs), axis=1) / (np.linalg.norm(fc, axis=1) * np.linalg.norm(gs, axis=1) + 1e-12)
            score = np.mean(cos)
            if best is None or score > best[0]:
                best = (score, frame, sign, np.mean(np.linalg.norm(fc, axis=1)) / 9.81)
    score, frame, sign, m_static = best
    rep = (f"wrench convention: expressed in {frame.upper()} frame, "
           f"{'CONTACT (sensor on object)' if sign > 0 else 'REACTION (object on sensor) -> negated'}; "
           f"static-gravity agreement {score:.3f}; static mass estimate {m_static:.4f} kg")
    if score < 0.95:
        rep += "   ** WARNING: poor static-gravity agreement -- check frames / sensor bias / grasp rigidity **"
    return frame, sign, rep


def wrench_to_sensor_contact(wrench, T_ws, frame, sign):
    R = T_ws[:, :3, :3]
    f, n = wrench[:, :3], wrench[:, 3:]
    if frame == "world":
        f = np.einsum("nji,nj->ni", R, f); n = np.einsum("nji,nj->ni", R, n)
    return sign * np.hstack([f, n])
