#!/usr/bin/env python
"""Arm-agnostic payload identification from JOINT TORQUE (no wrist F/T).

Needs only: a URDF (for kinematics), joint positions, joint torques, gripper state.
  1. self-calibration: from OPEN-gripper samples across many episodes, fit
        y = a*tau_g_model(q) + b + Y(q) @ theta_tool        (y = sign * tau_measured)
     per-joint scale a and offset b absorb model error / sensor bias; theta_tool = [m, m*c]
     is the unknown tool.  Nothing arm-specific is hard-coded.
  2. payload: in held samples the residual obeys  r = Y(q) @ theta_obj  -> least squares.
Y is the gravity regressor of a point mass rigidly attached to the last link:
     tau_hold = -(Jv^T g m  -  Jw^T [g]x R h),   h = m * c  (c in the link frame)
"""
import os, sys, glob, numpy as np
from pydrake.all import MultibodyPlant, Parser, MultibodyForces, JacobianWrtVariable

GW = np.array([0, 0, -9.81])
def skew(v): return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])

class Arm:
    def __init__(self, url, ee):
        for weld in (False, True):
            p = MultibodyPlant(0.0); m = Parser(p).AddModels(url=url)[0]
            if weld: p.WeldFrames(p.world_frame(), p.get_body(p.GetBodyIndices(m)[0]).body_frame())
            p.Finalize()
            if p.num_positions() == p.num_velocities() and p.num_positions() <= 9: break
        self.p, self.ctx, self.ee, self.n = p, p.CreateDefaultContext(), p.GetFrameByName(ee), p.num_positions()
    def terms(self, q):
        p, c = self.p, self.ctx; p.SetPositions(c, q)
        f = MultibodyForces(p); p.CalcForceElementsContribution(c, f)
        tg = p.CalcInverseDynamics(c, np.zeros(self.n), f)
        J = p.CalcJacobianSpatialVelocity(c, JacobianWrtVariable.kV, self.ee, np.zeros(3), p.world_frame(), p.world_frame())
        Jw, Jv = J[:3], J[3:]
        R = self.ee.CalcPoseInWorld(c).rotation().matrix()
        Y = -np.hstack([(Jv.T @ GW)[:, None], -(Jw.T @ skew(GW) @ R)])      # (n,4): [m, h]
        return tg, Y

def load_ep(e):
    if not all(os.path.exists(os.path.join(e, "transformed", f)) for f in ("joint.npy", "gripper.npy")): return None
    T = lambda f: np.load(os.path.join(e, "transformed", f), allow_pickle=True).item()
    J, gr = T("joint.npy"), T("gripper.npy"); s = [k for k in J if k in gr]
    if not s: return None
    s = s[0]; tk = np.array(sorted(J[s])); A = np.array([J[s][k] for k in tk], float)
    if A.shape[1] < 14 or len(tk) < 30: return None
    gk = np.array(sorted(gr[s])); gw = np.array([gr[s][k]["gripper_info"][0] for k in gk], float)
    w = np.interp(tk, gk, gw); t = (tk - tk[0]) / 1000.0
    qd = np.abs(np.gradient(A[:, :7], t, axis=0)).max(1)
    return dict(q=A[:, :7], tau=A[:, 7:14], w=w, slow=qd < 0.08, o=gw.max(), c=gw.min())

if __name__ == "__main__":
    arm = Arm("package://drake_models/iiwa_description/urdf/iiwa14_no_collision.urdf", "iiwa_link_7")
    SIGN = -1.0                                   # KUKA convention, found from the data (corr -0.96)
    eps = sorted(glob.glob("rh20t_data/cfg7/RH20T_cfg7/task_*_cfg_0007"))
    eps = [e for e in eps if not e.endswith("_human")]
    rng = np.random.default_rng(0); rng.shuffle(eps)
    train, test = eps[:120], eps[120:260]
    # ---- 1. self-calibration on open-gripper samples
    rows = []
    for e in train:
        d = load_ep(e)
        if d is None: continue
        m = (d["w"] > d["o"] - 3) & d["slow"]
        for i in np.nonzero(m)[0][::3]:
            tg, Y = arm.terms(d["q"][i]); rows.append((tg, Y, SIGN * d["tau"][i]))
    n = arm.n; N = len(rows); print(f"calibration samples: {N} from {len(train)} episodes")
    A_ = np.zeros((N * n, 2 * n + 4)); y_ = np.zeros(N * n)
    for k, (tg, Y, y) in enumerate(rows):
        sl = slice(k * n, (k + 1) * n)
        A_[sl, :n] = np.diag(tg); A_[sl, n:2 * n] = np.eye(n); A_[sl, 2 * n:] = Y; y_[sl] = y
    lam = np.zeros(2 * n + 4); lam[:n] = 50.0                      # ridge a -> 1 (weakly excited joints)
    prior = np.zeros(2 * n + 4); prior[:n] = 1.0
    th = np.linalg.solve(A_.T @ A_ + np.diag(lam), A_.T @ y_ + lam * prior)
    a, b, tool = th[:n], th[n:2 * n], th[2 * n:]
    res = (y_ - A_ @ th).reshape(N, n)
    print("per-joint scale a :", np.round(a, 3))
    print("per-joint offset b:", np.round(b, 2), "Nm")
    print(f"TOOL: mass {tool[0]:.3f} kg   CoM {np.round(tool[1:] / tool[0] * 1000, 0)} mm in link-7 frame"
          f"      [ATI calibration: 1.28 kg distal to the sensor]")
    print("fit residual RMS per joint (Nm):", np.round(np.sqrt((res ** 2).mean(0)), 3))
    # ---- 2. payload on held-out episodes: free phase must give ~0 (this IS the noise floor)
    def payload(d, mask):
        idx = np.nonzero(mask)[0]
        if len(idx) < 15: return None
        Ys, rs = [], []
        for i in idx:
            tg, Y = arm.terms(d["q"][i]); Ys.append(Y); rs.append(SIGN * d["tau"][i] - (a * tg + b + Y @ tool))
        Ys, rs = np.vstack(Ys), np.concatenate(rs)
        return float(np.linalg.lstsq(Ys[:, :1], rs, rcond=None)[0][0])       # mass only
    free_m, held_m = [], []
    for e in test:
        d = load_ep(e)
        if d is None: continue
        f = payload(d, (d["w"] > d["o"] - 3) & d["slow"]); h = payload(d, (d["w"] > d["c"] + 5) & (d["w"] < d["o"] - 5) & d["slow"])
        if f is not None: free_m.append(f * 1000)
        if h is not None and f is not None: held_m.append((h - f) * 1000)
    free_m, held_m = np.array(free_m), np.array(held_m)
    print(f"\nheld-out FREE-phase 'payload' (truth = 0): n={len(free_m)}  median {np.median(free_m):.0f} g   std {free_m.std():.0f} g"
          f"   robust sigma {1.4826 * np.median(np.abs(free_m - np.median(free_m))):.0f} g")
    print(f"held-out HELD-minus-FREE payload          : n={len(held_m)}  median {np.median(held_m):.0f} g   IQR {np.percentile(held_m, 25):.0f}..{np.percentile(held_m, 75):.0f} g")
