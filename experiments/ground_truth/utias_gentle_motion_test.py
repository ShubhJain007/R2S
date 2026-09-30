#!/usr/bin/env python
"""Ground-truth test of inertial identification from GENTLE (production-like) motion.

Data: utiasSTARS workshop-tools synthetic manipulation (20 objects x 1000 samples): wrist F/T sensor
pose / velocity / acceleration and the measured 6-D wrench; CAD ground-truth mass, CoM, inertia.
Accelerations <= ~1 m/s^2 linear, ~2.4 rad/s^2 angular.

Model (Newton-Euler, sensor frame S with origin O; h = m*c, I_O = inertia about O):
    f   = m (a_O - g) + alpha x h + w x (w x h)
    n_O = h x (a_O - g) + I_O alpha + w x (I_O w)
    =>  wrench = Phi(a_O, alpha, w, g) . theta,   theta = [m, h(3), I_O(6)]   (linear)

Protocol per object: determine frame/ordering conventions empirically (noise-free residual), noise
check, fit on first 70% (LS and physically-consistent SDP), held-out wrench RMSE on last 30%, error vs
ground truth, cond(Phi), and an excitation sweep on the gentlest subsets. Then repeat with injected
F/T noise (0.3 N / 0.005 Nm std, representative of a Robotiq FT-300 class sensor).
"""
import glob, json, os, pickle, sys, itertools
import numpy as np, yaml
from pydrake.all import MathematicalProgram, Solve, RigidTransform, RotationMatrix

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # repo root
D = f"{ROOT}/utias-inertial/data"
G = np.array([0, 0, -9.81])
NOISE = (0.3, 0.005)          # N, Nm  (std)
rng = np.random.default_rng(0)


class Dummy:
    def __setstate__(self, s): self.__dict__.update(s if isinstance(s, dict) else {"_s": s})
class U(pickle.Unpickler):
    def find_class(self, mod, name):
        try: return super().find_class(mod, name)
        except Exception: return Dummy


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

def Lmat(v):
    """L(v) such that I v = L(v) [Ixx Ixy Ixz Iyy Iyz Izz]^T."""
    x, y, z = v
    return np.array([[x, y, z, 0, 0, 0], [0, x, 0, y, z, 0], [0, 0, x, 0, y, z]])

def regressor(a_O, alpha, w, g_S):
    """6x10 block for one sample; all vectors in sensor frame."""
    ag = a_O - g_S
    Phi = np.zeros((6, 10))
    Phi[:3, 0] = ag
    Phi[:3, 1:4] = skew(alpha) + skew(w) @ skew(w)
    Phi[3:, 1:4] = -skew(ag)                            # h x ag = -ag x h
    Phi[3:, 4:] = Lmat(alpha) + skew(w) @ Lmat(w)
    return Phi

def theta_gt_in_S(m, c_S, I_cm_S):
    h = m * c_S
    I_O = I_cm_S + m * (c_S @ c_S * np.eye(3) - np.outer(c_S, c_S))
    return np.concatenate([[m], h, [I_O[0, 0], I_O[0, 1], I_O[0, 2], I_O[1, 1], I_O[1, 2], I_O[2, 2]]])

def unpack(theta):
    m = theta[0]; c = theta[1:4] / m
    I_O = np.array([[theta[4], theta[5], theta[6]], [theta[5], theta[7], theta[8]], [theta[6], theta[8], theta[9]]])
    I_cm = I_O - m * (c @ c * np.eye(3) - np.outer(c, c))
    return m, c, I_cm

def fit_ls(Phi, w):
    return np.linalg.lstsq(Phi, w, rcond=None)[0]

def fit_sdp(Phi, w):
    """LS with the pseudo-inertia J(theta) >= 0 physical-consistency constraint."""
    prog = MathematicalProgram(); th = prog.NewContinuousVariables(10, "th")
    m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz = th
    tr = (Ixx + Iyy + Izz) / 2
    Sig = np.array([[tr - Ixx, -Ixy, -Ixz], [-Ixy, tr - Iyy, -Iyz], [-Ixz, -Iyz, tr - Izz]], dtype=object)
    J = np.empty((4, 4), dtype=object); J[:3, :3] = Sig; J[:3, 3] = [hx, hy, hz]; J[3, :3] = [hx, hy, hz]; J[3, 3] = m
    prog.AddPositiveSemidefiniteConstraint(J)
    prog.Add2NormSquaredCost(Phi, w, th)
    r = Solve(prog)
    return r.GetSolution(th) if r.is_success() else None


def load(obj):
    d = U(open(f"{D}/Simulations/{obj}.pkl", "rb")).load()
    A = lambda v: np.asarray(v.A if hasattr(v, "A") else v).reshape(-1)
    T = np.array([np.asarray(s.ft_pose.A) for s in d])          # world <- sensor
    V = np.array([A(s.ft_vel) for s in d]); Acc = np.array([A(s.ft_acc) for s in d]); W = np.array([A(s.ft_value) for s in d])
    Tobj = np.array([np.asarray(s.object_pose.A) for s in d])
    y = yaml.safe_load(open(f"{D}/Workshop Tools Dataset/{obj.replace('_',' ')}/Inertia.yaml"))["OBJECT"]
    return T, V, Acc, W, Tobj, y


def build(T, V, Acc, W, order="vw", frame="body"):
    """Stack Phi and the CONTACT wrench in the sensor frame.
    Conventions determined empirically on the GT parameters (residual 1.0 -> 0.08):
      kinematics: [v; w] ordering, expressed in the sensor (body) frame
      wrench:     expressed in the WORLD frame, as the reaction (object-on-sensor) -> rotate into the
                  sensor frame with R^T and negate to get the contact wrench the model predicts."""
    n = len(T); Phi = np.zeros((6 * n, 10)); w = np.zeros(6 * n)
    for i in range(n):
        R = T[i][:3, :3]
        v, acc, wr = V[i], Acc[i], W[i]
        lin_a, ang_a, ang_v = acc[:3], acc[3:], v[3:]
        f_S, n_S = R.T @ wr[:3], R.T @ wr[3:]
        Phi[6*i:6*i+6] = regressor(lin_a, ang_a, ang_v, R.T @ G); w[6*i:6*i+6] = -np.r_[f_S, n_S]
    return Phi, w


def evaluate(obj, noise=None, verbose=True):
    T, V, Acc, W, Tobj, y = load(obj)
    m_gt, c_mesh, I_mesh = y["MASS"], np.array(y["COM_WRT_MESH"]), np.array(y["INERTIA_WRT_COM"])
    # sensor->object transform (constant), GT into sensor frame
    X_SO = np.linalg.inv(T[0]) @ Tobj[0]; R_SO, p_SO = X_SO[:3, :3], X_SO[:3, 3]
    c_S = R_SO @ c_mesh + p_SO; I_S = R_SO @ I_mesh @ R_SO.T
    th_gt = theta_gt_in_S(m_gt, c_S, I_S)
    Phi, w = build(T, V, Acc, W)
    res_gt = np.linalg.norm(Phi @ th_gt - w) / np.linalg.norm(w); order, frame = "vw", "world-wrench"
    if noise is not None:
        w = w.copy(); n = len(w) // 6
        w += np.tile(np.r_[np.full(3, noise[0]), np.full(3, noise[1])], n) * rng.standard_normal(len(w))
    n = len(w) // 6; ntr = int(0.7 * n)
    tr = np.arange(6 * ntr); te = np.arange(6 * ntr, 6 * n)
    out = dict(obj=obj, order=order, frame=frame, gt_residual=res_gt, cond=np.linalg.cond(Phi[tr]), m_gt=m_gt)
    accn = np.linalg.norm(Acc, axis=1)
    for name, fit in (("ls", fit_ls), ("sdp", fit_sdp)):
        th = fit(Phi[tr], w[tr])
        if th is None: out[name] = None; continue
        m, c, I = unpack(th)
        pred = Phi[te] @ th; wr = w[te].reshape(-1, 6)
        rel = np.sqrt(((pred - w[te]).reshape(-1, 6) ** 2).mean(0)) / (wr.std(0) + 1e-12)
        out[name] = dict(m=m, m_err=abs(m - m_gt) / m_gt * 100, com_mm=np.linalg.norm(c - c_S) * 1000,
                         I_err=np.linalg.norm(I - I_S) / np.linalg.norm(I_S) * 100,
                         pm_err=(np.abs(np.sort(np.linalg.eigvalsh(I)) - np.sort(np.linalg.eigvalsh(I_S))) / np.sort(np.linalg.eigvalsh(I_S)) * 100).tolist(),
                         holdout_rel=float(rel.mean()), holdout_f=float(rel[:3].mean()), holdout_n=float(rel[3:].mean()))
    # excitation sweep (LS): gentlest q% of TRAIN samples by |acc|
    sweep = {}
    for q in (100, 50, 25, 10):
        thr = np.percentile(accn[:ntr], q); sel = np.nonzero(accn[:ntr] <= thr)[0]
        rows = (sel[:, None] * 6 + np.arange(6)[None, :]).ravel()
        th = fit_ls(Phi[rows], w[rows]); m, c, I = unpack(th)
        sweep[q] = dict(n=len(sel), cond=np.linalg.cond(Phi[rows]), m_err=abs(m - m_gt) / m_gt * 100,
                        com_mm=np.linalg.norm(c - c_S) * 1000, I_err=np.linalg.norm(I - I_S) / np.linalg.norm(I_S) * 100,
                        acc_max=float(thr))
    out["sweep"] = sweep
    return out


if __name__ == "__main__":
    objs = sorted(os.path.basename(p)[:-4] for p in glob.glob(f"{D}/Simulations/*.pkl"))
    results = {}
    for label, noise in (("CLEAN", None), ("NOISY  (F/T noise 0.3 N / 0.005 Nm)", NOISE)):
        print(f"\n{'#'*100}\n {label}\n{'#'*100}")
        print(f"{'object':<19}{'conv':>10}{'gtres':>7}{'cond':>7} | {'LS: m%':>7}{'CoM mm':>7}{'I%':>7}{'hold':>6} | {'SDP: m%':>8}{'CoM mm':>7}{'I%':>7}{'hold':>6}")
        print("-" * 100)
        rs = []
        for o in objs:
            r = evaluate(o, noise); rs.append(r); results[f"{label[:5]}_{o}"] = r
            L, S = r["ls"], r["sdp"]
            s_str = f"{S['m_err']:>8.2f}{S['com_mm']:>7.1f}{S['I_err']:>7.1f}{S['holdout_rel']:>6.3f}" if S else f"{'solver fail':>28}"
            print(f"{o:<19}{'':>10}{r['gt_residual']:>7.0e}{r['cond']:>7.1f} | {L['m_err']:>7.2f}{L['com_mm']:>7.1f}{L['I_err']:>7.1f}{L['holdout_rel']:>6.3f} | {s_str}")
        for meth in ("ls", "sdp"):
            v = [r[meth] for r in rs if r[meth]]
            print(f"  {meth.upper():<4} median: mass {np.median([x['m_err'] for x in v]):.2f}%  CoM {np.median([x['com_mm'] for x in v]):.1f} mm  "
                  f"I {np.median([x['I_err'] for x in v]):.1f}%   | mass<=5% on {sum(x['m_err']<=5 for x in v)}/{len(v)}  "
                  f"I<=10% on {sum(x['I_err']<=10 for x in v)}/{len(v)}   | mean cond {np.mean([r['cond'] for r in rs]):.1f}")
        print(f"\n  excitation sweep (LS, gentlest q% of train samples by |acc|), medians over objects:")
        print(f"  {'q%':>4}{'n':>6}{'|acc|max':>10}{'cond':>9}{'mass%':>8}{'CoM mm':>8}{'I%':>8}")
        for q in (100, 50, 25, 10):
            v = [r["sweep"][q] for r in rs]
            print(f"  {q:>4}{int(np.median([x['n'] for x in v])):>6}{np.median([x['acc_max'] for x in v]):>10.2f}{np.median([x['cond'] for x in v]):>9.1f}"
                  f"{np.median([x['m_err'] for x in v]):>8.2f}{np.median([x['com_mm'] for x in v]):>8.1f}{np.median([x['I_err'] for x in v]):>8.1f}")
    json.dump(results, open(f"{ROOT}/results/utias_gentle_motion.json", "w"), indent=1, default=float)
    print(f"\nsaved -> results/utias_gentle_motion.json")
