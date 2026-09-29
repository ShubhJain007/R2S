"""Contact friction (object <-> surface, object <-> fingers) from joint torque.

Two frictions get confused in this project:
  * ROBOT joint friction (viscous + Coulomb per joint) -- a robot calibration, fitted as nuisance
    terms inside identify.py, or identified once per robot.  Best-identified quantity we have
    (2.5 % CV across independent calibrations) because it multiplies VELOCITY.
  * OBJECT contact friction mu -- a property of a MATERIAL PAIR (object/table, object/finger),
    which is what a simulation actually needs.  This module estimates that.

Method: press-and-slide.  The robot presses the grasped object onto the surface with its own normal
force and slides it at constant velocity.  Then

    mu_kinetic = |F_tangential| / |F_normal|

Both components come from the same end-effector wrench, so scale errors partly cancel.  Do NOT rely
on gravity for the normal load: at m*g a 0.5 kg object gives ~2 N of tangential force against a
~1-2 N force-estimation floor.  Pressing at 40-60 N puts mu within ~5 %, and makes the result
independent of the object's mass.

The end-effector wrench comes from the joint torques:  tau_ext = J(q)^T W  ->  least-squares W.
Feed it either KUKA's `external_torque` directly, or `measured_torque` minus the arm model
(identify.build_torque_problem does that subtraction).
"""
import numpy as np


def ee_wrench_from_joint_torque(q, tau_ext, robot_file, ee_frame):
    """(N,nj) joint angles + external joint torques -> (N,6) EE wrench [f; n] in WORLD frame.
    tau_ext must already have the arm's own dynamics removed (KUKA external_torque, or a residual)."""
    from pydrake.all import MultibodyPlant, Parser, JacobianWrtVariable
    plant = MultibodyPlant(0.0); Parser(plant).AddModels(robot_file); plant.Finalize()
    ctx = plant.CreateDefaultContext(); ee = plant.GetFrameByName(ee_frame); W = plant.world_frame()
    N = len(q); out = np.zeros((N, 6)); cond = np.zeros(N)
    for i in range(N):
        plant.SetPositions(ctx, q[i])
        J = plant.CalcJacobianSpatialVelocity(ctx, JacobianWrtVariable.kV, ee, np.zeros(3), W, W)  # 6xnj, [w; v]
        sol, *_ = np.linalg.lstsq(J.T, tau_ext[i], rcond=None)     # sol = [n; f] (torque, force)
        out[i] = np.r_[sol[3:], sol[:3]]                            # -> [f; n]
        cond[i] = np.linalg.cond(J.T)
    return out, cond


def estimate_mu(wrench, normal_dir, sliding_mask=None, min_normal_N=5.0):
    """wrench (N,6) [f; n] in the same frame as normal_dir (unit vector, surface normal).
    Returns kinetic mu over the sliding samples plus diagnostics."""
    f = wrench[:, :3]
    nrm = np.asarray(normal_dir, float); nrm = nrm / np.linalg.norm(nrm)
    Fn = f @ nrm                       # signed normal component
    Ft = np.linalg.norm(f - np.outer(Fn, nrm), axis=1)
    ok = np.abs(Fn) >= min_normal_N
    if sliding_mask is not None: ok &= np.asarray(sliding_mask, bool)
    if ok.sum() < 10:
        return dict(mu=None, reason=f"only {ok.sum()} samples with |F_n| >= {min_normal_N} N and sliding")
    mu_i = Ft[ok] / np.abs(Fn[ok])
    # total-least-squares style: slope of Ft vs |Fn| through the origin is more robust than the mean ratio
    slope = float((Ft[ok] * np.abs(Fn[ok])).sum() / (Fn[ok] ** 2).sum())
    return dict(mu=slope, mu_mean_ratio=float(mu_i.mean()), mu_std=float(mu_i.std()),
                n=int(ok.sum()), Fn_mean=float(np.abs(Fn[ok]).mean()), Ft_mean=float(Ft[ok].mean()),
                # a real Coulomb contact gives Ft proportional to Fn: check it
                linearity_r2=float(1 - ((Ft[ok] - slope * np.abs(Fn[ok])) ** 2).sum() /
                                   ((Ft[ok] - Ft[ok].mean()) ** 2).sum()))


def estimate_mu_static(wrench, normal_dir, speed, move_thresh=2e-3):
    """Breakaway (static) mu: the tangential/normal ratio in the last sample before motion starts."""
    f = wrench[:, :3]; nrm = np.asarray(normal_dir, float); nrm /= np.linalg.norm(nrm)
    Fn = np.abs(f @ nrm); Ft = np.linalg.norm(f - np.outer(f @ nrm, nrm), axis=1)
    moving = np.asarray(speed) > move_thresh
    if not moving.any(): return dict(mu_s=None, reason="object never moved")
    i = int(np.argmax(moving))
    if i == 0 or Fn[i - 1] < 1.0: return dict(mu_s=None, reason="motion started immediately or normal force too small")
    return dict(mu_s=float(Ft[i - 1] / Fn[i - 1]), breakaway_Fn=float(Fn[i - 1]), breakaway_Ft=float(Ft[i - 1]), index=i)


def mu_from_incline(angle_deg):
    """Ground truth, no robot: tilt the surface until the object slides.  mu_s = tan(theta)."""
    return float(np.tan(np.radians(angle_deg)))
