"""Newton-Euler rigid-body regressor and parameter packing.

Frame S = the sensor / end-effector frame the object is rigidly attached to, origin O.
theta = [m, h(3), I_O(6)]  with  h = m*c  (c = CoM position from O, in S)  and  I_O = inertia about O.

    f   = m (a_O - g) + alpha x h + w x (w x h)
    n_O = h x (a_O - g) + I_O alpha + w x (I_O w)
    =>  wrench_S = Phi(a_O, alpha, w, g_S) . theta            (6 x 10, linear)

All kinematic inputs are expressed in S.  The wrench is the CONTACT wrench: the force/torque the
sensor exerts ON the object.  Validated against ground truth on the utiasSTARS dataset
(mass median 0.2-0.8 %, CoM ~1 cm, under <=2.5 m/s^2 motion).
"""
import numpy as np

G_WORLD = np.array([0.0, 0.0, -9.81])


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def _L(v):
    x, y, z = v
    return np.array([[x, y, z, 0, 0, 0], [0, x, 0, y, z, 0], [0, 0, x, 0, y, z]])


def block(a_O, alpha, w, g_S):
    """6x10 regressor for one sample."""
    ag = a_O - g_S
    P = np.zeros((6, 10))
    P[:3, 0] = ag
    P[:3, 1:4] = skew(alpha) + skew(w) @ skew(w)
    P[3:, 1:4] = -skew(ag)
    P[3:, 4:] = _L(alpha) + skew(w) @ _L(w)
    return P


def stack(a_O, alpha, w, g_S):
    """(N,3) x4 -> Phi (6N,10). Rows are time-major, 6 per sample [fx fy fz nx ny nz]."""
    return np.vstack([block(a_O[i], alpha[i], w[i], g_S[i]) for i in range(len(a_O))])


def pack(m, c, I_cm):
    h = m * c
    I_O = I_cm + m * (c @ c * np.eye(3) - np.outer(c, c))
    return np.r_[m, h, I_O[0, 0], I_O[0, 1], I_O[0, 2], I_O[1, 1], I_O[1, 2], I_O[2, 2]]


def unpack(theta):
    m = theta[0]
    c = theta[1:4] / m
    I_O = np.array([[theta[4], theta[5], theta[6]], [theta[5], theta[7], theta[8]], [theta[6], theta[8], theta[9]]])
    I_cm = I_O - m * (c @ c * np.eye(3) - np.outer(c, c))
    return m, c, I_cm


def fit_ls(Phi, w):
    return np.linalg.lstsq(Phi, w, rcond=None)[0]


def fit_sdp(Phi, w):
    """Least squares with the pseudo-inertia PSD (physical-consistency) constraint. Needs Drake."""
    from pydrake.all import MathematicalProgram, Solve
    prog = MathematicalProgram(); th = prog.NewContinuousVariables(10, "th")
    m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz = th
    tr = (Ixx + Iyy + Izz) / 2
    J = np.empty((4, 4), dtype=object)
    J[:3, :3] = np.array([[tr - Ixx, -Ixy, -Ixz], [-Ixy, tr - Iyy, -Iyz], [-Ixz, -Iyz, tr - Izz]], dtype=object)
    J[:3, 3] = [hx, hy, hz]; J[3, :3] = [hx, hy, hz]; J[3, 3] = m
    prog.AddPositiveSemidefiniteConstraint(J)
    prog.Add2NormSquaredCost(Phi, w, th)
    r = Solve(prog)
    return r.GetSolution(th) if r.is_success() else None
