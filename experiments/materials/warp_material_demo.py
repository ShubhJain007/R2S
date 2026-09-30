#!/usr/bin/env python
"""Differentiable material identification in Warp -- the pattern, with three constitutive laws.

The architecture that matters:  the SIMULATION SKELETON is shared (integrate, contact, boundary);
only the CONSTITUTIVE LAW changes per material.  Every material parameter is a wp.array with
requires_grad=True, so identification is:

    simulate(theta) -> loss vs observations -> tape.backward() -> d(loss)/d(theta) -> optimise

Two observation channels are used, and having both is the point:
    deformation  (point cloud / mesh vertices, from a camera)
    contact FORCE (from the robot's joint torque -- what the r2s_pipeline already measures)
Force is directly conjugate to stiffness (F = kx), so it identifies stiffness far better than
shape alone.  The published cloth work fits stiffness from point clouds + total mass only; a
force-instrumented robot supplies a channel they did not have.
"""
import numpy as np
import warp as wp

wp.init()
DEV = "cuda:0" if wp.is_cuda_available() else "cpu"


# ---------------------------------------------------------------- constitutive laws
# Each takes a strain measure and material parameters, returns a force. Swap freely.

@wp.func
def law_linear(x: float, v: float, E: float, b: float) -> float:
    """Hookean spring + viscous damping.  Cloth, soft nets, small deformation."""
    return -E * x - b * v


@wp.func
def law_neohookean(x: float, v: float, E: float, b: float) -> float:
    """Stiffening (hyperelastic-like): foam and rubber get stiffer as they compress."""
    return -E * (x + 0.5 * x * x * x) - b * v


@wp.func
def law_plastic(x: float, v: float, E: float, b: float) -> float:
    """Elastoplastic: force saturates past a yield point. Dough, clay, putty."""
    f = -E * x - b * v
    yield_f = 0.35 * E
    return wp.clamp(f, -yield_f, yield_f)


# ---------------------------------------------------------------- shared skeleton
# NOTE: state is stored PER TIMESTEP, not mutated in place. In-place updates silently break the
# adjoint -- the gradient comes back wrong-but-plausible, which is the classic differentiable-sim
# bug. Cost: O(T) memory. Long rollouts need checkpointing.

@wp.kernel
def step(x: wp.array(dtype=float), v: wp.array(dtype=float),
         theta: wp.array(dtype=float), m: float, dt: float, law: int,
         f_out: wp.array(dtype=float), t: int):
    tid = wp.tid()
    E = theta[0]
    b = theta[1]
    xi = x[t]
    vi = v[t]
    if law == 0:
        f = law_linear(xi, vi, E, b)
    elif law == 1:
        f = law_neohookean(xi, vi, E, b)
    else:
        f = law_plastic(xi, vi, E, b)
    a = f / m
    vn = vi + a * dt
    xn = xi + vn * dt
    v[t + 1] = vn
    x[t + 1] = xn
    f_out[t] = f                      # the force a robot would measure at the contact


@wp.kernel
def set_initial(x: wp.array(dtype=float), x0: float):
    x[0] = x0


def rollout(theta, law, x0, steps=200, dt=1e-3, m=0.2, requires_grad=False):
    x = wp.zeros(steps + 1, dtype=float, device=DEV, requires_grad=requires_grad)
    v = wp.zeros(steps + 1, dtype=float, device=DEV, requires_grad=requires_grad)
    f = wp.zeros(steps, dtype=float, device=DEV, requires_grad=requires_grad)
    wp.launch(set_initial, dim=1, inputs=[x, x0], device=DEV)
    for t in range(steps):
        wp.launch(step, dim=1, inputs=[x, v, theta, m, dt, law, f, t], device=DEV)
    return x, f


@wp.kernel
def loss_kernel(traj: wp.array(dtype=float), obs_x: wp.array(dtype=float),
                f: wp.array(dtype=float), obs_f: wp.array(dtype=float),
                w_force: float, loss: wp.array(dtype=float)):
    t = wp.tid()
    dx = traj[t + 1] - obs_x[t]
    df = f[t] - obs_f[t]
    wp.atomic_add(loss, 0, dx * dx + w_force * df * df)


def identify(obs_x, obs_f, law, x0, theta0, w_force, iters=400, lr=0.08):
    """Gradient-based material identification, theta = [E, b], Adam in log-space
    (E and b differ by ~100x, so optimise log-parameters and stay positive for free)."""
    logt = np.log(np.array(theta0, dtype=np.float64))
    ox = wp.array(obs_x, dtype=float, device=DEV)
    of = wp.array(obs_f, dtype=float, device=DEV)
    n = len(obs_x)
    mt = np.zeros(2); vt = np.zeros(2)
    for it in range(1, iters + 1):
        theta = wp.array(np.exp(logt), dtype=float, device=DEV, requires_grad=True)
        tape = wp.Tape()
        with tape:
            traj, f = rollout(theta, law, x0, steps=n, requires_grad=True)
            loss = wp.zeros(1, dtype=float, device=DEV, requires_grad=True)
            wp.launch(loss_kernel, dim=n, inputs=[traj, ox, f, of, w_force, loss], device=DEV)
        tape.backward(loss=loss)
        g = theta.grad.numpy().astype(np.float64) * np.exp(logt)     # chain rule to log-space
        tape.zero()
        mt = 0.9 * mt + 0.1 * g
        vt = 0.999 * vt + 0.001 * g * g
        logt -= lr * (mt / (1 - 0.9 ** it)) / (np.sqrt(vt / (1 - 0.999 ** it)) + 1e-12)
    return np.exp(logt)


if __name__ == "__main__":
    MATERIALS = [("linear elastic  (cloth, soft net)", 0, [40.0, 0.8]),
                 ("neo-Hookean     (foam, rubber)",    1, [25.0, 0.5]),
                 ("elastoplastic   (dough, clay)",     2, [60.0, 1.2])]
    print(f"device: {DEV}\n")
    print(f"{'material':<36}{'true E':>8}{'true b':>8}   {'shape only':>22}   {'shape + measured force':>24}")
    print("-" * 104)
    for name, law, true in MATERIALS:
        th_true = wp.array(true, dtype=float, device=DEV)
        obs_x, obs_f = rollout(th_true, law, x0=0.02, steps=200)
        ox, of = obs_x.numpy()[1:], obs_f.numpy()   # x has steps+1 entries; align with f
        rng = np.random.default_rng(0)
        ox_n = ox + rng.normal(0, 2e-4, ox.shape)        # 0.2 mm camera noise
        of_n = of + rng.normal(0, 0.05, of.shape)        # 0.05 N force noise
        guess = [true[0] * 0.4, true[1] * 3.0]           # deliberately poor start
        a = identify(ox_n, of_n, law, 0.02, guess, w_force=0.0)    # deformation only
        b = identify(ox_n, of_n, law, 0.02, guess, w_force=1.0)    # + force channel
        eA = 100 * abs(a[0] - true[0]) / true[0]
        eB = 100 * abs(b[0] - true[0]) / true[0]
        print(f"{name:<36}{true[0]:>8.1f}{true[1]:>8.2f}   E={a[0]:>7.2f} ({eA:>5.1f}%)   E={b[0]:>7.2f} ({eB:>5.1f}%)")
    print("\nsame skeleton, three constitutive laws, one identification loop.")
