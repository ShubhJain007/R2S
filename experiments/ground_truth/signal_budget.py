#!/usr/bin/env python
"""Why mass is identifiable from gentle motion and inertia is not: how much of the measured wrench each
parameter group explains, on the 20 ground-truth workshop tools.

The Newton-Euler model is linear in theta = [m, h(3), I_O(6)], so with the GROUND-TRUTH parameters the
wrench splits exactly into three additive parts (same regressor as utias_gentle_motion_test.py):

    force   =  m (a - g)              +  alpha x h + w x (w x h)
    torque  =  h x (a - g)            +  I_O alpha + w x (I_O w)
               ^ mass / CoM, gravity     ^ inertia, needs angular acceleration

Reported per object: RMS of each part over the recording, next to the F/T noise used in the noisy test
(0.3 N / 0.005 N m, Robotiq FT-300 class).

Usage:  python experiments/ground_truth/signal_budget.py      -> results/utias_signal_budget.json
"""
import glob, json, os
import numpy as np
from utias_gentle_motion_test import D, NOISE, ROOT, build, load, theta_gt_in_S

rms = lambda x: float(np.sqrt((np.asarray(x) ** 2).sum(1).mean()))      # RMS of the 3-vector norm

if __name__ == "__main__":
    objs = sorted(os.path.basename(p)[:-4] for p in glob.glob(f"{D}/Simulations/*.pkl"))
    out = []
    print(f"{'object':<19}{'mass kg':>8} | {'m(a-g) N':>9}{'h-terms N':>10} | {'CoM Nm':>8}{'inertia Nm':>11}{'I / noise':>10}{'I share %':>10}")
    print("-" * 92)
    for o in objs:
        T, V, Acc, W, Tobj, y = load(o)
        X_SO = np.linalg.inv(T[0]) @ Tobj[0]; R_SO, p_SO = X_SO[:3, :3], X_SO[:3, 3]
        c_S = R_SO @ np.array(y["COM_WRT_MESH"]) + p_SO; I_S = R_SO @ np.array(y["INERTIA_WRT_COM"]) @ R_SO.T
        th = theta_gt_in_S(y["MASS"], c_S, I_S)
        Phi, _ = build(T, V, Acc, W)
        part = lambda cols: (Phi[:, cols] @ th[cols]).reshape(-1, 6)
        pm, ph, pI = part([0]), part([1, 2, 3]), part([4, 5, 6, 7, 8, 9])
        r = dict(obj=o, mass=y["MASS"], f_mass=rms(pm[:, :3]), f_com=rms(ph[:, :3]), n_com=rms(ph[:, 3:]), n_inertia=rms(pI[:, 3:]),
                 n_total=rms((pm + ph + pI)[:, 3:]), alpha_rms=rms(Acc[:, 3:]))
        r["inertia_share"] = r["n_inertia"] / r["n_total"] * 100
        out.append(r)
        print(f"{o:<19}{r['mass']:>8.3f} | {r['f_mass']:>9.3f}{r['f_com']:>10.4f} | {r['n_com']:>8.4f}{r['n_inertia']:>11.5f}"
              f"{r['n_inertia'] / NOISE[1]:>10.2f}{r['inertia_share']:>10.2f}")
    med = lambda k: float(np.median([r[k] for r in out]))
    print("-" * 92)
    print(f"median: gravity force {med('f_mass'):.2f} N = {med('f_mass') / NOISE[0]:.0f}x force noise | CoM torque {med('n_com'):.3f} N m = "
          f"{med('n_com') / NOISE[1]:.0f}x torque noise | inertia torque {med('n_inertia'):.5f} N m = {med('n_inertia') / NOISE[1]:.2f}x torque noise "
          f"({med('inertia_share'):.1f}% of the torque signal)")
    json.dump(dict(noise=dict(force_N=NOISE[0], torque_Nm=NOISE[1]), objects=out), open(f"{ROOT}/results/utias_signal_budget.json", "w"), indent=1)
    print("\nsaved -> results/utias_signal_budget.json")
