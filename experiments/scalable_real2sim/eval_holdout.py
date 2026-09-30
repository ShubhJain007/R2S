#!/usr/bin/env python
"""Held-out reconstruction test for the hybrid pipeline (Rung 2, ground-truth-free).

The object is grasped throughout, so the observable is joint torque: we fit parameters on part of
the trajectory and RECONSTRUCT the joint torques of the held-out part from the robot's measured
motion. Leave-one-block-out over K contiguous time blocks, so every sample is held out exactly once
and the result does not depend on which window was chosen.

Metrics on each held-out block, per method:
  relRMSE(0-4)  RMS torque error / torque std, averaged over joints 0-4 (where the payload signal lives)
  relRMSE(all)  same over all 7 joints (wrist joints 5-6 are noise-dominated; shown for honesty)
  absRMSE       RMS torque error in Nm over all joints

Usage:  python experiments/scalable_real2sim/eval_holdout.py [K] spam sugar lego
"""
import json, sys
import numpy as np
from hybrid_pipeline import HybridIdentifier, METHODS
from s2s_tests import NJ, ROOT


def block_masks(T, K):
    edges = np.linspace(0, T, K + 1).astype(int)
    return [(np.arange(T) >= a) & (np.arange(T) < b) for a, b in zip(edges[:-1], edges[1:])]


def errors(P, sol, rows):
    pred = P.W[rows] @ sol
    res = (pred - P.tau[rows]).reshape(-1, NJ); tru = P.tau[rows].reshape(-1, NJ)
    rel = np.sqrt((res ** 2).mean(0)) / (tru.std(0) + 1e-9)
    return dict(rel04=float(rel[:5].mean()), relall=float(rel.mean()), abs_nm=float(np.sqrt((res ** 2).mean())))


def run(obj, K):
    P = HybridIdentifier(obj)
    folds = block_masks(P.T, K)
    per = {m: [] for m in METHODS}; params = {m: [] for m in METHODS}
    for k, te_mask in enumerate(folds):
        tr = P.rows(~te_mask); te = P.rows(te_mask)
        out = P.identify(tr)
        if out is None:
            print(f"  fold {k}: solver failed"); continue
        res, _ = out
        for m in METHODS:
            p, sol = res[m]
            per[m].append(errors(P, sol, te)); params[m].append(p)
    summary = {}
    for m in METHODS:
        e = per[m]
        summary[m] = {k: (float(np.mean([x[k] for x in e])), float(np.std([x[k] for x in e]))) for k in e[0]}
        masses = [p["mass"] for p in params[m]]
        Is = [np.diag(p["I"]) for p in params[m]]
        summary[m]["mass_mean"] = float(np.mean(masses)); summary[m]["mass_cv"] = float(np.std(masses) / np.mean(masses) * 100)
        summary[m]["I_cv"] = float(np.mean(np.std(Is, 0) / (np.abs(np.mean(Is, 0)) + 1e-12) * 100))
        summary[m]["gyration"] = float(np.mean([P.gyration_ratio(p["mass"], p["I"]) for p in params[m]]))
    return summary


if __name__ == "__main__":
    args = sys.argv[1:]
    K = int(args[0]) if args and args[0].isdigit() else 5
    objs = [a for a in args if not a.isdigit()] or ["spam"]
    all_res = {}
    for obj in objs:
        s = run(obj, K); all_res[obj] = s
        print(f"\n{'='*96}\n {obj}: leave-one-block-out, K={K} blocks of {10/K:.0f}s\n{'='*96}")
        print(f"{'method':<16}{'relRMSE j0-4':>16}{'relRMSE all':>15}{'absRMSE Nm':>13}{'mass':>9}{'mass CV%':>10}{'I CV%':>8}{'gyr':>7}")
        print("-" * 96)
        for m in METHODS:
            r = s[m]
            print(f"{m:<16}{r['rel04'][0]:>10.4f}±{r['rel04'][1]:<5.3f}{r['relall'][0]:>9.4f}±{r['relall'][1]:<5.3f}"
                  f"{r['abs_nm'][0]:>9.3f}±{r['abs_nm'][1]:<4.2f}{r['mass_mean']:>8.4f}{r['mass_cv']:>9.1f}{r['I_cv']:>8.0f}{r['gyration']:>7.2f}")
        b, h = s["torque"], s["hybrid"]
        print(f"\n  hybrid vs torque-only:  relRMSE(j0-4) {(h['rel04'][0]/b['rel04'][0]-1)*100:+.1f}%   "
              f"absRMSE {(h['abs_nm'][0]/b['abs_nm'][0]-1)*100:+.1f}%   inertia fold-to-fold CV {b['I_cv']:.0f}% -> {h['I_cv']:.0f}%   "
              f"gyration ratio {b['gyration']:.2f} -> {h['gyration']:.2f}")
    # cross-object summary
    print(f"\n{'='*96}\n SUMMARY over {len(objs)} objects (mean of per-object means)\n{'='*96}")
    print(f"{'method':<16}{'relRMSE j0-4':>14}{'relRMSE all':>14}{'absRMSE Nm':>12}{'mass CV%':>10}{'I CV%':>8}{'gyration':>10}")
    for m in METHODS:
        print(f"{m:<16}{np.mean([all_res[o][m]['rel04'][0] for o in objs]):>14.4f}{np.mean([all_res[o][m]['relall'][0] for o in objs]):>14.4f}"
              f"{np.mean([all_res[o][m]['abs_nm'][0] for o in objs]):>12.3f}{np.mean([all_res[o][m]['mass_cv'] for o in objs]):>10.1f}"
              f"{np.mean([all_res[o][m]['I_cv'] for o in objs]):>8.0f}{np.mean([all_res[o][m]['gyration'] for o in objs]):>10.2f}")
    (ROOT / "results" / "hybrid" / f"holdout_K{K}.json").write_text(json.dumps(all_res, indent=1))
    print(f"\nsaved -> results/hybrid/holdout_K{K}.json")
