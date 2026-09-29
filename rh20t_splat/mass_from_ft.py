#!/usr/bin/env python
"""Object mass from the wrist F/T, segmented by gripper width and TCP motion.

`zeroed` is already compensated for the TOOL's own weight (configurations.py:117 uses
conf.gravity/centroid), which is why it reads ~0.05 N with an empty gripper.  So while
an object is held and the arm is nearly static,  |f_zeroed| = m*g  directly.

Segmentation matters more than the arithmetic:
  width ~84 mm -> fully open (nothing held)
  width ~9  mm -> fully closed on NOTHING (not a grasp!)
  in between   -> jaws stopped on an object; that width is the object's size
Getting this wrong was my first error -- it put the 9 mm "closed on air" samples into
the holding set.  Live TCP must come from tcp_base.npy; the `tcp` field inside
high_freq_data.npy is stale (constant z all episode).
"""
import os, sys, numpy as np

G = 9.81

def load(ep):
    T = lambda f: np.load(os.path.join(ep,"transformed",f), allow_pickle=True).item()
    hf, gr, tb = T("high_freq_data.npy"), T("gripper.npy"), T("tcp_base.npy")
    s = [k for k in hf if k in gr and k in tb][0]
    H = sorted(hf[s], key=lambda x: x["timestamp"])
    th = np.array([x["timestamp"] for x in H], dtype=np.int64)
    fz = np.linalg.norm(np.array([x["zeroed"] for x in H])[:, :3], axis=1)
    gk = np.array(sorted(gr[s].keys()), dtype=np.int64)
    gw = np.array([gr[s][k]["gripper_info"][0] for k in gk])
    B = sorted(tb[s], key=lambda x: x["timestamp"])
    tt = np.array([x["timestamp"] for x in B], dtype=np.int64)
    pos = np.array([x["tcp"] for x in B])[:, :3]
    return th, fz, gk, gw, tt, pos

def estimate(ep, verbose=True):
    th, fz, gk, gw, tt, pos = load(ep)
    w_at = np.interp(th, gk, gw)
    # speed from live TCP, interpolated onto the force clock
    dt = np.diff(tt) / 1000.0
    spd = np.r_[0, np.linalg.norm(np.diff(pos, axis=0), axis=1) / np.maximum(dt, 1e-3)]
    spd_at = np.interp(th, tt, spd)

    open_w, closed_w = gw.max(), gw.min()
    held = (w_at > closed_w + 5) & (w_at < open_w - 5)     # jaws stopped ON something
    free = w_at > open_w - 3                               # nothing in the gripper
    slow = spd_at < 0.03                                   # m/s, quasi-static
    base = fz[free & slow]
    grab = fz[held & slow]
    if verbose:
        print(f"  gripper: open {open_w:.0f} mm, closed {closed_w:.0f} mm, "
              f"held-window width {np.median(w_at[held]):.0f} mm" if held.any() else "  no grasp found")
        print(f"  samples: baseline {base.size}, held+slow {grab.size}")
    if base.size < 30 or grab.size < 30: return None
    m = (np.median(grab) - np.median(base)) / G * 1000
    lo, hi = (np.percentile(grab,25)-np.median(base))/G*1000, (np.percentile(grab,75)-np.median(base))/G*1000
    return dict(mass_g=m, iqr=(lo,hi), width_mm=float(np.median(w_at[held])),
                baseline_N=float(np.median(base)), held_N=float(np.median(grab)), n=int(grab.size))

if __name__ == "__main__":
    import glob
    root = "rh20t_data/cfg7/RH20T_cfg7"
    for task in sys.argv[1:]:
        print(f"\n===== {task}")
        for ep in sorted(glob.glob(os.path.join(root, f"{task}_*_cfg_0007")))[:6]:
            if not os.path.exists(os.path.join(ep,"transformed","high_freq_data.npy")): continue
            print(f"\n{os.path.basename(ep)}")
            r = estimate(ep)
            if r: print(f"  -> mass {r['mass_g']:6.1f} g   (IQR {r['iqr'][0]:.0f}-{r['iqr'][1]:.0f} g)  "
                        f"object width {r['width_mm']:.0f} mm  baseline {r['baseline_N']:.3f} N")
