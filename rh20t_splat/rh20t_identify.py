#!/usr/bin/env python
"""Run the validated r2s_pipeline Newton-Euler fit on an RH20T episode.

Why this rather than a quiet-window heuristic: measured on cfg7, the rolling force
std while holding never drops below 0.28 N and |F| swings 1.1-5.8 N (teleoperated
motion plus contact with the scale/table).  There IS no static plateau.  The
Newton-Euler regression uses the motion instead of avoiding it.

Mapping:
    t      <- force_torque_base timestamps (s)
    T_ws   <- T_world_base @ T_base_tcp(t)   (base Z is up, so base is a valid world)
    wrench <- force_torque_base 'zeroed'     (tool weight already removed by RH20T,
                                              so the fitted mass is the OBJECT's)
Only samples where the jaws are stopped on an object are used.
"""
import os, sys, numpy as np
sys.path.insert(0,"/home/kneepolean/shubhj/R2S")
sys.path.insert(0,"/home/kneepolean/shubhj/R2S/rh20t_splat")
sys.path.insert(0,"/home/kneepolean/shubhj/R2S/rh20t_api")
from rh20t_api.transforms import pose_array_quat_2_matrix
import importlib
ID = importlib.import_module("r2s_pipeline.identify")   # __init__ rebinds the name to a function
R  = importlib.import_module("r2s_pipeline.regressor")

def build(ep, calib, use_base_frame=True):
    from wrist_poses import setup
    conf,K,E,T_world_base = setup(calib)
    T=lambda f: np.load(os.path.join(ep,"transformed",f),allow_pickle=True).item()
    ftb, tb, gr = T("force_torque_base.npy"), T("tcp_base.npy"), T("gripper.npy")
    s=[k for k in ftb if k in tb and k in gr][0]
    F=sorted(ftb[s],key=lambda x:x["timestamp"])
    tf=np.array([x["timestamp"] for x in F],dtype=np.int64)
    W =np.array([x["zeroed"] for x in F],dtype=float)
    B=sorted(tb[s],key=lambda x:x["timestamp"])
    tt=np.array([x["timestamp"] for x in B],dtype=np.int64)
    P =np.array([x["tcp"] for x in B],dtype=float)
    gk=np.array(sorted(gr[s].keys()),dtype=np.int64)
    gw=np.array([gr[s][k]["gripper_info"][0] for k in gk])

    # poses at the wrench timestamps (nearest neighbour; both streams are ~10 Hz and aligned)
    idx=np.searchsorted(tt,tf).clip(0,len(tt)-1)
    Tws=np.zeros((len(tf),4,4))
    for i,j in enumerate(idx):
        M=pose_array_quat_2_matrix(P[j])
        Tws[i]= M if use_base_frame else (T_world_base @ M)
    w_at=np.interp(tf,gk,gw); o,c=gw.max(),gw.min()
    held=(w_at>c+5)&(w_at<o-5)
    return dict(t=(tf-tf[0])/1000.0, T_ws=Tws, wrench=W), held, float(np.median(w_at[held])) if held.any() else np.nan

if __name__=="__main__":
    import glob
    calib="rh20t_data/calib_cfg7/RH20T_cfg7/calib/1645940577589"
    print(f"{'episode':42s} {'n':>4s} {'cond':>8s} {'mass g':>8s} {'|CoM| mm':>9s}")
    print("-"*78)
    res=[]
    for pat in sys.argv[1:]:
        for ep in sorted(glob.glob(f"rh20t_data/cfg7/RH20T_cfg7/{pat}_*_cfg_0007")):
            if not os.path.exists(os.path.join(ep,"transformed","force_torque_base.npy")): continue
            try:
                rec, held, width = build(ep, calib)
                if held.sum()<40: continue
                rec={k:(v[held] if hasattr(v,"__len__") and len(v)==len(held) else v) for k,v in rec.items()}
                rec["t"]=rec["t"]-rec["t"][0]
                Phi,w,kin,rep,_=ID.build_wrench_problem(rec, fc_hz=2.0)
                th=R.fit_ls(Phi,w); m,c_,I=R.unpack(th)
                cond=np.linalg.cond(Phi)
                print(f"{os.path.basename(ep)[:42]:42s} {held.sum():4d} {cond:8.1f} {m*1000:8.1f} {np.linalg.norm(c_)*1000:9.1f}")
                res.append(m*1000)
            except Exception as e:
                print(f"{os.path.basename(ep)[:42]:42s}  FAILED: {str(e)[:40]}")
    if res: print(f"\nn={len(res)}  median {np.median(res):.0f} g   range {min(res):.0f}-{max(res):.0f} g")
