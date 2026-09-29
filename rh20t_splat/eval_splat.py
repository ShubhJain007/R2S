#!/usr/bin/env python
"""Per-view PSNR, split by camera type.

The in-hand camera sweeps a small volume just above the work surface, so it can only
densify the MANIPULATION REGION -- not the room.  If that is what is happening, wrist
holdout views should score well while distant fixed holdout views stay poor.  That
distinction matters: a twin for manipulation needs the tabletop right, not the walls.
"""
import os, sys, numpy as np, torch, cv2
sys.path.insert(0,"/home/kneepolean/shubhj/R2S/rh20t_splat")
from gsplat import rasterization
from train_splat import load_calib, psnr
from episode_views import load_episode_views
DEV="cuda"

ck, calib, ep = sys.argv[1], sys.argv[2], sys.argv[3]
P = torch.load(ck); P = {k:v.to(DEV) for k,v in P.items()}
views = load_episode_views(ep, calib, wrist_every=2)
rng = np.random.default_rng(0)
hold = set(rng.choice(len(views), 8, replace=False).tolist())

def render(v):
    vm = torch.tensor(v["E"],dtype=torch.float32,device=DEV)[None]
    Ks = torch.tensor(v["K"],dtype=torch.float32,device=DEV)[None]
    H,W = v["img"].shape[:2]
    out,_,_ = rasterization(P["means"], torch.nn.functional.normalize(P["quats"],dim=-1),
                            torch.exp(P["scales"]), torch.sigmoid(P["opacities"]),
                            torch.sigmoid(P["colors"]), vm, Ks, W, H, render_mode="RGB", packed=True)
    return out[0,:,:,:3]

res={"wrist/holdout":[], "wrist/train":[], "fixed/holdout":[], "fixed/train":[]}
with torch.no_grad():
    for i,v in enumerate(views):
        if i%3 and i not in hold: continue
        kind = "wrist" if "_f" in v["serial"] else "fixed"
        split = "holdout" if i in hold else "train"
        res[f"{kind}/{split}"].append(psnr(render(v), torch.tensor(v["img"],device=DEV)))
print(f"\n{'split':18s} {'n':>4s} {'PSNR':>8s}")
print("-"*34)
for k,v in res.items():
    if v: print(f"{k:18s} {len(v):4d} {np.mean(v):8.2f}")
