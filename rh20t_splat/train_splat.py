#!/usr/bin/env python
"""Stage 1: train a Gaussian splat of an RH20T workspace from KNOWN poses.

No COLMAP, no pose estimation.  RH20T ships calibrated intrinsics and extrinsics,
and verify_calib2.py established that extrinsics[serial] is world->camera -- which
is exactly what gsplat wants for `viewmats`.  So the camera parameters are an input,
not something to be solved for.

Gaussians are initialised from the fused RGB-D cloud (not random): with only ~11
views, initialisation is what decides whether this works at all.
Depth supervision keeps geometry honest where views are sparse, which matters
because the splat has to host shadow-casting proxy geometry later.
"""
import os, sys, math, json, argparse, time
import numpy as np, cv2, torch
from gsplat import rasterization
from gsplat.strategy import DefaultStrategy

DEV = "cuda"

def load_calib(cd):
    I = np.load(os.path.join(cd,"intrinsics.npy"), allow_pickle=True).item()
    E = np.load(os.path.join(cd,"extrinsics.npy"), allow_pickle=True).item()
    E = {k:(np.asarray(v)[0] if np.asarray(v).ndim==3 else np.asarray(v)) for k,v in E.items()}
    K = {k:np.asarray(v)[:3,:3] for k,v in I.items()}
    return K, E

def load_views(cd, K, E, scale=1.0):
    views = []
    for s in E:
        cp = os.path.join(cd,"imgs",f"cam_{s}_c.png"); dp = os.path.join(cd,"imgs",f"cam_{s}_d.npy")
        if not os.path.exists(cp): continue
        img = cv2.imread(cp)[:,:,::-1].astype(np.float32)/255.0
        dep = np.load(dp).astype(np.float32)/1000.0 if os.path.exists(dp) else None
        Ks = K[s].copy()
        if scale != 1.0:
            h,w = int(img.shape[0]*scale), int(img.shape[1]*scale)
            img = cv2.resize(img,(w,h),interpolation=cv2.INTER_AREA)
            if dep is not None: dep = cv2.resize(dep,(w,h),interpolation=cv2.INTER_NEAREST)
            Ks[:2] *= scale
        views.append(dict(serial=s, img=img, depth=dep, K=Ks, E=E[s]))
    return views

def init_cloud(views, stride=4, zmax=2.5):
    P, C = [], []
    for v in views:
        if v["depth"] is None: continue
        d, K = v["depth"], v["K"]; H,W = d.shape
        yy,xx = np.mgrid[0:H:stride, 0:W:stride]
        z = d[::stride,::stride]; col = v["img"][::stride,::stride]
        m = (z>0.2)&(z<zmax)
        u,vv,z,col = xx[m],yy[m],z[m],col[m]
        x=(u-K[0,2])*z/K[0,0]; y=(vv-K[1,2])*z/K[1,1]
        pts_cam = np.stack([x,y,z,np.ones_like(z)])
        pts_w = (np.linalg.inv(v["E"]) @ pts_cam)[:3].T     # cam_to_world = inv(E)  [verified]
        P.append(pts_w); C.append(col)
    return np.concatenate(P), np.concatenate(C)

def knn_scale(P, k=4, cap=0.05):
    import open3d as o3d
    pc = o3d.geometry.PointCloud(); pc.points = o3d.utility.Vector3dVector(P)
    kd = o3d.geometry.KDTreeFlann(pc)
    idx = np.random.default_rng(0).choice(len(P), min(20000,len(P)), replace=False)
    ds = []
    for i in idx:
        _,_,d2 = kd.search_knn_vector_3d(P[i], k)
        ds.append(np.sqrt(np.mean(d2[1:])))
    return float(np.clip(np.median(ds), 0.002, cap))

def psnr(a,b): return float(-10*torch.log10(((a-b)**2).mean()+1e-12))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("calib"); ap.add_argument("out")
    ap.add_argument("--episode", default=None, help="episode dir: use its frames instead of calib imgs")
    ap.add_argument("--fixed-frame", type=int, default=0)
    ap.add_argument("--wrist-every", type=int, default=2)
    ap.add_argument("--sh-degree", type=int, default=2)
    ap.add_argument("--iters", type=int, default=3000)
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--stride", type=int, default=4)
    ap.add_argument("--holdout", type=int, default=2)
    ap.add_argument("--depth-weight", type=float, default=0.1)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    K,E = load_calib(a.calib)
    if a.episode:
        import sys as _s; _s.path.insert(0,"/home/kneepolean/shubhj/R2S/rh20t_splat")
        from episode_views import load_episode_views
        views = load_episode_views(a.episode, a.calib, a.fixed_frame, a.wrist_every)
    else:
        views = load_views(a.calib, K, E, a.scale)
    print(f"{len(views)} views @ {views[0]['img'].shape[1]}x{views[0]['img'].shape[0]}")
    rng = np.random.default_rng(0)
    hold = set(rng.choice(len(views), a.holdout, replace=False).tolist())
    train = [v for i,v in enumerate(views) if i not in hold]
    test  = [v for i,v in enumerate(views) if i in hold]
    print(f"train {len(train)}  holdout {len(test)} ({[v['serial'] for v in test]})")

    if any(v["depth"] is not None for v in train):
        P,C = init_cloud([v for v in train if v["depth"] is not None], a.stride)
    else:
        # episode frames carry no depth yet -> initialise from the CALIB RGB-D cloud.
        # Same cameras, same room, same rig; only the tabletop contents changed, and
        # densification handles that.  Far better than a uniform box of noise.
        cviews = load_views(a.calib, K, E, 1.0)
        P, C = init_cloud([v for v in cviews if v["depth"] is not None], a.stride)
        print("  (init from calibration RGB-D cloud)")
    print(f"init cloud: {len(P)} points")
    s0 = knn_scale(P); print(f"init scale: {s0*1000:.1f} mm")
    N = len(P)
    means = torch.nn.Parameter(torch.tensor(P, dtype=torch.float32, device=DEV))
    scales = torch.nn.Parameter(torch.full((N,3), math.log(s0), device=DEV))
    quats = torch.nn.Parameter(torch.tensor(np.tile([1.,0,0,0],(N,1)), dtype=torch.float32, device=DEV))
    opac  = torch.nn.Parameter(torch.full((N,), torch.logit(torch.tensor(0.5)).item(), device=DEV))
    C0 = 0.28209479177387814                       # SH band-0 normalisation
    sh0 = torch.nn.Parameter(((torch.tensor(C, dtype=torch.float32, device=DEV)-0.5)/C0)[:,None,:])
    KSH = (a.sh_degree+1)**2
    shN = torch.nn.Parameter(torch.zeros((N, KSH-1, 3), device=DEV))

    params = torch.nn.ParameterDict(dict(means=means, scales=scales, quats=quats, opacities=opac, sh0=sh0, shN=shN))
    scene_scale = float(np.percentile(np.linalg.norm(P-P.mean(0),axis=1), 90))
    opt = {k: torch.optim.Adam([params[k]], lr=lr) for k,lr in
           dict(means=1.6e-4*scene_scale, scales=5e-3, quats=1e-3, opacities=5e-2,
                sh0=2.5e-3, shN=2.5e-3/20).items()}
    strat = DefaultStrategy(key_for_gradient="means2d",verbose=False, refine_start_iter=300, refine_stop_iter=int(a.iters*0.8),
                            reset_every=1500, refine_every=100)
    state = strat.initialize_state(scene_scale=scene_scale)

    def render(v, params):
        vm = torch.tensor(v["E"], dtype=torch.float32, device=DEV)[None]
        Ks = torch.tensor(v["K"], dtype=torch.float32, device=DEV)[None]
        H,W = v["img"].shape[:2]
        sh = torch.cat([params["sh0"], params["shN"]], dim=1)
        return rasterization(params["means"], torch.nn.functional.normalize(params["quats"],dim=-1),
                             torch.exp(params["scales"]), torch.sigmoid(params["opacities"]),
                             sh, vm, Ks, W, H, sh_degree=a.sh_degree,
                             render_mode="RGB+ED", packed=True)

    t0=time.time()
    for step in range(a.iters):
        v = train[step % len(train)]
        out, alpha, info = render(v, params)
        rgb = out[0,:,:,:3]
        gt = torch.tensor(v["img"], device=DEV)
        d = (rgb-gt).abs()
        mk = v.get("mask")
        if mk is not None:
            mt = torch.tensor(mk, device=DEV)
            l1 = d[mt].mean() if mt.any() else d.mean()
        else:
            l1 = d.mean()
        loss = l1
        if a.depth_weight>0 and v["depth"] is not None:
            dep = out[0,:,:,3]
            gtd = torch.tensor(v["depth"], device=DEV)
            m = (gtd>0.2)&(gtd<2.5)&(alpha[0,:,:,0]>0.5)
            if m.any(): loss = loss + a.depth_weight*((dep[m]-gtd[m]).abs().mean())
        strat.step_pre_backward(params, opt, state, step, info)
        loss.backward()
        for o in opt.values(): o.step(); o.zero_grad(set_to_none=True)
        strat.step_post_backward(params, opt, state, step, info, packed=True)
        if step % 500 == 0 or step == a.iters-1:
            with torch.no_grad():
                tr = np.mean([psnr(render(v,params)[0][0,:,:,:3], torch.tensor(v["img"],device=DEV)) for v in train[:40]])
                te = np.mean([psnr(render(v,params)[0][0,:,:,:3], torch.tensor(v["img"],device=DEV)) for v in test]) if test else float('nan')
            print(f"  step {step:5d}  loss {loss.item():.4f}  N {params['means'].shape[0]:7d}  "
                  f"train PSNR {tr:5.2f}  holdout PSNR {te:5.2f}   {time.time()-t0:5.0f}s")

    with torch.no_grad():
        for v in test+train[:1]:
            out,_,_ = render(v, params)
            im = (out[0,:,:,:3].clamp(0,1).cpu().numpy()*255).astype(np.uint8)[:,:,::-1]
            gt = (v["img"]*255).astype(np.uint8)[:,:,::-1]
            cv2.imwrite(os.path.join(a.out,f"cmp_{v['serial']}.png"), np.concatenate([gt,im],axis=1))
    torch.save({k:v.detach().cpu() for k,v in params.items()}, os.path.join(a.out,"splat.pt"))
    print(f"saved {a.out}/splat.pt  ({params['means'].shape[0]} gaussians)")

if __name__ == "__main__": main()
