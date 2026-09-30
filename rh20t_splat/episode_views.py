"""Views for a per-episode splat: fixed cameras at one instant + the wrist-camera sweep.

The point of this file is the second source.  Nine fixed cameras give nine views and
3DGS cannot generalise from that (measured: train PSNR 30.6, holdout 13.6).  The
in-hand camera supplies a few hundred more, with poses that are a kinematic
consequence of the recorded TCP -- no COLMAP, no alignment, no extra capture.
"""
import os, sys, numpy as np, cv2
from depth_io import read_depth_video
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wrist_poses import setup, load_tcp_series, frames, wrist_extrinsics, derive_tc_mat

def gripper_mask(fr, thresh=0.06):
    """In WRIST frames the gripper is rigidly attached to the camera, so it is the
    content that does NOT change while everything else does.  Low temporal variance
    therefore marks the robot.  Returns True where the pixel is usable scene."""
    a = np.stack([f.astype(np.float32)/255.0 for f in fr[::max(1,len(fr)//40)]])
    sd = a.std(0).mean(-1)
    m = sd > thresh
    m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((15,15),np.uint8))
    return m.astype(bool)

def median_plate(fr):
    """For a FIXED camera: the per-pixel temporal median removes the moving arm and
    object, leaving a static background plate that a static splat can actually explain."""
    a = np.stack([f for f in fr[::max(1,len(fr)//60)]])
    return np.median(a, axis=0).astype(np.uint8)

def load_episode_views(ep, calib, fixed_frame=0, wrist_every=2, use_plate=True, use_mask=True):
    conf, K, E, T_world_base = setup(calib)
    cams = sorted(d[4:] for d in os.listdir(ep) if d.startswith("cam_"))
    inhand = set(conf.in_hand_serial)
    views = []
    tt, pp = load_tcp_series(ep, conf.in_hand_serial[0])
    for s in cams:
        if s not in E or s not in K: continue
        try: ts, fr = frames(ep, s)
        except Exception: continue
        if not fr: continue
        H, W = fr[0].shape[:2]
        Ks = K[s].copy(); Ks[:2] *= (W/1280.0)
        dep = None
        dp = os.path.join(ep, f"cam_{s}", "depth.mp4")
        if os.path.exists(dp):
            try: dep = read_depth_video(dp, s)
            except Exception: dep = None
        if s in inhand:
            tc = derive_tc_mat(E, T_world_base, calib, conf, s)   # per-camera hand-eye
            gm = gripper_mask(fr) if use_mask else None
            if gm is not None:
                print(f"  wrist {s}: masking {100*(1-gm.mean()):.0f}% of pixels as robot")
            for i in range(0, len(fr), wrist_every):
                j = int(np.argmin(np.abs(tt - ts[i])))
                d_i = (dep[i].astype(np.float32)/1000.0) if (dep is not None and i < len(dep)) else None
                views.append(dict(serial=f"{s}_f{i}", img=fr[i][:,:,::-1].astype(np.float32)/255.0,
                                  depth=d_i, K=Ks, E=wrist_extrinsics(conf, T_world_base, pp[j], tc),
                                  mask=gm))
        else:
            base = median_plate(fr) if use_plate else fr[min(fixed_frame, len(fr)-1)]
            dmed = None
            if dep is not None:
                a = np.stack([x for x in dep[::max(1,len(dep)//40)]]).astype(np.float32)/1000.0
                a[a <= 0] = np.nan
                dmed = np.nanmedian(a, axis=0); dmed[~np.isfinite(dmed)] = 0.0   # static depth plate
            views.append(dict(serial=s, img=base[:,:,::-1].astype(np.float32)/255.0,
                              depth=dmed, K=Ks, E=E[s], mask=None))
    return views
