#!/usr/bin/env python
"""Stage-3 check: put the ROBOT BASE frame into the cameras and look at it.

Chain being tested (every link read from rh20t_api, none guessed):
    calib tcp.npy   -- KUKA: xyz in mm, rotation in EULER   (kuka_tcp_preprocessor)
    tc_mat          -- T_cam_tcp for the in-hand camera, metres   (configs.json)
    base_world_mat  = inv(E_inhand) @ tc_mat @ inv(T_base_tcp)  = T_world_base
    T_cam_base      = E_cam @ base_world_mat

If the chain is right, the drawn TCP axes land on the gripper and the base axes
land on the robot's base -- both visible in the calibration images.  That single
picture validates extrinsics, hand-eye and the base transform together, which is
the step real2sim-eval does by hand in SuperSplat.
"""
import os, sys, json
import numpy as np, cv2
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rh20t_api"))
from rh20t_api.configurations import load_conf
from rh20t_api.transforms import (calc_base_world_mat, pose_array_quat_2_matrix)

def draw_frame(img, K, T_cam_X, label, L=0.15):
    """T_cam_X: pose of frame X in camera coords."""
    pts = np.array([[0,0,0,1],[L,0,0,1],[0,L,0,1],[0,0,L,1]]).T
    Q = T_cam_X @ pts
    if (Q[2] <= 0.05).any(): return False
    uv = K @ Q[:3]; uv = (uv[:2]/uv[2]).T
    o = tuple(np.round(uv[0]).astype(int))
    H, W = img.shape[:2]
    if not (-W < o[0] < 2*W and -H < o[1] < 2*H): return False
    for i, c in [(1,(0,0,255)),(2,(0,255,0)),(3,(255,0,0))]:
        cv2.arrowedLine(img, o, tuple(np.round(uv[i]).astype(int)), c, 4, tipLength=0.2)
    cv2.circle(img, o, 8, (0,255,255), 3)
    cv2.putText(img, label, (o[0]+12, o[1]-12), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,255,255), 3)
    return True

if __name__ == "__main__":
    calib_dir, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    conf = [c for c in load_conf(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rh20t_api", "configs", "configs.json"))
            if c.conf_num == 7][0]
    I = np.load(os.path.join(calib_dir,"intrinsics.npy"), allow_pickle=True).item()
    E = np.load(os.path.join(calib_dir,"extrinsics.npy"), allow_pickle=True).item()
    E = {k:(np.asarray(v)[0] if np.asarray(v).ndim==3 else np.asarray(v)) for k,v in E.items()}
    K = {k:np.asarray(v)[:3,:3] for k,v in I.items()}

    tcp_raw = np.load(os.path.join(calib_dir,"tcp.npy"))
    tcp6 = conf.tcp_preprocessor(tcp_raw)               # mm->m, euler bounded
    from rh20t_api.configurations import tcp_as_q
    tcp_q = tcp_as_q(tcp6)
    T_base_tcp = pose_array_quat_2_matrix(tcp_q)
    print("calib tcp raw :", np.round(tcp_raw,4))
    print("calib tcp (m) :", np.round(tcp6,4))

    inhand = conf.in_hand_serial[0]
    T_world_base = calc_base_world_mat(E[inhand], tcp_q, conf.tcp_camera_mat)
    print(f"\nin-hand cam   : {inhand}")
    print("T_world_base  =\n", np.round(T_world_base,4))
    print("base origin in marker frame (m):", np.round(T_world_base[:3,3],3))

    T_world_tcp = T_world_base @ T_base_tcp
    print("tcp  origin in marker frame (m):", np.round(T_world_tcp[:3,3],3))

    n = 0
    for s in E:
        p = os.path.join(calib_dir,"imgs",f"cam_{s}_c.png")
        if not os.path.exists(p): continue
        img = cv2.imread(p).copy()
        a = draw_frame(img, K[s], E[s] @ T_world_base, "BASE")
        b = draw_frame(img, K[s], E[s] @ T_world_tcp,  "TCP")
        c = draw_frame(img, K[s], E[s],                "MARKER", L=0.1)
        if a or b:
            cv2.imwrite(os.path.join(out, f"base_{s}.png"), img); n += 1
    print(f"\nwrote {n} annotated images to {out}")
