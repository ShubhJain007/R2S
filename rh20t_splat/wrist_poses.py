#!/usr/bin/env python
"""In-hand camera pose per frame, from forward kinematics -- the free multi-view capture.

    tc_mat = T_cam_tcp                       (configs.json, metres)
    T_world_base                             (calc_base_world_mat, verified in verify_base.py)
    E_inhand(t) = tc_mat @ inv(T_world_base @ T_base_tcp(t))       [ world -> camera ]

This is why RH20T needs no COLMAP and no manual alignment: the wrist camera's pose
is a kinematic consequence of the recorded TCP, already in the same frame as the
fixed cameras.  Verification is the same trick as everywhere else in this project --
project the marker (world origin) into the frame and look at where it lands.
"""
import os, sys, json, numpy as np, cv2
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rh20t_api"))
from rh20t_api.configurations import load_conf, tcp_as_q
from rh20t_api.transforms import calc_base_world_mat, pose_array_quat_2_matrix

CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rh20t_api", "configs", "configs.json")

def setup(calib_dir, conf_num=7):
    conf = [c for c in load_conf(CFG) if c.conf_num == conf_num][0]
    I = np.load(os.path.join(calib_dir,"intrinsics.npy"), allow_pickle=True).item()
    E = np.load(os.path.join(calib_dir,"extrinsics.npy"), allow_pickle=True).item()
    E = {k:(np.asarray(v)[0] if np.asarray(v).ndim==3 else np.asarray(v)) for k,v in E.items()}
    K = {k:np.asarray(v)[:3,:3] for k,v in I.items()}
    tcp_q = tcp_as_q(conf.tcp_preprocessor(np.load(os.path.join(calib_dir,"tcp.npy"))))
    T_world_base = calc_base_world_mat(E[conf.in_hand_serial[0]], tcp_q, conf.tcp_camera_mat)
    return conf, K, E, T_world_base

def derive_tc_mat(E, T_world_base, calib_dir, conf, serial):
    """T_cam_tcp for ANY in-hand camera, derived from the calibration snapshot.

    configs.json ships ONE tc_mat, valid only for in_hand_serial[0].  cfg7 has two
    wrist cameras mounted 113 mm apart, so reusing it puts the second camera's poses
    11 cm off.  At calibration time both E[serial] and T_world_tcp are known, so
    T_cam_tcp = E[serial] @ T_world_tcp exactly.  Checked: for in_hand_serial[0] this
    reproduces configs.json to 0 mm, which is what makes it trustworthy for the other.
    """
    tcp_q = tcp_as_q(conf.tcp_preprocessor(np.load(os.path.join(calib_dir,"tcp.npy"))))
    T_world_tcp = T_world_base @ pose_array_quat_2_matrix(tcp_q)
    return E[serial] @ T_world_tcp

def wrist_extrinsics(conf, T_world_base, tcp7, tc_mat=None):
    """world -> in-hand camera, for a 7D tcp (xyz m + quat) in base coords."""
    T_world_tcp = T_world_base @ pose_array_quat_2_matrix(np.asarray(tcp7, dtype=np.float64))
    return (conf.tcp_camera_mat if tc_mat is None else tc_mat) @ np.linalg.inv(T_world_tcp)

def load_tcp_series(ep, serial):
    d = np.load(os.path.join(ep,"transformed","tcp_base.npy"), allow_pickle=True).item()
    lst = d[serial]
    t = np.array([x["timestamp"] for x in lst])
    p = np.array([x["tcp"] for x in lst])
    o = np.argsort(t)
    return t[o], p[o]

def frames(ep, serial):
    ts = np.load(os.path.join(ep,f"cam_{serial}","timestamps.npy"), allow_pickle=True)
    if ts.ndim == 0: ts = ts.item()
    if isinstance(ts, dict): ts = ts["color"]      # {'color': [...], 'depth': [...]}
    ts = np.asarray(ts)
    cap = cv2.VideoCapture(os.path.join(ep,f"cam_{serial}","color.mp4"))
    out = []
    while True:
        ok, fr = cap.read()
        if not ok: break
        out.append(fr)
    cap.release()
    n = min(len(out), len(ts))
    return ts[:n], out[:n]

if __name__ == "__main__":
    ep, calib, out = sys.argv[1], sys.argv[2], sys.argv[3]
    os.makedirs(out, exist_ok=True)
    conf, K, E, T_world_base = setup(calib)
    s = conf.in_hand_serial[0]
    tt, pp = load_tcp_series(ep, s)
    ts, fr = frames(ep, s)
    print(f"wrist {s}: {len(fr)} frames, {len(tt)} tcp samples")
    H, W = fr[0].shape[:2]
    Ks = K[s].copy(); Ks[:2] *= (W / 1280.0)         # intrinsics are for 1280x720
    axes = np.array([[0,0,0,1],[0.1,0,0,1],[0,0.1,0,1],[0,0,0.1,1]]).T
    hits = 0
    for i in range(0, len(fr), max(1, len(fr)//6)):
        j = int(np.argmin(np.abs(tt - ts[i])))
        Ec = wrist_extrinsics(conf, T_world_base, pp[j])
        Q = Ec @ axes
        img = fr[i].copy()
        if (Q[2] > 0.05).all():
            uv = Ks @ Q[:3]; uv = (uv[:2]/uv[2]).T
            o = tuple(np.round(uv[0]).astype(int))
            if -W < o[0] < 2*W and -H < o[1] < 2*H:
                for k,c in [(1,(0,0,255)),(2,(0,255,0)),(3,(255,0,0))]:
                    cv2.arrowedLine(img, o, tuple(np.round(uv[k]).astype(int)), c, 2, tipLength=0.25)
                cv2.circle(img, o, 5, (0,255,255), 2); hits += 1
        cv2.putText(img, f"f{i} dt={ts[i]-tt[j]}ms", (8,22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        cv2.imwrite(os.path.join(out, f"wrist_{i:04d}.png"), img)
    print(f"marker drawn in {hits} sampled frames -> {out}")
    # how much does the wrist actually move?  (this is the baseline that 9 fixed cams lack)
    C = np.array([np.linalg.inv(wrist_extrinsics(conf, T_world_base, p))[:3,3] for p in pp])
    print(f"wrist camera path: {np.linalg.norm(np.diff(C,axis=0),axis=1).sum():.2f} m total, "
          f"bbox {np.round(C.max(0)-C.min(0),3)} m")
