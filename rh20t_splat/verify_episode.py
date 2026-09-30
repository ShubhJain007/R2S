#!/usr/bin/env python
"""Stage-3 validation on EPISODE data (not calibration data).

All 917 cfg7 episodes reference one calibration timestamp, so the whole dataset
rests on the cameras not having moved since.  That is an assumption until it is
measured.  Here: draw the marker, the robot base, and the live TCP into fixed-camera
frames from an actual episode.  If the TCP tracks the gripper as the arm moves,
the calibration still holds for this episode.
"""
import os, sys, numpy as np, cv2
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rh20t_api"))
from wrist_poses import setup, load_tcp_series, frames
from rh20t_api.transforms import pose_array_quat_2_matrix

def draw(img, K, T_cam_X, label, L=0.12, col=(0,255,255)):
    pts=np.array([[0,0,0,1],[L,0,0,1],[0,L,0,1],[0,0,L,1]]).T
    Q=T_cam_X@pts
    if (Q[2]<=0.05).any(): return False
    uv=K@Q[:3]; uv=(uv[:2]/uv[2]).T
    o=tuple(np.round(uv[0]).astype(int)); H,W=img.shape[:2]
    if not(-W<o[0]<2*W and -H<o[1]<2*H): return False
    for i,c in [(1,(0,0,255)),(2,(0,255,0)),(3,(255,0,0))]:
        cv2.arrowedLine(img,o,tuple(np.round(uv[i]).astype(int)),c,2,tipLength=0.22)
    cv2.circle(img,o,5,col,2)
    cv2.putText(img,label,(o[0]+8,o[1]-8),cv2.FONT_HERSHEY_SIMPLEX,0.55,col,2)
    return True

if __name__=="__main__":
    ep, calib, out, cam = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
    os.makedirs(out, exist_ok=True)
    conf,K,E,T_world_base = setup(calib)
    tt,pp = load_tcp_series(ep, conf.in_hand_serial[0])
    ts,fr = frames(ep, cam)
    H,W = fr[0].shape[:2]
    Ks = K[cam].copy(); Ks[:2] *= (W/1280.0)
    print(f"{cam}: {len(fr)} frames {W}x{H}")
    for i in range(0, len(fr), max(1,len(fr)//5)):
        j = int(np.argmin(np.abs(tt-ts[i])))
        T_world_tcp = T_world_base @ pose_array_quat_2_matrix(np.asarray(pp[j],dtype=np.float64))
        img = fr[i].copy()
        draw(img, Ks, E[cam],                     "MARKER", 0.08, (255,255,0))
        draw(img, Ks, E[cam] @ T_world_base,      "BASE",   0.12, (0,255,255))
        draw(img, Ks, E[cam] @ T_world_tcp,       "TCP",    0.10, (0,255,0))
        cv2.putText(img,f"f{i}",(8,20),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),2)
        cv2.imwrite(os.path.join(out,f"ep_{cam}_{i:04d}.png"), img)
    print("wrote", out)
