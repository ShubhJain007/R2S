"""Decode RH20T's depth videos.

Encoding (from rh20t_api/extract.py:convert_depth): each frame is DOUBLE height and
greyscale; the top half holds the low byte and the bottom half the high byte, so
    depth_mm = high*256 + low
and L515 units (serials starting with 'f') are a further x4.  Getting this wrong
yields depth that looks plausible but is off by a factor or wrapped -- so it is
checked against cross-camera geometry rather than eyeballed.
"""
import os, numpy as np, cv2

def read_depth_video(path, serial, max_frames=None):
    cap = cv2.VideoCapture(path)
    is_l515 = serial.startswith("f")
    out = []
    while True:
        ok, fr = cap.read()
        if not ok: break
        g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
        h = g.shape[0] // 2
        lo = g[:h].astype(np.int32); hi = g[h:].astype(np.int32)
        d = (hi * 256 + lo).astype(np.uint16)
        if is_l515: d = (d.astype(np.int32) * 4).astype(np.uint16)
        out.append(d)
        if max_frames and len(out) >= max_frames: break
    cap.release()
    return out

def depth_at(ep, serial, frame_idx):
    p = os.path.join(ep, f"cam_{serial}", "depth.mp4")
    if not os.path.exists(p): return None
    d = read_depth_video(p, serial, max_frames=frame_idx+1)
    return d[frame_idx] if len(d) > frame_idx else None
