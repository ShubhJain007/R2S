#!/usr/bin/env python
"""Decisive test of what extrinsics[serial] actually is.

Only two hypotheses matter:
    H1:  E = T_cam_world   (world -> camera)   => cam_to_world = inv(E)
    H2:  E = T_world_cam   (camera -> world)   => cam_to_world = E

Three independent checks, because one number can lie:
  (a) cross-camera cloud consistency: fuse every camera's depth into world and
      measure nearest-neighbour distance between clouds from DIFFERENT cameras.
      Wrong hypothesis -> clouds land in different places -> large distance.
  (b) table planarity: the dominant plane of the fused cloud. Wrong hypothesis
      smears one plane into many -> large RMS.
  (c) world origin projected into each image, drawn as axes.  World = Aruco
      marker frame, so under the right hypothesis the axes sit ON the marker.
"""
import os, sys, itertools
import numpy as np, cv2, open3d as o3d

def load(cd):
    I = np.load(os.path.join(cd,"intrinsics.npy"), allow_pickle=True).item()
    E = np.load(os.path.join(cd,"extrinsics.npy"), allow_pickle=True).item()
    E = {k:(np.asarray(v)[0] if np.asarray(v).ndim==3 else np.asarray(v)) for k,v in E.items()}
    return {k:np.asarray(v)[:3,:3] for k,v in I.items()}, E

def cloud(cd, s, K, cam_to_world, stride=6, zmax=2.5):
    d = np.load(os.path.join(cd,"imgs",f"cam_{s}_d.npy")).astype(np.float64)/1000.0
    c = cv2.imread(os.path.join(cd,"imgs",f"cam_{s}_c.png"))
    H,W = d.shape
    v,u = np.mgrid[0:H:stride,0:W:stride]
    z = d[::stride,::stride]; col = c[::stride,::stride][:,:,::-1]/255.0
    m = (z>0.2)&(z<zmax)
    u,v,z,col = u[m],v[m],z[m],col[m]
    x=(u-K[0,2])*z/K[0,0]; y=(v-K[1,2])*z/K[1,1]
    P = np.stack([x,y,z,np.ones_like(z)])
    return (cam_to_world @ P)[:3].T, col

def nn_median(A, B):
    t = o3d.geometry.PointCloud(); t.points = o3d.utility.Vector3dVector(B)
    kd = o3d.geometry.KDTreeFlann(t)
    idx = np.random.default_rng(0).choice(len(A), min(3000,len(A)), replace=False)
    ds = []
    for p in A[idx]:
        _,_,d2 = kd.search_knn_vector_3d(p,1)
        ds.append(np.sqrt(d2[0]))
    return float(np.median(ds))*1000.0

if __name__ == "__main__":
    cd = sys.argv[1]; out = sys.argv[2]
    os.makedirs(out, exist_ok=True)
    K,E = load(cd)
    serials = [s for s in E if os.path.exists(os.path.join(cd,"imgs",f"cam_{s}_d.npy"))]
    results = {}
    for name, f in [("H1: E = T_cam_world (cam_to_world = inv(E))", lambda M: np.linalg.inv(M)),
                    ("H2: E = T_world_cam (cam_to_world = E)",      lambda M: M)]:
        clouds = {s: cloud(cd,s,K[s],f(E[s])) for s in serials}
        pairs = list(itertools.combinations(serials,2))[:12]
        nn = [nn_median(clouds[a][0], clouds[b][0]) for a,b in pairs]
        allP = np.concatenate([clouds[s][0] for s in serials])
        allC = np.concatenate([clouds[s][1] for s in serials])
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(allP); pc.colors = o3d.utility.Vector3dVector(allC)
        pl, inl = pc.segment_plane(0.01, 3, 400)
        n = np.array(pl[:3]); rms = float(np.sqrt(np.mean((allP@n + pl[3])**2)))*1000
        results[name] = (float(np.median(nn)), rms, len(inl)/len(allP), pc)
        print(f"{name}\n    cross-camera NN median : {np.median(nn):8.1f} mm"
              f"\n    dominant-plane RMS     : {rms:8.1f} mm"
              f"\n    plane inlier fraction  : {len(inl)/len(allP):8.2%}\n")
    best = min(results, key=lambda k: results[k][0])
    print(f"=> {best}")
    o3d.io.write_point_cloud(os.path.join(out,"fused_world.ply"), results[best][3])
    print(f"   wrote {out}/fused_world.ply  ({len(results[best][3].points)} pts)")
    # (c) draw the world origin + axes into three cameras, under the winning hypothesis
    inv = "inv(E)" in best
    axes = np.array([[0,0,0,1],[0.1,0,0,1],[0,0.1,0,1],[0,0,0.1,1]]).T
    for s in serials[:4]:
        M = E[s] if inv else np.linalg.inv(E[s])      # world -> camera
        Q = M @ axes
        if (Q[2] <= 0.05).any(): continue
        uv = (K[s] @ Q[:3]); uv = (uv[:2]/uv[2]).T
        img = cv2.imread(os.path.join(cd,"imgs",f"cam_{s}_c.png")).copy()
        o = tuple(np.round(uv[0]).astype(int))
        for i,col in [(1,(0,0,255)),(2,(0,255,0)),(3,(255,0,0))]:
            cv2.arrowedLine(img, o, tuple(np.round(uv[i]).astype(int)), col, 3, tipLength=0.25)
        cv2.circle(img, o, 7, (0,255,255), 2)
        cv2.putText(img, f"world origin {s}", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
        cv2.imwrite(os.path.join(out,f"origin_{s}.png"), img)
    print("   wrote origin_*.png")
