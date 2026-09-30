"""Command line.

  python -m r2s_pipeline identify  REC.npz --mesh OBJ.obj [--mode wrench|joint_torque] [--robot iiwa.urdf --ee link_7]
                                   [--out result.json] [--sdf-in shipped.sdf --sdf-out patched.sdf]
  python -m r2s_pipeline selftest                     # no hardware: runs on the utiasSTARS Hammer with known truth
  python -m r2s_pipeline convert-utias OBJECT OUT.npz  # turn a utias .pkl into a recording (example data)
"""
import argparse, json, os, sys
import numpy as np
from . import io as IO
from .identify import identify


def cmd_identify(a):
    rec = IO.load_recording(a.recording)
    info = IO.validate(rec, a.mode)
    print(f"recording: {info['n']} samples, {info['duration_s']:.1f} s @ {info['rate_hz']:.0f} Hz, mode={a.mode}")
    T_so = rec.get("T_so") if a.T_so is None else np.loadtxt(a.T_so).reshape(4, 4)
    base = IO.load_recording(a.baseline) if a.baseline else None
    if base is not None: IO.validate(base, a.mode)
    res = identify(rec, mesh_path=a.mesh, T_so=T_so, mode=a.mode, robot_file=a.robot, ee_frame=a.ee,
                   mesh_scale=a.mesh_scale, fc_hz=a.fc, use_sdp=a.sdp, wrench_frame=a.wrench_frame,
                   wrench_sign=a.wrench_sign, torque_sign=a.torque_sign, baseline=base)
    print(f"\nRESULT  mass {res['mass']:.4f} kg | CoM (sensor frame) {np.round(res['com_sensor_frame'],4)}")
    if "inertia_object_frame_about_com" in res:
        I = np.asarray(res["inertia_object_frame_about_com"])
        print(f"        inertia diag (object frame, about CoM) {np.round(np.diag(I),6)}  gyration {res['diagnostics']['gyration_ratio']:.2f}")
    if a.out: IO.write_json(a.out, res); print(f"-> {a.out}")
    if a.mesh:
        print("\nSDF inertial block:\n" + IO.sdf_inertial_block(res))
        print("\nURDF inertial block:\n" + IO.urdf_inertial_block(res))
    if a.sdf_in and a.sdf_out:
        IO.patch_sdf(a.sdf_in, a.sdf_out, res); print(f"-> {a.sdf_out}")


def utias_to_recording(obj):
    """Example converter: utiasSTARS synthetic .pkl -> recording dict (wrench mode)."""
    import pickle, yaml
    class Dummy:
        def __setstate__(self, s): self.__dict__.update(s if isinstance(s, dict) else {"_s": s})
    class U(pickle.Unpickler):
        def find_class(self, mod, name):
            try: return super().find_class(mod, name)
            except Exception: return Dummy
    D = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "utias-inertial", "data")   # clone: see third_party/UPSTREAM.md
    d = U(open(f"{D}/Simulations/{obj}.pkl", "rb")).load()
    A = lambda v: np.asarray(v.A if hasattr(v, "A") else v).reshape(-1)
    rec = dict(t=np.array([s.timestamp for s in d], float),
               T_ws=np.array([np.asarray(s.ft_pose.A) for s in d]),
               wrench=np.array([A(s.ft_value) for s in d]),
               vel=np.array([A(s.ft_vel) for s in d]), acc=np.array([A(s.ft_acc) for s in d]))
    Tobj = np.asarray(d[0].object_pose.A); rec["T_so"] = np.linalg.inv(rec["T_ws"][0]) @ Tobj
    if not np.all(np.diff(rec["t"]) > 0): rec["t"] = np.arange(len(d)) * 0.01
    y = yaml.safe_load(open(f"{D}/Workshop Tools Dataset/{obj.replace('_', ' ')}/Inertia.yaml"))["OBJECT"]
    gt = dict(mass=y["MASS"], com=np.array(y["COM_WRT_MESH"]), I=np.array(y["INERTIA_WRT_COM"]))
    mesh = f"{D}/Workshop Tools Dataset/{obj.replace('_', ' ')}/mesh.ply"
    return rec, gt, mesh


def cmd_selftest(a):
    objs = a.objects or ["Hammer", "Pliers", "Screwdriver", "Vise_Grip", "Box_Wrench"]
    print("self-test on utiasSTARS synthetic data with known ground truth (wrench mode)\n")
    ok = True
    for obj in objs:
        rec, gt, mesh = utias_to_recording(obj)
        print(f"== {obj} ==")
        res = identify(rec, mesh_path=mesh, T_so=rec["T_so"], mode="wrench", mesh_scale=0.001, verbose=True)
        m_err = abs(res["mass"] - gt["mass"]) / gt["mass"] * 100
        com_err = np.linalg.norm(np.asarray(res["com_object_frame"]) - gt["com"]) * 1000
        I = np.asarray(res["inertia_object_frame_about_com"]); I_err = np.linalg.norm(I - gt["I"]) / np.linalg.norm(gt["I"]) * 100
        status = "PASS" if m_err < 2 and com_err < 60 else "FAIL"; ok &= status == "PASS"
        print(f"  vs truth: mass {m_err:.2f}%  CoM {com_err:.1f} mm  geometry-inertia {I_err:.1f}%  -> {status}\n")
    print("SELF-TEST", "PASSED" if ok else "FAILED"); sys.exit(0 if ok else 1)


def cmd_convert(a):
    rec, gt, mesh = utias_to_recording(a.object)
    np.savez(a.out, **rec); print(f"-> {a.out}   (mesh: {mesh}, mesh units mm -> use --mesh-scale 0.001)")


def main(argv=None):
    p = argparse.ArgumentParser(prog="r2s_pipeline", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = p.add_subparsers(dest="cmd", required=True)
    q = sp.add_parser("identify"); q.add_argument("recording"); q.add_argument("--mesh"); q.add_argument("--mesh-scale", type=float, default=1.0)
    q.add_argument("--mode", choices=["wrench", "joint_torque"], default="wrench"); q.add_argument("--robot"); q.add_argument("--ee")
    q.add_argument("--T-so", dest="T_so", help="4x4 text file: object-mesh frame in sensor/EE frame (overrides recording)")
    q.add_argument("--fc", type=float, default=5.0, help="low-pass cutoff Hz for derived velocities/accelerations")
    q.add_argument("--sdp", action="store_true", help="physically-consistent SDP fit instead of least squares")
    q.add_argument("--wrench-frame", choices=["auto", "sensor", "world"], default="auto"); q.add_argument("--wrench-sign", type=int, default=0)
    q.add_argument("--torque-sign", type=int, default=1); q.add_argument("--baseline", help="recording of the same setup with NO object (empty gripper) to subtract"); q.add_argument("--out"); q.add_argument("--sdf-in"); q.add_argument("--sdf-out")
    q.set_defaults(fn=cmd_identify)
    s = sp.add_parser("selftest"); s.add_argument("objects", nargs="*"); s.set_defaults(fn=cmd_selftest)
    c = sp.add_parser("convert-utias"); c.add_argument("object"); c.add_argument("out"); c.set_defaults(fn=cmd_convert)
    a = p.parse_args(argv); a.fn(a)


if __name__ == "__main__":
    main()
