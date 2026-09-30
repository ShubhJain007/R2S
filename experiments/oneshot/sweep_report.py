#!/usr/bin/env python
"""Initialization-sensitivity sweep on One-Shot (arXiv 2412.00259 v4) stage 2, Drill.

Tests RigPI's core claim -- that a good VLM-derived seed fixes accuracy and stability --
using One-Shot's own code. Only init_masses/init_frictions differ between conditions;
all weights, LRs, epochs and the optimizable-parameter set are the shipped v4 values.

Ground truth: mass 0.8 kg, friction 0.5.
"""
import glob, json, math, statistics as st
from pathlib import Path
from wandb.sdk.internal.datastore import DataStore
from wandb.proto import wandb_internal_pb2 as pb

REPO = str(Path(__file__).resolve().parents[2] / "RigidWorldModel")   # clone: see third_party/UPSTREAM.md
GT_M, GT_MU = 0.8, 0.5

def history(f):
    ds = DataStore(); ds.open_for_scan(f); rows = []
    while True:
        try: d = ds.scan_data()
        except Exception: break
        if d is None: break
        r = pb.Record(); r.ParseFromString(d)
        if r.WhichOneof("record_type") == "history":
            h = {}
            for i in r.history.item:
                k = i.nested_key[0] if i.nested_key else i.key
                try: h[k] = json.loads(i.value_json)
                except Exception: pass
            rows.append(h)
    rows.sort(key=lambda x: x.get("epoch", 0)); return rows

def result(tag):
    for d in sorted(glob.glob(f"{REPO}/wandb/offline-run-*")):
        wf = glob.glob(f"{d}/run-*.wandb")
        if wf and tag.encode() in open(wf[0], "rb").read():
            rows = history(wf[0])
            if not rows: return None
            if len(rows) < 25: return None   # run still in progress -- partial history is misleading
            be = rows[-1]["best_epoch"]
            b = next((r for r in rows if r["epoch"] == be), rows[-1])
            return dict(m=b["obj1_mass"], mu=b["obj1_fric"], n=len(rows),
                        rot=math.degrees(b["obj_rot_loss"]), pos=b["obj_pos_loss"]*1000)
    return None

CONDS = [("baseline  m=0.2 mu=0.2", ["drill_v4run1","drill_v4run2","drill_v4run3"]),
         ("seed      m=0.5 mu=0.2", ["sweep_m05_a","sweep_m05_b"]),
         ("seed      m=0.9 mu=0.2", ["sweep_m09_a","sweep_m09_b"]),
         ("seed      m=1.3 mu=0.2", ["sweep_m13_a","sweep_m13_b"]),
         ("VLM full  m=0.9 mu=0.4", ["sweep_vlm_a","sweep_vlm_b"])]

print("Initialization sensitivity -- One-Shot stage 2, Drill (GT: mass 0.8 kg, mu 0.5)\n")
print(f"{'condition':<24}{'n':>3}{'mass':>9}{'|err| kg':>10}{'err %':>8}{'mu':>8}{'|err|':>8}{'rot deg':>9}")
print("-"*79)
for label, tags in CONDS:
    rs = [r for r in (result(t) for t in tags) if r]
    if not rs: print(f"{label:<24}{'--- pending ---':>50}"); continue
    me = [abs(r["m"]-GT_M) for r in rs]; ue = [abs(r["mu"]-GT_MU) for r in rs]
    sd = f"±{st.stdev(me):.3f}" if len(me) > 1 else ""
    print(f"{label:<24}{len(rs):>3}{st.mean(r['m'] for r in rs):>9.4f}"
          f"{st.mean(me):>10.4f}{st.mean(me)/GT_M*100:>7.1f}%{st.mean(r['mu'] for r in rs):>8.4f}"
          f"{st.mean(ue):>8.4f}{st.mean(r['rot'] for r in rs):>9.2f}  {sd}")
print("\nPaper v4 'Ours' (9-object avg): mass err 0.0728 kg | mu err 0.106 | rot 16.7 deg")
print("RigPI (5 objects):              mass err 1.5-7.2%")
