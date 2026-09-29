#!/usr/bin/env python
"""Replication of arXiv 2412.00259 v4 (RA-L) Table I, "Ours" row, Drill object only.

Setup = the repo's shipped configs/drill_stage_2.yaml, unmodified except save-dir and
video logging. v4 states: "The physical parameters we optimize for include the mass,
surface coefficient of friction, the center of mass, and the rotational inertia" and
"the surface coefficient of friction is initialized at 0.2 and the mass is initialized
at 0.2 kg" -- which is exactly the shipped config.

Ground truth from configs/drill_stage_2.yaml comments: mass 0.8 kg, friction 0.5.
Paper Table I numbers are AVERAGES OVER 9 OBJECTS; no per-object values are published,
and only the Drill ships with the repo. The comparison below is therefore
our-Drill vs their-9-object-average, which is the closest possible, not like-for-like.
"""
import glob, json, math, statistics as st
from wandb.sdk.internal.datastore import DataStore
from wandb.proto import wandb_internal_pb2 as pb

REPO = "/home/kneepolean/shubhj/R2S/RigidWorldModel"
GT_M, GT_MU = 0.8, 0.5
PAPER = {"mass": 0.0728, "mu": 0.106, "pos": 15.5, "rot": 16.7}

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

runs = []
for s in (1, 2, 3):
    tag = f"drill_v4run{s}"
    d = next((d for d in sorted(glob.glob(f"{REPO}/wandb/offline-run-*"))
              if glob.glob(f"{d}/run-*.wandb") and tag.encode() in open(glob.glob(f"{d}/run-*.wandb")[0], "rb").read()), None)
    if not d: print(f"run {s}: not found"); continue
    rows = history(glob.glob(f"{d}/run-*.wandb")[0])
    if not rows: print(f"run {s}: no history"); continue
    be = rows[-1]["best_epoch"]; b = next(r for r in rows if r["epoch"] == be)
    runs.append(dict(s=s, be=be, n=len(rows), m=b["obj1_mass"], mu=b["obj1_fric"],
                     pos=b["obj_pos_loss"]*1000, rot=math.degrees(b["obj_rot_loss"]),
                     merr=abs(b["obj1_mass"]-GT_M), muerr=abs(b["obj1_fric"]-GT_MU)))

print("arXiv 2412.00259 v4 (RA-L) replication -- Drill, shipped config, 25 epochs\n")
print(f"{'run':>4}{'bestep':>8}{'mass':>8}{'|err| kg':>10}{'mu':>8}{'|err|':>8}{'pos mm':>9}{'rot deg':>9}")
print("-"*64)
for r in runs:
    print(f"{r['s']:>4}{r['be']:>8}{r['m']:>8.4f}{r['merr']:>10.4f}{r['mu']:>8.4f}{r['muerr']:>8.4f}{r['pos']:>9.2f}{r['rot']:>9.2f}")

if runs:
    mean = lambda k: st.mean(r[k] for r in runs)
    sd = lambda k: st.stdev([r[k] for r in runs]) if len(runs) > 1 else 0.0
    print("-"*64)
    print(f"{'mean':>4}{'':>8}{mean('m'):>8.4f}{mean('merr'):>10.4f}{mean('mu'):>8.4f}{mean('muerr'):>8.4f}{mean('pos'):>9.2f}{mean('rot'):>9.2f}")
    print(f"{'sd':>4}{'':>8}{sd('m'):>8.4f}{sd('merr'):>10.4f}{sd('mu'):>8.4f}{sd('muerr'):>8.4f}{sd('pos'):>9.2f}{sd('rot'):>9.2f}")
    print("\n" + "="*64)
    print("Table I, 'Ours' row -- paper (9-object avg) vs this replication (Drill, n=%d)" % len(runs))
    print("="*64)
    print(f"{'metric':<20}{'paper':>12}{'ours (mean)':>14}")
    for k, lbl, ours in (("mass","mass error (kg)",mean('merr')), ("mu","mu error",mean('muerr')),
                         ("pos","pos error (mm)",mean('pos')), ("rot","rot error (deg)",mean('rot'))):
        print(f"{lbl:<20}{PAPER[k]:>12.4f}{ours:>14.4f}")
    print("\nNOT comparable like-for-like: paper averages 9 objects, we have 1 (only Drill ships).")
    print("Paper reports no per-object values and states no epoch/trial count.")
