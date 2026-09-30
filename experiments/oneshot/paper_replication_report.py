#!/usr/bin/env python
"""Replication of arXiv 2412.00259 Table I, Drill row, using the authors' code and data.

Protocol (paper Sec. V-A, quoted): mass known; COM at geometry centre of the initial
guess; inertia from the bounding-box box formula; three optimisation trials with random
initial friction in [0.1, 1]; the average across trials is reported.

Per trial we take the epoch the code itself selects (best training loss), then average.
Final-epoch values are also shown for transparency. Ground-truth friction = 0.5.
"""
import glob, json, math, os, yaml
from pathlib import Path
from wandb.sdk.internal.datastore import DataStore
from wandb.proto import wandb_internal_pb2 as pb

REPO = str(Path(__file__).resolve().parents[2] / "RigidWorldModel")   # clone: see third_party/UPSTREAM.md
GT_MU = 0.5
PAPER = {"mu_err": 0.0970, "pos_mm": 15.1, "rot_deg": 3.85}   # Table I, Drill, Train Dynamics Error

def history(wandb_file):
    ds = DataStore(); ds.open_for_scan(wandb_file); rows = []
    while True:
        try: data = ds.scan_data()
        except Exception: break
        if data is None: break
        rec = pb.Record(); rec.ParseFromString(data)
        if rec.WhichOneof("record_type") == "history":
            r = {}
            for i in rec.history.item:
                k = i.nested_key[0] if i.nested_key else i.key
                try: r[k] = json.loads(i.value_json)
                except Exception: pass
            rows.append(r)
    rows.sort(key=lambda r: r.get("epoch", 0)); return rows

def run_dir_for(task_name):
    for d in sorted(glob.glob(f"{REPO}/wandb/offline-run-*")):
        meta = f"{d}/files/wandb-metadata.json"
        cfg = f"{d}/files/config.yaml"
        for f in (meta, cfg):
            if os.path.exists(f) and task_name in open(f).read():
                return d
    # fallback: match by the config the run was launched with (stored in the .wandb stream)
    for d in sorted(glob.glob(f"{REPO}/wandb/offline-run-*")):
        wf = glob.glob(f"{d}/run-*.wandb")
        if wf and task_name.encode() in open(wf[0], "rb").read():
            return d
    return None

trials = []
for i in (1, 2, 3):
    cfg = yaml.safe_load(open(f"{REPO}/configs/drill_s2_paper{i}.yaml"))
    d = run_dir_for(cfg["task_name_stage_2"])
    if d is None:
        print(f"trial {i}: no wandb run found yet"); continue
    rows = history(glob.glob(f"{d}/run-*.wandb")[0])
    if not rows:
        print(f"trial {i}: no history yet"); continue
    best_ep = rows[-1]["best_epoch"]
    best = next(r for r in rows if r["epoch"] == best_ep)
    fin = rows[-1]
    def m(r): return dict(mu=r["obj1_fric"], mu_err=abs(r["obj1_fric"] - GT_MU),
                          pos_mm=r["obj_pos_loss"] * 1000.0, rot_deg=math.degrees(r["obj_rot_loss"]))
    trials.append(dict(i=i, init=cfg["init_frictions"][1], epochs=len(rows), best_ep=best_ep, best=m(best), final=m(fin)))

print("Trial results (Drill). mass fixed at 0.8 kg (known), COM/inertia from geometry, friction-only optimisation.\n")
print(f"{'trial':>5}{'init mu':>9}{'epochs':>7}{'best ep':>8} | {'mu':>7}{'|mu err|':>9}{'pos mm':>8}{'rot deg':>8} | {'final mu':>9}{'final err':>10}")
print("-" * 92)
for t in trials:
    b, f = t["best"], t["final"]
    print(f"{t['i']:>5}{t['init']:>9.3f}{t['epochs']:>7}{t['best_ep']:>8} | {b['mu']:>7.4f}{b['mu_err']:>9.4f}{b['pos_mm']:>8.2f}{b['rot_deg']:>8.2f} | {f['mu']:>9.4f}{f['mu_err']:>10.4f}")

if len(trials) == 3:
    avg = {k: sum(t["best"][k] for t in trials) / 3 for k in ("mu_err", "pos_mm", "rot_deg")}
    print("\n" + "=" * 92)
    print("Table I, Drill row, 'Train Dynamics Error' -- paper vs. this replication (mean of 3 trials, best-loss epoch)")
    print("=" * 92)
    print(f"{'metric':<22}{'paper':>10}{'ours':>10}")
    for k, lbl in (("mu_err", "mu error (abs)"), ("pos_mm", "pos error (mm)"), ("rot_deg", "rot error (deg)")):
        print(f"{lbl:<22}{PAPER[k]:>10.4f}{avg[k]:>10.4f}")
else:
    print(f"\n{len(trials)}/3 trials complete -- averages reported when all three finish.")
