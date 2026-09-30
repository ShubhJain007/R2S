#!/usr/bin/env python
"""Export the per-epoch mass / friction traces of the One-Shot stage-2 runs to results/oneshot_traces.json.

The traces live in RigidWorldModel/wandb (offline runs, not tracked). This writes the 11 runs behind the
v4 replication and the initialisation sweep into one small tracked file, so the figures in docs/figures
can be rebuilt without re-running 5 h of optimisation.

Usage:  python experiments/oneshot/export_traces.py
"""
import glob, json
from pathlib import Path
from wandb.sdk.internal.datastore import DataStore
from wandb.proto import wandb_internal_pb2 as pb

ROOT = Path(__file__).resolve().parents[2]
REPO = str(ROOT / "RigidWorldModel")   # clone: see third_party/UPSTREAM.md
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

# tag -> (initial mass, initial friction), from third_party/patches/RigidWorldModel_configs/<tag>.yaml
RUNS = {"drill_v4run1": (0.2, 0.2), "drill_v4run2": (0.2, 0.2), "drill_v4run3": (0.2, 0.2),
        "sweep_m05_a": (0.5, 0.2), "sweep_m05_b": (0.5, 0.2), "sweep_m09_a": (0.9, 0.2), "sweep_m09_b": (0.9, 0.2),
        "sweep_m13_a": (1.3, 0.2), "sweep_m13_b": (1.3, 0.2), "sweep_vlm_a": (0.9, 0.4), "sweep_vlm_b": (0.9, 0.4)}

out = dict(gt=dict(mass=GT_M, friction=GT_MU), runs={})
for tag, (m0, mu0) in RUNS.items():
    for d in sorted(glob.glob(f"{REPO}/wandb/offline-run-*")):
        wf = glob.glob(f"{d}/run-*.wandb")
        if wf and tag.encode() in open(wf[0], "rb").read():
            rows = history(wf[0])
            if len(rows) < 25: continue
            be = rows[-1]["best_epoch"]
            b = next((r for r in rows if r["epoch"] == be), rows[-1])
            out["runs"][tag] = dict(init_mass=m0, init_friction=mu0, best_epoch=be, mass=b["obj1_mass"], friction=b["obj1_fric"],
                                    epoch=[r["epoch"] for r in rows], mass_trace=[r["obj1_mass"] for r in rows],
                                    friction_trace=[r["obj1_fric"] for r in rows])
            print(f"{tag:<14} init {m0:.1f} kg -> best-epoch ({be:>2}) mass {b['obj1_mass']:.4f} kg, mu {b['obj1_fric']:.4f}")
            break
    else:
        print(f"{tag:<14} not found")

(ROOT / "results" / "oneshot_traces.json").write_text(json.dumps(out, indent=1))
print("\nsaved -> results/oneshot_traces.json")
