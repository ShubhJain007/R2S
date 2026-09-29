#!/usr/bin/env python
"""Phase-0 readout for the One-Shot Real-to-Sim sim round-trip (Drill object).

Ground truth, as documented in the comments of configs/drill_stage_2.yaml:
    mass     = 0.8 kg   (optimisation initialised at 0.2)
    friction = 0.5      (optimisation initialised at 0.2)

NOTE on inertia. The repo does not ship a ground-truth inertia tensor, and the
simulator does not treat inertia as a free parameter. It uses a box approximation
(diffworld/world/world.py:_custom_get_ang_inertia):

    I_ii = mass * inertia_scale_i * (sum of squared side lengths) / 12

so inertia is a *derived* quantity: it follows from the fitted mass, the fitted
`inertia_scale` (clamped to [0.5, 1.5], 1.0 = pure geometry-derived), and the
side lengths of the reconstructed geometry. A true "inertia error vs ground
truth" is therefore not computable from what ships. What we report instead is
the relative deviation of the (mass * inertia_scale) factor, which equals the
relative inertia error *on the assumption that the geometry's side lengths were
recovered correctly*. That assumption is stated, not verified, so treat this as
a proxy and not as the paper's inertia number.

Usage:  python phase0_report.py [checkpoint_dir]
"""
import sys, os, torch

GT   = {"mass": 0.8, "fric_coeff": 0.5}
INIT = {"mass": 0.2, "fric_coeff": 0.2}

ckpt_dir = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/kneepolean/shubhj/R2S/RigidWorldModel/checkpoints/drill2"
fn = os.path.join(ckpt_dir, "obj_1_full.pth")
if not os.path.exists(fn):
    sys.exit(f"no checkpoint at {fn}\n(stage 2 writes it on its best-loss epoch)")

d = torch.load(fn, map_location="cpu")
num = lambda x: x.item() if torch.is_tensor(x) and x.numel() == 1 else x
arr = lambda x: x.detach().cpu().numpy().ravel() if torch.is_tensor(x) else x

mass = num(d["mass"]); fric = num(d["fric_coeff"])
com  = arr(d["com"]);  scale = arr(d["inertia_scale"]); sides = arr(d["side_lengths"])

print(f"checkpoint: {fn}\n")
print(f"{'param':<12}{'init':>9}{'recovered':>12}{'truth':>9}{'err':>9}")
print("-" * 51)
for k, gt in GT.items():
    got = mass if k == "mass" else fric
    print(f"{k:<12}{INIT[k]:>9.3f}{got:>12.4f}{gt:>9.3f}{abs(got-gt)/gt*100:>8.1f}%")

print(f"\ncom offset (m)  : {com}")
print(f"inertia_scale   : {scale}  (1.0 = geometry-derived; clamped to [0.5, 1.5])")
print(f"side_lengths (m): {sides}")

mass_err = abs(mass - GT["mass"]) / GT["mass"] * 100
fric_err = abs(fric - GT["fric_coeff"]) / GT["fric_coeff"] * 100
inertia_proxy = max(abs(mass * s - GT["mass"]) / GT["mass"] * 100 for s in scale)

print("\n" + "=" * 51)
print("PHASE 0 CRITERION (Setup_guide.md sec.3)")
print("=" * 51)
print(f"mass error              {mass_err:>7.1f}%   {'PASS' if mass_err <= 5 else 'FAIL'}   (gate: within a few %)")
print(f"inertia proxy, worst ax {inertia_proxy:>7.1f}%   {'PASS' if inertia_proxy <= 10 else 'FAIL'}   (gate: ~10%; proxy - see docstring)")
print(f"friction error          {fric_err:>7.1f}%   n/a    (not a gate; reported for information)")

gate = mass_err <= 5 and inertia_proxy <= 10
print("\nPHASE 0: " + ("PASS - proceed to Phase 2." if gate else
      "FAIL - debug the install/optimisation loop before going further."))
