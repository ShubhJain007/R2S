#!/usr/bin/env python
"""Build the README figures from the tracked result files in results/.

Every chart is written twice, `<name>-light.svg` and `<name>-dark.svg`, with a transparent background so
it sits on GitHub's light and dark themes. Nothing here re-runs an experiment: the inputs are the JSON
files the experiment scripts saved (plus docs/figures/tools/*.png from render_tools.py).

Usage (env with matplotlib + trimesh, e.g. `s2s`):
    python docs/figures/make_figures.py            # all figures
    python docs/figures/make_figures.py hybrid     # only figures whose name contains "hybrid"
"""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.path, matplotlib.lines
from matplotlib import font_manager
from matplotlib.patches import PathPatch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
R = lambda *p: os.path.join(ROOT, "results", *p)
J = lambda *p: json.load(open(R(*p)))

# Categorical slots validated for both surfaces (adjacent pairs, and all pairs for the first three):
# light on #ffffff, dark on #0d1117. Order is fixed; a series keeps its slot in every figure.
THEMES = {
    "light": dict(ink="#1f2328", sub="#59636e", muted="#818b98", grid="#e4e8ec", axis="#c4cbd3", surface="#ffffff", band="#f1f4f7",
                  s=["#2a78d6", "#eb6834", "#1baf7a", "#eda100"], ctx="#a9b1bb"),
    "dark": dict(ink="#f0f6fc", sub="#9198a1", muted="#7d8590", grid="#21262d", axis="#3d444d", surface="#0d1117", band="#161b22",
                 s=["#3987e5", "#d95926", "#199e70", "#c98500"], ctx="#6e7681"),
}
FONT = next((f for f in ("Lato", "Liberation Sans", "DejaVu Sans") if f in {x.name for x in font_manager.fontManager.ttflist}), "sans-serif")
plt.rcParams.update({"font.family": FONT, "font.size": 10, "svg.fonttype": "path", "svg.hashsalt": "r2s",
                     "axes.spines.top": False, "axes.spines.right": False, "axes.spines.left": False,
                     "xtick.major.size": 0, "ytick.major.size": 0, "xtick.major.pad": 6, "ytick.major.pad": 6})
W = 8.6                                                   # figure width, inches; shown at 860 px in the README


def style(ax, t, grid="x"):
    ax.set_facecolor("none")
    ax.spines["bottom"].set_color(t["axis"]); ax.spines["bottom"].set_linewidth(1)
    ax.tick_params(colors=t["muted"], labelsize=9)
    if grid: ax.grid(axis=grid, color=t["grid"], linewidth=1); ax.set_axisbelow(True)


def title(fig, t, head, sub=None, x=0.012, y=0.975):
    fig.text(x, y, head, color=t["ink"], fontsize=12.5, fontweight="bold", va="top")
    if sub: fig.text(x, y - 0.26 / fig.get_figheight(), sub, color=t["sub"], fontsize=9.5, va="top")


def legend(fig, t, items, x, y, dx=None):
    """One row of `(colour, label)` swatches; text stays in ink, the dot carries identity."""
    for c, lab in items:
        fig.text(x, y, "●", color=c, fontsize=11, va="center")
        tx = fig.text(x + 0.018, y, lab, color=t["sub"], fontsize=9.5, va="center")
        if dx is None:
            fig.canvas.draw(); x = tx.get_window_extent().x1 / fig.bbox.width + 0.03
        else: x += dx


def save(fig, name, theme):
    out = os.path.join(HERE, f"{name}-{theme}.svg")
    fig.savefig(out, transparent=True, metadata={"Date": None, "Creator": "docs/figures/make_figures.py"})
    plt.close(fig); print("wrote", os.path.relpath(out, ROOT))


def swarm(u, r):
    """Greedy beeswarm: `u` are positions in points along the axis, returns offsets across it (points)."""
    order = np.argsort(u); off = np.zeros(len(u)); placed = []
    for i in order:
        for k in range(200):
            c = (k + 1) // 2 * (1 if k % 2 else -1) * r * 0.9
            if all((u[i] - pu) ** 2 + (c - pc) ** 2 >= (2 * r) ** 2 * 0.98 for pu, pc in placed):
                off[i] = c; placed.append((u[i], c)); break
    return off


def strip_rows(ax, t, rows, xlim, row_gap=1.0, r_pt=4.2):
    """Log-x strip plot. rows: list of (values, colour). One dot per object, jittered as a beeswarm."""
    ax.set_xscale("log"); ax.set_xlim(*xlim); ax.set_ylim(len(rows) * row_gap - row_gap / 2, -row_gap / 2)
    ax.figure.canvas.draw()
    bb = ax.get_window_extent(); pt_per_dec = bb.width * 72 / ax.figure.dpi / (np.log10(xlim[1]) - np.log10(xlim[0]))
    y_per_pt = (len(rows) * row_gap) / (bb.height * 72 / ax.figure.dpi)
    for k, (v, c) in enumerate(rows):
        v = np.clip(np.asarray(v, float), xlim[0], xlim[1])
        off = swarm(np.log10(v) * pt_per_dec, r_pt + 0.75)
        ax.scatter(v, k * row_gap + off * y_per_pt, s=(2 * r_pt) ** 2, color=c, edgecolors=t["surface"], linewidths=1.5, zorder=3, clip_on=False)
    ax.set_yticks([])


def median_mark(ax, t, k, v, text, row_gap=1.0, side="right", dy=-0.36):
    m = float(np.median(v))
    ax.plot([m, m], [k * row_gap - 0.30, k * row_gap + 0.30], color=t["ink"], linewidth=1.5, zorder=4, solid_capstyle="round")
    ax.annotate(text, (m, k * row_gap + dy), color=t["ink"], fontsize=9.5, fontweight="bold", va="center",
                ha="left" if side == "right" else "right", xytext=(6 if side == "right" else -6, 0), textcoords="offset points", zorder=5)


def pct(v):
    return f"{v:,.0f}%" if v >= 10 else (f"{v:.1f}%" if v >= 1 else f"{v:.2f}%")


# ------------------------------------------------------------------ 1. what is identifiable
def fig_identifiability(name, theme):
    t = THEMES[theme]
    g = J("utias_gentle_motion.json"); geo = J("utias_geometry_inertia.json")
    clean = [v for k, v in g.items() if k.startswith("CLEAN")]
    m_err = [r["ls"]["m_err"] for r in clean]; I_mot = [r["ls"]["I_err"] for r in clean]; I_geo = [r["errF"] for r in geo]
    fig = plt.figure(figsize=(W, 4.0)); ax = fig.add_axes([0.205, 0.13, 0.745, 0.56]); style(ax, t)
    xlim = (0.008, 1.2e5)
    ax.axvspan(xlim[0], 10, color=t["band"], zorder=0, linewidth=0)
    strip_rows(ax, t, [(m_err, t["s"][0]), (I_mot, t["s"][0]), (I_geo, t["s"][1])], xlim)
    ax.set_xticks([0.01, 0.1, 1, 10, 100, 1e3, 1e4, 1e5]); ax.set_xticklabels(["0.01%", "0.1%", "1%", "10%", "100%", "1,000%", "10,000%", "100,000%"])
    ax.xaxis.set_minor_locator(matplotlib.ticker.NullLocator())
    ax.set_xlabel("error against ground truth (log scale)", color=t["sub"], fontsize=9.5, labelpad=6)
    ax.text(10, -0.56, "within 10% ", color=t["muted"], fontsize=9, ha="right", va="bottom")
    for k, (v, lab) in enumerate([(m_err, "median {}"), (I_mot, "median {}"), (I_geo, "median {}")]):
        median_mark(ax, t, k, v, lab.format(pct(np.median(v))), side="right" if k == 0 else "left")
    for k, (a, b) in enumerate([("Mass", "from motion"), ("Inertia", "from motion"), ("Inertia", "from geometry")]):
        fig.text(0.012, ax.get_position().y1 - (k + 0.5) / 3 * ax.get_position().height + 0.028, a, color=t["ink"], fontsize=11, fontweight="bold", va="center")
        fig.text(0.012, ax.get_position().y1 - (k + 0.5) / 3 * ax.get_position().height - 0.028, b, color=t["sub"], fontsize=9.5, va="center")
    title(fig, t, "What a robot can measure about an object it is holding", "20 workshop tools with CAD ground truth, one dot per tool. Gentle, production-like motion (≤ 2.5 m/s²).")
    legend(fig, t, [(t["s"][0], "identified from the wrist wrench during motion"), (t["s"][1], "computed from the mesh, uniform density")], 0.012, 0.80)
    save(fig, name, theme)


# ------------------------------------------------------------------ 2. why: signal budget
def fig_signal_budget(name, theme):
    t = THEMES[theme]; d = J("utias_signal_budget.json"); o = d["objects"]; nf, nt = d["noise"]["force_N"], d["noise"]["torque_Nm"]
    rows = [[r["f_mass"] / nf for r in o], [r["n_com"] / nt for r in o], [r["n_inertia"] / nt for r in o]]
    fig = plt.figure(figsize=(W, 3.7)); ax = fig.add_axes([0.255, 0.14, 0.725, 0.62]); style(ax, t)
    xlim = (0.008, 400)
    ax.axvspan(xlim[0], 1, color=t["band"], zorder=0, linewidth=0)
    strip_rows(ax, t, [(v, t["s"][0]) for v in rows], xlim)
    ax.set_xticks([0.01, 0.1, 1, 10, 100]); ax.set_xticklabels(["0.01×", "0.1×", "1×", "10×", "100×"])
    ax.xaxis.set_minor_locator(matplotlib.ticker.NullLocator())
    ax.set_xlabel("RMS signal ÷ sensor noise, per sample (log scale)", color=t["sub"], fontsize=9.5, labelpad=6)
    ax.text(1, -0.56, "below the noise floor ", color=t["muted"], fontsize=9, ha="right", va="bottom")
    for k, v in enumerate(rows):
        m = np.median(v); median_mark(ax, t, k, v, f"median {m:.0f}×" if m >= 10 else f"median {m:.1f}×", side="left" if k < 2 else "right")
    labs = [("Mass", "gravity load  m·g"), ("Centre of mass", "gravity lever  m·c × g"), ("Inertia", "I·α + ω × I·ω")]
    for k, (a, b) in enumerate(labs):
        y = ax.get_position().y1 - (k + 0.5) / 3 * ax.get_position().height
        fig.text(0.012, y + 0.03, a, color=t["ink"], fontsize=11, fontweight="bold", va="center")
        fig.text(0.012, y - 0.03, b, color=t["sub"], fontsize=9.5, va="center")
    title(fig, t, "Where the signal is", "Wrench each parameter produces (from ground-truth parameters) relative to F/T noise of 0.3 N and 0.005 N·m. One dot per tool.")
    save(fig, name, theme)


# ------------------------------------------------------------------ 3. One-Shot: the estimate tracks its seed
def fig_oneshot(name, theme):
    t = THEMES[theme]; d = J("oneshot_traces.json"); gt = d["gt"]["mass"]
    groups = [0.2, 0.5, 0.9, 1.3]
    fig = plt.figure(figsize=(W, 4.1)); ax = fig.add_axes([0.075, 0.13, 0.62, 0.66]); style(ax, t, grid="y")
    ax.axhline(gt, color=t["ink"], linewidth=1.2, zorder=2)
    ax.plot([13.2, 14.6], [0.26, 0.26], color=t["ink"], linewidth=1.2); ax.text(15.0, 0.26, "ground truth, 0.80 kg", color=t["ink"], fontsize=9.5, fontweight="bold", va="center")
    for tag, r in d["runs"].items():
        c = t["s"][groups.index(r["init_mass"])]
        e = [0] + r["epoch"]; m = [r["init_mass"]] + r["mass_trace"]
        ax.plot(e, m, color=c, linewidth=1.6, alpha=0.85, solid_joinstyle="round", solid_capstyle="round", zorder=3)
        ax.scatter([r["best_epoch"]], [r["mass"]], s=64, color=c, edgecolors=t["surface"], linewidths=1.5, zorder=4)
    ys = []
    for k, m0 in enumerate(groups):
        best = [r["mass"] for r in d["runs"].values() if r["init_mass"] == m0]
        ax.scatter([0], [m0], s=64, color=t["s"][k], edgecolors=t["surface"], linewidths=1.5, zorder=4, clip_on=False)
        y = max(np.mean(best), ys[-1] + 0.135) if ys else np.mean(best); ys.append(y)
        ax.annotate("\u25cf", (25.9, y), color=t["s"][k], fontsize=11, va="center", annotation_clip=False)
        ax.annotate(f"{m0:.1f} \u2192 {np.mean(best):.2f} kg", (26.9, y), color=t["ink"], fontsize=9.5, fontweight="bold", va="center", annotation_clip=False)
        ax.annotate(f"{np.mean([abs(b - gt) for b in best]) / gt * 100:.0f}% off, {len(best)} runs", (31.6, y), color=t["sub"], fontsize=9, va="center", annotation_clip=False)
    ax.annotate("seed \u2192 reported mass", (25.9, 1.44), color=t["muted"], fontsize=9, va="center", annotation_clip=False)
    ax.set_xlim(-0.4, 25.4); ax.set_ylim(0.1, 1.45); ax.set_xticks([0, 5, 10, 15, 20, 25]); ax.set_yticks([0.2, 0.5, 0.8, 1.1, 1.4])
    ax.set_yticklabels(["0.2 kg", "0.5", "0.8", "1.1", "1.4"])
    ax.set_xlabel("optimisation epoch (stage 2, physics)", color=t["sub"], fontsize=9.5, labelpad=6)
    title(fig, t, "One-Shot Real-to-Sim: the mass estimate follows its starting value", "Drill, 11 runs of the shipped configuration; only the initial mass differs. Dots mark the epoch the method reports (best loss).")
    save(fig, name, theme)


# ------------------------------------------------------------------ 4. Scalable Real2Sim: how much motion mass needs
def fig_excitation(name, theme):
    t = THEMES[theme]; s = J("s2s_tests.json"); objs = ["spam", "sugar", "lego"]
    panels = [("Less time", ["first 4s", "first 2s", "first 1s", "first 0.5s"], ["first 4 s", "first 2 s", "first 1 s", "first 0.5 s"]),
              ("Less motion", ["most excited 25%", "most excited 50%", "gentlest 50%", "gentlest 25%", "gentlest 10%", "gentlest 5%"],
               ["most excited 25%", "most excited 50%", "gentlest 50%", "gentlest 25%", "gentlest 10%", "gentlest 5%"])]
    fig = plt.figure(figsize=(W, 4.0))
    for p, (head, keys, labs) in enumerate(panels):
        ax = fig.add_axes([0.125 + p * 0.50, 0.14, 0.345, 0.50]); style(ax, t)
        for k, key in enumerate(keys):
            for j, o in enumerate(objs):
                ax.scatter([s[o]["T2"][key]["dm"]], [k + (j - 1) * 0.2], s=50, color=t["s"][j], edgecolors=t["surface"], linewidths=1.2, zorder=3)
        ax.axvspan(0, 5, color=t["band"], zorder=0, linewidth=0)
        ax.set_ylim(5.6, -0.6); ax.set_xlim(0, 50); ax.set_yticks(range(len(keys))); ax.set_yticklabels(labs, color=t["sub"], fontsize=9.5)
        ax.set_xticks([0, 10, 20, 30, 40, 50]); ax.set_xticklabels(["0%", "10%", "20%", "30%", "40%", "50%"])
        ax.set_xlabel("mass change vs the full-recording fit", color=t["sub"], fontsize=9.5, labelpad=6)
        fig.text(ax.get_position().x0 - 0.11, ax.get_position().y1 + 0.045, head, color=t["ink"], fontsize=10.5, fontweight="bold", va="bottom")
        if p == 0: ax.text(0, -0.72, "within 5%", color=t["muted"], fontsize=9, va="bottom")
    title(fig, t, "Scalable Real2Sim: how little data mass identification needs", "Payload mass re-identified from subsets of each recording (iiwa joint torques).")
    legend(fig, t, [(t["s"][0], "spam, 0.38 kg"), (t["s"][1], "sugar, 0.50 kg"), (t["s"][2], "lego, 0.17 kg")], 0.012, 0.815)
    save(fig, name, theme)


# ------------------------------------------------------------------ 5. hybrid: inertia from geometry is stable and physical
def fig_hybrid(name, theme):
    t = THEMES[theme]; h = J("hybrid", "holdout_K5.json"); objs = ["spam", "sugar", "lego"]
    panels = [("Inertia spread across 5 folds", "I_cv", (0, 60), [0, 20, 40, 60], "{:.0f}%", None),
              ("Gyration ratio", "gyration", (0, 5.4), [0, 1, 2, 3, 4, 5], "{:.2f}", 1.0)]
    fig = plt.figure(figsize=(W, 2.9))
    for p, (head, key, xlim, ticks, fmt, limit) in enumerate(panels):
        ax = fig.add_axes([0.085 + p * 0.50, 0.17, 0.39, 0.46]); style(ax, t)
        if limit is not None:
            ax.axvspan(limit, xlim[1], color=t["band"], zorder=0, linewidth=0)
            ax.text(xlim[1], -0.62, "physically impossible  ", color=t["muted"], fontsize=9, va="bottom", ha="right")
        for k, o in enumerate(objs):
            a, b = h[o]["torque"][key], h[o]["hybrid"][key]
            ax.plot([a, b], [k, k], color=t["axis"], linewidth=2, zorder=2, solid_capstyle="round")
            ax.scatter([a], [k], s=84, color=t["ctx"], edgecolors=t["surface"], linewidths=1.5, zorder=3)
            ax.scatter([b], [k], s=84, color=t["s"][0], edgecolors=t["surface"], linewidths=1.5, zorder=4)
            ax.annotate(fmt.format(a), (a, k), xytext=(9, 0), textcoords="offset points", color=t["sub"], fontsize=9, va="center")
            ax.annotate(fmt.format(b), (b, k), xytext=(0, 9), textcoords="offset points", color=t["ink"], fontsize=9, fontweight="bold", ha="center")
        ax.set_ylim(2.5, -0.62); ax.set_xlim(*xlim); ax.set_xticks(ticks)
        ax.set_xticklabels([f"{x}%" if "%" in fmt else f"{x}" for x in ticks])
        ax.set_yticks(range(3)); ax.set_yticklabels(objs if p == 0 else [], color=t["sub"], fontsize=9.5)
        ax.text(xlim[0], -1.12, head, color=t["ink"], fontsize=10.5, fontweight="bold", va="bottom")
    title(fig, t, "Hybrid pipeline: same mass, an inertia that is stable and physically possible")
    legend(fig, t, [(t["ctx"], "inertia from joint torque (SDP)"), (t["s"][0], "inertia from geometry (hybrid)")], 0.012, 0.865)
    save(fig, name, theme)


# ------------------------------------------------------------------ 6. per-tool scorecard with thumbnails
def rbar(ax, y, v, h, color, r_pt=3.0):
    """Horizontal bar from 0 to v: square at the baseline, rounded at the data end."""
    bb = ax.get_window_extent(); (x0, x1), (y0, y1) = ax.get_xlim(), ax.get_ylim()
    ppx = bb.width / (x1 - x0); ppy = bb.height / abs(y1 - y0); r = r_pt * ax.figure.dpi / 72
    v = max(v, 1.0 / ppx); rx = min(r / ppx, v); ry = r / ppy; a, b = y - h / 2, y + h / 2
    P = matplotlib.path.Path
    ax.add_patch(PathPatch(P([(0, a), (v - rx, a), (v, a), (v, a + ry), (v, b - ry), (v, b), (v - rx, b), (0, b), (0, a)],
                             [P.MOVETO, P.LINETO, P.CURVE3, P.CURVE3, P.LINETO, P.CURVE3, P.CURVE3, P.LINETO, P.CLOSEPOLY]),
                           color=color, linewidth=0, zorder=3))


def fig_tools(name, theme):
    t = THEMES[theme]; g = J("utias_gentle_motion.json"); geo = sorted(J("utias_geometry_inertia.json"), key=lambda r: (r["errF"], r["name"]))
    n = len(geo); row = 0.36; top, bot = 1.05, 0.55; H = top + bot + n * row
    fig = plt.figure(figsize=(W, H)); y_of = lambda k: 1 - (top + (k + 0.5) * row) / H
    axs = []
    for p, (x, w, xlim, ticks) in enumerate([(0.43, 0.215, (0, 20), [0, 5, 10, 15, 20]), (0.725, 0.215, (0, 56), [0, 10, 20, 30, 40, 50])]):
        ax = fig.add_axes([x, bot / H, w, n * row / H]); style(ax, t)
        ax.set_xlim(*xlim); ax.set_ylim(n - 0.5, -0.5); ax.set_yticks([]); ax.set_xticks(ticks); ax.set_xticklabels([f"{v}%" for v in ticks])
        axs.append(ax)
    fig.canvas.draw()
    flagged = 0
    for k, r in enumerate(geo):
        key = "CLEAN_" + r["name"].replace(" ", "_"); me = g[key]["ls"]["m_err"]; bad = g[key]["gt_residual"] > 0.3; flagged += bad
        im = plt.imread(os.path.join(HERE, "tools", r["name"].replace(" ", "_") + ".png"))
        axi = fig.add_axes([0.008, y_of(k) - 0.47 * row / H, 0.11, 0.94 * row / H]); axi.imshow(im, interpolation="none"); axi.axis("off")
        fig.text(0.128, y_of(k) + 0.0, r["name"] + (" †" if bad else ""), color=t["ink"], fontsize=9.5, va="center")
        fig.text(0.318, y_of(k), f"{r['mass'] * 1000:,.0f} g", color=t["sub"], fontsize=9, va="center", ha="right")
        fig.text(0.41, y_of(k), "1 material" if r["nmat"] == 1 else f"{r['nmat']} materials", color=t["sub"], fontsize=9, va="center", ha="right")
        rbar(axs[0], k, me, 0.52, t["s"][0]); rbar(axs[1], k, r["errF"], 0.52, t["s"][0] if r["nmat"] == 1 else t["s"][1])
        axs[0].annotate(pct(me), (me, k), xytext=(5, 0), textcoords="offset points", color=t["sub"], fontsize=8.5, va="center")
        axs[1].annotate(pct(r["errF"]), (r["errF"], k), xytext=(5, 0), textcoords="offset points", color=t["sub"], fontsize=8.5, va="center")
    for ax, head, sub in ((axs[0], "Mass error", "from gentle motion"), (axs[1], "Inertia error", "from geometry, uniform density")):
        x = ax.get_position().x0
        fig.text(x, 1 - (top - 0.34) / H, head, color=t["ink"], fontsize=10.5, fontweight="bold", va="center")
        fig.text(x, 1 - (top - 0.13) / H, sub, color=t["sub"], fontsize=9, va="center")
    title(fig, t, "Per-tool scorecard against CAD ground truth")
    legend(fig, t, [(t["s"][0], "one material"), (t["s"][1], "several materials: uniform density is the approximation")], 0.43, 1 - 0.30 / H)
    fig.text(0.012, 0.13 / H, f"† {flagged} tools whose released simulation disagrees with its own ground truth (residual 0.4–0.5); they are the mass outliers.",
             color=t["muted"], fontsize=8.5, va="center")
    save(fig, name, theme)


# ------------------------------------------------------------------ 7. joint axes from geometry
def fig_axes(name, theme):
    sys.path.insert(0, os.path.join(ROOT, "experiments", "articulation"))
    import axis_recovery_test as A
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    t = THEMES[theme]; objs = A.make_objects()
    order = ["cabinet_door", "gate_post", "drawer", "box_lid", "laptop"]
    fig = plt.figure(figsize=(W, 3.75))
    for k, nm in enumerate(order):
        P, C, truth, hint = objs[nm]
        ax = fig.add_axes([0.004 + k * 0.199, 0.20, 0.196, 0.56], projection="3d"); ax.set_axis_off(); ax.set_facecolor("none"); ax.patch.set_alpha(0)
        for mesh, fc, al in ((P, t["ctx"], 0.22), (C, t["s"][0], 0.30)):
            ax.add_collection3d(Poly3DCollection(mesh.vertices[mesh.faces], facecolor=fc, edgecolor="none", alpha=al))
            for e in mesh.vertices[mesh.face_adjacency_edges[mesh.face_adjacency_angles > 0.5]]:
                ax.plot(*e.T, color=fc if fc != t["ctx"] else t["muted"], linewidth=0.9, alpha=0.9)
        V = np.vstack([P.vertices, C.vertices]); lo, hi = V.min(0), V.max(0); c = (lo + hi) / 2; L = (hi - lo).max() * 0.56
        ax.set_xlim(c[0] - L, c[0] + L); ax.set_ylim(c[1] - L, c[1] + L); ax.set_zlim(c[2] - L, c[2] + L); ax.set_box_aspect((1, 1, 1)); ax.view_init(22, -58)
        half = (hi - lo).max() * 0.62

        def line(r, col, lw, z):
            d = r["d"] / np.linalg.norm(r["d"]); p = r["p"] + d * ((c - r["p"]) @ d)
            ax.plot(*np.stack([p - d * half, p + d * half]).T, color=col, linewidth=lw, solid_capstyle="round", zorder=z)

        r1, _ = A.recover(P, C, truth["type"], hint, use_hint=True); r0, _ = A.recover(P, C, truth["type"], hint, use_hint=False)
        line(truth, t["ink"], 4.2, 10); line(r0, t["s"][1], 2.0, 11); line(r1, t["s"][2], 2.0, 12)
        a1, d1 = A.axis_errors(r1, truth); a0, d0 = A.axis_errors(r0, truth); ok = a1 < 2 and (d1 < 5 or truth["type"] == "prismatic")
        x = 0.004 + (k + 0.5) * 0.199
        # separation of two near-parallel lines = distance from a point on the true axis to the recovered line
        sep = lambda r: np.linalg.norm(np.cross(truth["p"] - r["p"], r["d"] / np.linalg.norm(r["d"]))) * 1000
        rev = truth["type"] != "prismatic"
        fig.text(x, 0.795, nm.replace("_", " "), color=t["ink"], fontsize=10.5, fontweight="bold", ha="center")
        fig.text(x, 0.165, ("pass" if ok else "fail") + f":  {a1:.1f}\u00b0" + (f", {d1:.1f} mm" if rev else ""), color=t["ink"], fontsize=9.5, fontweight="bold", ha="center")
        fig.text(x, 0.105, f"no hint:  {a0:.1f}\u00b0" + (f", {sep(r0):.0f} mm" if rev and a0 < 2 else ""), color=t["sub"], fontsize=9, ha="center")
        fig.text(x, 0.045, truth["type"] + " joint", color=t["muted"], fontsize=9, ha="center")
    title(fig, t, "Joint axes recovered from part geometry alone", "Direction and position error of the recovered axis. Pass: under 2\u00b0 and 5 mm.")
    fig.add_artist(matplotlib.lines.Line2D([0.575, 0.60], [0.945, 0.945], color=t["ink"], linewidth=4.2, solid_capstyle="round"))
    fig.text(0.61, 0.945, "true axis", color=t["sub"], fontsize=9.5, va="center")
    legend(fig, t, [(t["s"][2], "recovered with hint"), (t["s"][1], "without hint")], 0.70, 0.945)
    save(fig, name, theme)


FIGS = dict(identifiability=fig_identifiability, signal_budget=fig_signal_budget, oneshot_seed=fig_oneshot, s2s_excitation=fig_excitation,
            hybrid_inertia=fig_hybrid, tools_scorecard=fig_tools, axis_recovery=fig_axes)

if __name__ == "__main__":
    want = sys.argv[1:]
    for name, fn in FIGS.items():
        if want and not any(w in name for w in want): continue
        for theme in THEMES: fn(name, theme)
