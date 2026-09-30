#!/usr/bin/env python
"""Write the hand-laid-out diagrams: pipeline-{light,dark}.svg and the animated terminal_demo.svg.

Plain SVG with live text (system UI font), one file per GitHub theme. No dependencies.

Usage:  python docs/figures/make_diagrams.py
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
THEMES = {
    "light": dict(ink="#1f2328", sub="#59636e", muted="#818b98", line="#c4cbd3", box="#f6f8fa", motion="#2a78d6", geom="#eb6834", fric="#1baf7a", no="#d03b3b"),
    "dark": dict(ink="#f0f6fc", sub="#9198a1", muted="#7d8590", line="#3d444d", box="#161b22", motion="#3987e5", geom="#d95926", fric="#199e70", no="#e66767"),
}
FONT = "-apple-system, BlinkMacSystemFont, 'Segoe UI', 'Noto Sans', Helvetica, Arial, sans-serif"


def box(x, y, w, h, head, lines, t, accent):
    s = [f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{t["box"]}" stroke="{t["line"]}"/>',
         f'<rect x="{x}" y="{y + 10}" width="4" height="{h - 20}" rx="2" fill="{accent}"/>',
         f'<text x="{x + 18}" y="{y + 27}" class="h">{head}</text>']
    s += [f'<text x="{x + 18}" y="{y + 47 + 17 * i}" class="s">{ln}</text>' for i, ln in enumerate(lines)]
    return "\n".join(s)


def chip(x, y, w, head, note, t, accent):
    return (f'<rect x="{x}" y="{y}" width="{w}" height="46" rx="23" fill="none" stroke="{accent}" stroke-width="1.5"/>'
            f'<text x="{x + w / 2}" y="{y + 20}" class="h" text-anchor="middle">{head}</text>'
            f'<text x="{x + w / 2}" y="{y + 36}" class="n" text-anchor="middle">{note}</text>')


def arrow(pts, color, dashed=False, marker="a"):
    d = "M" + " L".join(f"{x},{y}" for x, y in pts)
    return f'<path d="{d}" fill="none" stroke="{color}" stroke-width="2" stroke-linejoin="round" {"stroke-dasharray=\'5 5\'" if dashed else ""} marker-end="url(#{marker}-{color[1:]})"/>'


def pipeline(t):
    M, G, F = t["motion"], t["geom"], t["fric"]
    heads = "".join(f'<marker id="a-{c[1:]}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
                    f'<path d="M0,1 L9,5 L0,9 z" fill="{c}"/></marker>' for c in (M, G, F, t["muted"]))
    body = [
        f'<text x="20" y="30" class="c">RECORD</text><text x="300" y="30" class="c">ESTIMATE</text>'
        f'<text x="620" y="30" class="c">PARAMETERS</text><text x="800" y="30" class="c">USE</text>',
        # record
        box(20, 50, 220, 96, "Force channel", ["joint torques + joint angles,", "or a wrist F/T wrench", "+ one empty-gripper baseline"], t, M),
        box(20, 236, 220, 80, "Cameras", ["calibrated RGB-D views", "&#8594; object mesh"], t, G),
        # estimate
        box(300, 50, 260, 96, "Newton&#8211;Euler fit", ["baseline subtracted, arm model removed", "least squares or physically-consistent SDP", "any motion: gravity carries the signal"], t, M),
        box(300, 236, 260, 80, "Geometry", ["voxelise the mesh (open meshes are fine)", "uniform density, scaled to the mass"], t, G),
        box(300, 356, 260, 62, "Press and slide", ["&#956; = |F tangential| / |F normal|"], t, F),
        # parameters
        chip(620, 50, 140, "mass", "0.2&#8211;1% error", t, M), chip(620, 104, 140, "centre of mass", "&#8776; 1 cm", t, M),
        chip(620, 253, 140, "inertia tensor", "4.4% median error", t, G), chip(620, 364, 140, "contact friction", "per material pair", t, F),
        # use
        f'<rect x="800" y="50" width="104" height="368" rx="8" fill="{t["box"]}" stroke="{t["line"]}"/>'
        f'<text x="852" y="80" class="h" text-anchor="middle">Simulator</text>'
        + "".join(f'<text x="852" y="{104 + 18 * i}" class="s" text-anchor="middle">{s}</text>' for i, s in enumerate(["SDF", "URDF", "JSON"]))
        + f'<line x1="816" y1="176" x2="888" y2="176" stroke="{t["line"]}"/>'
        f'<text x="852" y="202" class="h" text-anchor="middle">Self-checks</text>'
        + "".join(f'<text x="852" y="{226 + 18 * i}" class="s" text-anchor="middle">{s}</text>'
                  for i, s in enumerate(["held-out error", "conditioning", "gyration ratio", "symmetry flag", "static mass"])),
        # arrows
        arrow([(240, 98), (298, 98)], M), arrow([(560, 80), (618, 74)], M), arrow([(560, 116), (618, 126)], M),
        arrow([(240, 276), (298, 276)], G), arrow([(560, 276), (618, 276)], G),
        arrow([(130, 146), (130, 190), (270, 190), (270, 387), (298, 387)], F), arrow([(560, 387), (618, 387)], F),
        arrow([(760, 74), (798, 74)], M), arrow([(760, 127), (798, 127)], M), arrow([(760, 276), (798, 276)], G), arrow([(760, 387), (798, 387)], F),
        # mass feeds the geometry lane
        arrow([(430, 146), (430, 234)], M), f'<text x="440" y="176" class="n">mass</text>',
        # the route that is not taken
        arrow([(520, 146), (520, 200), (716, 200), (716, 251)], t["muted"], dashed=True),
        f'<circle cx="618" cy="200" r="10" fill="{t["box"]}" stroke="{t["no"]}" stroke-width="1.5"/>'
        f'<path d="M613,195 L623,205 M623,195 L613,205" stroke="{t["no"]}" stroke-width="2" stroke-linecap="round"/>'
        f'<text x="704" y="226" class="n" text-anchor="end">inertia from motion: 300&#8211;800% error, not used</text>',
    ]
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 924 438" width="924" height="438" font-family="{FONT}" role="img" '
            f'aria-label="Pipeline: mass and centre of mass come from the force channel, inertia from geometry, friction from a press-and-slide motion.">'
            f'<style>.h{{font-size:14px;font-weight:600;fill:{t["ink"]}}}.s{{font-size:12px;fill:{t["sub"]}}}.n{{font-size:11.5px;fill:{t["sub"]}}}'
            f'.c{{font-size:11px;font-weight:600;letter-spacing:.08em;fill:{t["muted"]}}}</style><defs>{heads}</defs>\n' + "\n".join(body) + "\n</svg>\n")


# The terminal animation replays a real run: the command and every output line are verbatim (see the "Try it"
# section of the README). Only the pill at the end is an annotation. SMIL, so it plays inside an <img> on GitHub;
# without animation support the attributes fall back to the finished frame.
TERMINAL = [
    ("cmd", "$ python -m r2s_pipeline identify results/hybrid/spam_recording.npz --mode joint_torque \\"),
    ("cmd", "      --robot scalable-real2sim/scalable_real2sim/robot_payload_id/models/iiwa.dmd.yaml --ee iiwa_link_7 \\"),
    ("cmd", "      --baseline results/hybrid/baseline_gripper0.05.npz"),
    ("out", "recording: 10000 samples, 10.0 s @ 1000 Hz, mode=joint_torque"),
    ("out", "  joint-torque mode: arm dynamics from nominal model + per-joint friction/offset terms"),
    ("out", "  baseline (no object): mass 2.5314 kg at [ 0.0392 -0.0215  0.0742]  ->  subtracted"),
    ("key", "  loaded 2.8972 kg - baseline 2.5314 kg = |object 0.3658 kg"),
    ("out", "  samples 10000 | excitation |a| max 6.08 m/s^2, median 2.36 | cond(full)=5.0 cond(mass,CoM)=3.3"),
    ("out", "  held-out (30%) RMSE / signal RMS = 0.724   abs RMSE per joint [ 3.774 11.056  3.568  3.938  3.26   2.062  3.341] Nm"),
    ("out", ""),
    ("key", "RESULT  |mass 0.3658 kg| | CoM (sensor frame) [0.0027 0.0012 0.2447]"),
]


def terminal(dur=16.0, W=900, lh=19, x0=22, y0=62):
    ink, sub, ok, bd, bg, bar = "#f0f6fc", "#9198a1", "#3fb950", "#30363d", "#0d1117", "#161b22"
    esc = lambda v: v.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    H = y0 + lh * len(TERMINAL) + 8; end = 0.95
    fade = lambda t0: (f'<animate attributeName="opacity" dur="{dur}s" repeatCount="indefinite" calcMode="discrete" '
                       f'values="0;1;0" keyTimes="0;{t0 / dur:.4f};{end}"/>')
    body, defs, t = [], [], 0.6
    for i, (kind, text) in enumerate(TERMINAL):
        y = y0 + lh * i
        if kind == "cmd":                                   # typed: a clip rectangle grows across the line
            t1 = t + max(0.9, len(text) * 0.022)
            defs.append(f'<clipPath id="c{i}"><rect x="0" y="{y - 14}" width="{W}" height="{lh}"><animate attributeName="width" dur="{dur}s" repeatCount="indefinite" '
                        f'values="0;0;{W};{W};0" keyTimes="0;{t / dur:.4f};{t1 / dur:.4f};{end};1"/></rect></clipPath>')
            head = f'<tspan fill="{ok}">$</tspan>' + esc(text[1:]) if text.startswith("$") else esc(text)
            body.append(f'<text x="{x0}" y="{y}" fill="{ink}" clip-path="url(#c{i})">{head}</text>'); t = t1 + 0.15
        else:
            t += 0.9 if i == 3 else (0.45 if text else 0.0)
            if not text: continue
            if kind == "key":
                parts = text.split("|", 2) if text.count("|") > 1 else text.split("|")
                a, b, c = (parts + [""])[:3]
                inner = f'{esc(a)}<tspan fill="{ok}">{esc(b)}</tspan>{esc(c)}'
                body.append(f'<text x="{x0}" y="{y}" fill="{ink}" font-weight="700">{inner}{fade(t)}</text>')
            else:
                body.append(f'<text x="{x0}" y="{y}" fill="{sub}">{esc(text)}{fade(t)}</text>')
    y = y0 + lh * (len(TERMINAL) - 1)
    body.append(f'<g font-family="{FONT}">{fade(t + 0.9)}<rect x="{W - 300}" y="{y - 17}" width="278" height="25" rx="12.5" fill="none" stroke="{ok}"/>'
                f'<text x="{W - 161}" y="{y}" fill="{ink}" font-size="12.5" text-anchor="middle">benchmark reference 0.3780 kg: 3.2% off</text></g>')
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" role="img" '
            f'aria-label="Terminal: r2s_pipeline identifies a 0.3658 kg object from a 10 second joint-torque recording after subtracting the empty-gripper baseline.">'
            f'<defs>{"".join(defs)}</defs><rect x="0.5" y="0.5" width="{W - 1}" height="{H - 1}" rx="10" fill="{bg}" stroke="{bd}"/>'
            f'<path d="M0.5,34 V10.5 a10,10 0 0 1 10,-10 H{W - 10.5} a10,10 0 0 1 10,10 V34 z" fill="{bar}"/><line x1="0.5" y1="34" x2="{W - 0.5}" y2="34" stroke="{bd}"/>'
            f'<circle cx="20" cy="17" r="5.5" fill="#f85149"/><circle cx="38" cy="17" r="5.5" fill="#d29922"/><circle cx="56" cy="17" r="5.5" fill="{ok}"/>'
            f'<text x="{W / 2}" y="21.5" fill="{sub}" font-family="{FONT}" font-size="12.5" text-anchor="middle">10 s of joint torques in, object mass out</text>'
            f'<g font-family="ui-monospace, SFMono-Regular, \'SF Mono\', Menlo, Consolas, \'Liberation Mono\', monospace" font-size="12" xml:space="preserve" style="white-space:pre">\n'
            + "\n".join(body) + "\n</g></svg>\n")


if __name__ == "__main__":
    for theme, t in THEMES.items():
        p = os.path.join(HERE, f"pipeline-{theme}.svg"); open(p, "w").write(pipeline(t)); print("wrote", os.path.relpath(p))
    p = os.path.join(HERE, "terminal_demo.svg"); open(p, "w").write(terminal()); print("wrote", os.path.relpath(p))
