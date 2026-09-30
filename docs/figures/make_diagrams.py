#!/usr/bin/env python
"""Write the hand-laid-out diagrams: pipeline-{light,dark}.svg.

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


if __name__ == "__main__":
    for theme, t in THEMES.items():
        p = os.path.join(HERE, f"pipeline-{theme}.svg"); open(p, "w").write(pipeline(t)); print("wrote", os.path.relpath(p))
