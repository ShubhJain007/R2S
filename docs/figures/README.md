# README figures

Every figure in the top-level README is generated, not drawn by hand, and comes in a light and a dark variant.

| Figure | Built by | From |
|---|---|---|
| `pipeline-*.svg` | `make_diagrams.py` | (layout in the script) |
| `identifiability-*.svg` | `make_figures.py` | `results/utias_gentle_motion.json`, `results/utias_geometry_inertia.json` |
| `signal_budget-*.svg` | `make_figures.py` | `results/utias_signal_budget.json` |
| `oneshot_seed-*.svg` | `make_figures.py` | `results/oneshot_traces.json` |
| `s2s_excitation-*.svg` | `make_figures.py` | `results/s2s_tests.json` |
| `hybrid_inertia-*.svg` | `make_figures.py` | `results/hybrid/holdout_K5.json` |
| `tools_scorecard-*.svg` | `make_figures.py` | the two utias result files and `tools/*.png` |
| `axis_recovery-*.svg` | `make_figures.py` | runs `experiments/articulation/axis_recovery_test.py` in-process (deterministic, under a second) |
| `tools/*.png` | `render_tools.py` | the 20 meshes in the `utias-inertial` clone |

The RH20T media (`results/rh20t_splat/demo/`) is built by `rh20t_splat/render_flythrough.py` and `rh20t_splat/make_demo_media.py`.

## Rebuilding

```bash
python docs/figures/make_diagrams.py                          # no dependencies
python docs/figures/make_figures.py                           # matplotlib + trimesh (conda env: s2s)
python docs/figures/make_figures.py hybrid                    # only figures whose name contains "hybrid"
PYOPENGL_PLATFORM=egl python docs/figures/render_tools.py     # pyrender (conda env: rigidworldmodel)
```

The charts have transparent backgrounds and are referenced from the README through `<picture>` so that GitHub serves the
variant matching the reader's theme. Colours are a fixed, colour-blind-checked categorical set; a series keeps its colour in
every figure (blue for "from motion", orange for "from geometry").

## Licences

`tools/*.png` are renders of meshes from the utiasSTARS workshop-tools dataset
([inertial-identification-with-part-segmentation](https://github.com/utiasSTARS/inertial-identification-with-part-segmentation), MIT).
