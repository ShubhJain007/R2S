# Third-party code used by this project (not vendored)

These repositories were cloned next to this project's own code and are **not** included here. Clone them at the pinned
commits below into the same folder names, then apply the patches in `third_party/patches/`.

| Folder | Upstream | Commit | Licence | Our changes |
|---|---|---|---|---|
| `RigidWorldModel/` | https://github.com/yifanzhu95/RigidWorldModel | `d034c13b61c5057f6d87fe49bdf21f08f9e9bae4` | MIT | `patches/RigidWorldModel.diff` (drop an import removed upstream); 15 experiment configs in `patches/RigidWorldModel_configs/` → copy into `RigidWorldModel/configs/` |
| `RigidWorldModel/diffworld/shape_as_points/` | https://github.com/yifanzhu95/shape_as_points | `25796bc466e34f7da84b70edf6710c06a4a7db3c` | MIT | none (required by RigidWorldModel, cloned manually as its README says) |
| `scalable-real2sim/` | https://github.com/nepfaff/scalable-real2sim | `a8e4d97cbb0c3ea887a69fa313bcd3a252c5a8a3` | MIT | `patches/scalable-real2sim.diff` (submodule URL over HTTPS instead of SSH); only the `robot_payload_id` submodule was initialised (`af16360`); note `robot_payload_id` has no LICENSE file |
| `rh20t_api/` | https://github.com/rh20t/rh20t_api | `aa3124434729ed622109a29b2cbb9f3bbb1c5eeb` | MIT | none |
| `utias-inertial/` | https://github.com/utiasSTARS/inertial-identification-with-part-segmentation | `48928088e4877c872ea5aaed92f2ef315190153a` | MIT | none |

```bash
git clone https://github.com/yifanzhu95/RigidWorldModel && (cd RigidWorldModel && git checkout d034c13 \
  && git apply ../third_party/patches/RigidWorldModel.diff && cp ../third_party/patches/RigidWorldModel_configs/*.yaml configs/ \
  && git clone https://github.com/yifanzhu95/shape_as_points diffworld/shape_as_points \
  && (cd diffworld/shape_as_points && git checkout 25796bc))
git clone https://github.com/nepfaff/scalable-real2sim && (cd scalable-real2sim && git checkout a8e4d97 \
  && git apply ../third_party/patches/scalable-real2sim.diff && git submodule update --init scalable_real2sim/robot_payload_id)
git clone https://github.com/rh20t/rh20t_api && (cd rh20t_api && git checkout aa31244)
git clone https://github.com/utiasSTARS/inertial-identification-with-part-segmentation utias-inertial \
  && (cd utias-inertial && git checkout 4892808)
```
