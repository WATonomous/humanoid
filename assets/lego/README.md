# LEGO piece assets (MuJoCo / mjlab)

Procedural MJCF LEGO bricks for use in `mjlab` (MuJoCo) scenes — built from
primitive geoms, so there are no mesh files to fetch or license.

## Layout

```
assets/lego/
├── lego_2x4.xml         # standard 2x4 brick, includable snippet
└── scene_lego_2x4.xml   # standalone viewer scene for the brick above
```

## Try it

```bash
pip install mujoco
python -m mujoco.viewer --mjcf assets/lego/scene_lego_2x4.xml
```

## Using a brick in your own scene

`lego_2x4.xml` is written to be spliced in via MJCF `<include>` (same pattern
as robot bodies in mujoco_menagerie):

```xml
<mujoco model="my_scene">
  <include file="../../assets/lego/lego_2x4.xml"/>
  <worldbody>
    <light .../>
    <geom name="floor" type="plane" size="1 1 0.05"/>
  </worldbody>
</mujoco>
```

The brick spawns as a free body (`lego_2x4` / joint `lego_2x4_freejoint`) at
the origin. To place multiple bricks in one scene, copy the file and rename
the `body`/`geom`/`joint` names — MJCF requires unique names within a model.

## Notes

- Dimensions follow real LEGO measurements (8mm stud pitch, 9.6mm brick
  height, 4.8mm stud diameter). The brick is a solid box (no hollow
  underside/tubes), so `density` in the `lego_2x4` default class is tuned
  down from ABS's real ~1050 kg/m³ to land on a realistic ~2.5g total mass.
- Only a 2x4 brick exists today; other sizes can be added the same way by
  copying `lego_2x4.xml` and adjusting the box half-size / stud grid.
