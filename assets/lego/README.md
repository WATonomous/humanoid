# LEGO piece assets (MuJoCo / mjlab)

Procedural MJCF LEGO pieces for use in `mjlab` (MuJoCo) scenes — built from
primitive geoms, so there are no mesh files to fetch or license.

## Layout

```
assets/lego/
├── generate.py                    # regenerates every piece below
├── lego_2x4.xml                   # 2x4 brick (red), hand-authored original
├── lego_brick_2x2.xml             # 2x2 brick (red)
├── lego_brick_2x1.xml             # 1x2 brick (red)
├── lego_plate_4x2.xml             # 2x4 plate (yellow)
├── lego_plate_2x2.xml             # 2x2 plate (yellow)
├── lego_tile_2x2.xml              # 2x2 tile, no studs (blue)
├── lego_tile_2x1.xml              # 1x2 tile, no studs (blue)
└── scene_<piece>.xml              # standalone viewer scene per piece above
```

Bricks are 9.6mm tall with studs, plates are 3.2mm tall with studs, tiles are
3.2mm tall with a flat (stud-less) top. Colors just distinguish the three
kinds in renders — pass a different `rgba`/material if you want piece-specific
colors instead.

## Try it

```bash
pip install mujoco
python -m mujoco.viewer --mjcf assets/lego/scene_lego_2x4.xml
```

## Using a piece in your own scene

Each piece file is written to be spliced in via MJCF `<include>` (same
pattern as robot bodies in mujoco_menagerie):

```xml
<mujoco model="my_scene">
  <include file="../../assets/lego/lego_brick_2x2.xml"/>
  <worldbody>
    <light .../>
    <geom name="floor" type="plane" size="1 1 0.05"/>
  </worldbody>
</mujoco>
```

Each piece spawns as a free body (e.g. `lego_brick_2x2` / joint
`lego_brick_2x2_freejoint`) at the origin. To place multiple pieces in one
scene, copy the file and rename the `body`/`geom`/`joint` names — MJCF
requires unique names within a model.

## Adding more pieces

Edit the `PIECES` list in `generate.py` (kind: `brick`/`plate`/`tile`, stud
counts along x/y) and rerun `python3 assets/lego/generate.py` — it rewrites
each `lego_<kind>_<n>x<n>.xml` / `scene_lego_<kind>_<n>x<n>.xml` pair. The
original `lego_2x4.xml` predates the generator and isn't produced by it, so
it's left alone.

## Notes

- Dimensions follow real LEGO measurements (8mm stud pitch, 9.6mm brick /
  3.2mm plate-and-tile height, 4.8mm stud diameter). Every piece is a solid
  box (no hollow underside/tubes), so `density` in each piece's default class
  is tuned down from ABS's real ~1050 kg/m³ to land on a realistic mass.
