<!-- SPDX-License-Identifier: MulanPSL-2.0 -->

# PAL Robotics TIAGo visual assets

The tracked URDF files in this directory are generated from the official PAL
Robotics ROS 2 Humble descriptions and distributed under Apache License 2.0.
See `TIAGO_VISUALS_LICENSE-APACHE-2.0.txt`. Files under `meshes/` are tracked
as part of the robot model and must be updated together with the URDF,
manifest, source revisions, and license. Generated textures remain local.

Source revisions:

- `https://github.com/pal-robotics/tiago_robot`:
  `f1c33c92bdde7c1dd79f0c3e739e98a233dbd30b`
- `https://github.com/pal-robotics/pmb2_robot`:
  `8c2758eb91eac80a626df0ded51cb18d3fd7d215`
- `https://github.com/pal-robotics/pal_gripper`:
  `485b535532c04b0ce31a00ad431d99e2714c0d54`
- `https://github.com/pal-robotics/pal_urdf_utils`:
  `775cdd6886296e6c00f17dbdfd9bcdd20e0e6622`
- `https://github.com/pal-robotics/pal_hey5`:
  `344de441fa3c378573dd7f6e04dc7ec4f8cb0cab`

Both models were expanded with Xacro 2.1.1. Lite uses
`arm_type:=no-arm ft_sensor:=no-ft-sensor end_effector:=no-end-effector`.
Full uses `arm_type:=tiago-arm ft_sensor:=schunk-ft
end_effector:=pal-gripper`. Both use `base_type:=pmb2`,
`is_public_sim:=true`, and `use_sim_time:=false`.

`examples/webots/sim/vendor_tiago_visuals.py` validates every asset reference,
copies it beside the Soma URDF, merges the Webots device and control nodes,
rewrites each resource to a URDF-local relative URL, and records file hashes in
`tiago_visuals.manifest.json`.

The generated layout and URDF reference format are:

```text
Local: resource/meshes/<format>/<package>__<upstream-path-with-__-separators>
URDF:  meshes/<format>/<package>__<upstream-path-with-__-separators>
```

For example:

```text
Upstream package path: tiago_description/meshes/arm/arm_1.stl
Local file:            resource/meshes/stl/tiago_description__meshes__arm__arm_1.stl
URDF filename:         meshes/stl/tiago_description__meshes__arm__arm_1.stl
```

DAE resources use `resource/meshes/dae/` and `meshes/dae/` in the same way.
Any future textures must use `resource/textures/<format>/` locally and
`textures/<format>/` in the URDF.

To prepare a fresh checkout, clone the official repositories above, check out
the exact revisions, and expand the Lite and Full Xacro descriptions in a ROS
2 Humble workspace where those packages are discoverable. Then run:

```bash
python3 examples/webots/sim/vendor_tiago_visuals.py \
  --lite-urdf /path/to/tiago-lite.urdf \
  --full-urdf /path/to/tiago-full.urdf \
  --tiago-description /path/to/tiago_robot/tiago_description \
  --pmb2-description /path/to/pmb2_robot/pmb2_description \
  --pal-gripper-description /path/to/pal_gripper/pal_gripper_description \
  --pal-urdf-utils /path/to/pal_urdf_utils/pal_urdf_utils
```

Before requesting `get_urdf(include_assets=true)`, every path in
`tiago_visuals.manifest.json` must exist below `resource/` with the recorded
checksum. The Client does not download missing files from PAL Robotics; Soma
loads these local files on demand and transfers them with the URDF.
