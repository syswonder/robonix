<!-- SPDX-License-Identifier: MulanPSL-2.0 -->

# Piper model source

The tracked Piper and parallel-gripper URDF is generated from:

- Repository: `https://github.com/agilexrobotics/agx_arm_urdf`
- Revision: `f6642ce0d7872c686f29c99e9e10cd23d1d49313`
- Base: `piper/urdf/piper_description.urdf`
- Gripper: `piper/urdf/piper_with_gripper_description.xacro`
- License: MIT

The generator expands the gripper file's single Xacro include, removes
collision geometry, rewrites `package://agx_arm_description/...` Mesh URIs to
URDF-local relative paths, copies only referenced visual assets, and records
their hashes in `model/manifest.json`. Generated files under `model/meshes/`
are tracked robot-model assets. Update the URDF, Mesh files, manifest, source
revision, and license information together whenever the model is regenerated.

The path mapping is stable and resolved relative to
`model/piper_with_gripper.urdf`:

```text
Upstream: piper/meshes/dae/<name>.dae
Local:    model/meshes/dae/<name>.dae
URDF:     meshes/dae/<name>.dae
```

Regenerate from a checkout pinned to the revision above:

```bash
python3 examples/piper_vitals/vendor_model.py \
  --source /path/to/agx_arm_urdf \
  --revision f6642ce0d7872c686f29c99e9e10cd23d1d49313
```

Run the generator after cloning the official repository and checking out the
exact revision above. Before starting Soma, every resource listed in
`model/manifest.json` must exist below `model/` with the same case-sensitive
relative path and checksum.
