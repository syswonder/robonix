<!-- SPDX-License-Identifier: MulanPSL-2.0 -->

# Unitree B2-W model source

This example uses the official Unitree B2-W description from:

- repository: `https://github.com/unitreerobotics/unitree_ros`
- revision: `daadf41ee9afce8f90fdc09a98506012691fa122`
- source package: `robots/b2w_description`
- source URDF: `robots/b2w_description/urdf/b2w_description.urdf`
- license: BSD-3-Clause

The checked-in `model/unitree_b2w_visual.urdf` is generated from that pinned
source. Collision elements are removed, material identifiers are normalized to
ASCII, and visual Mesh references are rewritten from
`package://b2w_description/meshes/...` to `meshes/...` relative to the URDF.

The official repository contains about 1.38 GB of files across many robots.
This example's URDF references about 79 MiB of uncompressed visual resources;
its largest single DAE is about 68 MiB. Those binary files are distributed as
a versioned GitHub Release artifact instead of ordinary Git objects:

- distribution repository:
  `https://github.com/LittleRookie1115/robonix-model-assets-test`
- release: `unitree-b2w-daadf41-r2`
- artifact: `unitree-b2w-visuals-daadf41.tar.gz`

The distribution repository contains the reproducible build script. The
Robonix manifest at `model/model-assets.json` pins the release URL, archive
size, archive SHA-256, extracted paths, extracted sizes, and extracted hashes.
Run `./fetch_model.sh` before Soma starts. The downloaded license is installed
beside the Mesh directory under `model/licenses/`.
