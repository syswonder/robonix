<!-- SPDX-License-Identifier: MulanPSL-2.0 -->

# Unitree B2-W large-model Vitals demo

This static deployment validates the external model artifact workflow with an
official Unitree B2-W visual model. It starts Atlas, Soma, and Vitals only. It
does not run Webots, ROS, Pilot, Executor, Liaison, or a hardware driver.

## Prepare the model

From the Robonix repository root:

```bash
examples/unitree_b2w_vitals/fetch_model.sh
```

The first run downloads a pinned GitHub Release artifact, verifies its declared
size and SHA-256, verifies every extracted file, and atomically installs the
`model/meshes/` and `model/licenses/` directories. The verified archive remains
under `${XDG_CACHE_HOME:-$HOME/.cache}/robonix/model-artifacts/`. Re-running the
command verifies the installed model and performs no network download when it
is unchanged.

Use an institution mirror without editing the checked-in manifest:

```bash
examples/unitree_b2w_vitals/fetch_model.sh \
  --artifact-url https://artifacts.example.org/robots/unitree-b2w-daadf41.tar.gz
```

## Run

Terminal 1:

```bash
source ~/.bashrc
robonix-env
cd /path/to/robonix/examples/unitree_b2w_vitals
rbnx boot --no-update-check
```

Terminal 2:

```bash
cd /path/to/robonix-client
source .venv/bin/activate
robonix-client --host 127.0.0.1 --port 7860 --robot-host 127.0.0.1
```

Open `http://127.0.0.1:7860/` and select Vitals. The first browser model load
causes the Client to stream missing resources from Soma into its verified disk
cache. The largest DAE is 71,748,110 bytes, so this also exercises the large
single-file path beyond ordinary Git hosting guidance and unary gRPC defaults.

Stop the deployment with `Ctrl-C` or:

```bash
cd /path/to/robonix/examples/unitree_b2w_vitals
rbnx shutdown
```

See `MODEL_SOURCE.md` for provenance and `model/model-assets.json` for exact
artifact and extracted-file checksums. See
`system/soma/ROBOT_MODEL_ASSETS.md` in the Robonix repository for the complete
distribution and streaming specification.
