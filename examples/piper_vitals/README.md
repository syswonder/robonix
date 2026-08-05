<!-- SPDX-License-Identifier: MulanPSL-2.0 -->

# Piper Vitals demo

This minimal deployment renders an AgileX Piper arm with its parallel gripper
in the Robonix Client Vitals page. It does not run Webots, ROS, Liaison, audio,
perception, or a hardware controller.

The deployment starts these system components:

- Atlas
- Executor
- Soma
- Pilot
- Vitals
- `piper_health`, a deterministic nominal-health primitive

Executor and Pilot are included so Vitals can poll their module-health
contracts. The Chat and Audio workspaces have no interaction backend in this
deployment because Liaison and audio packages are intentionally absent.

## Build

Install the current Rust binaries once from the repository root:

```bash
env -u CC -u CXX -u CFLAGS -u CXXFLAGS \
  -u CPPFLAGS -u LDFLAGS -u OPENSSL_DIR -u PKG_CONFIG_PATH \
  make install
```

The Piper URDF and its generated DAE resources are tracked as one model bundle.
Regenerate them from the pinned official repository revision whenever the
model is updated:

```bash
cd /path/to/robonix

PIPER_SOURCE="${XDG_CACHE_HOME:-$HOME/.cache}/robonix/model-sources/agx_arm_urdf"
mkdir -p "$(dirname "$PIPER_SOURCE")"
test -d "$PIPER_SOURCE/.git" || \
  git clone https://github.com/agilexrobotics/agx_arm_urdf.git "$PIPER_SOURCE"
git -C "$PIPER_SOURCE" fetch origin f6642ce0d7872c686f29c99e9e10cd23d1d49313
git -C "$PIPER_SOURCE" checkout --detach f6642ce0d7872c686f29c99e9e10cd23d1d49313

python3 examples/piper_vitals/vendor_model.py \
  --source "$PIPER_SOURCE" \
  --revision f6642ce0d7872c686f29c99e9e10cd23d1d49313
```

This creates `model/meshes/dae/` beside the tracked URDF. The URDF references
resources with paths such as `meshes/dae/link1.dae`, resolved relative to
`model/piper_with_gripper.urdf`. The generated Mesh files are part of the
tracked robot model; stage them together with the URDF, manifest, source
metadata, and license. See `system/soma/README.md` for the general resource
path convention and `MODEL_SOURCE.md` for provenance.

Load the VLM settings needed for Pilot, then build the one primitive:

```bash
source ~/.bashrc
robonix-env
cd /path/to/robonix/examples/piper_vitals
rbnx build
```

## Run

This example uses the standard Robonix and Client ports. Stop any existing
Webots deployment and Client process before starting it.

Terminal 1 starts the minimal Robonix deployment:

```bash
source ~/.bashrc
robonix-env
cd /path/to/robonix/examples/piper_vitals
rbnx boot --no-update-check
```

Terminal 2 starts the client on the same machine:

```bash
cd /path/to/robonix-client
source .venv/bin/activate
robonix-client \
  --host 127.0.0.1 \
  --port 7860 \
  --robot-host 127.0.0.1 \
  --atlas-port 50051
```

Open `http://127.0.0.1:7860/` and select Vitals. The Chat Connect action is not
needed because this deployment intentionally has no Liaison. A remote browser
can forward only the Client port; Webots ports are not needed:

```text
ssh -N -T -o ExitOnForwardFailure=no -L 17860:127.0.0.1:7860 user@server
```

Then open `http://127.0.0.1:17860/`.

Expected hardware entries are the arm, six joints, parallel gripper, and its
actuator. The software module list includes Vitals, Executor, and Pilot.

### Test hardware failure and recovery

Keep the deployment running and inject faults from a third terminal. This does
not restart any Robonix process:

```bash
cd /path/to/robonix/examples/piper_vitals
python3 health_demo.py fault joint_2 joint_5 gripper
```

Within a few seconds, Vitals should open an alert dialog, render Joint 2,
Joint 5, and the parallel gripper in red, and retain the incidents in the Alert
Center. Restore nominal telemetry with:

```bash
python3 health_demo.py recover
```

The components return to normal, while the incidents change to `recovered` and
remain visible until `Confirm resolved` is clicked. For an automatically
recovering demonstration:

```bash
python3 health_demo.py fault joint_3 gripper --duration 15
```

Inspect the current injected profile with:

```bash
python3 health_demo.py status
```

### Test module failure and recovery

Use the PID recorded by `rbnx boot` so only this deployment's Pilot is affected:

```bash
cd /path/to/robonix/examples/piper_vitals
PILOT_PID=$(jq -er '.components[] | select(.name == "pilot") | .pid' \
  rbnx-boot/state.json)
kill -STOP "$PILOT_PID"
```

Within roughly 5 to 10 seconds, Vitals reports Pilot as `stale`. Resume the
same process before Atlas's 90-second provider lease expires:

```bash
kill -CONT "$PILOT_PID"
```

Pilot returns to `active` after the next health poll. If it remains paused for
more than 90 seconds, restart the deployment instead because Atlas has removed
the expired provider registration:

```bash
rbnx shutdown
rbnx boot --no-update-check
```

Stop the deployment with `Ctrl-C` in Terminal 1 or:

```bash
cd /path/to/robonix/examples/piper_vitals
rbnx shutdown
```

## Model provenance

`model/piper_with_gripper.urdf` contains only visual geometry because this
deployment does not simulate collisions. The URDF, source metadata, and
expected-resource manifest are generated from the MIT licensed official
`agilexrobotics/agx_arm_urdf` repository. See `MODEL_SOURCE.md`,
`model/manifest.json`, and `model/AGX_ARM_URDF_LICENSE-MIT.txt`. The binary DAE
resources are tracked with those files so Soma can serve a complete,
checksum-verifiable model without depending on an external package resolver at
runtime.
