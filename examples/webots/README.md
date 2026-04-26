# Webots Tiago example

End-to-end demo: Webots Tiago sim + per-device robonix driver packages
+ Nav2 service + agent loop. Goal: bring it up and control the robot
from `rbnx chat`.

## Layout

```
examples/webots/
├── sim/                       NOT a robonix package. Plain docker
│   ├── start.sh               compose stack (Webots + eaios_webots).
│   └── ...                    Bring up FIRST, before anything else.
├── primitives/                One device = one package.
│   ├── tiago_chassis/         /amcl_pose + /cmd_vel  → chassis caps
│   ├── tiago_camera/          /head_front_camera/*   → camera caps
│   ├── tiago_lidar/           /scanner/scan          → lidar caps
│   └── audio_driver/          (separate, mic/spkr — old schema)
├── services/
│   └── tiago_nav2/            Nav2 launch + ActionClient wrapper
└── robonix_manifest.yaml      Top-level deploy manifest.
```

Drivers run **inside** the sim container via `docker exec` so they share
the simulator's DDS graph. They are NOT host-side processes; the host
only needs `rbnx`, Docker, and an X11 display.

## Bring-up

Two terminals only:

```bash
# T1 — sim (Ctrl-C stops):
bash examples/webots/sim/start.sh

# T2 — robonix: atlas + executor + pilot + 4 drivers + nav2:
cd examples/webots
rbnx deploy
```

Then a third terminal for `rbnx chat`. `rbnx nodes` should show the
4 driver registrations + 3 system caps.

## Env vars

Pre-export before `rbnx deploy` (or set in shell rc):

```
VLM_BASE_URL=https://api.openai.com/v1   # or your OpenAI-compatible endpoint
VLM_API_KEY=sk-...
VLM_MODEL=gpt-4o
```

The deploy manifest references these via `${VLM_*}`.

## What deploy does

1. Brings up `system:` (atlas, executor, pilot) using their installed binaries.
2. For each `primitive:` and `service:` entry:
   - Spawns the package via `rbnx start -p <path>` (which runs that
     package's `scripts/start.sh` — for tiago drivers that's a
     `docker exec` into the sim container).
   - Polls atlas until the package registers its first cap.
   - If the new cap declared a `*/driver` gRPC interface, also calls
     `Driver(CMD_INIT, config_json)` against it; otherwise (legacy
     RegisterNode/DeclareInterface API) just records the registration
     and moves on.
3. Sits on Ctrl-C, then tears down.
