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
└── robonix_manifest.yaml      Top-level deploy manifest (atlas + pilot
                               + executor under `system:`; primitives
                               and services TBD pending driver lifecycle).
```

Drivers run **inside** the sim container via `docker exec` so they share
the simulator's DDS graph. They are NOT host-side processes; the host
only needs `rbnx`, Docker, and an X11 display.

## Bring-up (current — manual driver start)

Until each driver grows a `*/driver` lifecycle interface, `rbnx deploy`
won't auto-start them (it waits on Driver(CMD_INIT) which the legacy
registration API doesn't provide). Manual flow:

```bash
# T1 — sim (Ctrl-C stops):
bash examples/webots/sim/start.sh

# T2 — robonix system services (atlas, executor, pilot):
cd examples/webots
rbnx deploy

# T3..T6 — drivers + nav (one per terminal):
rbnx start -p ./primitives/tiago_chassis
rbnx start -p ./primitives/tiago_camera
rbnx start -p ./primitives/tiago_lidar
rbnx start -p ./services/tiago_nav2

# T7 — agent chat:
rbnx chat
```

`rbnx nodes` should show 4 driver registrations + 3 system caps.

## Bring-up (target — single `rbnx deploy`)

After the migration to `RegisterCapability` + LifecycleDriver, the four
driver entries move into `primitive:` / `service:` blocks of
`robonix_manifest.yaml` and `rbnx deploy` brings everything up
serially. The user-side step that doesn't go away: starting the sim
docker (`sim/start.sh`) — that's environment, not robonix.

## Env vars

Pre-export before `rbnx deploy` (or set in shell rc):

```
VLM_BASE_URL=https://api.openai.com/v1   # or your OpenAI-compatible endpoint
VLM_API_KEY=sk-...
VLM_MODEL=gpt-4o
```

The deploy manifest references these via `${VLM_*}`.
