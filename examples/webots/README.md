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
│   ├── tiago_chassis/         /amcl_pose + /cmd_vel  → chassis/{state, move}
│   ├── tiago_camera/          /head_front_camera/*   → camera/{snapshot, depth_snapshot}
│   ├── tiago_lidar/           /scanner               → lidar/snapshot
│   └── audio_driver/          (separate, mic/spkr — old schema, not deployed yet)
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
rbnx boot
```

Then a third terminal for `rbnx chat`. `rbnx caps` should show the
4 driver registrations + 3 system caps (atlas / pilot / executor).
`rbnx tools` lists every MCP tool the LLM agent sees.

To tear everything down: `bash sim/stop.sh`.

## Env vars

Pre-export before `rbnx boot` (or set in shell rc):

```bash
export VLM_BASE_URL=https://api.openai.com/v1   # or your OpenAI-compatible endpoint
export VLM_API_KEY=sk-...
export VLM_MODEL=gpt-5.4-mini
```

The deploy manifest references these via `${VLM_*}`.

## What `rbnx boot` does

1. Reads `robonix_manifest.yaml`, brings up the `system:` block (atlas,
   executor, pilot) using their installed binaries — args (listen
   address, log level, VLM endpoint) come straight out of the manifest.
2. For each `primitive:` / `service:` entry, in declaration order:
   - Spawns the package via `rbnx start -p <path>` (which runs that
     package's `scripts/start.sh` — for tiago drivers that's a
     `docker exec` into the sim container that runs the Python driver).
   - Polls atlas until the package registers its first capability.
   - If the new cap declared a `*/driver` gRPC interface, also calls
     `LifecycleDriver.Driver(CMD_INIT, config_json)` to initialize it.
     Caps without a `*/driver` interface are deployed as soon as they
     register (no init dance) — tiago drivers fall in this bucket.
3. Sits on Ctrl-C / SIGTERM, then tears down all children.

## How the LLM picks tools

After `rbnx boot` is up, pilot's system prompt lists each cap's
`CAPABILITY.md` path; the LLM uses the executor's `read_file` builtin
to lazy-load the docs it needs (e.g. `read_file("/path/to/tiago_chassis/CAPABILITY.md")`).

Tools are exposed to the LLM as `<area>_<leaf>` to avoid leaf-name
collisions (camera and lidar both have a `snapshot` leaf — the LLM
sees `camera_snapshot` and `lidar_snapshot`). The MCP server inside
each driver still registers tools by leaf name.

The pilot's persistence prompt instructs the LLM to keep iterating
tools-then-reason until the task is *verifiably* done — taking a
fresh `chassis/state` or `camera/snapshot` after every physical action
to confirm progress.
