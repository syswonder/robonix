# Webots demo deploy

End-to-end Robonix scene exercised on Webots. One deploy = one
`robonix_manifest.yaml` + the packages it brings up (primitives, scene
services, skills). System services (atlas, pilot, executor, memory,
speech) live under `/system/` and are wired in via the manifest's
`system:` block, not duplicated here.

## Layout

```
examples/webots/
├── robonix_manifest.yaml      # global env, system, services, primitives, skills
├── primitives/
│   └── audio_driver/          # ALSA mic + speaker primitives
└── services/
    └── tiago_sim_stack/       # Tiago + nav2 simulation orchestration
```

Per-package layout (when fully migrated):

```
<package>/
├── package_manifest.yaml      # this package's build/start/capabilities
├── capabilities/              # *.toml contract definitions this package implements
├── CAPABILITY.md              # human-readable description
├── src/                       # source (Python / ROS2 / Rust / …)
└── bin/                       # built binaries / scripts (generated)
```

## Bringing it up

```sh
rbnx deploy -f robonix_manifest.yaml
```

`rbnx deploy` start order:

1. **`system:`** — atlas, pilot, executor, memory, speech, … come up
   first. They register / declare their interfaces, then idle.
2. **`primitives:`** — each primitive process is spawned and registers
   its `driver` interface to atlas. `rbnx` then drives the driver
   interface (`Driver(CMD_INIT, config_json=…)`) on each in
   topological order. After init succeeds, the primitive
   lazy-declares its real interfaces (e.g. `base/move`, `base/odom`).
3. **`services:`** — scene services are spawned only after all
   primitives have finished init. Services may depend on primitive
   data streams (e.g. `map_service` consumes a lidar primitive's
   pointcloud), so they wait until step 2 is fully green.
4. **`skills:`** — NOT started. Skills are RPC-invocable applications
   that executor brings up on demand when pilot dispatches a tool
   call. Their packages just have to be installed; the runtime starts
   each invocation per call.
