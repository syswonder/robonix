# webots-aibox — full-stack split deployment

One `rbnx boot` on the AIBOX board brings up the whole stack for the TIAGo
Webots simulation:

- **AIBOX board**: Atlas, soma, executor, pilot, liaison, vitals, mapping
  (native), nav2 (native), explore (docker).
- **Ubuntu sim host**: the four TIAGo primitives, started through SSH
  wrappers (`primitives/*` forward their `build`/`start`/`stop` over
  `remote.env:HOST_SSH`), each docker-exec'ing its driver into the sim
  container.

The Webots sim itself stays outside boot — it is a GUI container owned by
`examples/webots/sim/start.sh` / `sim/stop.sh`.

## One-time setup

1. Board → sim host SSH, BatchMode-capable: generate a keypair on
   the board (`ssh-keygen`), authorize its public key in the sim host's
   `~/.ssh/authorized_keys`, and add the `aibox-ubuntu` alias (host, user,
   identity key) to the board's `~/.ssh/config`. Verify with
   `ssh -o BatchMode=yes aibox-ubuntu true`.
2. `remote.env` — copy `remote.env.example` to `remote.env`, then check
   `SIM_HOST_IP` and `BOARD_ATLAS` still match the two machines' current
   addresses.
3. Board env — copy the three examples in this directory into the board's
   `/root/robonix/run/` (drop the `.example` suffix) and replace the
   placeholder addresses: `webots-ubuntu.env` (Zenoh client wiring),
   `zenoh-aibox.json5` (the session config it points to), and `vlm.env`
   (VLM credentials; the real file never leaves the board). `boot.env`
   sources all three.
4. The three service/skill packages are upstream repos referenced by
   `url:` in the manifest; `rbnx build` / `rbnx boot` clone them into
   `examples/webots/rbnx-boot/cache/` on the board and keep them built.

## Bring up

```bash
# Ubuntu sim host:
bash examples/webots/sim/start.sh --world office.wbt
# wait for "[sim/start] live odom, RGB, and lidar data received"

# AIBOX board:
cd /root/robonix/examples/webots-aibox
. ./boot.env
rbnx build -f robonix_manifest.yaml
rbnx boot -f robonix_manifest.yaml --no-update-check -v
```

Boot waits for each layer to register before the next: the four primitives
must be ACTIVE before mapping starts, mapping before nav2, explore last.

Verify with `rbnx caps -v` (expect the four TIAGo primitives, mapping, nav2,
explore) and a `rbnx ask "What can you see in front of the robot?"`.

## Tear down

```bash
rbnx shutdown        # on the board, in the deployment directory
```

Shutdown reaches each remote driver via Atlas, then the stop hooks sweep any
leftover remote wrapper or driver (wrapper first — a live wrapper would
respawn its driver), then the board-side processes. Stop the sim last on the
Ubuntu host with `bash examples/webots/sim/stop.sh`.
