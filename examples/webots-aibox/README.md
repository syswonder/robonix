# webots-aibox — full-stack split deployment

One `rbnx boot` on the AIBOX board brings up the whole stack for the TIAGo
Webots simulation:

- **AIBOX board**: Atlas, Chronos, soma, executor, pilot, liaison, vitals, mapping,
  nav2, and Scene (native), plus explore (docker).
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
4. Mapping, nav2, and explore are upstream packages referenced by `url:` in
   the manifest; `rbnx build` / `rbnx boot` keeps them built in the board's
   `examples/webots-aibox/rbnx-boot/cache/`. Scene is built from `system/scene`.

## Bring up

```bash
# Ubuntu sim host (opens Webots on the desktop):
  bash examples/webots/sim/start.sh --world office.wbt
# wait for "[sim/start] live odom, RGB, and lidar data received"

# AIBOX board:
cd /root/robonix/examples/webots-aibox
. ./boot.env
rbnx build -f robonix_manifest.yaml
rbnx boot -f robonix_manifest.yaml --no-update-check -v
```

Boot waits for each layer to register before the next: the four primitives
must be ACTIVE before mapping starts, followed by nav2, Scene, and explore.

Verify with `rbnx caps -v` (expect the four TIAGo primitives, mapping, nav2,
Scene, and explore) and a `rbnx ask "What can you see in front of the robot?"`.

## Tear down

```bash
rbnx shutdown        # on the board, in the deployment directory
```

Shutdown reaches each remote driver via Atlas, then the stop hooks sweep any
leftover remote wrapper or driver (wrapper first — a live wrapper would
respawn its driver), then the board-side processes. Stop the sim last on the
Ubuntu host with `bash examples/webots/sim/stop.sh`.

## Scene NPU models

Scene runs four fixed-shape FP16 RKNN graphs on the RK1828:

- `yoloworld.rknn`: YOLO-World v2s detector, with a fixed class vocabulary.
- `mobilesam.rknn`: MobileSAM image encoder at 1024×1024.
- `mobilesam_decoder.rknn`: MobileSAM prompt and mask decoder (one box per
  call, low-res mask token 0).
- `clip_image.rknn`: OpenAI CLIP ViT-B/32 visual tower at 224×224.

Image resizing and letterboxing, YOLO box decoding and NMS, and mask
upsampling and thresholding stay on the CPU. The CLIP ViT-B/32 text tower
also runs on the CPU; its checkpoint is `/root/weights/clip/ViT-B-32.pt`.
MobileSAM does not load a Torch checkpoint at runtime.

The board bundle at `/root/robonix/dualmap/model/npu/` contains each graph
with its `.weight` file and `npu_meta.json`. Keep the
detector vocabulary in sync with `SCENE_DUALMAP_CLASSES`.

The NPU backend itself (`use_npu` / `npu_model_dir` in DualMap's
`config/system_config.yaml`) lives in the `li041/DualMap` fork at commit
`c1a294d` — `SCENE_DUALMAP_ROOT` (`/opt/dualmap` on the board) must be that
checkout, not the upstream `Eku127/DualMap` `157235e` that
`docker/Dockerfile.dualmap` pins for the CPU path. At this commit both
MobileSAM stages run from the NPU bundle (`mobilesam.rknn` +
`mobilesam_decoder.rknn`, one box per call, low-res mask token 0, letterbox
pad cropped off the logits before upsampling); only the CLIP text tower
stays on torch:

```sh
git clone --filter=blob:none https://github.com/li041/DualMap.git /opt/dualmap
git -C /opt/dualmap checkout c1a294d
```

`convert_aibox_models.py deploy --dualmap-repo /opt/dualmap` performs the
same clone-and-pin as part of bundle assembly.

`export mobilesam_decoder` imports `SamOnnxModel` from a pinned
`li041/MobileSAM` checkout (`--mobilesam-repo`):

```sh
git clone --filter=blob:none https://github.com/li041/MobileSAM.git ~/MobileSAM
git -C ~/MobileSAM checkout 0cd7b9d
```

The graphs come from `system/scene/scripts/`: `convert_aibox_models.py`
self-exports and builds the YOLO-World detector and both MobileSAM graphs
(`deploy` assembles the bundle and can clone the pinned DualMap checkout via
`--dualmap-repo`), and `fetch_zoo_aibox_models.py` fetches and
converts the CLIP ViT-B/32 graph from the rknn-model-zoo. Numeric checks of the finished
graphs run on the board: the PC toolkit can only simulate a graph it built in
the same process, since `load_rknn` of a finished `.rknn` needs the board
runtime. A prebuilt, board-validated bundle of these graphs is published as
the [`aibox-npu-bundle`](https://github.com/li041/DualMap/releases/tag/aibox-npu-bundle)
release on the DualMap fork.
