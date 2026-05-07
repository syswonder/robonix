# audio_macos_bridge

Local-only audio primitive that runs on a Linux atlas host but routes
mic + speaker through a daemon on a macOS box across the LAN. The
manifest-side cap (`com.robonix.primitive.audio`) is identical to the
default `audio_driver` package, so liaison / scene / pilot don't see a
difference; the only thing that changes is where the actual ADC/DAC
lives.

Both halves (the Linux primitive package and the macOS server) live
in this directory and are committed to the repo. The macOS box just
needs `mac_server/server.py` + `mac_server/requirements.txt` — no
robonix install, no codegen.

## Layout

```
audio_macos_bridge/
├── package_manifest.yaml          # cap_id = com.robonix.primitive.audio
├── audio_macos_bridge/node.py     # Linux side: gRPC servicer ↔ WebSocket client
├── mac_server/server.py           # macOS side: headless WebSocket server
├── mac_server/server_web.py       # macOS side: same protocol + browser debug UI
├── mac_server/requirements.txt    # sounddevice + websockets
└── scripts/{build,start}.sh       # rbnx codegen + entry
```

## Ports

| Port | Side | Protocol | What |
| --- | --- | --- | --- |
| `60000` | macOS | WebSocket (`0.0.0.0`) | `/mic`, `/speaker`, `/health`, `/devices`, `/vu`, `/log`, `/set_device` |
| `60001` | macOS | HTTP (`127.0.0.1`) | `server_web.py` debug UI — open in a browser |

`60000` listens on all interfaces so the Linux atlas host can dial it
across LAN/Tailscale. `60001` only binds loopback by default; pass
`--ui-host 0.0.0.0` if you want to drive the UI from another machine
(no auth — don't expose to the public internet).

## Setup (one-shot)

### macOS side

```sh
cd ~/robonix-scripts/mac_server          # or wherever you scp'd this dir
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt

# Option A — headless, you already know the device ids
python3 server.py --list-devices
python3 server.py --port 60000 --input-device 1 --output-device 2

# Option B — web UI for picking devices + watching VU + tailing log
python3 server_web.py --port 60000        # WS 60000 + UI on 60001
open http://localhost:60001/
```

Leave it running. The first time, macOS will prompt for mic + network
permission — accept both. The web UI auto-skips devices whose name
matches `airpods|bluetooth|iphone|ipad` because BT-HFP can't open a
16 kHz mono `RawInputStream` on Apple silicon.

### Linux side

In `examples/webots/robonix_manifest.yaml` swap the audio_driver entry
for the bridge:

```yaml
primitive:
  # - name: audio_driver
  #   path: ./primitives/audio_driver
  - name: audio_macos_bridge
    path: ./primitives/audio_macos_bridge
    config:
      host: 192.168.1.42      # macOS LAN IP
      port: 60000
```

Then `rbnx build && rbnx boot` as usual. Driver(CMD_INIT) probes
`/health` on the macOS box; if unreachable, this primitive defers
instead of advertising dead mic/speaker streams.

## Wire format

Both directions: 16 kHz, mono, s16le PCM. Frames are 100 ms (3200 B
each). `/mic` is server-stream binary frames; `/speaker` is
client-stream binary frames. `/health` is a single text JSON message.

No auth, no TLS — assume LAN. Don't expose `mac_server` on a public
interface.
