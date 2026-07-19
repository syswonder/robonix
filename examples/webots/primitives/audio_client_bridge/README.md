# audio_client_bridge

Audio primitive that runs on the Robonix host but routes mic + speaker through
the selected `robonix-client` device. It implements the same audio contracts
as `audio_driver`; both providers can be registered at once, and Liaison uses
the provider selected by the client for F2, voice-steer, and hands-free turns.

Both halves (the Linux primitive package and the macOS server) live
in this directory and are committed to the repo. The client machine just
needs `client_audio_server/server.py` + `client_audio_server/requirements.txt` — no
robonix install, no codegen.

## Layout

```
audio_client_bridge/
├── package_manifest.yaml          # provider_id = com.robonix.primitive.audio
├── audio_client_bridge/main.py     # Linux side: gRPC servicer ↔ WebSocket client
├── client_audio_server/server.py           # client side: headless WebSocket server
├── client_audio_server/server_web.py       # client side: same protocol + browser debug UI
├── client_audio_server/requirements.txt    # sounddevice + websockets
└── scripts/{build,start}.sh       # rbnx codegen + entry
```

## Transport

The default is a reverse WebSocket transport. During driver init the primitive
binds `0.0.0.0:60002`, publishes its endpoint through Atlas, and waits for
`robonix-client` to connect. The deployment never stores a client IP.

The client discovers this endpoint through Atlas after the operator selects
`audio_client_bridge` as an input or output route. Its Audio page owns local
device selection, audio level display, and the reverse connection lifecycle.

## Setup (one-shot)

### Client side

```sh
robonix-client --host <robot-host> --port 50051
```

Open **Audio**, select input/output primitives and devices, then apply the
route. The client starts its local audio endpoint and connects to the selected
reverse bridge when needed. macOS may request microphone and local-network
permission on first use.

### Linux side

In `examples/webots/robonix_manifest.yaml`, keep both providers registered:

```yaml
primitive:
  - name: audio_driver
    path: ./primitives/audio_driver
  - name: audio_client_bridge
    path: ./primitives/audio_client_bridge
    config:
      transport: reverse
      listen_port: 60002
```

Then `rbnx build && rbnx boot` as usual. The bridge stays active while no
client is connected; it is selected only when the operator chooses it.

## Wire format

Both directions use 16 kHz, mono, s16le PCM. Frames are 100 ms (3200 B).

The current bridge has no authentication or TLS. Use a trusted LAN, Tailscale,
or an authenticated tunnel; do not expose the bridge port publicly.
