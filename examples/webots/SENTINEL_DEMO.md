# Webots Sentinel demo

The demo manifest seeds the local Sentinel rule file from
`sentinel-rules.v1.json` on first boot. The seed allows administrators to call
`robonix/skill/*` and denies those calls for other users. Rules changed later
through Sentinel persist across restarts.

```bash
rbnx boot -f examples/webots/robonix_manifest.sentinel-demo.yaml
```

The management endpoint listens on `127.0.0.1:50062` and requires an
administrator session. Calls outside `robonix/skill/*` are allowed unless
another rule matches.
