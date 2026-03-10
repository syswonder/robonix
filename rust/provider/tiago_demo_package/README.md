# Tiago Simulator Package

Before registering this package or starting programs via recipes, start the simulator and supporting services manually.

First, you need to make sure you've installed this package and `rbnx package list` will show it.

```bash
rbnx package build tiago_demo_package
```

## 1. Start Webots simulator

```bash
cd rust/provider/tiago_demo_package
./run.sh
```

## 2. Start Nav2 (new terminal)

```bash
cd rust/provider/tiago_demo_package/nav2_webots_tiago
./run.sh
```

## 3. (Optional) Start RViz2 (new terminal)

```bash
cd rust/provider/tiago_demo_package
./start_rviz.sh
```

## 4. Start robonix-server and register (new terminal)

From the **rust** directory, source the SDK and start robonix-server with environment variables (no CLI flags). Then use `rbnx` to register and deploy.

```bash
cd rust
eval $(make source-sdk)
ROBONIX_WEB_ASSETS_DIR="$(pwd)/robonix-server/web" \
ROBONIX_WEB_PORT=8000 \
RUST_LOG=robonix_server=info \
robonix-server
```

In another terminal (with `eval $(make source-sdk)` from `rust`): register the recipe, start services, and create tasks. See the main [rust/README.md](../../README.md) for full steps (Step 4–8).