# Robonix (rust/)

Rust/ROS 2 implementation of the Robonix embodied intelligence framework. This directory contains the RIDL toolchain, interface definitions, runtime, and examples.

## Getting the source (with submodules)

```bash
git clone https://github.com/syswonder/robonix
cd robonix
git submodule update --init --recursive
```

Submodules include ROS message dependencies under `robonix-interfaces/lib/` (rcl_interfaces, common_interfaces) and the docs. **You must run `git submodule update --init --recursive` after cloning**, or ridlc and robonix-server builds will fail.

## Components

| Component | Description |
|-----------|-------------|
| `ridlc` | RIDL compiler; compiles `.ridl` into ROS 2 workspace and generates Python/Rust code |
| `robonix-interfaces` | RIDL definitions and ROS msg packages; `ridl/` for query/command/stream/event, `lib/` for msg deps |
| `robonix-server` | Runtime gRPC meta API for channel registration and resolution |
| `robonix-cli` (rbnx) | Package build, start/stop, manifest validation |
| `examples/` | Python examples (ping, skill, etc.) |

## Quick start

Requirements: Ubuntu 22.04, ROS 2 Humble, Rust, Python 3.10+.

```bash
cd rust
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp   # optional, Zenoh by default

make build
make install
```

Start robonix-server:

```bash
cd rust
./start_server   # or run robonix-server after make install (wrapper requires rust/ dir)
```

**Note**: `make install` installs a wrapper for robonix-server that sources the colcon workspace; the `rust/` directory must stay in place. Do not move or delete it after install.

See [Quick Start](https://github.com/syswonder/robonix-book/blob/main/src/chapter1-getting-started/quickstart.md) for details (run `./scripts/build-highlight.sh` in docs first to build the book).

## Directory layout

```
rust/
├── ridlc/                 # RIDL compiler
├── robonix-interfaces/    # Interface definitions (lib/ is submodule)
├── robonix-server/        # Runtime
├── robonix-cli/           # rbnx CLI
├── examples/              # Python examples
├── provider/              # Provider implementations (some legacy)
└── Makefile               # Top-level build
```

## Documentation

- [RFC001 RIDL](https://github.com/syswonder/robonix-book/blob/main/src/rfc/RFC001-RIDL.md)
- [RFC002 Package Management](https://github.com/syswonder/robonix-book/blob/main/src/rfc/RFC002-Package-Management.md)
- [Package Manifest](ROBONIX_PACKAGE_MANIFEST.md)
