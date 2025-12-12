# Robonix

**Robonix** is an open-source embodied intelligence framework built with Rust and ROS2, implementing the EAIOS (Embodied AI Operating System) architecture.

## Architecture

Robonix follows the EAIOS architecture with four core components:

- **Task Manager**: Global scheduling and control core, responsible for task parsing, planning, and execution coordination
- **Skill Library**: Stores reusable skills that can be called at runtime
- **Service Registry**: Manages standardized algorithm capabilities (perception, planning, evaluation, verification)
- **Primitive Abstraction Layer**: Provides standardized hardware capability mapping, managing access to actuators and sensors

## Quick Start

### Prerequisites

- ROS2 Humble: https://docs.ros.org/en/humble/Installation.html
- Rust (stable): https://rustup.rs/

### Using Docker (Recommended)

The easiest way to get started is using the pre-built Docker image:

```bash
cd docker
./run.sh  # Pulls and runs docker.io/enkerewpo/robonix_ros:latest
```

Or build locally:

```bash
cd docker
./run.sh -b  # Builds and runs local image
```

### Manual Setup

1. **Build robonix-cli**:

```bash
cd rust/robonix-cli
cargo build
```

2. **Setup packages directory**:

```bash
mkdir -p ~/.robonix
rm -rf ~/.robonix/packages
ln -s "$(realpath ../provider/)" ~/.robonix/packages
export FASTRTPS_DEFAULT_PROFILES_FILE=
```

3. **Build robonix-msg**:

```bash
cd rust/robonix-msg
./build_ros2.sh
```

4. **Start robonix-core** (in a separate terminal):

```bash
cd rust/robonix-core
cargo run --
```

robonix-core provides the following EAIOS API services:
- **Primitive API** (`/rbnx/prm/*`): Primitive registration and query
- **Service API** (`/rbnx/srv/*`): Standard service registration and query
- **Skill API** (`/rbnx/skl/*`): Skill registration and query
- **Task API** (`/rbnx/task/*`): Task submission, status query, and result retrieval

5. **Configure robonix-cli**:

```bash
cd rust/robonix-cli
cargo run -- config --set-msg-path ../robonix-msg
cargo run -- config -s
```

## Install and Register Packages

### Install Packages

```bash
cd rust/robonix-cli

# Install package from GitHub
cargo run -- package install --github https://github.com/enkerewpo/demo-package-01-robonix
cargo run -- package list

# Build all packages
cargo run -- package build all
```

### Register Packages

Use recipe files to register primitives, services, and skills:

```bash
cd rust/robonix-cli

# Register recipe
cargo run -- deploy register demo_recipe.yaml

# Start all registered packages
cargo run -- deploy start

# View status
cargo run -- deploy status
```

## Create and Execute Tasks

### Create Task

```bash
cd rust/robonix-cli

# Create task from natural language
cargo run -- task create "Pick up the red box on the table"

# View task details
cargo run -- task get task_0
```

### Task Execution Flow

1. **Task Submission**: Task submitted via `/rbnx/task/submit`, status: `pending`
2. **Task Planning**: Task manager calls planning service, converts to RTDL code, status: `planning`
3. **Plan Simulation** (optional): Validates RTDL feasibility, status: `simulating`
4. **Task Execution**: Executes RTDL code, status: `running`
5. **Result Feedback** (optional): Validates execution results, status: `finished` or `failed`

## Core Concepts

### Primitives

Standardized hardware capability mapping (e.g., `prm::arm_move_ee`, `prm::camera_capture`). Must conform to specifications defined in EAIOS.

### Services

Standardized algorithm capabilities:
- **Spatial Map Service** (`spatial_map`): Geometric structure information
- **Semantic Map Service** (`semantic_map`): Entity-level representation
- **Task Planning Service** (`task_plan`): Natural language to RTDL conversion
- **Plan Simulation Service** (`plan_simulate`): Feasibility validation
- **Result Feedback Service** (`result_feedback`): Execution verification

### Skills

User-defined high-level action logic, written in RTDL (Robot Task Description Language). Skills are flexible and can call primitives, services, or other skills.

Example RTDL:

```python
def skl::close_window(room: str):
    skl::navigate_to(target_label = room)
    srv::semantic_map.update(entity = room)
    pose = srv::semantic_map.query_pose(
        entity_type = "window",
        parent_room = room
    )
    prm::arm_move_ee(pose = pose)
    prm::gripper.close()
    return True
```

## Project Structure

```
robonix/
├── rust/
│   ├── robonix-core/      # Core EAIOS implementation
│   ├── robonix-cli/        # Command-line interface
│   ├── robonix-msg/        # ROS2 message definitions
│   └── provider/           # Example provider packages
├── docker/                 # Docker configuration
└── README.md              # This file
```

## Documentation

- [Rust Quick Start Guide](rust/README.md)
- [Package Specification](rust/PACKAGE_SPEC.md)
- [Docker Setup](docker/README.md)

## Troubleshooting

```bash
# Check if robonix-core is running
ros2 service list | grep rbnx

# Check service types
ros2 service type /rbnx/prm/register
ros2 service type /rbnx/task/submit
```

## License

See LICENSE file for details.
