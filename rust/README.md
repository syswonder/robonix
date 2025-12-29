# Robonix Quick Start

wheatfox

## Overview

Robonix is a robot task execution system that allows you to:
- Register and manage robot capabilities (primitives, services, skills)
- Submit natural language tasks
- Automatically plan and execute tasks using registered capabilities

This guide will walk you through setting up and using Robonix step by step.

## Prerequisites

- ROS2 environment (Humble recommended)
- Rust toolchain
- Python 3.10

If you want to use the prebuilt robonix ROS2 dev container:

```bash
cd docker
./run.sh # pull image and enter container, you can use CTRL+P CTRL+Q to detach
./run.sh -b # build a local image if you modified some docker config
```

## Step 1: Build and Install Robonix

Navigate to the rust directory and build all components:

```bash
cd rust
# View all available commands
make help
# Build SDK (ROS2 interface package)
make build-sdk
# Build CLI and Core
make build
# Install binaries to local rust folder, normally ~/.cargo/bin
make install
```

After installation, you can run binaries directly (if rust toolchains are correctly installed and configured):
- `rbnx` - Robonix CLI
- `rbnx-daemon` - Robonix daemon
- `robonix-core` - Robonix core service

## Step 2: Setup Development Environment (Optional)

**Important**: The `setup-dev` command creates a symbolic link from `~/.robonix/packages` to the `provider` directory. This is only needed if you want to use packages from the local `provider` folder for development/testing.

**If you don't need to use local provider packages, skip this step.**

```bash
# This links ~/.robonix/packages to rust/provider/
# Only run this if you need to use local provider packages
make setup-dev
```

## Step 3: Start robonix-core

robonix-core provides the core services for the system. You need to start it in a separate terminal before using the CLI.

```bash
# In terminal 1: Start robonix-core
eval $(make source-sdk) # source robonix-sdk environment
robonix-core
```

robonix-core will start the following services:
- **Primitive API** (`/rbnx/prm/*`): Primitive registration and query
- **Service API** (`/rbnx/srv/*`): Standard service registration and query
- **Skill API** (`/rbnx/skl/*`): Skill registration and query
- **Task API** (`/rbnx/task/*`): Task submission, status query, and result retrieval

Keep this terminal running.

### Configuring Log Levels

You can control the verbosity of robonix-core logs by setting the `RUST_LOG` environment variable:

```bash
# Show only info, warn, and error messages (default)
robonix-core

# Show debug messages for robonix-core module
RUST_LOG=robonix_core=debug robonix-core
# Show debug messages for all modules, you can see rustdds logs too for example
RUST_LOG=debug robonix-core
# Show debug for task_manager only
RUST_LOG=robonix_core::task_manager=debug robonix-core
# Show trace messages (most verbose)
RUST_LOG=robonix_core=trace robonix-core
# Customize log levels for different modules
RUST_LOG=robonix_core::task_manager=debug,robonix_core=info,rustdds=error robonix-core
```
## Step 4: Configure robonix-cli

In a new terminal (terminal 2), configure the CLI:

```bash
cd rust

eval $(make source-sdk) # source robonix-sdk environment for new terminal

# View current configuration
rbnx config --show # or just -s
# Set robonix-sdk path (if not already set)
rbnx config --set-sdk-path $(realpath ./robonix-sdk)
rbnx config --show
```

## Step 5: Install and Build Packages

Install packages that provide primitives, services, and skills:

```bash
# Install a package from GitHub (example)
rbnx package install --github https://github.com/enkerewpo/demo-package-01-robonix
# List installed packages
rbnx package list
# View package details
rbnx package info <package_name>
# Build all packages
rbnx package build all
```

## Step 6: Register Packages

Use a recipe file to register primitives, services, and skills to robonix-core:

```bash
# rbnx daemon restart # if anything went wrong

# Register recipe (automatically registers primitives, services, and skills)
rbnx deploy register demo_recipe.yaml
# View registration status
rbnx deploy status
```

## Step 7: Start Services and Skills

Start the registered services and skills:

```bash
# Start all registered packages
rbnx deploy start
# View status
rbnx deploy status
# To stop all packages
rbnx deploy stop
# To restart
rbnx deploy restart
```

## Step 8: Create and Execute Tasks

Now you can submit tasks in natural language:

```bash
# wait several seconds for semantic map service to be able to provide an object graph, then you can issue a task
# Create a task
rbnx task create "Pick up the red box on the table"
# View task status
rbnx task get task_0
# Cancel a task if needed
rbnx task cancel task_0
```

### Task Execution Flow

When you create a task, the system automatically:

1. **Submits** the task (status: `pending`)
2. **Plans** the task by calling the task planning service, converting natural language to RTDL code (status: `planning`)
3. **Simulates** the plan (optional, status: `simulating`)
4. **Executes** the RTDL code, calling skills in sequence (status: `running`)
5. **Completes** with result feedback (status: `finished` or `failed`)

## Step 9: Using Standard Services

The system provides standard services that can be queried:

- **Spatial Map Service** (`spatial_map`): Geometric structure information
- **Semantic Map Service** (`semantic_map`): Object-level environment representation
- **Task Planning Service** (`task_plan`): Converts natural language to RTDL code
- **Plan Simulation Service** (`plan_simulate`): Validates task plan feasibility
- **Result Feedback Service** (`result_feedback`): Validates execution results

These services are automatically called by the task manager during task execution. You can also query them directly via ROS2 services.

## Common Commands Reference

### Build Commands
```bash
make build          # Build robonix-cli and robonix-core
make build-cli      # Build robonix-cli only
make build-core     # Build robonix-core only
make build-sdk      # Build robonix-sdk ROS2 interface package
```

### Install Commands
```bash
make install        # Install all binaries to ~/.local/bin
make install-cli    # Install robonix-cli binaries only
make install-core   # Install robonix-core binary only
```

### Run Commands (after installation)
```bash
rbnx <command>      # Run CLI with any command
rbnx-daemon <command>  # Run daemon
robonix-core        # Run robonix-core
```

### Environment Commands
```bash
make env            # Show environment information
eval $(make source-sdk)  # Source SDK environment in current shell
```

### Development Commands
```bash
make setup-dev      # Link provider directory (only if needed for local packages)
make fmt            # Format code using cargo fmt
make check          # Run cargo check
make clean          # Clean build artifacts
```

## Troubleshooting

### Check if robonix-core is running
```bash
ros2 service list | grep rbnx
```

### Check service availability
```bash
ros2 service type /rbnx/prm/register
ros2 service type /rbnx/srv/register
ros2 service type /rbnx/skl/register
ros2 service type /rbnx/task/submit
```

### Clean up all ROS2 processes
```bash
pkill -9 -f "ros2|robonix|rclpy|rclcpp|demo_rgb_provider"
```

## System Architecture

### Components

- **Primitives**: Standardized hardware capability mapping (e.g., `prm::arm.move.ee`), must conform to specifications
- **Services**: Standardized algorithm capabilities (e.g., `spatial_map`, `semantic_map`), must conform to specifications
- **Skills**: User-defined high-level action logic, written in RTDL, flexible and do not need to conform to specifications
- Skills can call primitives and services, and can also call other skills

### RTDL Format

RTDL (Robot Task Description Language) is the task description language. Example:

```python
def skl::close_window(room: str):
    skl::navigate_to(target_label = room)
    srv::semantic_map.update(entity = room)
    pose = srv::semantic_map.query_pose(
        entity_type = "window",
        parent_room = room
    )
    prm::arm.move.ee(pose = pose)
    prm::gripper.close()
    return True
```

### Data Types

- System supports Robonix custom message types (Point3D, Object, BoundingBox, etc.)
- Also supports standard ROS2 message types (geometry_msgs, sensor_msgs, std_msgs, etc.)

## Notes

1. **robonix-sdk Setup**: robonix-sdk setup will be automatically sourced by startup scripts. CLI will:
   - First check configuration file (set via `rbnx config --set-sdk-path`)
   - Then check `ROBONIX_SDK_PATH` environment variable

2. **Development Setup**: The `make setup-dev` command creates a symbolic link from `~/.robonix/packages` to the local `provider` directory. This is useful for development but not required for normal usage. Only run it if you need to use packages from the local provider folder.
