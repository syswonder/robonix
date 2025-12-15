# Robonix Quick Start

wheatfox

## Environment Setup

First, you need ROS2 environment and Rust.

```bash
cd rust # at robonix src root folder
cd robonix-cli
cargo build
# in robonix-cli, run:
mkdir -p ~/.robonix; rm -rf ~/.robonix/packages; ln -s "$(realpath ../provider/)" ~/.robonix/packages;
export FASTRTPS_DEFAULT_PROFILES_FILE=

# then build the robonix-sdk
cd robonix-sdk
./build_ros2.sh
```

## Start robonix-core

Before starting everything, you need to start robonix-core first (in a separate terminal).

```bash
cd robonix-core
cargo run --
```

robonix-core will start the following EAIOS API services:
- **Primitive API** (`/rbnx/prm/*`): Primitive registration and query
- **Service API** (`/rbnx/srv/*`): Standard service registration and query (e.g., spatial map, semantic map, task planning, etc.)
- **Skill API** (`/rbnx/skl/*`): Skill registration and query
- **Task API** (`/rbnx/task/*`): Task submission, status query, and result retrieval

Then you can use `robonix-cli` in another terminal.

## Configure robonix-cli

```bash
cd robonix-cli
# View current configuration
cargo run -- config -s

# Set robonix-sdk path
cargo run -- config --set-sdk-path ../robonix-sdk
cargo run -- config -s
```

## Install and Register Packages (Primitives, Services, Skills)

### Install Packages

```bash
cd robonix-cli

# Install package from GitHub
cargo run -- package install --github https://github.com/enkerewpo/demo-package-01-robonix
cargo run -- package list
cargo run -- package info demo_package_01_github

# Build all packages, or specify package name
cargo run -- package build all
```

### Register Packages to robonix-core (using recipe)

Use recipe files to register packages. The recipe specifies which primitives, services, and skills need to be registered.

```bash
cd robonix-cli

# Build all packages
cargo build && cargo run -- daemon restart # if daemon code was modified
cargo run -- package build all

# Register recipe (will automatically register primitives, services, and skills specified in recipe)
cargo run -- deploy register demo_recipe.yaml

# Start all registered packages
cargo run -- deploy start

# View status
cargo run -- deploy status

# Restart/Stop
cargo run -- deploy restart
cargo run -- deploy stop

# Clean up all ROS2 processes
# pkill -9 -f "ros2|robonix|rclpy|rclcpp|demo_rgb_provider"
```

## Using Standard Services

The system provides multiple standard services, including:

- **Spatial Map Service** (`spatial_map`): Provides geometric structure information of the environment (2D/3D occupancy grids, point clouds, etc.)
- **Semantic Map Service** (`semantic_map`): Provides entity-level environment representation (objects, rooms, robots, etc.)
- **Task Planning Service** (`task_plan`): Converts natural language tasks to RTDL code
- **Plan Simulation Service** (`plan_simulate`): Validates task plan feasibility in simulation environment
- **Result Feedback Service** (`result_feedback`): Validates task execution results

These services can be queried via `/rbnx/srv/query` and will be automatically called by the task manager during task execution.

```bash
# View registered services
# Call ROS2 service /rbnx/srv/query
```

## Create and Execute Tasks

### Create Task (Natural Language Input)

```bash
cd robonix-cli

# Create task
cargo run -- task create "Pick up the red box on the table"

# View task list
cargo run -- task list

# View task details
cargo run -- task get task_0
```

### Task Execution Flow

After creating a task, the system will automatically execute the following flow:

1. **Task Submission**: Submit task via `/rbnx/task/submit`, task status is `pending`
2. **Task Planning**: Task manager calls task planning service (`task_plan`), converts natural language description to RTDL code
   - Planning service will query:
     - All entities in semantic map and their supported skills
     - All registered skills list
     - Available primitives and services
   - Task status changes to `planning`
3. **Plan Simulation** (optional): Task manager calls plan simulation service (`plan_simulate`), validates RTDL code feasibility
   - Task status changes to `simulating`
4. **Task Execution**: Task manager parses RTDL code, executes each skill call in sequence
   - Send parameters to skill's start_topic
   - Receive status updates from skill's status_topic
   - Task status changes to `running`
5. **Result Feedback** (optional): Task manager calls result feedback service (`result_feedback`), validates execution results
   - Task status changes to `finished` or `failed`

### View Task Status

```bash
# List all tasks
cargo run -- task list

# Get task details (including generated RTDL code)
cargo run -- task get <task_id>

# Cancel task
cargo run -- task cancel <task_id>
```

## Notes

1. **robonix-sdk Setup**: robonix-sdk setup will be automatically sourced by startup scripts. CLI will:
   - First check configuration file (set via `rbnx config --set-sdk-path`)
   - Then check `ROBONIX_SDK_PATH` environment variable

2. **Primitives, Services, and Skills**:
   - **Primitives**: Standardized hardware capability mapping (e.g., `prm::arm_move_ee`), must conform to specifications
   - **Services**: Standardized algorithm capabilities (e.g., `spatial_map`, `semantic_map`), must conform to specifications
   - **Skills**: User-defined high-level action logic, written in RTDL, flexible and do not need to conform to specifications
   - Skills can call primitives and services, and can also call other skills

3. **RTDL Format**:
   - RTDL (Robot Task Description Language) is the task description language
   - Format example:
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

4. **Data Types**:
   - System supports Robonix custom message types (Point3D, Entity, BoundingBox, etc.)
   - Also supports standard ROS2 message types (geometry_msgs, sensor_msgs, std_msgs, etc.)

## Troubleshooting

```bash
# Check if robonix-core is running
ros2 service list | grep rbnx

# Check if primitive register service is available
ros2 service type /rbnx/prm/register

# Check if service register service is available
ros2 service type /rbnx/srv/register

# Check if skill register service is available
ros2 service type /rbnx/skl/register

# Check if task submit service is available
ros2 service type /rbnx/task/submit
```
