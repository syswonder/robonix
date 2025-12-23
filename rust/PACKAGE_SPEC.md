# Robonix Package Specification

## Overview

This document defines the specification for robonix provider packages. A package can provide multiple primitives (prm), services (srv), and skills (skl).

**Important**: Robonix does not care how developers implement primitives, services, and skills. Developers can use any technology stack (Python, C++, Rust, ROS2, etc.). Robonix only requires packages to provide standard manifest files and necessary control scripts, and provide ROS2 communication interfaces.

## Package Directory Structure

A standard robonix provider package should contain the following structure:

```
provider_package/
├── rbnx_manifest.yaml    # Robonix package manifest (required)
└── [source code]         # Source code (implementation-related, any technology stack)
```

## Required Configuration Files

### 1. rbnx_manifest.yaml

This is the core configuration file for robonix packages, defining all primitives, services, and skills provided by the package.

#### Format Specification

```yaml
package:
  name: string              # Package name
  version: string           # Version number (semantic versioning)
  description: string       # Description
  maintainer: string        # Maintainer
  maintainer_email: string  # Maintainer email
  license: string           # License
  build_script: string      # Optional: build script path (relative to package root, e.g., "rbnx/build.sh"). If omitted, defaults to "rbnx/build.sh"

primitives:
  - name: string            # Standard primitive name (e.g., prm::arm_move_ee)
    input_schema: string    # JSON string: {"argname0":"/topic0", ...}
    output_schema: string   # JSON string: {"argname1":"/topic1", ...}
    metadata: string        # JSON string: metadata for instance filtering (e.g., {"resolution":">=720p"}, {"index":0})
    version: string         # Implementation version (e.g., "1.0.0", "1.0.0-alpha", "2.0.0-beta.1")
    start_script: string    # Optional: start script path (relative to package root, e.g., "rbnx/start_capture_rgb.sh")
    stop_script: string     # Optional: stop script path (relative to package root, e.g., "rbnx/stop_capture_rgb.sh")
    # Note: provider is automatically set to package name by CLI during registration
    # Note: The same package can provide multiple implementations of the same standard primitive name, distinguished by different version values

services:
  - name: string            # Standard service name (e.g., spatial_map, semantic_map, task_plan)
    srv_type: string        # ROS2 service type (e.g., "robonix_sdk/srv/service/spatial_map/GetSpatialMap")
    entry: string           # Actual ROS2 service name (e.g., "/mapping/get_spatial_map")
    metadata: string        # JSON string: metadata for instance filtering (e.g., {"model":"deepseek"}, {"backend":"webots"})
    version: string         # Implementation version (e.g., "1.0.0", "1.0.0-alpha", "2.0.0-beta.1")
    start_script: string    # Optional: start script path (relative to package root, e.g., "rbnx/start_spatial_map.sh")
    stop_script: string     # Optional: stop script path (relative to package root, e.g., "rbnx/stop_spatial_map.sh")
    # Note: provider is automatically set to package name by CLI during registration
    # Note: The same package can provide multiple implementations of the same standard service name, distinguished by different version values

skills:
  - name: string            # Skill name (e.g., close_window)
    type: string            # Skill type: "basic" | "rtdl"
    start_topic: string     # Skill start topic (e.g., "/robot1/skill/close_window/start")
    status_topic: string    # Status feedback topic (e.g., "/robot1/skill/close_window/status")
    # For basic skills:
    entry: string           # Basic skill entry point (required if type="basic", e.g., "/path/to/skill_executable" or "python3 /path/to/skill.py")
    # For RTDL skills:
    skill_dir: string       # Skill directory path (required if type="rtdl", relative to package root, e.g., "skills/close_window")
    main_rtdl: string       # Main RTDL file name (required if type="rtdl", e.g., "close_window.rtdl")
    start_args: string      # JSON string: input parameter schema (e.g., '{"room":"string"}')
    status: string          # JSON string: status feedback schema (e.g., '{"state":"string","result":"any"}')
    metadata: string         # JSON string: metadata for instance filtering (e.g., '{"domain":"indoor","robot":"arm6dof"}')
    version: string         # Skill version (e.g., "1.0.0", "1.0.0-alpha")
    start_script: string    # Optional: start script path (relative to package root, e.g., "rbnx/start_pick.sh")
    stop_script: string     # Optional: stop script path (relative to package root, e.g., "rbnx/stop_pick.sh")
    # Note: provider is automatically set to package name by CLI during registration
```

**Important Notes:**
- **The `name` field for primitive/service is required**: Must specify the standard name in the manifest (e.g., `prm::arm_move_ee`, `spatial_map`) to identify which standard specification the primitive/service conforms to
- **Primitives and services must conform to standard specifications**: The system will validate input/output schemas (for primitives) and service types (for services) against the spec during registration
- **Multiple implementations**: The same package can provide multiple implementations of the same standard primitive/service name. All implementations share the same `provider` (package name), but are distinguished by different `version` values. Version can follow semantic versioning (e.g., "1.0.0") or include suffixes (e.g., "1.0.0-alpha", "2.0.0-beta.1")
- **Skills are flexible**: Skills do not need to conform to a spec. Skills can be either "basic" (static program) or "rtdl" (RTDL-based). Basic skills require `entry` field, RTDL skills require `skill_dir` and `main_rtdl` fields
- **JSON fields**: Fields like `input_schema`, `output_schema`, `metadata`, `start_args`, `status` must be valid JSON strings
- **Multiple instances**: A package can provide multiple instances of the same primitive/service/skill (distinguished by different `version` or `metadata`)

#### Field Descriptions

**Package Fields:**
- `name`: Package name
- `version`: Semantic version number (e.g., 1.0.0)
- `description`: Package functionality description
- `maintainer`: Maintainer name
- `maintainer_email`: Maintainer email
- `license`: License type (e.g., Apache-2.0)
- `build_script`: Build script path (optional), relative path to package root, e.g., `rbnx/build.sh`. If omitted, CLI will default to `rbnx/build.sh`. If neither exists, the build command will skip the package

**Primitive Fields:**
- `name`: Standard primitive name, must start with `prm::`, format is `prm::category.action` (e.g., `prm::arm_move_ee`). Name and schema definitions are defined by the standard specification (spec)
- `input_schema`: JSON string mapping input argument names to ROS2 topic channels, e.g., `'{"pose":"/arm/pose_goal"}'`
- `output_schema`: JSON string mapping output argument names to ROS2 topic channels, e.g., `'{"success":"/arm/status"}'`
- `metadata`: JSON string for instance filtering, e.g., `'{"resolution":">=720p"}'` or `'{"index":0}'`
- `version`: Implementation version string. Can follow semantic versioning (e.g., "1.0.0") or include suffixes (e.g., "1.0.0-alpha", "2.0.0-beta.1"). Used to distinguish different implementations of the same standard primitive name from the same package
- `start_script`: Optional. Start script path (relative to package root, e.g., `"rbnx/start_capture_rgb.sh"`). Used to start the primitive's ROS2 node or process. If not specified, the primitive will not be started via CLI
- `stop_script`: Optional. Stop script path (relative to package root, e.g., `"rbnx/stop_capture_rgb.sh"`). Used for cleanup when stopping the primitive. CLI will also manage the process by PID
- **Note**: `provider` is automatically set to the package name by CLI during registration (no need to specify in manifest)

**Service Fields:**
- `name`: Standard service name (e.g., `spatial_map`, `semantic_map`, `task_plan`, `plan_simulate`, `result_feedback`). Name and service type are defined by the standard specification (spec)
- `srv_type`: ROS2 service type, e.g., `"robonix_sdk/srv/service/spatial_map/GetSpatialMap"`
- `entry`: Actual ROS2 service name that implements this service, e.g., `"/mapping/get_spatial_map"`
- `metadata`: JSON string for instance filtering, e.g., `'{"model":"deepseek"}'` or `'{"backend":"webots"}'`
- `version`: Implementation version string. Can follow semantic versioning (e.g., "1.0.0") or include suffixes (e.g., "1.0.0-alpha", "2.0.0-beta.1"). Used to distinguish different implementations of the same standard service name from the same package
- `start_script`: Optional. Start script path (relative to package root, e.g., `"rbnx/start_spatial_map.sh"`). Used to start the service's ROS2 node or process. If not specified, the service will not be started via CLI
- `stop_script`: Optional. Stop script path (relative to package root, e.g., `"rbnx/stop_spatial_map.sh"`). Used for cleanup when stopping the service. CLI will also manage the process by PID
- **Note**: `provider` is automatically set to the package name by CLI during registration (no need to specify in manifest)

**Skill Fields:**
- `name`: Skill name (e.g., `close_window`, `pick_object`). Skills are flexible and do not need to conform to a spec
- `type`: Skill type, must be either `"basic"` or `"rtdl"`. Basic skills are static programs provided by developers, RTDL skills are RTDL-based skills generated from task execution
- `start_topic`: ROS2 topic name for starting the skill (triggering execution), e.g., `"/robot1/skill/close_window/start"`
- `status_topic`: ROS2 topic name for receiving skill status updates, e.g., `"/robot1/skill/close_window/status"`
- `entry`: **Required for basic skills**. Basic skill entry point (e.g., executable path or command, e.g., `"/path/to/skill_executable"` or `"python3 /path/to/skill.py"`)
- `skill_dir`: **Required for RTDL skills**. Directory path (relative to package root) containing the skill's RTDL files and data, e.g., `"skills/close_window"`
- `main_rtdl`: **Required for RTDL skills**. Main RTDL file name, e.g., `"close_window.rtdl"`
- `start_args`: JSON string describing the input parameter schema, e.g., `'{"room":"string"}'`
- `status`: JSON string describing the status feedback schema, e.g., `'{"state":"string","result":"any"}'`
- `metadata`: JSON string for instance filtering, e.g., `'{"domain":"indoor","robot":"arm6dof"}'`
- `version`: Skill version (e.g., `"1.0.0"`, `"1.0.0-alpha"`)
- `start_script`: Optional. Start script path (relative to package root, e.g., `"rbnx/start_pick.sh"`). Used to start the skill's process (e.g., Python script that listens to `start_topic`). If not specified, the skill will not be started via CLI
- `stop_script`: Optional. Stop script path (relative to package root, e.g., `"rbnx/stop_pick.sh"`). Used for cleanup when stopping the skill. CLI will also manage the process by PID
- **Note**: `provider` is automatically set to the package name by CLI during registration (no need to specify in manifest)

**Example:**

```yaml
primitives:
  - name: prm::arm_move_ee
    # Spec definition: INPUT: {"pose":"geometry_msgs/PoseStamped"}, OUTPUT: {"success":"bool"}
    input_schema: '{"pose":"/arm/pose_goal"}'
    output_schema: '{"success":"/arm/status"}'
    metadata: '{"robot":"arm6dof"}'
    version: 1.0.0
    start_script: rbnx/start_arm_move.sh
    stop_script: rbnx/stop_arm_move.sh

  - name: prm::arm_move_ee
    # Another implementation of the same standard primitive
    input_schema: '{"pose":"/arm/pose_goal"}'
    output_schema: '{"success":"/arm/status"}'
    metadata: '{"robot":"arm7dof"}'
    version: 2.0.0-beta.1
    start_script: rbnx/start_arm_move_v2.sh
    stop_script: rbnx/stop_arm_move_v2.sh

  - name: prm::camera_capture
    # Spec definition: OUTPUT: {"image":"sensor_msgs/Image"}
    input_schema: '{}'
    output_schema: '{"image":"/camera/image"}'
    metadata: '{"resolution":"720p","index":0}'
    version: 1.0.0
    start_script: rbnx/start_camera_capture.sh
    stop_script: rbnx/stop_camera_capture.sh

services:
  - name: spatial_map
    srv_type: robonix_sdk/srv/service/spatial_map/GetSpatialMap
    entry: /mapping/get_spatial_map
    metadata: '{"supported_types":["2d","3d","cloud"]}'
    version: 1.0.0
    start_script: rbnx/start_spatial_map.sh
    stop_script: rbnx/stop_spatial_map.sh

  - name: task_plan
    srv_type: robonix_sdk/srv/service/task_plan/PlanTask
    entry: /planner/plan
    metadata: '{"model":"qwen2.5-vl","capabilities":["navigation","manipulation"],"rtdl_type":"BT"}'
    version: 1.0.0
    start_script: rbnx/start_task_plan.sh
    stop_script: rbnx/stop_task_plan.sh

skills:
  - name: close_window
    type: basic
    start_topic: /robot1/skill/close_window/start
    status_topic: /robot1/skill/close_window/status
    entry: python3 /path/to/close_window_skill.py
    start_args: '{"room":"string"}'
    status: '{"state":"string","result":"any"}'
    metadata: '{"domain":"indoor","capability":["navigation","manipulation"]}'
    version: 1.0.0
    start_script: rbnx/start_close_window.sh
    stop_script: rbnx/stop_close_window.sh

  - name: pick_object
    type: rtdl
    start_topic: /robot1/skill/pick_object/start
    status_topic: /robot1/skill/pick_object/status
    skill_dir: skills/pick_object
    main_rtdl: pick_object.rtdl
    start_args: '{"target_label":"string"}'
    status: '{"state":"string","result":"any"}'
    metadata: '{"domain":"indoor","robot":"arm6dof"}'
    version: 1.0.0
    start_script: rbnx/start_pick_object.sh
    stop_script: rbnx/stop_pick_object.sh
```

**Multiple Instance Notes:**
- The same package can provide multiple instances of the same primitive/service/skill
- For primitives and services: Multiple implementations of the same standard name are distinguished by different `version` values (all share the same `provider` which is the package name)
- For skills: Multiple instances can be distinguished by different `version` or `metadata` values
- When querying, you can filter by `metadata` or `version` to select specific instances

### 2. rbnx/ Directory

(Optional) It is recommended to use the `rbnx/` directory to contain all robonix-specific configuration and scripts.

**Important**: 
- Primitives, services, and skills may require start/stop scripts if they run as separate processes or ROS2 nodes that need to be managed by the CLI
- Start scripts are used to launch the primitive/service/skill process (e.g., ROS2 nodes, Python scripts)
- Stop scripts are used for cleanup when stopping the process (CLI will also manage the process by PID)
- If `start_script` is not specified in the manifest, the item will not be started via CLI (it may be started manually or by other means)
- A package can have an optional build script `rbnx/build.sh` (or specified in the manifest through the `build_script` field) for compilation, dependency installation, and other build operations

#### Build Script (build_script)

The package's build script is used for compilation, dependency installation, and other build operations.

Script Requirements:
- Must be an executable file (`chmod +x`)
- Should be executed in the package root directory
- Should handle error cases and return appropriate exit codes (0 for success, non-zero for failure)
- Script path is relative to package root directory
- If the script does not exist, the build command will skip the package (no error)

**Default Location**: If `build_script` is not specified in the manifest, CLI will default to `rbnx/build.sh`.

#### Start/Stop Scripts

Start and stop scripts are used to manage primitive/service/skill processes. These scripts are specified in the manifest via `start_script` and `stop_script` fields.

**Start Scripts:**
- Used to launch the primitive/service/skill process (e.g., ROS2 nodes, Python scripts)
- Must be an executable file (`chmod +x`)
- Should use `exec` to start the target process so CLI can properly manage the process
- Should set necessary environment variables (ROS2, Python paths, robonix-sdk setup, etc.)
- Should handle error cases and return appropriate exit codes
- Script path is relative to package root directory (e.g., `"rbnx/start_capture_rgb.sh"`)
- If `start_script` is not specified, the item will not be started via CLI

**Stop Scripts:**
- Used for cleanup when stopping the process
- CLI will also manage the process by PID, but stop scripts can be used for additional cleanup
- Must be an executable file (`chmod +x`)
- Script path is relative to package root directory (e.g., `"rbnx/stop_capture_rgb.sh"`)
- Optional: If not specified, CLI will only stop the process by PID

**Naming Convention:**
- Start scripts: `rbnx/start_<name>.sh` (e.g., `rbnx/start_capture_rgb.sh`, `rbnx/start_pick.sh`)
- Stop scripts: `rbnx/stop_<name>.sh` (e.g., `rbnx/stop_capture_rgb.sh`, `rbnx/stop_pick.sh`)

## Standard Specification Validation

Primitives and standard services must conform to the standard specifications defined in `robonix-core`. The system will validate during registration:

1. **Primitives**: 
   - Primitive names must match standard specifications
   - Input/output schemas must exactly match the specifications
   - Schema validation ensures topic mappings are correct

2. **Services**:
   - Service names must match standard specifications (e.g., `spatial_map`, `semantic_map`, `task_plan`)
   - Service types must match the specifications
   - Service entry points must be valid ROS2 service names

3. **Skills**:
   - Skills are flexible and do not need to conform to a spec
   - Skills can be either "basic" (static program) or "rtdl" (RTDL-based)
   - Basic skills must provide valid start/status topics and entry point
   - RTDL skills must provide valid start/status topics and RTDL files

## Registration Process

After installation, packages need to register their provided primitives, services, and skills through the CLI command `rbnx deploy register <recipe_file>`.

### Recipe File Format

Recipe files define the packages to register and their corresponding primitives/services/skills:

```yaml
name: recipe_name
description: Optional description

packages:
  - name: package_name
    primitives:                # Optional, if not specified, register all primitives
      - prm::arm_move_ee
    services:                  # Optional, if not specified, register all services
      - spatial_map
    skills:                    # Optional, if not specified, register all skills
      - close_window
```

### Deployment Workflow

The deployment process is divided into multiple steps, executed in order:

1. **Registration Phase** (`rbnx deploy register <recipe_file>`): Register each primitive/service/skill to robonix-core
   - Each registration request includes `provider` (package name) and metadata
   - **Note**: Registration does not start processes, it only registers primitive/service/skill information to the system

2. **Build Phase** (`rbnx deploy build [target]`): Build packages (compile, install dependencies, etc.)
   - Can specify `target` parameter to build a specific package (e.g., `"demo_rgb_provider"`), or use `"all"` to build all packages (default)
   - Execute each package's build script (`rbnx/build.sh` or `build_script` specified in manifest)
   - If the build script does not exist, skip the package (no error)
   - **Note**: Build is optional, if a package does not need building (e.g., pure Python scripts), this step can be skipped

3. **Start Process Phase** (`rbnx deploy start [target]`): Start corresponding processes (if needed)
   - Primitives, services, and skills that have `start_script` defined in the manifest will be started
   - Items without `start_script` will be skipped (they may be started manually or by other means)
   - All process stdout/stderr will be redirected to log files (stored in `{package_storage_path}/logs/`)
   - CLI will record all processes started on this machine and their process information
   - Can specify `target` parameter to start a specific item, or use `"all"` to start all (default)

4. **Stop Process Phase** (`rbnx deploy stop [target]`): Stop running processes
   - Can specify `target` parameter to stop a specific item, or use `"all"` to stop all (default)

5. **Restart Process Phase** (`rbnx deploy restart [target]`): Restart processes
   - Equivalent to executing `stop` then `start`

6. **View Status** (`rbnx deploy status`): View status of all running processes

7. **Unregister Phase** (`rbnx deploy unregister <target>`): Unregister primitive/service/skill from the system
   - Need to stop all related processes first
   - Can unregister entire recipe, package, or specific primitive/service/skill

**Complete Workflow Example:**
```bash
# 1. Register recipe
rbnx deploy register demo_recipe.yaml

# 2. Build all packages (optional, if compilation etc. is needed)
rbnx deploy build

# 3. Start all processes
rbnx deploy start

# 4. View status
rbnx deploy status

# 5. Stop all processes
rbnx deploy stop

# 6. Unregister recipe
rbnx deploy unregister demo_recipe.yaml
```

Standard primitive and service specifications are defined in [`robonix-core/src/specs_table.rs`](robonix-core/src/specs_table.rs).

## Examples

For complete examples, please refer to:
- Package example: `rust/provider/demo_package/` directory (contains complete `rbnx_manifest.yaml` and start/stop scripts)
- Recipe example: `rust/robonix-cli/demo_recipe.yaml` file
