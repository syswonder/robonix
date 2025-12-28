# Demo RGB Provider Package

SPDX-License-Identifier: MulanPSL-2.0

This package provides demo implementations of Robonix capabilities and skills:
- **cap::vision.capture_rgb**: RGB camera capability that publishes random color images
- **cap::grasp.move**: Grasp movement capability
- **skl::pick**: Pick skill that combines vision and grasp capabilities

## Components

### Capabilities

#### cap::vision.capture_rgb

RGB camera capability that publishes random color images to `/demo_rgb/image` topic.

**Publisher**: `/demo_rgb/image` (sensor_msgs/Image)

#### cap::grasp.move

Grasp movement capability that subscribes to pose goals and publishes status.

**Subscriber**: `/demo_grasp/pose_goal` (geometry_msgs/PoseStamped)
**Publisher**: `/demo_grasp/pose_status` (std_msgs/Bool)

### Skills

#### skl::pick

Pick skill that combines vision and grasp capabilities to perform pick operations.

**Start Topic**: `/robot1/skill/pick/start` (std_msgs/String)
**Status Topic**: `/robot1/skill/pick/status` (std_msgs/String)

## Building and Installation

### Prerequisites

- ROS2 (Humble or later)
- Python 3.8+
- colcon build tools
- numpy (for image generation)

### Build

```bash
# From package directory
colcon build --packages-select demo_rgb_provider
```

Or use robonix-cli:

```bash
rbnx deploy build
```

### Install Dependencies

The package requires the following Python packages (automatically installed via setup.py):
- `numpy`: For image array generation

## Usage

### Start Capabilities and Skills

Components can be started individually or via robonix-cli:

```bash
# Start RGB capture capability
rbnx deploy start cap::vision.capture_rgb

# Start grasp move capability
rbnx deploy start cap::grasp.move

# Start pick skill
rbnx deploy start skl::pick

# Or start all components
rbnx deploy start all
```

### Stop Components

```bash
# Stop specific component
rbnx deploy stop skl::pick

# Stop all components
rbnx deploy stop all
```

## Topics and Services

### Published Topics

- `/demo_rgb/image` (sensor_msgs/Image): RGB camera images
- `/demo_grasp/pose_status` (std_msgs/Bool): Grasp movement status
- `/robot1/skill/pick/status` (std_msgs/String): Pick skill execution status

### Subscribed Topics

- `/demo_grasp/pose_goal` (geometry_msgs/PoseStamped): Target pose for grasp movement
- `/robot1/skill/pick/start` (std_msgs/String): Pick skill start command

## Example Usage

### Using Pick Skill

1. Start the required capabilities:
   ```bash
   rbnx deploy start cap::vision.capture_rgb
   rbnx deploy start cap::grasp.move
   ```

2. Start the pick skill:
   ```bash
   rbnx deploy start skl::pick
   ```

3. Send a pick command:
   ```bash
   ros2 topic pub /robot1/skill/pick/start std_msgs/String "data: 'pick cup_001'"
   ```

4. Monitor skill status:
   ```bash
   ros2 topic echo /robot1/skill/pick/status
   ```

## Troubleshooting

### Components Not Starting

1. Ensure ROS2 is properly sourced: `source /opt/ros/humble/setup.bash`
2. Check that robonix-sdk is built and sourced
3. Verify Python dependencies are installed (numpy)
4. Check component logs in `rbnx/` directory

### Image Not Publishing

1. Verify RGB publisher is running: `ros2 topic list | grep demo_rgb`
2. Check topic data: `ros2 topic echo /demo_rgb/image`
3. Ensure numpy is installed: `pip3 install numpy`

## License

Apache-2.0

