# Demo Service Provider Package

SPDX-License-Identifier: MulanPSL-2.0

This package provides demo implementations of Robonix services:
- **semantic_map**: Provides object-level representation of the environment using front camera and Qwen3-VL VLM
- **task_plan**: Converts natural language task descriptions to RTDL code using DeepSeek LLM

## Services

### Semantic Map Service

The semantic map service (`semantic_map_service`) uses the front camera primitive to capture RGB images and camera info, then uses Qwen3-VL VLM to detect objects and estimate their 3D coordinates in camera frame. It provides an object-level representation of the environment.

**Service Interface**: `/demo_service/semantic_map/query`
**Service Type**: `robonix_sdk/srv/service/semantic_map/QuerySemanticMap`

**Features**:
- Queries front camera primitive from OS using RobonixClient (from robonix_sdk.client)
- Subscribes to RGB images and camera_info topics
- Uses Qwen3-VL VLM to detect objects and estimate 3D positions in camera coordinate system
- Returns objects with camera frame coordinates (world coordinates are empty for now)

### Task Plan Service

The task plan service (`task_plan_service`) converts natural language task descriptions into RTDL (Robot Task Description Langauge) code. It uses DeepSeek LLM API for intelligent task planning.

**Service Interface**: `/demo_service/task_plan/plan`
**Service Type**: `robonix_sdk/srv/service/task_plan/PlanTask`

## Configuration

### API Keys Configuration

Both services require API keys to function. Create a `.env` file in the package root directory (same level as `setup.py`) with the following content:

```
DEEPSEEK_API_KEY=sk-your-deepseek-api-key-here
QWEN3_VL_API_KEY=sk-your-qwen3-vl-api-key-here
```

### Qwen3-VL API Configuration

The semantic map service requires a Qwen3-VL API key to function. Follow these steps to configure it:

1. **Get a Qwen3-VL API Key**
   - Visit [DashScope Platform](https://dashscope.aliyun.com/)
   - Sign up or log in to your account
   - Navigate to API keys section
   - Create a new API key

2. **Configure the API Key**
   - Add `QWEN3_VL_API_KEY` to your `.env` file:
     ```
     QWEN3_VL_API_KEY=sk-your-actual-api-key-here
     ```
   - The service uses model `qwen-vl-plus` with base URL `https://dashscope.aliyuncs.com/compatible-mode/v1`

3. **Verify Configuration**
   - The service will automatically load the `.env` file on startup
   - Check service logs to confirm Qwen3-VL API is initialized
   - Ensure front camera primitive is registered and running

### DeepSeek API Configuration

The task plan service requires a DeepSeek API key to function. Follow these steps to configure it:

1. **Get a DeepSeek API Key**
   - Visit [DeepSeek Platform](https://platform.deepseek.com/)
   - Sign up or log in to your account
   - Navigate to API keys section
   - Create a new API key

2. **Configure the API Key**
   - Add `DEEPSEEK_API_KEY` to your `.env` file:
     ```
     DEEPSEEK_API_KEY=sk-your-actual-api-key-here
     ```

3. **Verify Configuration**
   - The service will automatically load the `.env` file on startup
   - If the API key is not found, the service will fall back to simple keyword-based planning
   - Check service logs to confirm DeepSeek API is initialized

## Building and Installation

### Prerequisites

- ROS2 (Humble or later)
- Python 3.8+
- colcon build tools
- **robonix-core** running (start from `rust` with `ROBONIX_WEB_ASSETS_DIR`, `ROBONIX_WEB_PORT`, and `eval $(make source-sdk)`; see [rust/README.md](../../README.md) Step 3)
- DeepSeek API key (for task planning service)
- Qwen3-VL API key (for semantic map service)
- Front camera primitive registered and running (for semantic map service)

### Build

```bash
# From package directory
colcon build --packages-select demo_service_provider
```

Or use robonix-cli:

```bash
rbnx deploy build
```

### Install Dependencies

The package requires the following Python packages (automatically installed via setup.py):
- `python-dotenv`: For loading environment variables from .env file
- `openai`: For DeepSeek and Qwen3-VL API clients (OpenAI-compatible)
- `robonix_sdk`: For querying primitives from Robonix OS (use `from robonix_sdk.client import RobonixClient`)
- `cv-bridge`: For converting ROS images to OpenCV format
- `numpy`: For numerical operations
- `Pillow`: For image processing

## Usage

### Start Services

After registering a recipe that includes this package (`rbnx deploy register <recipe.yaml>`), start services via robonix-cli (pattern matches recipe item names):

```bash
# Start semantic map service (pattern matches srv::semantic_map)
rbnx deploy start semantic_map

# Start task plan service (pattern matches srv::task_plan)
rbnx deploy start task_plan

# Or start all items in the active recipe
rbnx deploy start all
```

### Service Endpoints

- **Semantic Map**: `/demo_service/semantic_map/query`
- **Task Plan**: `/demo_service/task_plan/plan`

## RTDL Format

The task plan service generates RTDL code in JSON format:

```json
[
  {
    "type": "skill",
    "name": "pick",
    "params": {
      "target": "cup_001"
    }
  },
  {
    "type": "skill",
    "name": "place",
    "params": {
      "destination": "table_001"
    }
  }
]
```

## Troubleshooting

### API Not Working

**Qwen3-VL API Issues:**
1. Check that `.env` file exists and contains `QWEN3_VL_API_KEY`
2. Verify the API key is correct and has sufficient credits
3. Check service logs for API errors
4. Ensure front camera primitive is registered and publishing images

**DeepSeek API Issues:**
1. Check that `.env` file exists and contains `DEEPSEEK_API_KEY`
2. Verify the API key is correct and has sufficient credits
3. Check service logs for API errors
4. The service will automatically fall back to simple planning if API fails

### Service Not Starting

1. Ensure ROS2 is properly sourced: `source /opt/ros/humble/setup.bash`
2. Check that robonix-sdk is built and sourced
3. Verify Python dependencies are installed
4. Check service logs in `rbnx/` directory

## License

MulanPSL-2.0

