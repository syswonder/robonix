# Demo Service Provider Package

SPDX-License-Identifier: MulanPSL-2.0

This package provides demo implementations of Robonix services:
- **semantic_map**: Provides object-level representation of the environment
- **task_plan**: Converts natural language task descriptions to RTDL code using DeepSeek LLM

## Services

### Semantic Map Service

The semantic map service (`semantic_map_service`) simulates an object graph by constructing mock objects. It provides an object-level representation on top of spatial maps.

**Service Interface**: `/demo_service/semantic_map/query`
**Service Type**: `robonix_sdk/srv/service/semantic_map/QuerySemanticMap`

### Task Plan Service

The task plan service (`task_plan_service`) converts natural language task descriptions into RTDL (Real-Time Decision Logic) code. It uses DeepSeek LLM API for intelligent task planning.

**Service Interface**: `/demo_service/task_plan/plan`
**Service Type**: `robonix_sdk/srv/service/task_plan/PlanTask`

## Configuration

### DeepSeek API Configuration

The task plan service requires a DeepSeek API key to function. Follow these steps to configure it:

1. **Get a DeepSeek API Key**
   - Visit [DeepSeek Platform](https://platform.deepseek.com/)
   - Sign up or log in to your account
   - Navigate to API keys section
   - Create a new API key

2. **Configure the API Key**
   - Copy the `.env.example` file to `.env` in this package directory:
     ```bash
     cp .env.example .env
     ```
   - Edit `.env` and replace `your_deepseek_api_key_here` with your actual API key:
     ```
     DEEPSEEK_API_KEY=sk-your-actual-api-key-here
     ```
   - Make sure `.env` file is in the package root directory (same level as `setup.py`)

3. **Verify Configuration**
   - The service will automatically load the `.env` file on startup
   - If the API key is not found, the service will fall back to simple keyword-based planning
   - Check service logs to confirm DeepSeek API is initialized

## Building and Installation

### Prerequisites

- ROS2 (Humble or later)
- Python 3.8+
- colcon build tools
- DeepSeek API key (for task planning service)

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
- `openai`: For DeepSeek API client (compatible with DeepSeek API)

## Usage

### Start Services

Services can be started individually or via robonix-cli:

```bash
# Start semantic map service
rbnx deploy start semantic_map

# Start task plan service
rbnx deploy start task_plan

# Or start all services
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

### DeepSeek API Not Working

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

Apache-2.0

