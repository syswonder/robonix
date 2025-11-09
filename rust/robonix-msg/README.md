# Robonix Core Message Definitions

This package provides ROS2 service definitions and a simple Python client library for Robonix Core services.

## Services

- **Query**: Query capability/skill information by standard name
- **Register**: Register a capability or skill with robonix core

## Python Client Library

The package includes a simple Python client library that makes it easy to use robonix_core services without manually creating service clients.

### Installation

After building the package with `colcon build`, source the workspace:

```bash
source install/setup.bash
```

### Usage

#### Query Client

Instead of manually creating a Query service client:

```python
# Old way (complicated)
from robonix_core.srv import Query
query_client = node.create_client(Query, '/rbnx/srv/query')
if not query_client.wait_for_service(timeout_sec=5.0):
    # handle error
request = Query.Request()
request.std_name = 'cap::vision.capture_rgb'
future = query_client.call_async(request)
rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
response = future.result()
# ... process response
```

Use the simplified client:

```python
# New way (simple!)
from robonix_core.client import QueryClient

query_client = QueryClient(node)
result = query_client.query('cap::vision.capture_rgb')

if result.success:
    image_topic = result.output_channels[0]
    print(f"Image topic: {image_topic}")
else:
    print(f"Error: {result.error_message}")
```

#### Register Client

Similarly, registering capabilities/skills is much simpler:

```python
from robonix_core.client import RegisterClient

register_client = RegisterClient(node)
result = register_client.register(
    package_name='demo_package',
    package_type='capability',
    std_name='cap::vision.capture_rgb',
    description='Capture RGB image from camera',
    code_path='/opt/demo_package',
    input_names=[],
    input_ros_types=[],
    input_channels=[],
    output_names=['image'],
    output_ros_types=['sensor_msgs/msg/Image'],
    output_channels=['/demo_rgb/image'],
    config_services=[],
    config_names=[]
)

if result.success:
    print("Registration successful!")
else:
    print(f"Error: {result.error_message}")
```

### API Reference

#### QueryClient

```python
client = QueryClient(node, service_name='/rbnx/srv/query', timeout=5.0)
result = client.query(std_name, requirements=None, wait_for_service=True)
```

**QueryResult** fields:
- `success: bool` - Whether the query succeeded
- `error_message: str` - Error message if failed
- `input_channels: List[str]` - Input topic channels
- `output_channels: List[str]` - Output topic channels
- `input_names: List[str]` - Input parameter names
- `output_names: List[str]` - Output parameter names
- `input_types: List[str]` - Input ROS message types
- `output_types: List[str]` - Output ROS message types

#### RegisterClient

```python
client = RegisterClient(node, service_name='/rbnx/srv/register', timeout=5.0)
result = client.register(
    package_name: str,
    package_type: str,  # 'capability' or 'skill'
    std_name: str,
    description: str,
    code_path: str,
    input_names: List[str] = None,
    input_ros_types: List[str] = None,
    input_channels: List[str] = None,
    output_names: List[str] = None,
    output_ros_types: List[str] = None,
    output_channels: List[str] = None,
    config_services: List[str] = None,
    config_names: List[str] = None,
    wait_for_service: bool = True
)
```

**RegisterResult** fields:
- `success: bool` - Whether the registration succeeded
- `error_message: str` - Error message if failed
