"""
Simple Python client wrapper for Robonix Core services.

This module provides easy-to-use Python interfaces for Query and Register services,
simplifying the usage of robonix_core services in ROS2 nodes.
"""

import rclpy
from typing import Optional, List
from dataclasses import dataclass


@dataclass
class QueryResult:
    """Result of a Query service call."""
    success: bool
    error_message: str = ""
    input_channels: List[str] = None
    output_channels: List[str] = None
    input_names: List[str] = None
    output_names: List[str] = None
    input_types: List[str] = None
    output_types: List[str] = None

    def __post_init__(self):
        """Initialize empty lists if None."""
        if self.input_channels is None:
            self.input_channels = []
        if self.output_channels is None:
            self.output_channels = []
        if self.input_names is None:
            self.input_names = []
        if self.output_names is None:
            self.output_names = []
        if self.input_types is None:
            self.input_types = []
        if self.output_types is None:
            self.output_types = []
    
    def get_input_channel(self, name: str) -> Optional[str]:
        """
        Get input channel by parameter name.
        
        Args:
            name: Parameter name (e.g., 'target_pose')
        
        Returns:
            Channel name if found, None otherwise
        
        Example:
            ```python
            result = client.query('cap::grasp.move')
            pose_topic = result.get_input_channel('target_pose')
            ```
        """
        try:
            index = self.input_names.index(name)
            return self.input_channels[index] if index < len(self.input_channels) else None
        except ValueError:
            return None
    
    def get_output_channel(self, name: str) -> Optional[str]:
        """
        Get output channel by parameter name.
        
        Args:
            name: Parameter name (e.g., 'image', 'status')
        
        Returns:
            Channel name if found, None otherwise
        
        Example:
            ```python
            result = client.query('cap::vision.capture_rgb')
            image_topic = result.get_output_channel('image')
            ```
        """
        try:
            index = self.output_names.index(name)
            return self.output_channels[index] if index < len(self.output_channels) else None
        except ValueError:
            return None


@dataclass
class RegisterResult:
    """Result of a Register service call."""
    success: bool
    error_message: str = ""


class QueryClient:
    """
    Simple client for Query service.
    
    Example:
        ```python
        from robonix_core.client import QueryClient
        
        client = QueryClient(node)
        result = client.query('cap::vision.capture_rgb')
        if result.success:
            image_topic = result.get_output_channel('image')
        ```
    """
    
    def __init__(self, node, service_name: str = '/rbnx/srv/query', timeout: float = 5.0):
        """
        Initialize Query client.
        
        Args:
            node: ROS2 node instance
            service_name: Service name (default: '/rbnx/srv/query')
            timeout: Timeout for service calls in seconds (default: 5.0)
        """
        self.node = node
        self.service_name = service_name
        self.timeout = timeout
        self._client = None
        self._ensure_client()
    
    def _ensure_client(self):
        """Ensure service client is created."""
        if self._client is None:
            try:
                from robonix_core.srv import Query
                self._client = self.node.create_client(Query, self.service_name)
            except ImportError:
                raise ImportError(
                    "robonix_core.srv.Query not available. "
                    "Make sure robonix_core package is built and sourced."
                )
    
    def query(
        self, 
        std_name: str, 
        requirements: Optional[List[str]] = None,
        wait_for_service: bool = True
    ) -> QueryResult:
        """
        Query a capability or skill by standard name.
        
        Args:
            std_name: Standard name (e.g., 'cap::vision.capture_rgb')
            requirements: Optional list of requirements/filters
            wait_for_service: Whether to wait for service to be available (default: True)
        
        Returns:
            QueryResult with success status and channel information
        
        Example:
            ```python
            result = client.query('cap::vision.capture_rgb')
            if result.success:
                image_topic = result.get_output_channel('image')
                if image_topic:
                    print(f"Image topic: {image_topic}")
            ```
        """
        self._ensure_client()
        
        if requirements is None:
            requirements = []
        
        # Wait for service if requested
        if wait_for_service:
            if not self._client.wait_for_service(timeout_sec=self.timeout):
                return QueryResult(
                    success=False,
                    error_message=f"Service {self.service_name} not available"
                )
        
        # Create request
        request = self._client.srv_type.Request()
        request.std_name = std_name
        request.requirements = requirements
        
        # Call service
        future = self._client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
        
        if not future.done():
            return QueryResult(
                success=False,
                error_message="Service call timeout"
            )
        
        try:
            response = future.result()
            return QueryResult(
                success=response.success,
                error_message=response.error_message,
                input_channels=list(response.input_channels),
                output_channels=list(response.output_channels),
                input_names=list(response.input_names),
                output_names=list(response.output_names),
                input_types=list(response.input_types),
                output_types=list(response.output_types),
            )
        except Exception as e:
            return QueryResult(
                success=False,
                error_message=f"Error getting response: {str(e)}"
            )


class RegisterClient:
    """
    Simple client for Register service.
    
    Example:
        ```python
        from robonix_core.client import RegisterClient
        
        client = RegisterClient(node)
        result = client.register(
            package_name='demo_rgb_provider',
            package_type='capability',
            std_name='cap::vision.capture_rgb',
            description='Capture RGB image',
            code_path='/opt/demo_rgb_package_v1',
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
            print("Registration successful")
        ```
    """
    
    def __init__(self, node, service_name: str = '/rbnx/srv/register', timeout: float = 5.0):
        """
        Initialize Register client.
        
        Args:
            node: ROS2 node instance
            service_name: Service name (default: '/rbnx/srv/register')
            timeout: Timeout for service calls in seconds (default: 5.0)
        """
        self.node = node
        self.service_name = service_name
        self.timeout = timeout
        self._client = None
        self._ensure_client()
    
    def _ensure_client(self):
        """Ensure service client is created."""
        if self._client is None:
            try:
                from robonix_core.srv import Register
                self._client = self.node.create_client(Register, self.service_name)
            except ImportError:
                raise ImportError(
                    "robonix_core.srv.Register not available. "
                    "Make sure robonix_core package is built and sourced."
                )
    
    def register(
        self,
        package_name: str,
        package_type: str,  # 'capability' or 'skill'
        std_name: str,
        description: str,
        code_path: str,
        input_names: Optional[List[str]] = None,
        input_ros_types: Optional[List[str]] = None,
        input_channels: Optional[List[str]] = None,
        output_names: Optional[List[str]] = None,
        output_ros_types: Optional[List[str]] = None,
        output_channels: Optional[List[str]] = None,
        config_services: Optional[List[str]] = None,
        config_names: Optional[List[str]] = None,
        wait_for_service: bool = True
    ) -> RegisterResult:
        """
        Register a capability or skill.
        
        Args:
            package_name: Package name
            package_type: 'capability' or 'skill'
            std_name: Standard name (e.g., 'cap::vision.capture_rgb')
            description: Description of the capability/skill
            code_path: Code path
            input_names: List of input parameter names
            input_ros_types: List of input ROS message types
            input_channels: List of input topic channels
            output_names: List of output parameter names
            output_ros_types: List of output ROS message types
            output_channels: List of output topic channels
            config_services: List of configuration service names
            config_names: List of configuration parameter names
            wait_for_service: Whether to wait for service to be available (default: True)
        
        Returns:
            RegisterResult with success status
        """
        self._ensure_client()
        
        # Initialize empty lists if None
        if input_names is None:
            input_names = []
        if input_ros_types is None:
            input_ros_types = []
        if input_channels is None:
            input_channels = []
        if output_names is None:
            output_names = []
        if output_ros_types is None:
            output_ros_types = []
        if output_channels is None:
            output_channels = []
        if config_services is None:
            config_services = []
        if config_names is None:
            config_names = []
        
        # Wait for service if requested
        if wait_for_service:
            if not self._client.wait_for_service(timeout_sec=self.timeout):
                return RegisterResult(
                    success=False,
                    error_message=f"Service {self.service_name} not available"
                )
        
        # Create request
        request = self._client.srv_type.Request()
        request.package_name = package_name
        request.package_type = package_type
        request.std_name = std_name
        request.description = description
        request.code_path = code_path
        request.input_names = input_names
        request.input_ros_types = input_ros_types
        request.input_channels = input_channels
        request.output_names = output_names
        request.output_ros_types = output_ros_types
        request.output_channels = output_channels
        request.config_services = config_services
        request.config_names = config_names
        
        # Call service
        future = self._client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
        
        if not future.done():
            return RegisterResult(
                success=False,
                error_message="Service call timeout"
            )
        
        try:
            response = future.result()
            return RegisterResult(
                success=response.success,
                error_message=response.error_message
            )
        except Exception as e:
            return RegisterResult(
                success=False,
                error_message=f"Error getting response: {str(e)}"
            )

