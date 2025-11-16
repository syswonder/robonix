#!/usr/bin/env python3
"""
Robonix Client - High-level client for Robonix Core services.

This module provides a thread-safe, concurrent client for all Robonix Core services.
It handles service discovery, connection management, and concurrent request handling.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from concurrent.futures import ThreadPoolExecutor, Future, TimeoutError as FutureTimeoutError
import threading
import time
from typing import Optional, Dict, List, Tuple, Any


class RobonixClient(Node):
    """
    High-level client for Robonix Core services.
    
    This client provides thread-safe, concurrent access to all Robonix services.
    It uses a thread pool to handle concurrent requests and automatically manages
    service connections.
    
    Example:
        ```python
        import rclpy
        from robonix_core.client import RobonixClient
        
        rclpy.init()
        client = RobonixClient()
        
        # Query a capability
        result = client.query_capability('cap::vision.capture_rgb')
        if result.success:
            print(f"Found at: {result.output_channels[0]}")
        
        # Add an entity (concurrent-safe)
        entity = Entity()
        # ... set entity fields ...
        result = client.add_entity(entity)
        
        rclpy.spin_once(client, timeout_sec=0.1)  # Process callbacks
        ```
    """
    
    def __init__(self, node_name: str = 'robonix_client', max_workers: int = 10):
        """
        Initialize Robonix Client.
        
        Args:
            node_name: Name for the ROS2 node
            max_workers: Maximum number of worker threads for concurrent requests
        """
        super().__init__(node_name)
        
        # Thread pool for concurrent service calls
        self._executor = ThreadPoolExecutor(max_workers=max_workers)
        self._lock = threading.Lock()
        
        # Service clients (lazy initialization)
        # Note: Using _service_clients instead of _clients to avoid conflict with Node._clients
        self._service_clients: Dict[str, Any] = {}
        self._client_locks: Dict[str, threading.Lock] = {}
        
        # Service names
        self._service_names = {
            'ping': '/rbnx/srv/mgmt/ping',
            'query_cap_skl': '/rbnx/srv/mgmt/query_cap_skl',
            'register_cap_skl': '/rbnx/srv/mgmt/register_cap_skl',
            'register_model': '/rbnx/srv/mgmt/register_model',
            'query_model': '/rbnx/srv/mgmt/query_model',
            'add_entity': '/rbnx/srv/perception/add_entity',
            'get_semantic_map': '/rbnx/srv/perception/get_semantic_map',
            'get_spatial_map': '/rbnx/srv/perception/get_spatial_map',
            'add_spatial_map_entry': '/rbnx/srv/perception/add_spatial_map_entry',
            'get_map_status': '/rbnx/srv/perception/get_map_status',
            'create_task': '/rbnx/srv/planning/create_task',
            'get_task': '/rbnx/srv/planning/get_task',
            'list_tasks': '/rbnx/srv/planning/list_tasks',
            'cancel_task': '/rbnx/srv/planning/cancel_task',
        }
        
        # Create QoS profile matching server configuration
        # Server uses: KeepLast depth=1000, Reliable, Volatile
        self._service_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.get_logger().info('RobonixClient initialized')
    
    def _get_client(self, service_name: str, service_type):
        """
        Get or create a service client (thread-safe, lazy initialization).
        
        Args:
            service_name: Key name for the service (e.g., 'ping', 'query_cap_skl')
            service_type: ROS2 service type class
            
        Returns:
            Service client instance
        """
        if service_name not in self._service_clients:
            with self._lock:
                # Double-check locking pattern
                if service_name not in self._service_clients:
                    full_name = self._service_names[service_name]
                    # Create client with matching QoS profile
                    client = self.create_client(
                        service_type, 
                        full_name,
                        qos_profile=self._service_qos
                    )
                    self._service_clients[service_name] = client
                    self._client_locks[service_name] = threading.Lock()
        
        return self._service_clients[service_name]
    
    def _call_service(self, service_name: str, service_type, request, timeout_sec: float = 5.0):
        """
        Call a service (thread-safe, supports concurrent calls).
        
        Args:
            service_name: Key name for the service
            service_type: ROS2 service type class
            request: Service request object
            timeout_sec: Timeout in seconds
            
        Returns:
            Service response object, or None on timeout/error
        """
        client = self._get_client(service_name, service_type)
        client_lock = self._client_locks[service_name]
        
        # Wait for service to be available (with lock to avoid race condition)
        with client_lock:
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'{service_name} service not available')
                return None
        
        # Make async call (thread-safe - each call gets its own future)
        # Note: call_async is thread-safe, no lock needed here
        future = client.call_async(request)
        
        # Wait for response with timeout
        # Use executor to handle spin in a thread-safe way
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            # Spin once to process callbacks (thread-safe)
            # Use a very short timeout to avoid blocking
            rclpy.spin_once(self, timeout_sec=0.05)
            if not future.done():
                time.sleep(0.01)  # Small sleep to avoid busy waiting
        
        if future.done():
            try:
                return future.result()
            except Exception as e:
                self.get_logger().error(f'{service_name} service call error: {e}')
                return None
        else:
            self.get_logger().warn(f'{service_name} service call timeout after {timeout_sec}s')
            return None
    
    def ping(self, sequence: int = 0, timeout_sec: float = 5.0):
        """
        Ping service for testing.
        
        Args:
            sequence: Sequence number
            timeout_sec: Timeout in seconds
            
        Returns:
            PingResponse or None on error
        """
        try:
            from robonix_core.srv import Ping
            request = Ping.Request()
            request.sequence = sequence
            return self._call_service('ping', Ping, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import Ping service: {e}')
            return None
    
    def query_capability(self, std_name: str, impl_id: str = '', requirements: List[str] = None, timeout_sec: float = 5.0):
        """
        Query a capability or skill.
        
        Args:
            std_name: Standard name (e.g., 'cap::vision.capture_rgb')
            impl_id: Implementation ID (empty for any)
            requirements: Optional requirements/filters
            timeout_sec: Timeout in seconds
            
        Returns:
            QueryCapSklResponse or None on error
        """
        try:
            from robonix_core.srv import QueryCapSkl
            self.get_logger().debug(f'query_capability: Creating QueryCapSkl.Request for {std_name}')
            request = QueryCapSkl.Request()
            self.get_logger().debug(f'query_capability: Setting std_name={std_name}, impl_id={impl_id}')
            request.std_name = std_name
            request.impl_id = impl_id
            # Convert to list to ensure proper type (not dict-like)
            self.get_logger().debug(f'query_capability: requirements type={type(requirements)}, value={requirements}')
            try:
                if requirements is not None:
                    self.get_logger().debug(f'query_capability: Converting requirements to list, type={type(requirements)}')
                    request.requirements = list(requirements)
                    self.get_logger().debug(f'query_capability: requirements converted, type={type(request.requirements)}')
                else:
                    self.get_logger().debug('query_capability: requirements is None, setting to empty list')
                    request.requirements = []
                    self.get_logger().debug(f'query_capability: requirements set to empty list, type={type(request.requirements)}')
            except Exception as e:
                self.get_logger().error(f'query_capability: Error setting requirements: {e}')
                self.get_logger().error(f'query_capability: requirements type={type(requirements)}, value={requirements}')
                import traceback
                self.get_logger().error(f'query_capability: Traceback:\n{traceback.format_exc()}')
                raise
            self.get_logger().debug('query_capability: Calling service...')
            return self._call_service('query_cap_skl', QueryCapSkl, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import QueryCapSkl service: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'query_capability: Unexpected error: {e}')
            import traceback
            self.get_logger().error(f'query_capability: Traceback:\n{traceback.format_exc()}')
            raise
    
    def register_capability(self, package_name: str, package_type: str, std_name: str,
                           impl_id: str, description: str, code_path: str,
                           input_names: List[str], input_ros_types: List[str], input_channels: List[str],
                           output_names: List[str], output_ros_types: List[str], output_channels: List[str],
                           config_services: List[str] = None, config_names: List[str] = None,
                           hostname: str = '', entity_name: str = '', timeout_sec: float = 10.0):
        """
        Register a capability or skill.
        
        Args:
            package_name: Package name
            package_type: 'cap' or 'skl'
            std_name: Standard name
            impl_id: Implementation ID
            description: Description
            code_path: Code path
            input_names: Input parameter names
            input_ros_types: Input ROS types
            input_channels: Input channels (topics)
            output_names: Output parameter names
            output_ros_types: Output ROS types
            output_channels: Output channels (topics)
            config_services: Config service names
            config_names: Config parameter names
            hostname: Hostname
            entity_name: Entity name
            timeout_sec: Timeout in seconds
            
        Returns:
            RegisterCapSklResponse or None on error
        """
        try:
            from robonix_core.srv import RegisterCapSkl
            request = RegisterCapSkl.Request()
            request.package_name = package_name
            request.package_type = package_type
            request.std_name = std_name
            request.impl_id = impl_id
            request.description = description
            request.code_path = code_path
            # Convert all list fields to proper lists to ensure proper type (not dict-like)
            request.input_names = list(input_names) if input_names else []
            request.input_ros_types = list(input_ros_types) if input_ros_types else []
            request.input_channels = list(input_channels) if input_channels else []
            request.output_names = list(output_names) if output_names else []
            request.output_ros_types = list(output_ros_types) if output_ros_types else []
            request.output_channels = list(output_channels) if output_channels else []
            request.config_services = list(config_services) if config_services else []
            request.config_names = list(config_names) if config_names else []
            request.hostname = hostname
            request.entity_name = entity_name
            return self._call_service('register_cap_skl', RegisterCapSkl, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import RegisterCapSkl service: {e}')
            return None
    
    def add_entity(self, entity, timeout_sec: float = 5.0):
        """
        Add an entity to the semantic map.
        
        Args:
            entity: Entity message object
            timeout_sec: Timeout in seconds
            
        Returns:
            AddEntityResponse or None on error
        """
        try:
            from robonix_core.srv import AddEntity
            request = AddEntity.Request()
            request.entity = entity
            return self._call_service('add_entity', AddEntity, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import AddEntity service: {e}')
            return None
    
    def get_semantic_map(self, entity_id: Optional[str] = None, label: Optional[str] = None,
                        path: Optional[str] = None, timeout_sec: float = 5.0):
        """
        Get semantic map entities.
        
        Args:
            entity_id: Filter by entity ID (None for all)
            label: Filter by label
            path: Filter by path
            timeout_sec: Timeout in seconds
            
        Returns:
            GetSemanticMapResponse or None on error
        """
        try:
            from robonix_core.srv import GetSemanticMap
            request = GetSemanticMap.Request()
            request.entity_id = entity_id
            request.label = label
            request.path = path
            return self._call_service('get_semantic_map', GetSemanticMap, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import GetSemanticMap service: {e}')
            return None
    
    def get_spatial_map(self, frame_id: Optional[str] = None, timeout_sec: float = 5.0):
        """
        Get spatial map entries.
        
        Args:
            frame_id: Filter by frame ID (None for all)
            timeout_sec: Timeout in seconds
            
        Returns:
            GetSpatialMapResponse or None on error
        """
        try:
            from robonix_core.srv import GetSpatialMap
            request = GetSpatialMap.Request()
            request.frame_id = frame_id
            return self._call_service('get_spatial_map', GetSpatialMap, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import GetSpatialMap service: {e}')
            return None
    
    def add_spatial_map_entry(self, entry, timeout_sec: float = 5.0):
        """
        Add a spatial map entry.
        
        Args:
            entry: SpatialMapEntry message object
            timeout_sec: Timeout in seconds
            
        Returns:
            AddSpatialMapEntryResponse or None on error
        """
        try:
            from robonix_core.srv import AddSpatialMapEntry
            request = AddSpatialMapEntry.Request()
            request.entry = entry
            return self._call_service('add_spatial_map_entry', AddSpatialMapEntry, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import AddSpatialMapEntry service: {e}')
            return None
    
    def get_map_status(self, timeout_sec: float = 5.0):
        """
        Get map updating status.
        
        Args:
            timeout_sec: Timeout in seconds
            
        Returns:
            GetMapStatusResponse or None on error
        """
        try:
            from robonix_core.srv import GetMapStatus
            request = GetMapStatus.Request()
            return self._call_service('get_map_status', GetMapStatus, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import GetMapStatus service: {e}')
            return None
    
    def register_model(self, model_id: str, model_name: str, model_type: str, provider: str,
                      api_endpoint: str, api_key: Optional[str] = None, description: str = '',
                      capabilities: List[str] = None, timeout_sec: float = 10.0):
        """
        Register an AI model.
        
        Args:
            model_id: Model ID
            model_name: Model name
            model_type: 'llm' or 'vlm'
            provider: Provider name
            api_endpoint: API endpoint URL
            api_key: Optional API key
            description: Description
            capabilities: List of capabilities
            timeout_sec: Timeout in seconds
            
        Returns:
            RegisterModelResponse or None on error
        """
        try:
            from robonix_core.srv import RegisterModel
            request = RegisterModel.Request()
            request.model_id = model_id
            request.model_name = model_name
            request.model_type = 0 if model_type.lower() == 'llm' else 1  # 0=LLM, 1=VLM
            request.provider = provider
            request.api_endpoint = api_endpoint
            request.api_key = api_key
            request.description = description
            request.capabilities = capabilities or []
            return self._call_service('register_model', RegisterModel, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import RegisterModel service: {e}')
            return None
    
    def query_model(self, model_id: Optional[str] = None, model_type: Optional[str] = None,
                   capability: Optional[str] = None, timeout_sec: float = 5.0):
        """
        Query AI models.
        
        Args:
            model_id: Filter by model ID (None for all)
            model_type: Filter by type 'llm' or 'vlm' (None for all)
            capability: Filter by capability (None for all)
            timeout_sec: Timeout in seconds
            
        Returns:
            QueryModelResponse or None on error
        """
        try:
            from robonix_core.srv import QueryModel
            request = QueryModel.Request()
            request.model_id = model_id
            if model_type:
                request.model_type = 0 if model_type.lower() == 'llm' else 1
            request.capability = capability
            return self._call_service('query_model', QueryModel, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import QueryModel service: {e}')
            return None
    
    def create_task(self, natural_language: str, timeout_sec: float = 10.0):
        """
        Create a task from natural language.
        
        Args:
            natural_language: Natural language task description
            timeout_sec: Timeout in seconds
            
        Returns:
            CreateTaskResponse or None on error
        """
        try:
            from robonix_core.srv import CreateTask
            request = CreateTask.Request()
            request.natural_language = natural_language
            return self._call_service('create_task', CreateTask, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import CreateTask service: {e}')
            return None
    
    def get_task(self, task_id: str, timeout_sec: float = 5.0):
        """
        Get task by ID.
        
        Args:
            task_id: Task ID
            timeout_sec: Timeout in seconds
            
        Returns:
            GetTaskResponse or None on error
        """
        try:
            from robonix_core.srv import GetTask
            request = GetTask.Request()
            request.task_id = task_id
            return self._call_service('get_task', GetTask, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import GetTask service: {e}')
            return None
    
    def list_tasks(self, timeout_sec: float = 5.0):
        """
        List all tasks.
        
        Args:
            timeout_sec: Timeout in seconds
            
        Returns:
            ListTasksResponse or None on error
        """
        try:
            from robonix_core.srv import ListTasks
            request = ListTasks.Request()
            return self._call_service('list_tasks', ListTasks, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import ListTasks service: {e}')
            return None
    
    def cancel_task(self, task_id: str, timeout_sec: float = 5.0):
        """
        Cancel a task.
        
        Args:
            task_id: Task ID
            timeout_sec: Timeout in seconds
            
        Returns:
            CancelTaskResponse or None on error
        """
        try:
            from robonix_core.srv import CancelTask
            request = CancelTask.Request()
            request.task_id = task_id
            return self._call_service('cancel_task', CancelTask, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import CancelTask service: {e}')
            return None
    
    def shutdown(self):
        """Shutdown the client and cleanup resources."""
        self._executor.shutdown(wait=True)
        self.destroy_node()


# Convenience function for creating a client instance
def create_client(node_name: str = 'robonix_client', max_workers: int = 10) -> RobonixClient:
    """
    Create a RobonixClient instance.
    
    Args:
        node_name: Name for the ROS2 node
        max_workers: Maximum number of worker threads
        
    Returns:
        RobonixClient instance
    """
    return RobonixClient(node_name, max_workers)

