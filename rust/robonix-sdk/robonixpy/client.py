#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Robonix Python Client
#
# Robonix Python Client - High-level client for Robonix Core services.
# This module provides a thread-safe, concurrent client for all Robonix Core services.
# It handles service discovery, connection management, and concurrent request handling.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from concurrent.futures import ThreadPoolExecutor
import threading
import time
import json
from typing import Optional, Dict, List, Any


class RobonixClient(Node):
    """
    High-level client for Robonix Core services.
    
    This client provides thread-safe, concurrent access to all Robonix services.
    It uses a thread pool to handle concurrent requests and automatically manages
    service connections.
    
    Example:
        ```python
        import rclpy
        from robonixpy import RobonixClient
        
        rclpy.init()
        client = RobonixClient()
        
        # Query a primitive
        result = client.query_primitive('prm::camera.capture')
        if result and result.instances:
            instance = result.instances[0]
            print(f"Found primitive: provider={instance.provider}, version={instance.version}")
        
        # Query a skill
        result = client.query_skill('skl::pick', filter={'type': 'basic'})
        if result and result.instances:
            skill = result.instances[0]
            print(f"Found skill: skill_id={skill.skill_id}, type={skill.type}")
        
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
        self._service_clients: Dict[str, Any] = {}
        self._client_locks: Dict[str, threading.Lock] = {}
        
        # Service names
        self._service_names = {
            # Primitive services
            'register_primitive': '/rbnx/prm/register',
            'query_primitive': '/rbnx/prm/query',
            # Service services
            'register_service': '/rbnx/srv/register',
            'query_service': '/rbnx/srv/query',
            # Skill services
            'register_skill': '/rbnx/skl/register',
            'query_skill': '/rbnx/skl/query',
            # Task services
            'submit_task': '/rbnx/task/submit',
            'task_status': '/rbnx/task/status',
            'task_result': '/rbnx/task/result',
            # Ping pong service
            'ping_pong': '/rbnx/ping',
        }
        
        # Create QoS profile matching server configuration
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
            service_name: Key name for the service (e.g., 'query_primitive')
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
        future = client.call_async(request)
        
        # Wait for response with timeout
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            # Spin once to process callbacks (thread-safe)
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
    
    def query_primitive(self, name: str, filter: Optional[Dict[str, Any]] = None, timeout_sec: float = 5.0):
        """
        Query primitives by standard name.
        
        Args:
            name: Standard primitive name (e.g., 'prm::camera.capture')
            filter: Optional filter dictionary for metadata (e.g., {'resolution': '>=720p'})
            timeout_sec: Timeout in seconds
            
        Returns:
            QueryPrimitiveResponse or None on error
        """
        try:
            from robonix_sdk.srv import QueryPrimitive
            request = QueryPrimitive.Request()
            request.name = name
            request.filter = json.dumps(filter) if filter else ''
            return self._call_service('query_primitive', QueryPrimitive, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import QueryPrimitive service: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'query_primitive error: {e}')
            return None
    
    def register_primitive(self, name: str, input_schema: Dict[str, str], output_schema: Dict[str, str],
                          metadata: Optional[Dict[str, Any]] = None, provider: str = '', 
                          version: str = '1.0.0', timeout_sec: float = 10.0):
        """
        Register a primitive.
        
        Args:
            name: Standard primitive name (e.g., 'prm::camera.capture')
            input_schema: Input schema mapping argument names to topic channels
            output_schema: Output schema mapping argument names to topic channels
            metadata: Optional metadata dictionary for instance filtering
            provider: Provider identifier (usually package name)
            version: Implementation version (e.g., '1.0.0', '1.0.0-alpha')
            timeout_sec: Timeout in seconds
            
        Returns:
            RegisterPrimitiveResponse or None on error
        """
        try:
            from robonix_sdk.srv import RegisterPrimitive
            request = RegisterPrimitive.Request()
            request.name = name
            request.input_schema = json.dumps(input_schema)
            request.output_schema = json.dumps(output_schema)
            request.metadata = json.dumps(metadata) if metadata else '{}'
            request.provider = provider
            request.version = version
            return self._call_service('register_primitive', RegisterPrimitive, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import RegisterPrimitive service: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'register_primitive error: {e}')
            return None
    
    def query_service(self, name: str, filter: Optional[Dict[str, Any]] = None, timeout_sec: float = 5.0):
        """
        Query services by standard name.
        
        Args:
            name: Standard service name (e.g., 'spatial_map')
            filter: Optional filter dictionary for metadata
            timeout_sec: Timeout in seconds
            
        Returns:
            QueryServiceResponse or None on error
        """
        try:
            from robonix_sdk.srv import QueryService
            request = QueryService.Request()
            request.name = name
            request.filter = json.dumps(filter) if filter else ''
            return self._call_service('query_service', QueryService, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import QueryService service: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'query_service error: {e}')
            return None
    
    def register_service(self, name: str, srv_type: str, entry: str,
                        metadata: Optional[Dict[str, Any]] = None, provider: str = '',
                        version: str = '1.0.0', timeout_sec: float = 10.0):
        """
        Register a service.
        
        Args:
            name: Standard service name (e.g., 'spatial_map')
            srv_type: ROS2 service type (e.g., 'robonix_sdk/srv/service/spatial_map/GetSpatialMap')
            entry: Actual ROS2 service name (e.g., '/mapping/get_spatial_map')
            metadata: Optional metadata dictionary for instance filtering
            provider: Provider identifier (usually package name)
            version: Implementation version (e.g., '1.0.0', '1.0.0-alpha')
            timeout_sec: Timeout in seconds
            
        Returns:
            RegisterServiceResponse or None on error
        """
        try:
            from robonix_sdk.srv import RegisterService
            request = RegisterService.Request()
            request.name = name
            request.srv_type = srv_type
            request.entry = entry
            request.metadata = json.dumps(metadata) if metadata else '{}'
            request.provider = provider
            request.version = version
            return self._call_service('register_service', RegisterService, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import RegisterService service: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'register_service error: {e}')
            return None
    
    def query_skill(self, name: str, filter: Optional[Dict[str, Any]] = None, timeout_sec: float = 5.0):
        """
        Query skills by name.
        
        Args:
            name: Skill name (e.g., 'skl::pick' or 'pick')
            filter: Optional filter dictionary (e.g., {'type': 'basic'}, {'domain': 'indoor'})
            timeout_sec: Timeout in seconds
            
        Returns:
            QuerySkillResponse or None on error
        """
        try:
            from robonix_sdk.srv import QuerySkill
            request = QuerySkill.Request()
            # Ensure name starts with 'skl::'
            if not name.startswith('skl::'):
                name = f'skl::{name}'
            request.name = name
            request.filter = json.dumps(filter) if filter else ''
            return self._call_service('query_skill', QuerySkill, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import QuerySkill service: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'query_skill error: {e}')
            return None
    
    def register_skill(self, name: str, skill_type: str, start_topic: str, status_topic: str,
                      start_args: Dict[str, str], status: Dict[str, str],
                      metadata: Optional[Dict[str, Any]] = None, provider: str = '',
                      version: str = '1.0.0',
                      entry: Optional[str] = None,  # For basic skills
                      skill_dir: Optional[str] = None,  # For RTDL skills
                      main_rtdl: Optional[str] = None,  # For RTDL skills
                      timeout_sec: float = 10.0):
        """
        Register a skill.
        
        Args:
            name: Skill name (e.g., 'pick')
            skill_type: Skill type ('basic' or 'rtdl')
            start_topic: Skill start topic (e.g., '/robot1/skill/pick/start')
            status_topic: Status feedback topic (e.g., '/robot1/skill/pick/status')
            start_args: Input parameter schema (e.g., {'target_label': 'string'})
            status: Status feedback schema (e.g., {'state': 'string', 'result': 'any'})
            metadata: Optional metadata dictionary for instance filtering
            provider: Provider identifier (usually package name)
            version: Skill version (e.g., '1.0.0')
            entry: Basic skill entry point (required if type='basic')
            skill_dir: Skill directory path (required if type='rtdl')
            main_rtdl: Main RTDL file name (required if type='rtdl')
            timeout_sec: Timeout in seconds
            
        Returns:
            RegisterSkillResponse or None on error
        """
        try:
            from robonix_sdk.srv import RegisterSkill
            request = RegisterSkill.Request()
            # Ensure name starts with 'skl::'
            if not name.startswith('skl::'):
                name = f'skl::{name}'
            request.name = name
            request.type = skill_type
            request.start_topic = start_topic
            request.status_topic = status_topic
            request.entry = entry if entry else ''
            request.skill_dir = skill_dir if skill_dir else ''
            request.main_rtdl = main_rtdl if main_rtdl else ''
            request.start_args = json.dumps(start_args)
            request.status = json.dumps(status)
            request.metadata = json.dumps(metadata) if metadata else '{}'
            request.provider = provider
            request.version = version
            return self._call_service('register_skill', RegisterSkill, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import RegisterSkill service: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'register_skill error: {e}')
            return None
    
    def submit_task(self, description: str, params: Optional[Dict[str, Any]] = None, timeout_sec: float = 30.0):
        """
        Submit a task.
        
        Args:
            description: Task description
            params: Optional task parameters
            timeout_sec: Timeout in seconds
            
        Returns:
            SubmitTaskResponse or None on error
        """
        try:
            from robonix_sdk.srv import SubmitTask
            request = SubmitTask.Request()
            request.description = description
            request.params = json.dumps(params) if params else '{}'
            return self._call_service('submit_task', SubmitTask, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import SubmitTask service: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'submit_task error: {e}')
            return None
    
    def task_status(self, task_id: str, timeout_sec: float = 5.0):
        """
        Get task status.
        
        Args:
            task_id: Task ID
            timeout_sec: Timeout in seconds
            
        Returns:
            TaskStatusResponse or None on error
        """
        try:
            from robonix_sdk.srv import TaskStatus
            request = TaskStatus.Request()
            request.task_id = task_id
            return self._call_service('task_status', TaskStatus, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import TaskStatus service: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'task_status error: {e}')
            return None
    
    def task_result(self, task_id: str, timeout_sec: float = 5.0):
        """
        Get task result.
        
        Args:
            task_id: Task ID
            timeout_sec: Timeout in seconds
            
        Returns:
            TaskResultResponse or None on error
        """
        try:
            from robonix_sdk.srv import TaskResult
            request = TaskResult.Request()
            request.task_id = task_id
            return self._call_service('task_result', TaskResult, request, timeout_sec)
        except ImportError as e:
            self.get_logger().error(f'Failed to import TaskResult service: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'task_result error: {e}')
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
