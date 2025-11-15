"""
High-performance Robonix SDK client library.

This module provides efficient, long-lived client connections for all Robonix Core services.
Clients are designed to be reused across multiple service calls for maximum performance.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from typing import Optional, List
from dataclasses import dataclass


@dataclass
class QueryCapSklResult:
    """Result of a QueryCapSkl service call."""

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
        """Get input channel by parameter name."""
        try:
            index = self.input_names.index(name)
            return (
                self.input_channels[index] if index < len(self.input_channels) else None
            )
        except ValueError:
            return None

    def get_output_channel(self, name: str) -> Optional[str]:
        """Get output channel by parameter name."""
        try:
            index = self.output_names.index(name)
            return (
                self.output_channels[index]
                if index < len(self.output_channels)
                else None
            )
        except ValueError:
            return None


@dataclass
class RegisterResult:
    """Result of a Register service call."""

    success: bool
    error_message: str = ""


@dataclass
class CreateTaskResult:
    """Result of a CreateTask service call."""

    success: bool
    error_message: str = ""
    task_id: Optional[str] = None


@dataclass
class GetTaskResult:
    """Result of a GetTask service call."""

    success: bool
    error_message: str = ""
    task_id: Optional[str] = None
    natural_language: str = ""
    dsl_code: str = ""
    state: str = ""
    created_at: int = 0
    updated_at: int = 0


@dataclass
class ListTasksResult:
    """Result of a ListTasks service call."""

    success: bool
    error_message: str = ""
    task_ids: List[str] = None
    natural_languages: List[str] = None
    states: List[str] = None
    created_at: List[int] = None
    updated_at: List[int] = None

    def __post_init__(self):
        """Initialize empty lists if None."""
        if self.task_ids is None:
            self.task_ids = []
        if self.natural_languages is None:
            self.natural_languages = []
        if self.states is None:
            self.states = []
        if self.created_at is None:
            self.created_at = []
        if self.updated_at is None:
            self.updated_at = []


@dataclass
class CancelTaskResult:
    """Result of a CancelTask service call."""

    success: bool
    error_message: str = ""


@dataclass
class GetSemanticMapResult:
    """Result of a GetSemanticMap service call."""

    success: bool
    error_message: str = ""
    entities: List = None  # List of Entity messages

    def __post_init__(self):
        """Initialize empty list if None."""
        if self.entities is None:
            self.entities = []


@dataclass
class GetSpatialMapResult:
    """Result of a GetSpatialMap service call."""

    success: bool
    error_message: str = ""
    entries: List = None  # List of SpatialMapEntry messages

    def __post_init__(self):
        """Initialize empty list if None."""
        if self.entries is None:
            self.entries = []


@dataclass
class GetMapStatusResult:
    """Result of a GetMapStatus service call."""

    is_updating: bool = False


@dataclass
class ModelQueryResult:
    """Result of a QueryModel service call."""

    success: bool
    error_message: str = ""
    model_ids: List[str] = None
    model_names: List[str] = None
    model_types: List[str] = None
    providers: List[str] = None
    api_endpoints: List[str] = None
    descriptions: List[str] = None
    capabilities: List[str] = None  # List of comma-separated capability strings

    def __post_init__(self):
        """Initialize empty lists if None."""
        if self.model_ids is None:
            self.model_ids = []
        if self.model_names is None:
            self.model_names = []
        if self.model_types is None:
            self.model_types = []
        if self.providers is None:
            self.providers = []
        if self.api_endpoints is None:
            self.api_endpoints = []
        if self.descriptions is None:
            self.descriptions = []
        if self.capabilities is None:
            self.capabilities = []

    def get_model(self, model_id: str) -> Optional[dict]:
        """Get model information by model_id."""
        try:
            index = self.model_ids.index(model_id)
            return {
                "model_id": self.model_ids[index],
                "model_name": self.model_names[index]
                if index < len(self.model_names)
                else "",
                "model_type": self.model_types[index]
                if index < len(self.model_types)
                else "",
                "provider": self.providers[index]
                if index < len(self.providers)
                else "",
                "api_endpoint": self.api_endpoints[index]
                if index < len(self.api_endpoints)
                else "",
                "description": self.descriptions[index]
                if index < len(self.descriptions)
                else "",
                "capabilities": self.capabilities[index].split(",")
                if index < len(self.capabilities) and self.capabilities[index]
                else [],
            }
        except ValueError:
            return None


class RobonixSDK:
    """
    High-performance Robonix SDK with long-lived connections.

    This class maintains persistent connections to all Robonix Core services,
    allowing efficient reuse across multiple calls.

    Example:
        ```python
        from robonix_core.client import RobonixSDK

        # Create SDK instance (reuse across your application)
        sdk = RobonixSDK()

        # Query capabilities (fast, reuses connection)
        result = sdk.query_cap_skl('cap::vision.capture_rgb')

        # Create tasks (fast, reuses connection)
        task_result = sdk.create_task("Pick up the red box")
        ```
    """

    def __init__(self, node_name: Optional[str] = None):
        """
        Initialize SDK with long-lived connections.

        Each process should create its own SDK instance.
        The SDK creates its own node and executor. The executor is only used
        temporarily during service calls, avoiding conflicts with the skill's executor.

        Args:
            node_name: Optional custom node name (default: 'robonix_sdk_client_<pid>')
        """
        import os
        import logging
        
        # Setup logger for debug output
        self._logger = logging.getLogger('robonix_sdk')
        if not self._logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter('[DEBUG] [robonix_sdk] %(message)s')
            handler.setFormatter(formatter)
            self._logger.addHandler(handler)
            self._logger.setLevel(logging.DEBUG)
        
        # Initialize ROS2 if not already initialized
        try:
            rclpy.init()
            self._logger.debug("Initialized rclpy")
        except RuntimeError:
            # Already initialized, that's fine
            self._logger.debug("rclpy already initialized")
            pass

        # Create a dedicated node for this SDK instance
        if node_name is None:
            node_name = f"robonix_sdk_client_{os.getpid()}"
        self._logger.debug(f"Creating SDK node: {node_name}")
        self._node = Node(node_name)
        self._own_node = True
        
        # Create a dedicated executor for this SDK instance
        # IMPORTANT: We do NOT spin this executor in a background thread.
        # It is only used temporarily during service calls via spin_until_future_complete.
        # This avoids conflicts with the skill's executor.
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._logger.debug("Created SDK executor (not spinning in background)")

        # Initialize service clients (lazy initialization)
        self._query_client = None
        self._register_client = None
        self._query_model_client = None
        self._register_model_client = None
        self._add_entity_client = None
        self._add_spatial_map_entry_client = None
        self._get_semantic_map_client = None
        self._get_spatial_map_client = None
        self._get_map_status_client = None
        self._create_task_client = None
        self._get_task_client = None
        self._list_tasks_client = None
        self._cancel_task_client = None
        
        self._logger.debug("RobonixSDK initialized")

    def _spin_until_complete(self, future, timeout_sec: float):
        """
        Spin until future completes using SDK's own executor.
        
        This is safe because the executor is only used temporarily during service calls,
        and is not running in a background thread, so it won't conflict with the skill's executor.
        """
        self._logger.debug(f"Starting spin_until_future_complete, timeout={timeout_sec}s")
        try:
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=timeout_sec, executor=self._executor
            )
            self._logger.debug("spin_until_future_complete returned")
        except Exception as e:
            self._logger.debug(f"Exception in spin_until_future_complete: {e}")
            import traceback
            self._logger.debug(f"Traceback: {traceback.format_exc()}")
            raise

    def _ensure_query_client(self):
        """Ensure query client is created."""
        if self._query_client is None:
            try:
                from robonix_core.srv import QueryCapSkl

                self._query_client = self._node.create_client(
                    QueryCapSkl, "/rbnx/srv/mgmt/query_cap_skl"
                )
                self._logger.debug("Created query_cap_skl client")
            except ImportError:
                raise ImportError(
                    "robonix_core.srv.QueryCapSkl not available. "
                    "Make sure robonix_core package is built and sourced."
                )

    def _ensure_register_client(self):
        """Ensure register client is created."""
        if self._register_client is None:
            try:
                from robonix_core.srv import RegisterCapSkl

                self._register_client = self._node.create_client(
                    RegisterCapSkl, "/rbnx/srv/mgmt/register_cap_skl"
                )
            except ImportError:
                raise ImportError(
                    "robonix_core.srv.RegisterCapSkl not available. "
                    "Make sure robonix_core package is built and sourced."
                )

    def _ensure_query_model_client(self):
        """Ensure query model client is created."""
        if self._query_model_client is None:
            try:
                from robonix_core.srv import QueryModel

                self._query_model_client = self._node.create_client(
                    QueryModel, "/rbnx/srv/mgmt/query_model"
                )
            except ImportError:
                raise ImportError(
                    "robonix_core.srv.QueryModel not available. "
                    "Make sure robonix_core package is built and sourced."
                )

    def _ensure_register_model_client(self):
        """Ensure register model client is created."""
        if self._register_model_client is None:
            try:
                from robonix_core.srv import RegisterModel

                self._register_model_client = self._node.create_client(
                    RegisterModel, "/rbnx/srv/mgmt/register_model"
                )
            except ImportError:
                raise ImportError(
                    "robonix_core.srv.RegisterModel not available. "
                    "Make sure robonix_core package is built and sourced."
                )

    def _ensure_add_entity_client(self):
        """Ensure add entity client is created."""
        if self._add_entity_client is None:
            try:
                from robonix_core.srv import AddEntity

                self._add_entity_client = self._node.create_client(
                    AddEntity, "/rbnx/srv/perception/add_entity"
                )
            except ImportError:
                raise ImportError(
                    "robonix_core.srv.AddEntity not available. "
                    "Make sure robonix_core package is built and sourced."
                )

    def _ensure_add_spatial_map_entry_client(self):
        """Ensure add spatial map entry client is created."""
        if self._add_spatial_map_entry_client is None:
            try:
                from robonix_core.srv import AddSpatialMapEntry

                self._add_spatial_map_entry_client = self._node.create_client(
                    AddSpatialMapEntry, "/rbnx/srv/perception/add_spatial_map_entry"
                )
            except ImportError:
                raise ImportError(
                    "robonix_core.srv.AddSpatialMapEntry not available. "
                    "Make sure robonix_core package is built and sourced."
                )

    def _ensure_get_semantic_map_client(self):
        """Ensure get semantic map client is created."""
        if self._get_semantic_map_client is None:
            try:
                from robonix_core.srv import GetSemanticMap

                self._get_semantic_map_client = self._node.create_client(
                    GetSemanticMap, "/rbnx/srv/perception/get_semantic_map"
                )
            except ImportError:
                raise ImportError(
                    "robonix_core.srv.GetSemanticMap not available. "
                    "Make sure robonix_core package is built and sourced."
                )

    def _ensure_get_spatial_map_client(self):
        """Ensure get spatial map client is created."""
        if self._get_spatial_map_client is None:
            try:
                from robonix_core.srv import GetSpatialMap

                self._get_spatial_map_client = self._node.create_client(
                    GetSpatialMap, "/rbnx/srv/perception/get_spatial_map"
                )
            except ImportError:
                raise ImportError(
                    "robonix_core.srv.GetSpatialMap not available. "
                    "Make sure robonix_core package is built and sourced."
                )

    def _ensure_get_map_status_client(self):
        """Ensure get map status client is created."""
        if self._get_map_status_client is None:
            try:
                from robonix_core.srv import GetMapStatus

                self._get_map_status_client = self._node.create_client(
                    GetMapStatus, "/rbnx/srv/perception/get_map_status"
                )
            except ImportError:
                raise ImportError(
                    "robonix_core.srv.GetMapStatus not available. "
                    "Make sure robonix_core package is built and sourced."
                )

    def _ensure_task_clients(self):
        """Ensure task service clients are created."""
        try:
            from robonix_core.srv import CreateTask, GetTask, ListTasks, CancelTask
        except (ImportError, ModuleNotFoundError):
            # Task services may not be available yet
            return

        if self._create_task_client is None:
            self._create_task_client = self._node.create_client(
                CreateTask, "/rbnx/srv/planning/create_task"
            )

        if self._get_task_client is None:
            self._get_task_client = self._node.create_client(
                GetTask, "/rbnx/srv/planning/get_task"
            )

        if self._list_tasks_client is None:
            self._list_tasks_client = self._node.create_client(
                ListTasks, "/rbnx/srv/planning/list_tasks"
            )

        if self._cancel_task_client is None:
            self._cancel_task_client = self._node.create_client(
                CancelTask, "/rbnx/srv/planning/cancel_task"
            )

    def query_cap_skl(
        self,
        std_name: str,
        requirements: Optional[List[str]] = None,
        timeout: float = 5.0,
    ) -> QueryCapSklResult:
        """
        Query a capability or skill by standard name.

        Thread-safe: Multiple skills can call this concurrently.

        Args:
            std_name: Standard name (e.g., 'cap::vision.capture_rgb')
            requirements: Optional list of requirements/filters
            timeout: Timeout in seconds (default: 5.0)

        Returns:
            QueryCapSklResult with success status and channel information
        """
        self._logger.debug(f"query_cap_skl called: std_name={std_name}, timeout={timeout}")
        self._ensure_query_client()
        self._logger.debug("Query client ensured")

        if requirements is None:
            requirements = []

        # Wait for service with detailed logging and periodic status updates
        self._logger.debug(f"Waiting for service /rbnx/srv/mgmt/query_cap_skl (timeout={timeout}s)...")
        import time
        wait_start = time.time()
        
        # Check service availability with periodic logging
        check_interval = 1.0  # Log every 1 second
        last_log_time = wait_start
        service_available = False
        
        while time.time() - wait_start < timeout:
            service_available = self._query_client.service_is_ready()
            if service_available:
                break
            
            # Log progress every check_interval seconds
            current_time = time.time()
            if current_time - last_log_time >= check_interval:
                elapsed = current_time - wait_start
                remaining = timeout - elapsed
                self._logger.debug(f"Service not ready yet (elapsed: {elapsed:.1f}s, remaining: {remaining:.1f}s)")
                last_log_time = current_time
            
            time.sleep(0.1)  # Small sleep to avoid busy waiting
        
        wait_elapsed = time.time() - wait_start
        
        if not service_available:
            self._logger.debug(f"Service not available after {wait_elapsed:.2f}s timeout")
            return QueryCapSklResult(
                success=False,
                error_message=f"Service /rbnx/srv/mgmt/query_cap_skl not available after {timeout}s",
            )
        self._logger.debug(f"Service is available (waited {wait_elapsed:.2f}s)")

        # Create and send request
        request = self._query_client.srv_type.Request()
        request.std_name = std_name
        request.requirements = requirements

        self._logger.debug(f"Calling query_cap_skl for: {std_name}")
        # Use async call with executor spin_until_future_complete
        future = self._query_client.call_async(request)
        self._logger.debug("Service call sent, waiting for response...")
        self._spin_until_complete(future, timeout)
        self._logger.debug(f"Spin completed, future.done()={future.done()}")

        if not future.done():
            self._logger.debug("Future not done after spin, timeout occurred")
            return QueryCapSklResult(
                success=False, error_message="Service call timeout"
            )

        self._logger.debug("Future is done, getting result...")
        try:
            response = future.result()
            self._logger.debug(f"Got response: success={response.success}, error_message={response.error_message}")
            return QueryCapSklResult(
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
            self._logger.debug(f"Exception getting future result: {e}")
            import traceback
            self._logger.debug(f"Traceback: {traceback.format_exc()}")
            return QueryCapSklResult(
                success=False, error_message=f"Error getting response: {str(e)}"
            )

    def register_cap_skl(
        self,
        package_name: str,
        package_type: str,  # 'cap' or 'skl'
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
        hostname: str = "",
        entity_name: str = "",
        timeout: float = 5.0,
    ) -> RegisterResult:
        """
        Register a capability or skill.

        Thread-safe: Multiple skills can call this concurrently.

        Args:
            package_name: Package name
            package_type: 'cap' or 'skl'
            std_name: Standard name
            description: Description
            code_path: Code path
            input_names: Input parameter names
            input_ros_types: Input ROS message types
            input_channels: Input topic channels
            output_names: Output parameter names
            output_ros_types: Output ROS message types
            output_channels: Output topic channels
            config_services: Configuration service names
            config_names: Configuration parameter names
            hostname: Hostname
            entity_name: Entity name
            timeout: Timeout in seconds

        Returns:
            RegisterResult with success status
        """
        self._ensure_register_client()

        # Initialize empty lists
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

        # Wait for service
        if not self._register_client.wait_for_service(timeout_sec=timeout):
            return RegisterResult(
                success=False,
                error_message=f"Service /rbnx/srv/mgmt/register_cap_skl not available",
            )

        # Create and send request
        request = self._register_client.srv_type.Request()
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
        request.hostname = hostname
        request.entity_name = entity_name

        self._logger.debug(f"Calling register_cap_skl for: {std_name}")
        future = self._register_client.call_async(request)
        self._spin_until_complete(future, timeout)

        if not future.done():
            return RegisterResult(success=False, error_message="Service call timeout")

        try:
            response = future.result()
            return RegisterResult(
                success=response.success, error_message=response.error_message
            )
        except Exception as e:
            return RegisterResult(
                success=False, error_message=f"Error getting response: {str(e)}"
            )

    def query_model(
        self,
        model_id: str = "",
        model_type: str = "",
        capability: str = "",
        timeout: float = 5.0,
    ) -> ModelQueryResult:
        """
        Query AI models (LLM, VLM, etc.).

        Thread-safe: Multiple skills can call this concurrently.

        Args:
            model_id: Optional specific model ID (empty string to query all)
            model_type: Optional filter by type "LLM" or "VLM" (empty string for all)
            capability: Optional filter by capability (empty string for all)
            timeout: Timeout in seconds (default: 5.0)

        Returns:
            ModelQueryResult with success status and model information
        """
        self._ensure_query_model_client()

        # Wait for service
        if not self._query_model_client.wait_for_service(timeout_sec=timeout):
            return ModelQueryResult(
                success=False,
                error_message=f"Service /rbnx/srv/mgmt/query_model not available",
            )

        # Create and send request
        request = self._query_model_client.srv_type.Request()
        request.model_id = model_id
        request.model_type = model_type
        request.capability = capability

        self._logger.debug(f"Calling query_model: model_id={model_id}, model_type={model_type}, capability={capability}")
        future = self._query_model_client.call_async(request)
        self._spin_until_complete(future, timeout)

        if not future.done():
            return ModelQueryResult(success=False, error_message="Service call timeout")

        try:
            response = future.result()
            return ModelQueryResult(
                success=response.success,
                error_message=response.error_message,
                model_ids=list(response.model_ids),
                model_names=list(response.model_names),
                model_types=list(response.model_types),
                providers=list(response.providers),
                api_endpoints=list(response.api_endpoints),
                descriptions=list(response.descriptions),
                capabilities=list(response.capabilities),
            )
        except Exception as e:
            return ModelQueryResult(
                success=False, error_message=f"Error getting response: {str(e)}"
            )

    def register_model(
        self,
        model_id: str,
        model_name: str,
        model_type: str,  # "LLM" or "VLM"
        provider: str,
        api_endpoint: str,
        api_key: str = "",
        description: str = "",
        capabilities: Optional[List[str]] = None,
        timeout: float = 5.0,
    ) -> RegisterResult:
        """
        Register an AI model (LLM, VLM, etc.).

        Args:
            model_id: Model identifier
            model_name: Human-readable model name
            model_type: "LLM" or "VLM"
            provider: Provider name (e.g., "openai", "anthropic", "local")
            api_endpoint: API endpoint URL
            api_key: Optional API key (empty string if not needed)
            description: Model description
            capabilities: List of supported capabilities
            timeout: Timeout in seconds

        Returns:
            RegisterResult with success status
        """
        self._ensure_register_model_client()

        # Initialize empty list
        if capabilities is None:
            capabilities = []

        # Wait for service
        if not self._register_model_client.wait_for_service(timeout_sec=timeout):
            return RegisterResult(
                success=False,
                error_message=f"Service /rbnx/srv/mgmt/register_model not available",
            )

        # Create and send request
        request = self._register_model_client.srv_type.Request()
        request.model_id = model_id
        request.model_name = model_name
        request.model_type = model_type
        request.provider = provider
        request.api_endpoint = api_endpoint
        request.api_key = api_key
        request.description = description
        request.capabilities = capabilities

        self._logger.debug(f"Calling register_model: model_id={model_id}, model_name={model_name}")
        future = self._register_model_client.call_async(request)
        self._spin_until_complete(future, timeout)

        if not future.done():
            return RegisterResult(success=False, error_message="Service call timeout")

        try:
            response = future.result()
            return RegisterResult(
                success=response.success, error_message=response.error_message
            )
        except Exception as e:
            return RegisterResult(
                success=False, error_message=f"Error getting response: {str(e)}"
            )

    def add_entity(self, entity, timeout: float = 5.0) -> RegisterResult:
        """
        Add an entity to the semantic map.

        Args:
            entity: Entity message object
            timeout: Timeout in seconds

        Returns:
            RegisterResult with success status
        """
        self._ensure_add_entity_client()

        if not self._add_entity_client.wait_for_service(timeout_sec=timeout):
            return RegisterResult(
                success=False,
                error_message=f"Service /rbnx/srv/perception/add_entity not available",
            )

        request = self._add_entity_client.srv_type.Request()
        request.entity = entity

        self._logger.debug(f"Calling add_entity: entity_id={entity.id}, label={entity.label}")
        future = self._add_entity_client.call_async(request)
        self._spin_until_complete(future, timeout)

        if not future.done():
            return RegisterResult(success=False, error_message="Service call timeout")

        try:
            response = future.result()
            return RegisterResult(
                success=response.success, error_message=response.error_message
            )
        except Exception as e:
            return RegisterResult(
                success=False, error_message=f"Error getting response: {str(e)}"
            )

    def add_spatial_map_entry(self, entry, timeout: float = 5.0) -> RegisterResult:
        """
        Add a spatial map entry.

        Args:
            entry: SpatialMapEntry message object
            timeout: Timeout in seconds

        Returns:
            RegisterResult with success status
        """
        self._ensure_add_spatial_map_entry_client()

        if not self._add_spatial_map_entry_client.wait_for_service(timeout_sec=timeout):
            return RegisterResult(
                success=False,
                error_message=f"Service /rbnx/srv/perception/add_spatial_map_entry not available",
            )

        request = self._add_spatial_map_entry_client.srv_type.Request()
        request.entry = entry

        self._logger.debug(f"Calling add_spatial_map_entry: frame_id={entry.frame_id}")
        future = self._add_spatial_map_entry_client.call_async(request)
        self._spin_until_complete(future, timeout)

        if not future.done():
            return RegisterResult(success=False, error_message="Service call timeout")

        try:
            response = future.result()
            return RegisterResult(
                success=response.success, error_message=response.error_message
            )
        except Exception as e:
            return RegisterResult(
                success=False, error_message=f"Error getting response: {str(e)}"
            )

    def create_task(
        self, natural_language: str, timeout: float = 10.0
    ) -> CreateTaskResult:
        """
        Create a new task from natural language.

        Args:
            natural_language: Natural language task description
            timeout: Timeout in seconds

        Returns:
            TaskResult with task_id if successful
        """
        self._ensure_task_clients()

        if self._create_task_client is None:
            return CreateTaskResult(
                success=False, error_message="Task services not available"
            )

        if not self._create_task_client.wait_for_service(timeout_sec=timeout):
            return CreateTaskResult(
                success=False,
                error_message="Service /rbnx/srv/planning/create_task not available",
            )

        request = self._create_task_client.srv_type.Request()
        request.natural_language = natural_language

        self._logger.debug(f"Calling create_task: natural_language={natural_language[:50]}...")
        future = self._create_task_client.call_async(request)
        self._spin_until_complete(future, timeout)

        if not future.done():
            return CreateTaskResult(success=False, error_message="Service call timeout")

        try:
            response = future.result()
            return CreateTaskResult(
                success=response.success,
                error_message=response.error_message,
                task_id=response.task_id if response.success else None,
            )
        except Exception as e:
            return CreateTaskResult(
                success=False, error_message=f"Error getting response: {str(e)}"
            )

    def get_task(self, task_id: str, timeout: float = 5.0) -> GetTaskResult:
        """
        Get task information by task ID.

        Args:
            task_id: Task ID
            timeout: Timeout in seconds

        Returns:
            GetTaskResult with task information
        """
        self._ensure_task_clients()

        if self._get_task_client is None:
            return GetTaskResult(
                success=False, error_message="Task services not available"
            )

        if not self._get_task_client.wait_for_service(timeout_sec=timeout):
            return GetTaskResult(
                success=False,
                error_message="Service /rbnx/srv/planning/get_task not available",
            )

        request = self._get_task_client.srv_type.Request()
        request.task_id = task_id

        self._logger.debug(f"Calling get_task: task_id={task_id}")
        future = self._get_task_client.call_async(request)
        self._spin_until_complete(future, timeout)

        if not future.done():
            return GetTaskResult(success=False, error_message="Service call timeout")

        try:
            response = future.result()
            return GetTaskResult(
                success=response.success,
                error_message=response.error_message,
                task_id=response.task_id,
                natural_language=response.natural_language,
                dsl_code=response.dsl_code,
                state=response.state,
                created_at=response.created_at,
                updated_at=response.updated_at,
            )
        except Exception as e:
            return GetTaskResult(
                success=False, error_message=f"Error getting response: {str(e)}"
            )

    def list_tasks(self, timeout: float = 5.0) -> ListTasksResult:
        """
        List all tasks.

        Args:
            timeout: Timeout in seconds

        Returns:
            ListTasksResult with list of tasks
        """
        self._ensure_task_clients()

        if self._list_tasks_client is None:
            return ListTasksResult(
                success=False, error_message="Task services not available"
            )

        if not self._list_tasks_client.wait_for_service(timeout_sec=timeout):
            return ListTasksResult(
                success=False,
                error_message="Service /rbnx/srv/planning/list_tasks not available",
            )

        request = self._list_tasks_client.srv_type.Request()

        self._logger.debug("Calling list_tasks")
        future = self._list_tasks_client.call_async(request)
        self._spin_until_complete(future, timeout)

        if not future.done():
            return ListTasksResult(success=False, error_message="Service call timeout")

        try:
            response = future.result()
            return ListTasksResult(
                success=response.success,
                error_message=response.error_message,
                task_ids=list(response.task_ids),
                natural_languages=list(response.natural_languages),
                states=list(response.states),
                created_at=list(response.created_at),
                updated_at=list(response.updated_at),
            )
        except Exception as e:
            return ListTasksResult(
                success=False, error_message=f"Error getting response: {str(e)}"
            )

    def cancel_task(self, task_id: str, timeout: float = 5.0) -> CancelTaskResult:
        """
        Cancel a task.

        Args:
            task_id: Task ID
            timeout: Timeout in seconds

        Returns:
            CancelTaskResult with success status
        """
        self._ensure_task_clients()

        if self._cancel_task_client is None:
            return CancelTaskResult(
                success=False, error_message="Task services not available"
            )

        if not self._cancel_task_client.wait_for_service(timeout_sec=timeout):
            return CancelTaskResult(
                success=False,
                error_message="Service /rbnx/srv/planning/cancel_task not available",
            )

        request = self._cancel_task_client.srv_type.Request()
        request.task_id = task_id

        self._logger.debug(f"Calling cancel_task: task_id={task_id}")
        future = self._cancel_task_client.call_async(request)
        self._spin_until_complete(future, timeout)

        if not future.done():
            return CancelTaskResult(success=False, error_message="Service call timeout")

        try:
            response = future.result()
            return CancelTaskResult(
                success=response.success,
                error_message=response.error_message,
            )
        except Exception as e:
            return CancelTaskResult(
                success=False, error_message=f"Error getting response: {str(e)}"
            )

    def get_semantic_map(
        self, entity_id: str = "", label: str = "", path: str = "", timeout: float = 5.0
    ) -> GetSemanticMapResult:
        """
        Get semantic map entities.

        Args:
            entity_id: Optional entity ID (empty to get all)
            label: Optional filter by label (empty to ignore)
            path: Optional filter by path (empty to ignore)
            timeout: Timeout in seconds

        Returns:
            GetSemanticMapResult with entities
        """
        self._ensure_get_semantic_map_client()

        if not self._get_semantic_map_client.wait_for_service(timeout_sec=timeout):
            return GetSemanticMapResult(
                success=False,
                error_message="Service /rbnx/srv/perception/get_semantic_map not available",
            )

        request = self._get_semantic_map_client.srv_type.Request()
        request.entity_id = entity_id
        request.label = label
        request.path = path

        self._logger.debug(f"Calling get_semantic_map: entity_id={entity_id}, label={label}, path={path}")
        future = self._get_semantic_map_client.call_async(request)
        self._spin_until_complete(future, timeout)

        if not future.done():
            return GetSemanticMapResult(
                success=False, error_message="Service call timeout"
            )

        try:
            response = future.result()
            return GetSemanticMapResult(
                success=response.success,
                error_message=response.error_message,
                entities=list(response.entities),
            )
        except Exception as e:
            return GetSemanticMapResult(
                success=False, error_message=f"Error getting response: {str(e)}"
            )

    def get_spatial_map(
        self, frame_id: str = "", timeout: float = 5.0
    ) -> GetSpatialMapResult:
        """
        Get spatial map entries.

        Args:
            frame_id: Optional filter by frame_id (empty to get all)
            timeout: Timeout in seconds

        Returns:
            GetSpatialMapResult with entries
        """
        self._ensure_get_spatial_map_client()

        if not self._get_spatial_map_client.wait_for_service(timeout_sec=timeout):
            return GetSpatialMapResult(
                success=False,
                error_message="Service /rbnx/srv/perception/get_spatial_map not available",
            )

        request = self._get_spatial_map_client.srv_type.Request()
        request.frame_id = frame_id

        self._logger.debug(f"Calling get_spatial_map: frame_id={frame_id}")
        future = self._get_spatial_map_client.call_async(request)
        self._spin_until_complete(future, timeout)

        if not future.done():
            return GetSpatialMapResult(
                success=False, error_message="Service call timeout"
            )

        try:
            response = future.result()
            return GetSpatialMapResult(
                success=response.success,
                error_message=response.error_message,
                entries=list(response.entries),
            )
        except Exception as e:
            return GetSpatialMapResult(
                success=False, error_message=f"Error getting response: {str(e)}"
            )

    def get_map_status(self, timeout: float = 5.0) -> GetMapStatusResult:
        """
        Get map update status.

        Args:
            timeout: Timeout in seconds

        Returns:
            GetMapStatusResult with is_updating flag
        """
        self._ensure_get_map_status_client()

        if not self._get_map_status_client.wait_for_service(timeout_sec=timeout):
            return GetMapStatusResult(is_updating=False)

        request = self._get_map_status_client.srv_type.Request()

        self._logger.debug("Calling get_map_status")
        future = self._get_map_status_client.call_async(request)
        self._spin_until_complete(future, timeout)

        if not future.done():
            return GetMapStatusResult(is_updating=False)

        try:
            response = future.result()
            return GetMapStatusResult(is_updating=response.is_updating)
        except Exception:
            return GetMapStatusResult(is_updating=False)

    def shutdown(self):
        """Shutdown SDK and cleanup resources."""
        self._logger.debug("Shutting down SDK")
        if self._executor is not None:
            try:
                self._executor.shutdown()
                self._logger.debug("Shut down SDK executor")
            except Exception:
                pass
        if self._node is not None:
            try:
                self._node.destroy_node()
                self._logger.debug("Destroyed SDK node")
            except Exception:
                pass
