#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Robonix Python Helper
#
# Helper module for interacting with Robonix services.
# Simplifies common operations like querying primitives, services, and skills.

import json
import time
from typing import Optional, Dict, Any
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
    LivelinessPolicy,
)
from rclpy.duration import Duration
from robonix_sdk.srv import QueryPrimitive


class RobonixClient:
    """Helper class for interacting with Robonix services."""

    def __init__(self, node: Node, logger: Optional[Any] = None) -> None:
        """
        Initialize Robonix client.

        Args:
            node: ROS2 node instance
            logger: Optional logger (defaults to node.get_logger())
        """
        self.node = node
        self.logger = logger if logger else node.get_logger()
        self._query_primitive_client: Optional[Any] = None

    def create_query_primitive_client(
        self, service_name: str = "/rbnx/prm/query"
    ) -> Any:
        """
        Create a QueryPrimitive service client with standard QoS settings.

        Args:
            service_name: Service name (default: "/rbnx/prm/query")

        Returns:
            Service client instance
        """
        service_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        service_qos.deadline = Duration(seconds=0)
        service_qos.lifespan = Duration(seconds=0)
        service_qos.liveliness = LivelinessPolicy.AUTOMATIC
        service_qos.liveliness_lease_duration = Duration(seconds=0)

        self._query_primitive_client = self.node.create_client(
            QueryPrimitive, service_name, qos_profile=service_qos
        )
        self.logger.info(f"QueryPrimitive service client created: {service_name}")
        return self._query_primitive_client

    def query_primitive(
        self,
        primitive_name: str,
        filter_dict: Optional[Dict[str, Any]] = None,
        max_retries: int = 5,
        retry_delay: float = 2.0,
        wait_timeout: float = 10.0,
        call_timeout: float = 3.0,
        raise_on_error: bool = True,
    ) -> Optional[Any]:
        """
        Query a primitive from Robonix OS with retry logic.

        Args:
            primitive_name: Name of the primitive (e.g., "prm::camera.rgb")
            filter_dict: Optional filter dictionary (will be JSON-encoded)
            max_retries: Maximum number of retry attempts
            retry_delay: Delay between retries (seconds)
            wait_timeout: Timeout for waiting for service availability (seconds)
            call_timeout: Timeout for service call (seconds)
            raise_on_error: If True, raise exception on failure; if False, return None

        Returns:
            Response object if successful, None if failed and raise_on_error=False

        Raises:
            RuntimeError: If service is unavailable or call times out after all retries
            ValueError: If response is invalid
        """
        if not self._query_primitive_client:
            self.create_query_primitive_client()

        # Type check: ensure client is not None after creation
        if self._query_primitive_client is None:
            error_msg = "Failed to create query_primitive client"
            if raise_on_error:
                raise RuntimeError(error_msg)
            return None

        client = self._query_primitive_client  # Type narrowing for linter
        filter_str = json.dumps(filter_dict) if filter_dict else "{}"

        for attempt in range(max_retries):
            self.logger.info(
                f"Querying {primitive_name}... (attempt {attempt + 1}/{max_retries})"
            )
            try:
                # Wait for service availability
                wait_timeout_actual = wait_timeout if attempt < 2 else wait_timeout / 2
                if not client.wait_for_service(timeout_sec=wait_timeout_actual):
                    self.logger.warn(
                        f"  query_primitive service not available (attempt {attempt + 1}/{max_retries})"
                    )
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    else:
                        error_msg = (
                            "query_primitive service not available after all retries"
                        )
                        self.logger.error(f"  {error_msg}")
                        if raise_on_error:
                            raise RuntimeError(error_msg)
                        return None

                # Create and send request
                request = QueryPrimitive.Request()
                request.name = primitive_name
                request.filter = filter_str
                future = client.call_async(request)

                # Wait for response with timeout
                start_time = time.time()
                while not future.done() and (time.time() - start_time) < call_timeout:
                    rclpy.spin_once(self.node, timeout_sec=0.01)

                if not future.done():
                    elapsed = time.time() - start_time
                    self.logger.warn(
                        f"  Service call timeout after {elapsed:.1f}s (attempt {attempt + 1}/{max_retries})"
                    )
                    try:
                        future.cancel()
                    except Exception:
                        pass
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    else:
                        error_msg = f"Service call timeout after {elapsed:.1f}s after all retries"
                        if raise_on_error:
                            raise RuntimeError(error_msg)
                        return None

                # Get response
                response = future.result()
                if response and response.instances:
                    self.logger.info(
                        f"  Successfully queried {primitive_name} (attempt {attempt + 1}/{max_retries})"
                    )
                    return response
                else:
                    self.logger.warn(
                        f"  No {primitive_name} primitive found (attempt {attempt + 1}/{max_retries})"
                    )
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    else:
                        error_msg = (
                            f"No {primitive_name} primitive found after all retries"
                        )
                        if raise_on_error:
                            raise RuntimeError(error_msg)
                        return None

            except (RuntimeError, ValueError):
                if raise_on_error:
                    raise
                return None
            except Exception as e:
                self.logger.error(f"Error querying {primitive_name}: {e}")
                import traceback

                self.logger.error(f"Traceback:\n{traceback.format_exc()}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    continue
                else:
                    error_msg = (
                        f"Failed to query {primitive_name} after all retries: {e}"
                    )
                    if raise_on_error:
                        raise RuntimeError(error_msg)
                    return None

        error_msg = f"Failed to query {primitive_name}: unknown error"
        if raise_on_error:
            raise RuntimeError(error_msg)
        return None

    def get_primitive_output_schema(
        self, response: Any, instance_index: int = 0
    ) -> Optional[Dict[str, Any]]:
        """
        Extract and parse output_schema from a QueryPrimitive response.

        Args:
            response: QueryPrimitive response object
            instance_index: Index of instance to extract (default: 0)

        Returns:
            Parsed output_schema dictionary, or None if not found
        """
        if not response or not response.instances:
            return None

        if instance_index >= len(response.instances):
            return None

        instance = response.instances[instance_index]
        output_schema = (
            json.loads(instance.output_schema)
            if isinstance(instance.output_schema, str)
            else instance.output_schema
        )
        return output_schema

    def query_primitive_and_get_schema(
        self,
        primitive_name: str,
        filter_dict: Optional[Dict[str, Any]] = None,
        max_retries: int = 5,
        retry_delay: float = 2.0,
        wait_timeout: float = 10.0,
        call_timeout: float = 3.0,
        raise_on_error: bool = True,
    ) -> Optional[Dict[str, Any]]:
        """
        Query a primitive and return its output_schema directly.

        Args:
            primitive_name: Name of the primitive (e.g., "prm::camera.rgb")
            filter_dict: Optional filter dictionary (will be JSON-encoded)
            max_retries: Maximum number of retry attempts
            retry_delay: Delay between retries (seconds)
            wait_timeout: Timeout for waiting for service availability (seconds)
            call_timeout: Timeout for service call (seconds)
            raise_on_error: If True, raise exception on failure; if False, return None

        Returns:
            output_schema dictionary if successful, None if failed and raise_on_error=False
        """
        response = self.query_primitive(
            primitive_name,
            filter_dict=filter_dict,
            max_retries=max_retries,
            retry_delay=retry_delay,
            wait_timeout=wait_timeout,
            call_timeout=call_timeout,
            raise_on_error=raise_on_error,
        )

        if not response:
            return None

        return self.get_primitive_output_schema(response)

    def query_primitive_and_extract_field(
        self,
        primitive_name: str,
        field_name: str,
        filter_dict: Optional[Dict[str, Any]] = None,
        max_retries: int = 5,
        retry_delay: float = 2.0,
        wait_timeout: float = 10.0,
        call_timeout: float = 3.0,
        raise_on_error: bool = True,
        raise_on_missing_field: bool = True,
        log_success: bool = True,
    ) -> Optional[Any]:
        """
        Query a primitive and extract a specific field from output_schema.

        This is a convenience method that combines querying and field extraction,
        with proper error handling and logging.

        Args:
            primitive_name: Name of the primitive (e.g., "prm::camera.rgb")
            field_name: Name of the field to extract from output_schema (e.g., "image", "pose")
            filter_dict: Optional filter dictionary (will be JSON-encoded)
            max_retries: Maximum number of retry attempts
            retry_delay: Delay between retries (seconds)
            wait_timeout: Timeout for waiting for service availability (seconds)
            call_timeout: Timeout for service call (seconds)
            raise_on_error: If True, raise exception if query fails; if False, return None
            raise_on_missing_field: If True, raise ValueError if field not found in schema; if False, return None
            log_success: If True, log success message with extracted value

        Returns:
            Field value if successful, None if failed and raise_on_error/raise_on_missing_field=False

        Raises:
            RuntimeError: If query fails and raise_on_error=True
            ValueError: If field not found in output_schema and raise_on_missing_field=True
        """
        output_schema = self.query_primitive_and_get_schema(
            primitive_name,
            filter_dict=filter_dict,
            max_retries=max_retries,
            retry_delay=retry_delay,
            wait_timeout=wait_timeout,
            call_timeout=call_timeout,
            raise_on_error=raise_on_error,
        )

        if not output_schema:
            if raise_on_error:
                raise RuntimeError(f"Failed to query {primitive_name}")
            return None

        if field_name in output_schema:
            field_value = output_schema[field_name]
            if log_success:
                self.logger.info(
                    f"  Found {field_name} from {primitive_name}: {field_value}"
                )
            return field_value
        else:
            error_msg = f'{primitive_name} found but no "{field_name}" in output_schema'
            if raise_on_missing_field:
                raise ValueError(error_msg)
            else:
                self.logger.warn(error_msg)
                return None
