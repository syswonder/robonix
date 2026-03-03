#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Move to Object Skill
#
# Skill to navigate to a specific object nearby.
# Uses semantic_map service to find the object and navigates to it.
""""""

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
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import json
import time
import signal
import math
from robonix_sdk.srv import QuerySemanticMap, QueryPrimitive, QueryService


class MoveToObjectSkill(Node):
    """Implements move_to_object skill that navigates to a specific object."""

    def __init__(self):
        super().__init__("move_to_object_skill")

        self.start_topic = "/robot1/skill/move_to_object/start"
        self.status_topic = "/robot1/skill/move_to_object/status"

        self.semantic_map_service_entry = None
        self.pose_topic = None
        self.navigate_goal_topic = None
        self.navigate_status_topic = None

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

        self.query_primitive_client = self.create_client(
            QueryPrimitive, "/rbnx/prm/query", qos_profile=service_qos
        )
        self.get_logger().info("QueryPrimitive service client created")

        self.query_service_client = self.create_client(
            QueryService, "/rbnx/srv/query", qos_profile=service_qos
        )
        self.get_logger().info("QueryService service client created")

        self._query_services_and_primitives()

        start_topic_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.start_subscriber = self.create_subscription(
            String, self.start_topic, self.start_callback, start_topic_qos
        )
        self.get_logger().info(
            f"Subscribing to start topic: {self.start_topic} with RELIABLE QoS"
        )

        status_topic_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, status_topic_qos
        )
        self.get_logger().info(
            f"Publishing to status topic: {self.status_topic} with RELIABLE QoS"
        )

        if self.pose_topic:
            self.pose_subscriber = self.create_subscription(
                PoseWithCovarianceStamped, self.pose_topic, self.pose_cov_callback, 10
            )
            self.get_logger().info(
                f"Subscribing to pose topic (PoseWithCovarianceStamped from prm::base.pose.cov): {self.pose_topic}"
            )
        else:
            self.pose_subscriber = None

        if self.navigate_goal_topic:
            self.navigate_goal_publisher = self.create_publisher(
                PoseStamped, self.navigate_goal_topic, 10
            )
            self.get_logger().info(
                f"Publishing to navigate goal: {self.navigate_goal_topic}"
            )
        else:
            self.navigate_goal_publisher = None

        if self.navigate_status_topic:
            status_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self.navigate_status_subscriber = self.create_subscription(
                Bool, self.navigate_status_topic, self.navigate_status_callback, status_qos
            )
            self.get_logger().info(
                f"Subscribing to navigate status: {self.navigate_status_topic}"
            )
        else:
            self.navigate_status_subscriber = None

        if self.semantic_map_service_entry:
            semantic_map_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.VOLATILE,
            )
            semantic_map_qos.deadline = Duration(seconds=0)
            semantic_map_qos.lifespan = Duration(seconds=0)
            semantic_map_qos.liveliness = LivelinessPolicy.AUTOMATIC
            semantic_map_qos.liveliness_lease_duration = Duration(seconds=0)

            self.semantic_map_client = self.create_client(
                QuerySemanticMap,
                self.semantic_map_service_entry,
                qos_profile=semantic_map_qos,
            )
            self.get_logger().info(
                f"Semantic map service client created: {self.semantic_map_service_entry}"
            )
        else:
            self.semantic_map_client = None

        self.current_skill_id = None
        self.moving_in_progress = False
        self._cancel_requested = False
        self._target_replaced = False
        self.target_object_id = None
        self.stop_radius = 0.5
        self.navigation_complete = False
        self.latest_pose = None
        self.status_timer = None
        self.status_timer_running = False

        self.get_logger().info("Move to object skill initialized")

    def _query_services_and_primitives(self):
        """Query robonix core for required services and primitives."""
        max_retries = 5
        retry_delay = 2.0

        self.get_logger().info("Querying srv::semantic_map service...")
        for attempt in range(max_retries):
            try:
                wait_timeout = 10.0 if attempt < 2 else 5.0
                if not self.query_service_client.wait_for_service(
                    timeout_sec=wait_timeout
                ):
                    self.get_logger().warn(
                        f"  query_service not available (attempt {attempt + 1}/{max_retries})"
                    )
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    else:
                        self.get_logger().error(
                            "  query_service not available after all retries"
                        )
                        break

                request = QueryService.Request()
                request.name = "srv::semantic_map"
                request.filter = "{}"

                future = self.query_service_client.call_async(request)
                start_time = time.time()
                timeout_sec = 3.0
                while not future.done() and (time.time() - start_time) < timeout_sec:
                    rclpy.spin_once(self, timeout_sec=0.01)

                if not future.done():
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    else:
                        break

                response = future.result()
                if response and response.instances:
                    instance = response.instances[0]
                    self.semantic_map_service_entry = instance.entry
                    self.get_logger().info(
                        f"  Found semantic_map service: {self.semantic_map_service_entry}"
                    )
                    break
            except Exception as e:
                self.get_logger().error(
                    f"Error querying srv::semantic_map service: {e}"
                )
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                else:
                    break

        self.get_logger().info("Querying prm::base.pose.cov...")
        for attempt in range(max_retries):
            try:
                wait_timeout = 10.0 if attempt < 2 else 5.0
                if not self.query_primitive_client.wait_for_service(
                    timeout_sec=wait_timeout
                ):
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    else:
                        break

                request = QueryPrimitive.Request()
                request.name = "prm::base.pose.cov"
                request.filter = "{}"

                future = self.query_primitive_client.call_async(request)
                start_time = time.time()
                timeout_sec = 3.0
                while not future.done() and (time.time() - start_time) < timeout_sec:
                    rclpy.spin_once(self, timeout_sec=0.01)

                if not future.done():
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    else:
                        break

                response = future.result()
                if response and response.instances:
                    instance = response.instances[0]
                    output_schema = (
                        json.loads(instance.output_schema)
                        if isinstance(instance.output_schema, str)
                        else instance.output_schema
                    )
                    if "pose" in output_schema:
                        self.pose_topic = output_schema["pose"]
                        self.get_logger().info(
                            f"  Found pose topic: {self.pose_topic} (from prm::base.pose.cov)"
                        )
                        break
            except Exception as e:
                self.get_logger().error(
                    f"Error querying prm::base.pose.cov: {e}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                else:
                    break

        self.get_logger().info("Querying prm::base.navigate...")
        for attempt in range(max_retries):
            try:
                wait_timeout = 10.0 if attempt < 2 else 5.0
                if not self.query_primitive_client.wait_for_service(
                    timeout_sec=wait_timeout
                ):
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    else:
                        break

                request = QueryPrimitive.Request()
                request.name = "prm::base.navigate"
                request.filter = "{}"

                future = self.query_primitive_client.call_async(request)
                start_time = time.time()
                timeout_sec = 3.0
                while not future.done() and (time.time() - start_time) < timeout_sec:
                    rclpy.spin_once(self, timeout_sec=0.01)

                if not future.done():
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        continue
                    else:
                        break

                response = future.result()
                if response and response.instances:
                    instance = response.instances[0]
                    input_schema = (
                        json.loads(instance.input_schema)
                        if isinstance(instance.input_schema, str)
                        else instance.input_schema
                    )
                    output_schema = (
                        json.loads(instance.output_schema)
                        if isinstance(instance.output_schema, str)
                        else instance.output_schema
                    )
                    if "goal" in input_schema:
                        self.navigate_goal_topic = input_schema["goal"]
                        self.get_logger().info(
                            f"  Found navigate goal topic: {self.navigate_goal_topic}"
                        )
                    if "status" in output_schema:
                        self.navigate_status_topic = output_schema["status"]
                        self.get_logger().info(
                            f"  Found navigate status topic: {self.navigate_status_topic}"
                        )
                    break
            except Exception as e:
                self.get_logger().error(
                    f"Error querying prm::base.navigate: {e}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                else:
                    break

    def start_callback(self, msg):
        """Handle skill start request (input params or robonix terminate)."""
        try:
            data = json.loads(msg.data)
            # Robonix can send terminate to start_topic to force-stop this skill
            if data.get("terminate") and data.get("skill_id"):
                sid = data.get("skill_id")
                if self.moving_in_progress and sid == self.current_skill_id:
                    self.get_logger().info(
                        f"Received terminate for skill_id={sid}, stopping move_to_object"
                    )
                    self._cancel_requested = True
                    self.moving_in_progress = False
                    self._stop_status_timer()
                    self._publish_status(
                        sid,
                        "cancelled",
                        {"message": "Cancelled by robonix (terminate on start_topic)"},
                        errno=0,
                    )
                return

            skill_id = data.get("skill_id", "unknown")
            params = data.get("params", {})
            target_object_id = params.get("target_object_id", "")

            if "stop_radius" in params:
                self.stop_radius = float(params["stop_radius"])
            elif "radius" in params:
                self.stop_radius = float(params["radius"])
            else:
                self.stop_radius = 0.5

            self.get_logger().info(
                f"Received move_to_object request: skill_id={skill_id}, "
                f"target_object_id={target_object_id}, stop_radius={self.stop_radius}m"
            )

            if self.moving_in_progress:
                self.get_logger().info(
                    "New target received while moving, replacing with target_object_id=%s"
                    % target_object_id
                )
                self.current_skill_id = skill_id
                self.target_object_id = target_object_id
                self._cancel_requested = False
                self._target_replaced = True
                self._publish_status(
                    skill_id,
                    "running",
                    {
                        "message": "Target replaced, navigating to new object %s"
                        % target_object_id
                    },
                    errno=0,
                )
                return

            if not target_object_id:
                self.get_logger().error("target_object_id parameter is required!")
                self._publish_status(
                    skill_id,
                    "error",
                    {"error": "target_object_id parameter is required"},
                    errno=2,
                )
                return

            if not self.semantic_map_client:
                self.get_logger().error("Semantic map service not available!")
                self._publish_status(
                    skill_id,
                    "error",
                    {"error": "Semantic map service not available"},
                    errno=3,
                )
                return

            if not self.navigate_goal_publisher:
                self.get_logger().error("Navigation primitive not available!")
                self._publish_status(
                    skill_id,
                    "error",
                    {"error": "Navigation primitive not available"},
                    errno=4,
                )
                return

            self.current_skill_id = skill_id
            self.target_object_id = target_object_id
            self._cancel_requested = False
            self.moving_in_progress = True

            self._publish_status(
                skill_id,
                "running",
                {
                    "message": f"Received request, searching for object with ID: {target_object_id}"
                },
                errno=0,
            )

            self._start_status_timer(skill_id)
            self._start_move_operation()

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse start message JSON: {e}")
            self._publish_status(
                "unknown", "error", {"error": f"Invalid JSON: {e}"}, errno=5
            )
        except Exception as e:
            self.get_logger().error(f"Error in start_callback: {e}")
            import traceback

            self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
            self._publish_status("unknown", "error", {
                                 "error": str(e)}, errno=6)

    def _start_status_timer(self, skill_id):
        """Start periodic status reporting (every 1 second)."""
        import threading

        self.status_timer_running = True

        def status_timer_loop():
            while self.status_timer_running and self.moving_in_progress:
                time.sleep(1.0)
                if self.status_timer_running and self.moving_in_progress:
                    if self.navigation_complete:
                        break
                    else:
                        current_message = (
                            f"Processing: searching for object {self.target_object_id}"
                        )
                        if self.latest_pose:
                            current_message = f"Processing: navigating to object {self.target_object_id}"
                        self._publish_status(
                            skill_id, "running", {"message": current_message}, errno=0
                        )

        self.status_timer = threading.Thread(
            target=status_timer_loop, daemon=True)
        self.status_timer.start()

    def _stop_status_timer(self):
        """Stop periodic status reporting."""
        self.status_timer_running = False

    def _start_move_operation(self):
        """Start the move operation in a separate thread."""
        import threading

        thread = threading.Thread(target=self._move_operation, daemon=True)
        thread.start()

    def _move_operation(self):
        """Main move operation that finds and navigates to the target object."""
        try:
            while True:
                self.get_logger().info(
                    f"Querying semantic map for object with ID: {self.target_object_id}"
                )
                try:
                    objects = self._query_semantic_map()
                except TimeoutError as e:
                    error_msg = f"Semantic map query timeout: {str(e)}"
                    self.get_logger().error(error_msg)
                    self._stop_status_timer()
                    self._publish_status(
                        self.current_skill_id,
                        "error",
                        {"error": error_msg, "error_type": "timeout"},
                        errno=7,
                    )
                    self.moving_in_progress = False
                    return
                except RuntimeError as e:
                    error_msg = f"Semantic map service error: {str(e)}"
                    self.get_logger().error(error_msg)
                    self._stop_status_timer()
                    self._publish_status(
                        self.current_skill_id,
                        "error",
                        {"error": error_msg, "error_type": "service_error"},
                        errno=7,
                    )
                    self.moving_in_progress = False
                    return

                if not objects:
                    error_msg = "Semantic map query returned no objects (map may be empty)"
                    self.get_logger().error(error_msg)
                    self._stop_status_timer()
                    self._publish_status(
                        self.current_skill_id,
                        "error",
                        {"error": error_msg, "error_type": "empty_map"},
                        errno=7,
                    )
                    self.moving_in_progress = False
                    return

                target_object = None
                for obj in objects:
                    if obj.id == self.target_object_id:
                        target_object = obj
                        break

                if not target_object:
                    self.get_logger().error(
                        f'Object with ID "{self.target_object_id}" not found in semantic map'
                    )
                    available_ids = [obj.id for obj in objects]
                    available_labels = [obj.label for obj in objects]
                    error_msg = (
                        f'Object with ID "{self.target_object_id}" not found in semantic map. '
                        f"Available object IDs: {available_ids[:10]}{'...' if len(available_ids) > 10 else ''}. "
                        f"This may happen if the object was removed or the ID from planning is incorrect."
                    )
                    self.get_logger().error(error_msg)
                    self._stop_status_timer()
                    self._publish_status(
                        self.current_skill_id,
                        "error",
                        {
                            "error": error_msg,
                            "requested_object_id": self.target_object_id,
                            "available_object_ids": available_ids,
                            "available_labels": available_labels,
                        },
                        errno=8,
                    )
                    self.moving_in_progress = False
                    return

                self.get_logger().info(
                    f"Found target object: {target_object.id} ({target_object.label})"
                )

                object_pose = self._get_object_pose_in_map(target_object)
                if not object_pose:
                    self.get_logger().error(
                        f"Could not get pose for object {target_object.id}"
                    )
                    self._stop_status_timer()
                    self._publish_status(
                        self.current_skill_id,
                        "error",
                        {"error": f"Could not get pose for object {target_object.id}"},
                        errno=9,
                    )
                    self.moving_in_progress = False
                    return

                goal_pose = self._calculate_goal_pose_near_object(
                    object_pose, self.stop_radius
                )

                actual_distance = math.sqrt(
                    (goal_pose.pose.position.x - object_pose.pose.position.x) ** 2
                    + (goal_pose.pose.position.y - object_pose.pose.position.y) ** 2
                )

                self.get_logger().info(
                    f"Navigating to object {target_object.id} ({target_object.label}) - "
                    f"Goal at ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}), "
                    f"will stop {actual_distance:.2f}m from object (requested: {self.stop_radius:.2f}m)"
                )

                self.navigation_complete = False
                assert self.navigate_goal_publisher is not None, (
                    "Navigation goal publisher must be available"
                )
                self.navigate_goal_publisher.publish(goal_pose)

                # Reach = at goal (xyz position only, no orientation) and stationary for a while
                POSITION_DELTA_M = 0.35
                STILL_DURATION_S = 1.0
                STILL_MOVE_THRESHOLD_M = 0.05
                time_first_at_goal = None
                pose_first_at_goal = None
                last_log_time = 0.0
                LOG_INTERVAL_S = 2.0

                self.get_logger().info(
                    "Waiting for pose-based completion: xyz dist<=%.2fm, still %.1fs"
                    % (POSITION_DELTA_M, STILL_DURATION_S)
                )

                timeout = 90.0
                start_time = time.time()
                while (
                    not self.navigation_complete
                    and (time.time() - start_time) < timeout
                    and not self._cancel_requested
                    and not self._target_replaced
                ):
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if self.navigation_complete or self._cancel_requested or self._target_replaced:
                        break
                    now = time.time()
                    if self.latest_pose is None:
                        if now - last_log_time >= LOG_INTERVAL_S:
                            self.get_logger().warn(
                                "No pose yet (amcl_pose?). elapsed=%.1fs" % (now - start_time)
                            )
                            last_log_time = now
                        continue
                    dist = math.sqrt(
                        (self.latest_pose.pose.position.x - goal_pose.pose.position.x) ** 2
                        + (self.latest_pose.pose.position.y - goal_pose.pose.position.y) ** 2
                        + (self.latest_pose.pose.position.z - goal_pose.pose.position.z) ** 2
                    )
                    at_goal = dist <= POSITION_DELTA_M
                    if at_goal:
                        if time_first_at_goal is None:
                            time_first_at_goal = time.time()
                            pose_first_at_goal = (
                                self.latest_pose.pose.position.x,
                                self.latest_pose.pose.position.y,
                                self.latest_pose.pose.position.z,
                            )
                            self.get_logger().info(
                                "At goal (xyz dist=%.3fm). Starting still timer (%.1fs)."
                                % (dist, STILL_DURATION_S)
                            )
                        elif (time.time() - time_first_at_goal) >= STILL_DURATION_S:
                            assert pose_first_at_goal is not None
                            dx = self.latest_pose.pose.position.x - pose_first_at_goal[0]
                            dy = self.latest_pose.pose.position.y - pose_first_at_goal[1]
                            dz = self.latest_pose.pose.position.z - pose_first_at_goal[2]
                            drift = math.sqrt(dx * dx + dy * dy + dz * dz)
                            if drift <= STILL_MOVE_THRESHOLD_M:
                                self.navigation_complete = True
                                self.get_logger().info(
                                    "Navigation complete (at goal and stationary for %.1fs, drift=%.3fm)"
                                    % (STILL_DURATION_S, drift)
                                )
                            elif now - last_log_time >= LOG_INTERVAL_S:
                                self.get_logger().info(
                                    "At goal but moved too much: drift=%.3fm (max %.2fm). Resetting still timer."
                                    % (drift, STILL_MOVE_THRESHOLD_M)
                                )
                                time_first_at_goal = None
                                pose_first_at_goal = None
                                last_log_time = now
                        elif now - last_log_time >= LOG_INTERVAL_S:
                            remaining = STILL_DURATION_S - (now - time_first_at_goal)
                            self.get_logger().info(
                                "At goal, waiting still: %.1fs left" % max(0, remaining)
                            )
                            last_log_time = now
                    else:
                        if time_first_at_goal is not None:
                            self.get_logger().info(
                                "Left goal (xyz dist=%.3fm). Resetting still timer."
                                % (dist,)
                            )
                        time_first_at_goal = None
                        pose_first_at_goal = None
                        if now - last_log_time >= LOG_INTERVAL_S:
                            self.get_logger().info(
                                "Approaching: xyz dist=%.3fm (need<=%.2f), pos=(%.2f,%.2f,%.2f)"
                                % (
                                    dist,
                                    POSITION_DELTA_M,
                                    self.latest_pose.pose.position.x,
                                    self.latest_pose.pose.position.y,
                                    self.latest_pose.pose.position.z,
                                )
                            )
                            last_log_time = now

                if self._target_replaced:
                    self._target_replaced = False
                    self.get_logger().info(
                        "Target replaced, switching to new object %s"
                        % self.target_object_id
                    )
                    continue

                if self._cancel_requested:
                    self._stop_status_timer()
                    self._publish_status(
                        self.current_skill_id,
                        "cancelled",
                        {"message": "Cancelled by robonix (terminate on start_topic)"},
                        errno=0,
                    )
                    self.get_logger().info("Move to object cancelled by robonix")
                    self.moving_in_progress = False
                    return

                if self.navigation_complete:
                    final_distance = None
                    if self.latest_pose:
                        final_distance = math.sqrt(
                            (self.latest_pose.pose.position.x -
                             object_pose.pose.position.x)
                            ** 2
                            + (
                                self.latest_pose.pose.position.y
                                - object_pose.pose.position.y
                            )
                            ** 2
                        )

                    result = {
                        "message": f"Successfully navigated to object: {target_object.label}",
                        "object_id": target_object.id,
                        "object_label": target_object.label,
                        "object_position": {
                            "x": object_pose.pose.position.x,
                            "y": object_pose.pose.position.y,
                            "z": object_pose.pose.position.z,
                        },
                        "goal_position": {
                            "x": goal_pose.pose.position.x,
                            "y": goal_pose.pose.position.y,
                            "z": goal_pose.pose.position.z,
                        },
                        "stop_radius": self.stop_radius,
                    }
                    if final_distance is not None:
                        result["final_distance_to_object"] = final_distance

                    self._stop_status_timer()
                    self._publish_status(self.current_skill_id,
                                         "finished", result, errno=0)
                    self.get_logger().info(
                        f"Successfully navigated to object: {target_object.label} "
                        f"(stopped {final_distance:.2f}m away, requested: {self.stop_radius:.2f}m)"
                        if final_distance is not None
                        else f"Successfully navigated to object: {target_object.label}"
                    )
                    self.moving_in_progress = False
                    return
                else:
                    self._stop_status_timer()
                    # Log why we timed out
                    if self.latest_pose is not None:
                        dist = math.sqrt(
                            (self.latest_pose.pose.position.x - goal_pose.pose.position.x) ** 2
                            + (self.latest_pose.pose.position.y - goal_pose.pose.position.y) ** 2
                            + (self.latest_pose.pose.position.z - goal_pose.pose.position.z) ** 2
                        )
                        self.get_logger().error(
                            "Navigation timeout. Last: xyz dist=%.3fm (need<=%.2f)"
                            % (dist, POSITION_DELTA_M)
                        )
                    else:
                        self.get_logger().error(
                            "Navigation timeout. No pose received (check amcl_pose topic)."
                        )
                    self._publish_status(
                        self.current_skill_id,
                        "error",
                        {"error": "Navigation timeout"},
                        errno=10,
                    )
                    self.moving_in_progress = False
                    return

        except Exception as e:
            self._stop_status_timer()
            self.get_logger().error(f"Error in move operation: {e}")
            import traceback

            self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
            self._publish_status(
                self.current_skill_id, "error", {"error": str(e)}, errno=11
            )
            self.moving_in_progress = False

    def _query_semantic_map(self):
        """Query semantic map service for objects.

        Returns:
            list: List of objects from semantic map

        Raises:
            TimeoutError: If the query times out
            RuntimeError: If the service is not available or other errors occur
        """
        if not self.semantic_map_client:
            raise RuntimeError("Semantic map client not initialized")

        # Wait for service
        if not self.semantic_map_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Semantic map service not available")

        request = QuerySemanticMap.Request()
        request.types = []

        future = self.semantic_map_client.call_async(request)

        start_time = time.time()
        timeout = 10.0
        while not future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not future.done():
            error_msg = f"Semantic map query timeout after {timeout}s"
            self.get_logger().error(error_msg)
            raise TimeoutError(error_msg)

        try:
            response = future.result()
            assert response is not None, "Semantic map query response must not be None"
            objects = list(response.objects)
            self.get_logger().info(
                f"Query semantic map returned {len(objects)} objects"
            )
            if objects:
                object_ids = [obj.id for obj in objects]
                object_labels = [obj.label for obj in objects]
                self.get_logger().debug(
                    f"Available objects: {list(zip(object_ids[:10], object_labels[:10]))}{'...' if len(object_ids) > 10 else ''}"
                )
            return objects
        except Exception as e:
            error_msg = f"Error querying semantic map: {e}"
            self.get_logger().error(error_msg)
            import traceback

            self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
            raise RuntimeError(error_msg) from e

    def _get_object_pose_in_map(self, obj):
        """Get object pose in map frame."""
        for frame_mapping in obj.frame_mapping:
            if frame_mapping.frame_id == "map":
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = float(frame_mapping.center.x)
                pose.pose.position.y = float(frame_mapping.center.y)
                pose.pose.position.z = float(frame_mapping.center.z)
                pose.pose.orientation.w = 1.0
                return pose
        return None

    def _yaw_from_quat(self, q):
        """Extract yaw (radians) from geometry_msgs Quaternion (x, y, z, w)."""
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _calculate_goal_pose_near_object(self, object_pose, stop_radius):
        """
        Calculate goal pose near object, stopping at specified radius to avoid collision.

        Args:
            object_pose: PoseStamped of the object in map frame
            stop_radius: Distance in meters to stop before reaching the object

        Returns:
            PoseStamped: Goal pose that is stop_radius meters away from the object
        """
        goal_pose = PoseStamped()
        goal_pose.header = object_pose.header
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        object_x = object_pose.pose.position.x
        object_y = object_pose.pose.position.y

        if self.latest_pose:
            robot_x = self.latest_pose.pose.position.x
            robot_y = self.latest_pose.pose.position.y

            dx = object_x - robot_x
            dy = object_y - robot_y
            distance_to_object = math.sqrt(dx * dx + dy * dy)

            if distance_to_object > stop_radius:
                unit_dx = dx / distance_to_object
                unit_dy = dy / distance_to_object
                goal_pose.pose.position.x = object_x - unit_dx * stop_radius
                goal_pose.pose.position.y = object_y - unit_dy * stop_radius
            else:
                if distance_to_object > 0.1:
                    goal_pose.pose.position.x = robot_x
                    goal_pose.pose.position.y = robot_y
                else:
                    goal_pose.pose.position.x = object_x - stop_radius
                    goal_pose.pose.position.y = object_y
        else:
            goal_pose.pose.position.x = object_x - stop_radius
            goal_pose.pose.position.y = object_y

        goal_pose.pose.position.z = object_pose.pose.position.z

        dx_to_object = object_x - goal_pose.pose.position.x
        dy_to_object = object_y - goal_pose.pose.position.y
        yaw = math.atan2(dy_to_object, dx_to_object)

        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        return goal_pose

    def pose_cov_callback(self, msg):
        """Handle PoseWithCovarianceStamped updates and convert to PoseStamped."""
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.latest_pose = pose_stamped

    def navigate_status_callback(self, msg):
        """Handle navigation status updates."""
        if hasattr(msg, "data"):
            success = msg.data
            if success:
                self.navigation_complete = True
                self.get_logger().info("Navigation completed successfully")

    def _publish_status(self, skill_id, state, result, errno=0):
        """
        Publish skill status to status_topic.

        Standard status format (matching executor expectations):
        {
            "skill_id": string,      # Skill execution ID
            "state": string,         # "running" | "finished" | "error"
            "result": any,           # Result data (any JSON-serializable value)
            "errno": int,            # Error number (0 = success, non-zero = error)
            "error": string,         # Optional: Error message (extracted from result if present)
            "message": string,       # Optional: Status message (extracted from result if present)
            "error_message": string  # Optional: Alternative error message field
        }

        The executor extracts error information from these fields in order:
        1. "error"
        2. "message"
        3. "error_message"
        4. If none found and errno != 0, generates: "Skill execution failed with errno={errno}"
        """
        status_msg = {
            "skill_id": skill_id,
            "state": state,
            "result": result,
            "errno": errno,
        }

        if isinstance(result, dict):
            if "error" in result:
                status_msg["error"] = result["error"]
            if "message" in result:
                status_msg["message"] = result["message"]
            if "error_message" in result:
                status_msg["error_message"] = result["error_message"]

        if (state == "error" or errno != 0) and "error" not in status_msg:
            if isinstance(result, dict) and "error" in result:
                status_msg["error"] = result["error"]
            else:
                status_msg["error"] = (
                    f"Skill execution failed: state={state}, errno={errno}"
                )

        msg = String()
        msg.data = json.dumps(status_msg)
        self.status_publisher.publish(msg)
        self.get_logger().info(
            f"Published status: skill_id={skill_id}, state={state}, errno={errno}"
        )


def main(args=None):
    rclpy.init(args=args)
    move_to_object_skill = MoveToObjectSkill()

    shutdown_requested = False

    def signal_handler(signum, frame):
        nonlocal shutdown_requested
        shutdown_requested = True
        move_to_object_skill.get_logger().info(
            f"Received signal {signum}, shutting down..."
        )
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(move_to_object_skill)
    except KeyboardInterrupt:
        shutdown_requested = True
        move_to_object_skill.get_logger().info(
            "Received KeyboardInterrupt, shutting down..."
        )
    finally:
        move_to_object_skill.destroy_node()
        rclpy.shutdown()
        if shutdown_requested:
            move_to_object_skill.get_logger().info(
                "Move to object skill shutdown complete"
            )


if __name__ == "__main__":
    main()
