#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# Wandering Skill
#
# Random wandering skill that explores the environment by navigating to random positions.
# The skill generates random goal poses and navigates to them repeatedly.
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
import random
import time
import signal
import math
import sys
from robonix_sdk.srv import QueryPrimitive


class WanderingSkill(Node):
    """Implements wandering skill that randomly explores the environment."""

    def __init__(self):
        super().__init__("wandering_skill")

        self.start_topic = "/robot1/skill/wandering/start"
        self.status_topic = "/robot1/skill/wandering/status"

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

        self._query_primitives()

        self.start_subscriber = self.create_subscription(
            String, self.start_topic, self.start_callback, 10
        )
        self.get_logger().info(
            f"Subscribing to start topic: {self.start_topic}")

        self.status_publisher = self.create_publisher(
            String, self.status_topic, 10)
        self.get_logger().info(
            f"Publishing to status topic: {self.status_topic}")

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
            self.navigate_status_subscriber = self.create_subscription(
                Bool, self.navigate_status_topic, self.navigate_status_callback, 10
            )
            self.get_logger().info(
                f"Subscribing to navigate status: {self.navigate_status_topic}"
            )
        else:
            self.navigate_status_subscriber = None

        self.current_skill_id = None
        self.wandering_in_progress = False
        self._cancel_requested = False
        self.navigation_complete = False
        self.latest_pose = None
        self.wander_radius = 5.0
        self.starting_position = None

        self.get_logger().info("Wandering skill initialized")

    def _query_primitives(self):
        """Query robonix core for required primitives. Exits if any required primitive is not found."""
        max_retries = 5
        retry_delay = 2.0

        self.get_logger().info("Querying prm::base.pose.cov...")
        pose_found = False
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
                        pose_found = True
                        break
            except Exception as e:
                self.get_logger().error(
                    f"Error querying prm::base.pose.cov: {e}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                else:
                    break

        if not pose_found:
            self.get_logger().error(
                "Failed to query prm::base.pose.cov after all retries. Exiting."
            )
            sys.exit(1)

        self.get_logger().info("Querying prm::base.navigate...")
        navigate_found = False
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
                        navigate_found = True
                    if "status" in output_schema:
                        self.navigate_status_topic = output_schema["status"]
                        self.get_logger().info(
                            f"  Found navigate status topic: {self.navigate_status_topic}"
                        )
                    if navigate_found:
                        break
            except Exception as e:
                self.get_logger().error(
                    f"Error querying prm::base.navigate: {e}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                else:
                    break

        if not navigate_found:
            self.get_logger().error(
                "Failed to query prm::base.navigate after all retries. Exiting."
            )
            sys.exit(1)

    def start_callback(self, msg):
        """Handle skill start request (input params or robonix terminate)."""
        try:
            data = json.loads(msg.data)
            # Robonix can send terminate to start_topic to force-stop this skill
            if data.get("terminate") and data.get("skill_id"):
                sid = data.get("skill_id")
                if self.wandering_in_progress and sid == self.current_skill_id:
                    self.get_logger().info(
                        f"Received terminate for skill_id={sid}, stopping wandering"
                    )
                    self._cancel_requested = True
                    self.wandering_in_progress = False
                    self._publish_status(
                        sid,
                        "cancelled",
                        {"message": "Cancelled by robonix (terminate on start_topic)"},
                        errno=0,
                    )
                return

            skill_id = data.get("skill_id", "unknown")
            params = data.get("params", {})

            if "wander_radius" in params:
                self.wander_radius = float(params["wander_radius"])

            self.get_logger().info(
                f"Received wandering request: skill_id={skill_id}, wander_radius={self.wander_radius}"
            )

            if self.wandering_in_progress:
                self.get_logger().warn(
                    "Wandering already in progress, ignoring request"
                )
                self._publish_status(
                    skill_id,
                    "error",
                    {"error": "Operation already in progress"},
                    errno=1,
                )
                return

            if not self.navigate_goal_publisher:
                self.get_logger().error("Navigation primitive not available!")
                self._publish_status(
                    skill_id,
                    "error",
                    {"error": "Navigation primitive not available"},
                    errno=2,
                )
                return

            if not self.latest_pose:
                self.get_logger().warn("No initial pose available, waiting...")
                for _ in range(10):
                    rclpy.spin_once(self, timeout_sec=0.5)
                    if self.latest_pose:
                        break

                if not self.latest_pose:
                    self.get_logger().error("Could not get initial pose!")
                    self._publish_status(
                        skill_id,
                        "error",
                        {"error": "Could not get initial pose"},
                        errno=5,
                    )
                    return

            self.starting_position = (
                self.latest_pose.pose.position.x,
                self.latest_pose.pose.position.y,
            )

            self.current_skill_id = skill_id
            self._cancel_requested = False
            self.wandering_in_progress = True

            self._publish_status(
                skill_id, "running", {"message": "Starting wandering..."}
            )

            self._start_wandering_loop()

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse start message JSON: {e}")
            self._publish_status(
                "unknown", "error", {"error": f"Invalid JSON: {e}"}, errno=3
            )
        except Exception as e:
            self.get_logger().error(f"Error in start_callback: {e}")
            import traceback

            self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
            self._publish_status("unknown", "error", {
                                 "error": str(e)}, errno=4)

    def _start_wandering_loop(self):
        """Start the wandering loop."""
        import threading

        thread = threading.Thread(target=self._wandering_loop, daemon=True)
        thread.start()

    def _wandering_loop(self):
        """Main wandering loop that generates random goals and navigates to them."""
        max_iterations = 20
        iteration = 0

        while (
            self.wandering_in_progress
            and not self._cancel_requested
            and iteration < max_iterations
        ):
            iteration += 1
            self.get_logger().info(
                f"Wandering iteration {iteration}/{max_iterations}")

            try:
                if not self.latest_pose:
                    self.get_logger().warn("No current pose available, waiting...")
                    time.sleep(1.0)
                    continue

                current_x = self.latest_pose.pose.position.x
                current_y = self.latest_pose.pose.position.y

                if self.starting_position:
                    angle = random.uniform(0, 2 * math.pi)
                    distance = random.uniform(1.0, self.wander_radius)
                    goal_x = self.starting_position[0] + \
                        distance * math.cos(angle)
                    goal_y = self.starting_position[1] + \
                        distance * math.sin(angle)
                else:
                    angle = random.uniform(0, 2 * math.pi)
                    distance = random.uniform(1.0, 3.0)
                    goal_x = current_x + distance * math.cos(angle)
                    goal_y = current_y + distance * math.sin(angle)

                goal_yaw = random.uniform(-math.pi, math.pi)

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = "map"
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose.position.x = goal_x
                goal_pose.pose.position.y = goal_y
                goal_pose.pose.position.z = 0.0

                goal_pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
                goal_pose.pose.orientation.w = math.cos(goal_yaw / 2.0)

                self.get_logger().info(
                    f"Navigating to random goal at ({goal_x:.2f}, {goal_y:.2f}), "
                    f"yaw={math.degrees(goal_yaw):.1f}°"
                )

                self.navigation_complete = False
                if self.navigate_goal_publisher is None:
                    raise RuntimeError(
                        "Navigation goal publisher is not available")
                self.navigate_goal_publisher.publish(goal_pose)

                timeout = 90.0
                start_time = time.time()
                while (
                    not self.navigation_complete
                    and (time.time() - start_time) < timeout
                    and not self._cancel_requested
                ):
                    rclpy.spin_once(self, timeout_sec=0.1)

                if self.navigation_complete:
                    self.get_logger().info("Navigation completed successfully")
                    self._publish_status(
                        self.current_skill_id,
                        "running",
                        {
                            "message": f"Reached goal {iteration}",
                            "iteration": iteration,
                            "goal_position": {"x": goal_x, "y": goal_y},
                        },
                        errno=0,
                    )
                else:
                    self.get_logger().warn(
                        f"Navigation timeout for iteration {iteration}"
                    )

                time.sleep(1.0)

            except Exception as e:
                self.get_logger().error(f"Error in wandering loop: {e}")
                import traceback

                self.get_logger().error(
                    f"Traceback:\n{traceback.format_exc()}")
                time.sleep(2.0)

        self.wandering_in_progress = False
        if self._cancel_requested:
            self._publish_status(
                self.current_skill_id,
                "cancelled",
                {"message": "Cancelled by robonix (terminate on start_topic)"},
                errno=0,
            )
            self.get_logger().info("Wandering cancelled by robonix")
        else:
            result = {"message": "Wandering completed", "iterations": iteration}
            self._publish_status(
                self.current_skill_id, "finished", result, errno=0
            )
            self.get_logger().info(
                f"Wandering completed after {iteration} iterations."
            )

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
    wandering_skill = WanderingSkill()

    shutdown_requested = False

    def signal_handler(signum, frame):
        nonlocal shutdown_requested
        shutdown_requested = True
        wandering_skill.get_logger().info(
            f"Received signal {signum}, shutting down...")
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(wandering_skill)
    except KeyboardInterrupt:
        shutdown_requested = True
        wandering_skill.get_logger().info(
            "Received KeyboardInterrupt, shutting down..."
        )
    finally:
        wandering_skill.destroy_node()
        rclpy.shutdown()
        if shutdown_requested:
            wandering_skill.get_logger().info("Wandering skill shutdown complete")


if __name__ == "__main__":
    main()
