# SPDX-License-Identifier: MulanPSL-2.0
"""Report every time the robot touches something it should not.

The simulator knows each contact the robot's solids make. Contacts at floor
level are its wheels and casters driving; any contact higher up is a
collision: a body part touching furniture, or a wheel rubbing a wall. Height
decides, not which solid touched, because TIAGo's wheels are PROTO internals
the supervisor cannot name. Navigation tests use this as the judge of "never
hits anything", which the robot's own sensors cannot give: TIAGo's bumper
covers only the base, and a table top meets the torso.

This is a reference signal for evaluation. Nothing in the navigation stack
should consume it.
"""

from __future__ import annotations

import json

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String, UInt32

#: A contact higher than this above the floor is not the wheels on the floor.
FLOOR_TOLERANCE_M = 0.03
#: Contacts closer together than this are one collision, not several.
DEBOUNCE_S = 0.5


class CollisionMonitor:
    """webots_ros2_driver plugin publishing the robot's collisions.

    Needs `supervisor TRUE` on the robot node; without it the plugin stays a
    no-op rather than taking the driver down.
    """

    _node = None
    _self_node = None

    def init(self, webots_node, properties):  # noqa: D102 - driver-called hook
        self._robot = webots_node.robot
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node("webots_collision_monitor")
        self._node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        get_self = getattr(self._robot, "getSelf", None)
        self._self_node = get_self() if get_self else None
        if self._self_node is None:
            self._node.get_logger().warn("collision monitor disabled: the robot is not a supervisor")
            return
        step = int(self._robot.getBasicTimeStep())
        self._self_node.enableContactPointsTracking(step, True)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._events = self._node.create_publisher(String, properties.get("topic", "/webots/collision"), 10)
        self._count_pub = self._node.create_publisher(UInt32, properties.get("countTopic", "/webots/collision_count"), latched)
        self._names: dict[int, str] = {}
        self._count = 0
        self._last = -1e9
        self._touching = False
        self._publish_count()
        self._node.get_logger().info("publishing robot collisions on /webots/collision")

    def _name(self, node_id: int) -> str:
        if node_id not in self._names:
            node = self._robot.getFromId(node_id)
            field = node.getField("name") if node else None
            self._names[node_id] = field.getSFString() if field else (node.getTypeName() if node else f"solid {node_id}")
        return self._names[node_id]

    def _publish_count(self):
        self._count_pub.publish(UInt32(data=self._count))

    def step(self):  # noqa: D102 - driver-called hook
        if self._self_node is None:
            return
        rclpy.spin_once(self._node, timeout_sec=0)
        hits = []
        for cp in self._self_node.getContactPoints(True):
            if cp.point[2] > FLOOR_TOLERANCE_M:
                hits.append((self._name(cp.node_id), cp.point))
        now = self._robot.getTime()
        if hits and not self._touching and now - self._last >= DEBOUNCE_S:
            self._count += 1
            pose = self._self_node.getPosition()
            event = {
                "t": round(now, 3),
                "count": self._count,
                "solids": sorted({n for n, _ in hits}),
                "point": [round(v, 3) for v in hits[0][1]],
                "robot": [round(pose[0], 3), round(pose[1], 3)],
            }
            self._events.publish(String(data=json.dumps(event)))
            self._publish_count()
            self._node.get_logger().warn(f"collision {self._count}: {event['solids']} at {event['point']}")
        if hits:
            self._last = now
        self._touching = bool(hits)
