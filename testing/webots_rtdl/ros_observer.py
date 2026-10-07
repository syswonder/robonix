# SPDX-License-Identifier: MulanPSL-2.0
"""Observe Webots navigation without commanding it; run inside the ROS container."""
import json
import math
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def position(p, q):
    """Flatten a ROS position and quaternion for independent motion checks."""
    return {'x': p.x, 'y': p.y,
            'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}


def main():
    """Emit fresh TF/odometry, action status, and occupancy observations as JSONL."""
    rclpy.init()
    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    node = rclpy.create_node('rtdl_navigation_observer')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    latest = {}
    last_map = 0.0

    def odom(message):
        """Retain measured pose and velocity, with wall-clock receipt time."""
        p = message.pose.pose
        v = message.twist.twist
        latest['odom'] = dict(position(p.position, p.orientation),
                              v=math.hypot(v.linear.x, v.linear.y), w=v.angular.z,
                              received=time.time())

    def status(message):
        """Retain Nav2 action goal identities and lifecycle states."""
        latest['goals'] = [{'id': bytes(s.goal_info.goal_id.uuid).hex(),
                            'status': s.status} for s in message.status_list]

    def grid(message):
        """Record a current map at most every five seconds for goal selection."""
        nonlocal last_map
        if time.monotonic() - last_map < 5:
            return
        last_map = time.monotonic()
        info = message.info
        print(json.dumps({'type': 'map', 'time': time.time(),
                          'frame': message.header.frame_id,
                          'resolution': info.resolution, 'width': info.width,
                          'height': info.height,
                          'origin': position(info.origin.position, info.origin.orientation),
                          'data': list(message.data)}), flush=True)

    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Odometry, '/odom', odom, 10)
    node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', status, durable)
    node.create_subscription(OccupancyGrid, '/map', grid, durable)
    last_sample = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - last_sample < 0.2:
                continue
            last_sample = time.monotonic()
            try:
                t = buffer.lookup_transform('map', 'base_link', Time())
            except TransformException:
                continue
            print(json.dumps({'type': 'sample', 'time': time.time(),
                              'pose': position(t.transform.translation, t.transform.rotation),
                              'tf_stamp': t.header.stamp.sec + t.header.stamp.nanosec/1e9,
                              **latest}), flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
