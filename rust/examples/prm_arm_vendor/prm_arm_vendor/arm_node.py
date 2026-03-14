# SPDX-License-Identifier: MulanPSL-2.0
"""Example arm vendor: implements prm::arm (move_ee, joint_trajectory) and prm::gripper (close, open).
Partial implementation - does not implement move_joint, state_joint, set_width, state_width.

Code structure:
- [Generated] ridlc generates: create_*_server, RobonixRuntimeStub, msg types
- [You write] In main(): connect to meta, create servers, implement execute callbacks, start + spin
- [Hook] Command server: attach callback via server.execute = your_callback
"""

import grpc
import rclpy

# [Generated] ridlc generates create_*_server from RIDL
from robonix.prm.arm.move_ee_command import create_move_ee_server
from robonix.prm.arm.joint_trajectory_command import create_joint_trajectory_server
from robonix.prm.gripper.close_command import create_close_server
from robonix.prm.gripper.open_command import create_open_server
from robonix_runtime_pb2_grpc import RobonixRuntimeStub
from std_msgs.msg import Bool
from robonix_msgs.msg import CommandResult


def main() -> None:
    endpoint = "127.0.0.1:50051"
    node_id = "com.robonix.example.arm"

    # [Required] 1) Init ROS2  2) Connect to meta gRPC  3) Get runtime_client (for channel registration/resolution)
    rclpy.init()
    grpc_channel = grpc.insecure_channel(endpoint)
    runtime_client = RobonixRuntimeStub(grpc_channel)

    # [Generated] create_*_server returns server with execute placeholder; you assign your impl
    # This example registers only the interfaces this hardware supports (partial impl)
    move_ee_srv = create_move_ee_server(runtime_client, node_id=node_id)
    joint_traj_srv = create_joint_trajectory_server(runtime_client, node_id=node_id)
    close_srv = create_close_server(runtime_client, node_id=node_id)
    open_srv = create_open_server(runtime_client, node_id=node_id)

    # [You write] execute callback: request is action Goal with input fields; return action Result
    def move_ee_execute(request, goal_handle=None):
        # Real impl: arm_driver.move_to_pose(request.pose)
        result = move_ee_srv._action_type.Result()
        result.status = Bool()
        result.status.data = True
        return result

    def joint_trajectory_execute(request, goal_handle=None):
        # Real impl: arm_driver.execute_trajectory(request.trajectory)
        result = joint_traj_srv._action_type.Result()
        result.status = CommandResult()
        result.status.success = True
        result.status.message = "ok"
        return result

    def close_execute(request, goal_handle=None):
        # Real impl: gripper_driver.close()
        result = close_srv._action_type.Result()
        result.status = Bool()
        result.status.data = True
        return result

    def open_execute(request, goal_handle=None):
        # Real impl: gripper_driver.open()
        result = open_srv._action_type.Result()
        result.status = Bool()
        result.status.data = True
        return result

    # [Hook] Attach callbacks to server.execute; invoked when action receives a request after start()
    move_ee_srv.execute = move_ee_execute
    joint_traj_srv.execute = joint_trajectory_execute
    close_srv.execute = close_execute
    open_srv.execute = open_execute

    move_ee_srv.start()
    joint_traj_srv.start()
    close_srv.start()
    open_srv.start()

    try:
        rclpy.spin(move_ee_srv)
    finally:
        move_ee_srv.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
