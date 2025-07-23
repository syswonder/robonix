import grpc
import robot_control_pb2 as robot_control_pb2
import robot_control_pb2_grpc as robot_control_pb2_grpc
import math
import time

# gRPC client setup
channel = grpc.insecure_channel('localhost:50051')
stub = robot_control_pb2_grpc.RobotControlStub(channel)

def get_pose():
    response = stub.GetPose(robot_control_pb2.GetPoseRequest())
    return response.x, response.y, response.yaw

def rotate_to_yaw(target_yaw_deg, tolerance=2.0):
    while True:
        _, _, current_yaw = get_pose()
        diff = (target_yaw_deg - current_yaw + 180) % 360 - 180
        if abs(diff) < tolerance:
            stub.Stop(robot_control_pb2.StopRequest())
            break
        angle = math.radians(diff)
        stub.Rotate(robot_control_pb2.RotateRequest(angle=angle))
        time.sleep(0.1)
        stub.Stop(robot_control_pb2.StopRequest())
        time.sleep(0.05)

def move_forward(distance):
    stub.MoveTo(robot_control_pb2.MoveToRequest(forward=distance, lateral=0.0))

def move_to_point(target_x, target_y):
    x, y, yaw = get_pose()
    dx = target_x - x
    dy = target_y - y
    distance = math.hypot(dx, dy)
    if distance < 1e-3:
        return
    target_yaw_deg = math.degrees(math.atan2(dx, dy)) % 360
    rotate_to_yaw(target_yaw_deg)
    move_forward(distance)
    time.sleep(max(distance, 1.0))

if __name__ == "__main__":
    # print("move to (1.0, 1.0)")
    # move_to_point(1.0, 1.0)
    # print("move to (2.5, 3.0)")
    # move_to_point(2.5, 3.0)
    # print("move to (1.5, 0.0)")
    # move_to_point(1.5, 0.0)
    # print("move to (0.0, 0.0)")
    # move_to_point(0.0, 0.0)
    # print("goodbye")
    move_to_point(-2.2, 1.8)
    move_to_point(0,0)