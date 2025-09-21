import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # 导入 TaskResult
import sys

def setgoal(x, y, yaw):
    rclpy.init()

    navigator = BasicNavigator()

    # 创建目标位姿消息
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg() # 使用当前时间戳
    goal_pose.pose.position.x = float(x)
    goal_pose.pose.position.y = float(y)
    goal_pose.pose.position.z = 0.0 # 通常在2D导航中z为0，但根据你的原始命令保留
    goal_pose.pose.orientation.x = float(yaw)
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    # 使用BasicNavigator发送目标位姿
    print('Going to goal pose...')
    navigator.goThroughPoses([goal_pose])

    # 可选：等待导航完成
    # 确保在spin_once循环中处理事件
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        # if feedback and feedback.current_pose:
        #     print(f"Current pose: x={feedback.current_pose.pose.position.x:.2f}, "
        #           f"y={feedback.current_pose.pose.position.y:.2f}")
        # 使用 navigator 对象本身进行 spin_once
        rclpy.spin_once(navigator, timeout_sec=5.0)

    rclpy.shutdown()
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
        return True
    else:
        print('Goal failed!')
        return False

    
if __name__ == '__main__':
    ret = setgoal(sys.argv[1], sys.argv[2], sys.argv[3])
    if ret == True:
        print("到达目标点成功")
    else:
        print("到达目标点失败")
