# 导入所需的库
# Import the required libraries
import cv2  # OpenCV library for image processing
import numpy as np  # NumPy library for array and matrix operations

# 相机内参矩阵K，包括相机的焦距和主点坐标
# Camera intrinsic matrix K, including camera's focal length and principal point coordinates
K = [[602.7175003324863, 0, 351.305582038406],
     [0, 601.6330312976042, 240.0929104708551],
     [0, 0, 1]]

# 相机畸变参数D，用于校正图像畸变
# Camera distortion parameters D, used for correcting image distortion
D = [0.06712174262966401, -0.2636999208734844,
     0.006484443443073637, 0.01111161327049835, 0]

# 定义一个函数px2xy，用于将像素坐标转换为相机坐标系下的二维坐标
# Define a function px2xy to convert pixel coordinates to 2D coordinates in camera coordinate system
def px2xy(point, camera_k, camera_d, z=1.0):
    # 将相机内参矩阵K和相机畸变参数D转换为NumPy数组
    # Convert camera intrinsic matrix K and camera distortion parameters D to NumPy arrays
    MK = np.array(camera_k, dtype=float).reshape(3, 3)
    MD = np.array(camera_d, dtype=float)
    
    # 将输入的像素坐标点转换为NumPy数组
    # Convert the input pixel coordinate point to a NumPy array
    point = np.array(point, dtype=float)
    
    # 使用OpenCV的cv2.undistortPoints函数对输入点进行畸变矫正，并乘以深度值z
    # Use OpenCV's cv2.undistortPoints function to correct distortion of input points and multiply by depth value z
    pts_uv = cv2.undistortPoints(point, MK, MD) * z
    
    # 返回相机坐标系下的二维坐标
    # Return 2D coordinates in the camera coordinate system
    return pts_uv[0][0]

# 调用函数并打印结果（如果需要）
# Call the function and print the result (if needed)
# print(px2xy([0, 0], K, D, 1))
