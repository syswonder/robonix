CameraManager 相机管理
========

CameraManager 是 Genesis 模拟器的相机管理模块，提供了完整的机器人视觉仿真能力。该系统通过多线程架构实现了实时图像采集、处理和存储功能，支持RGB图像、深度图像和相机参数的获取，为机器人的视觉算法提供了高质量的数据支持。

系统架构设计
-----------

CameraManager 类
~~~~~~~~~~~~~~~~

CameraManager类采用生产者-消费者模式设计，通过独立线程处理图像采集任务，避免阻塞主仿真循环：

**多线程处理架构**
  - 主线程负责仿真控制和用户交互
  - 相机线程专门处理图像采集和处理
  - 线程间通过事件机制进行同步和通信

**资源管理机制**
  - 自动创建输出目录结构
  - 智能的文件命名和版本管理
  - 完善的线程生命周期管理

**配置参数系统**
  - 灵活的采集间隔设置
  - 可配置的输出路径和格式
  - 动态的图像质量参数调节

核心功能模块
-----------

相机定位系统
~~~~~~~~~~~

**动态跟随机制**
  相机系统实现了智能的机器人跟随功能，相机位置根据机器人的实时状态动态调整：

.. code-block:: python

   # 获取机器人当前状态
   car_pos = self.car.get_pos()
   car_yaw = getattr(self.car, "_my_yaw", 0.0)
   
   # 计算相机在机器人前方的位置
   camera_offset_x = 0.3 * np.sin(car_yaw)
   camera_offset_y = 0.3 * np.cos(car_yaw)
   camera_x = car_x + camera_offset_x
   camera_y = car_y + camera_offset_y
   camera_z = car_z + 0.2  # 略高于机器人中心

**视线方向计算**
  系统根据机器人朝向自动计算相机的观察方向，确保相机始终朝向机器人的前进方向：

.. code-block:: python

   # 计算前视方向的观察点
   lookat_x = camera_x + np.sin(car_yaw)
   lookat_y = camera_y + np.cos(car_yaw)
   lookat_z = camera_z

**多视角支持**
  系统支持多种相机安装模式：
  - 前置相机：安装在机器人前方，用于导航和避障
  - 顶置相机：俯视角度，用于全局定位和地图构建
  - 侧置相机：侧面视角，用于环境感知和物体检测

图像采集系统
~~~~~~~~~~~

**RGB图像采集**
  系统提供高质量的RGB图像采集功能，支持多种分辨率和格式：

.. code-block:: python

   def capture_rgb_image(self, width=640, height=480, save_to_disk=True):
       # 渲染RGB图像
       self.camera.render()
       rgb_image = self.camera.get_picture("Color")
       
       # 格式转换和处理
       if hasattr(rgb_image, "cpu"):
           rgb_image = rgb_image.cpu().numpy()
       
       # 图像后处理
       rgb_image = (rgb_image * 255).astype(np.uint8)
       rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
       
       return rgb_image

**深度图像采集**
  深度图像提供了精确的距离信息，支持机器人的3D感知能力：

.. code-block:: python

   def capture_depth_image(self, width=640, height=480, save_to_disk=True):
       # 渲染深度图像
       self.camera.render()
       depth_image = self.camera.get_picture("Depth")
       
       # 深度值处理
       if hasattr(depth_image, "cpu"):
           depth_image = depth_image.cpu().numpy()
       
       # 深度范围标准化
       min_depth = np.min(depth_image)
       max_depth = np.max(depth_image)
       
       return depth_image, min_depth, max_depth

**相机参数获取**
  系统提供完整的相机内参和外参信息：

.. code-block:: python

   def get_camera_info(self):
       # 获取相机内参矩阵
       K = self.camera.get_intrinsic_matrix()
       
       # 获取相机外参信息
       cam_pos = self.camera.get_pos()
       cam_lookat = self.camera.get_lookat()
       
       return {
           "intrinsic_matrix": K.tolist(),
           "position": cam_pos,
           "lookat": cam_lookat,
           "fov": self.camera.fov,
           "resolution": [self.camera.W, self.camera.H]
       }