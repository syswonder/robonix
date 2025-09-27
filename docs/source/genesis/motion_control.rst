CarController 运动控制
============

CarController 是 Genesis 模拟器的运动控制模块，实现了基于物理的运动控制模型，支持键盘输入处理、平滑速度控制、精确位置导航和实时姿态管理，为机器人提供了真实可信的运动行为。

控制架构设计
-----------

CarController 类
~~~~~~~~~~~~~~~~

CarController类采用分层控制架构，将运动控制分解为多个相互协作的子系统：

**输入处理层**
  负责处理来自键盘、程序接口和网络命令的控制输入。该层实现了输入的标准化和优先级管理，确保不同控制源的协调工作。

**运动规划层**
  基于当前状态和目标状态，计算最优的运动轨迹。采用平滑的加速度控制模型，避免突兀的速度变化，提高运动的真实性。

**物理执行层**
  将运动规划的结果转换为Genesis引擎可执行的物理命令，包括位置更新、姿态调整和碰撞处理。

**状态管理层**
  维护机器人的完整运动状态，包括位置、速度、加速度和姿态信息。提供状态查询和历史记录功能。

运动学模型
---------

**坐标系统**
  系统采用右手坐标系，X轴向前，Y轴向左，Z轴向上。机器人的运动在XY平面内进行，Z轴方向的旋转表示机器人的朝向角度。

**速度控制模型**
  机器人的运动通过三个速度分量控制：

  - ``vx`` - 前进/后退速度（X轴方向）
  - ``vy`` - 左移/右移速度（Y轴方向）  
  - ``wz`` - 旋转角速度（绕Z轴）

**加速度平滑机制**
  为了避免机器人运动的突兀变化，系统实现了平滑的加速度控制：

.. code-block:: python

   def smooth_approach(current, target, accel, dt):
       diff = target - current
       max_change = accel * dt
       if abs(diff) <= max_change:
           return target
       else:
           return current + np.sign(diff) * max_change

这种机制确保了机器人运动的连续性和真实感。

键盘控制
-----------

**按键映射配置**
  系统定义了标准的键盘控制映射：

  - 方向键：前进/后退/左移/右移
  - ``[`` / ``]``：左转/右转
  - ``-``：重置到初始位置
  - ``ESC``：退出程序

**实时输入处理**
  键盘输入通过独立线程进行监听和处理，确保了控制的实时响应性。系统支持多键同时按下，实现复合运动控制。

**优先级管理**
  当同时存在键盘输入和程序控制时，键盘输入具有更高优先级，这种设计便于调试和紧急干预。

位置导航
-----------

MoveTo 功能
~~~~~~~~~~~~~~

位置导航系统实现了精确的点到点导航能力：

**目标设定机制**
  通过设置机器人的 ``_move_to_target`` 属性来启动导航任务。目标包含坐标位置和激活状态信息。

**路径规划算法**
  采用简单而有效的直线路径规划：

.. code-block:: python

   # Calculate vector to target
   rem_x = target_x - curr_x
   rem_y = target_y - curr_y
   rem_dist = np.hypot(rem_x, rem_y)
   
   # Normalize direction vector
   if rem_dist > distance_threshold:
       norm_x = rem_x / rem_dist
       norm_y = rem_y / rem_dist
       
       # Calculate target velocity
       self.move_to_vx = norm_x * self.max_speed * 0.8
       self.move_to_vy = norm_y * self.max_speed * 0.8

**到达判定机制**
  系统使用距离阈值（默认0.1米）来判定是否到达目标位置。到达目标后自动停止导航任务。

**超时保护机制**
  导航任务设有超时保护，防止因环境障碍或算法问题导致的无限导航。

姿态管理
-----------

**姿态表示方法**
  机器人的姿态使用欧拉角表示，主要关注绕Z轴的旋转角度（yaw角）。系统维护了机器人的当前朝向和目标朝向。

**姿态更新算法**
  姿态更新基于角速度积分：

.. code-block:: python

   # Update robot orientation
   self.car._my_yaw += self.wz * self.dt
   
   # Normalize angle to [-π, π] range
   while self.car._my_yaw > np.pi:
       self.car._my_yaw -= 2 * np.pi
   while self.car._my_yaw < -np.pi:
       self.car._my_yaw += 2 * np.pi

**坐标变换处理**
  系统支持本体坐标系和世界坐标系之间的变换，确保运动控制的正确性：

.. code-block:: python

   # Transform from body coordinate system to world coordinate system
   world_vx = body_vx * np.cos(yaw) - body_vy * np.sin(yaw)
   world_vy = body_vx * np.sin(yaw) + body_vy * np.cos(yaw)

物理集成
-----------

**Genesis引擎接口**
  运动控制系统通过标准接口与Genesis物理引擎集成：

  - ``get_qpos()``：获取机器人当前位置和姿态
  - ``set_qpos()``：设置机器人位置和姿态
  - ``get_qvel()``：获取机器人当前速度
  - ``set_qvel()``：设置机器人速度

**物理约束处理**
  系统考虑了物理世界的约束条件：

  - 重力影响：机器人受重力作用，需要地面支撑
  - 碰撞检测：与墙体和障碍物的碰撞会影响运动
  - 摩擦力：地面摩擦影响机器人的加速和减速

**实时同步机制**
  控制系统与物理仿真保持同步更新，确保控制指令的及时执行和状态反馈的准确性。

状态监控
-----------

**状态记录机制**
  系统持续记录机器人的运动状态，包括：

  - 位置历史：记录机器人的运动轨迹
  - 速度历史：监控速度变化趋势
  - 控制输入：记录各种控制命令的执行情况

**性能监控指标**
  - 位置精度：实际位置与目标位置的偏差
  - 响应时间：从控制输入到运动响应的延迟
  - 运动平滑度：速度和加速度的连续性指标

**调试信息输出**
  系统提供详细的调试信息输出，便于开发者监控和调试运动控制的行为。

重置和恢复
~~~~~~~~~~~~~

**状态重置功能**
  系统支持将机器人重置到初始状态：

.. code-block:: python

   def reset_car(self):
       # Reset position to initial position
       initial_pos = getattr(self.car, '_initial_pos', (0.0, -2.0, 0.15))
       initial_yaw = getattr(self.car, '_initial_yaw', 0.0)
       
       # Set new position and orientation
       new_qpos = [initial_pos[0], initial_pos[1], initial_pos[2], 
                   0.0, 0.0, 0.0, 1.0]  # Position + quaternion orientation
       self.car.set_qpos(new_qpos)
       
       # Reset velocity and state
       self.car._my_yaw = initial_yaw
       self.vx = self.vy = self.wz = 0.0