.. _skill_specs_table:

Capability/Skill 调用规范表
==============================================

版本：2025年9月2日

本文档展示了DeepEmbody OS系统中所有可用的能力(Capabilities)和技能(Skills)的完整规范。

技能规范表格
------------

能力 (Capabilities)
~~~~~~~~~~~~~~~~~~~

.. list-table:: 系统能力列表
   :header-rows: 1
   :widths: 20 50 15 15

   * - 能力名称
     - 描述
     - 输入类型
     - 输出类型
   * - cap_camera_rgb
     - 从指定摄像头获取RGB图像
     - camera_name: str, timeout_sec: float
     - opencv图像 (numpy array)
   * - cap_camera_dep_rgb
     - 从指定摄像头获取RGB和深度图像
     - camera_name: str, timeout_sec: float
     - Tuple[rgb_image, depth_image]
   * - cap_camera_info
     - 获取指定摄像头的参数信息
     - camera_name: str, timeout_sec: float
     - Dict[str, Any]
   * - cap_save_rgb_image
     - 捕获并保存RGB图像到文件
     - filename: str, camera_name: str, width: int, height: int
     - success: bool
   * - cap_save_depth_image
     - 捕获并保存深度图像到文件
     - filename: str, camera_name: str, width: int, height: int
     - success: bool
   * - cap_get_robot_pose
     - 获取机器人当前位姿
     - timeout_sec: float
     - x: float, y: float, z: float, yaw: float
   * - cap_set_goal
     - 设置机器人目标点
     - x: float, y: float, yaw: float
     - str
   * - cap_stop_goal
     - 停止机器人目标点
     - None
     - str
   * - cap_get_object_global_pos
     - 基于机器人位姿、像素坐标、深度和相机参数计算物体全局位置
     - pixel_x: float, pixel_y: float, depth: float, camera_info: Dict, robot_pose: Dict
     - Tuple[float, float, float]
   * - cap_get_pose
     - 获取机器人当前位姿
     - None 或 timeout_sec: float
     - Tuple[float, float, float]
   * - cap_tf_transform
     - 坐标系变换
     - source_frame: str, target_frame: str, x: float, y: float, z: float
     - Tuple[float, float, float]

技能 (Skills)
~~~~~~~~~~~~~~

.. list-table:: 系统技能列表
   :header-rows: 1
   :widths: 20 40 20 20

   * - 技能名称
     - 描述
     - 输入类型
     - 依赖能力
   * - skl_debug_test_skill
     - 测试技能
     - input_val: int
     - 无
   * - skl_detect_objs
     - 在指定摄像头的当前视野中检测物体
     - camera_name: str
     - cap_camera_dep_rgb, cap_camera_info, cap_get_object_global_pos, cap_get_robot_pose
   * - skl_move_to_goal
     - 移动机器人到目标点
     - goal_name: str
     - cap_set_goal
   * - skl_move_to_ab_pos
     - 移动机器人到绝对位置
     - x: float, y: float, yaw: float
     - cap_set_goal
   * - skl_move_to_rel_pos
     - 移动机器人到相对位置
     - dx: float, dy: float, dyaw: float
     - cap_set_goal, cap_get_pos
   * - skl_update_map
     - 更新语义地图
     - camera_name: str
     - skl_detect_objs

已弃用的能力
~~~~~~~~~~~~

.. list-table:: 已弃用的能力列表
   :header-rows: 1
   :widths: 20 50 15 15

   * - 能力名称
     - 描述
     - 输入类型
     - 输出类型
   * - cap_space_getpos
     - 获取实体位置 (已弃用)
     - None
     - x: float, y: float, z: float
   * - cap_space_move
     - 移动实体到指定位置 (已弃用)
     - x: float, y: float, z: float
     - success: bool

技能规范源码
------------

.. literalinclude:: ../../../uapi/specs/skill_specs.py
   :language: python
   :linenos: