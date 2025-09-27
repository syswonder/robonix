Virtual Entity Graph（VEG）虚拟实体图
===================================

虚拟实体图（VEG）系统是UAPI的核心建模组件，通过层次化的实体结构来表示具身应用的认知世界。该系统提供了完整的实体生命周期管理、关系建模和技能绑定机制。

**层次化建模**
  实体图采用树状层次结构组织世界中的所有对象。根节点通常是一个房间（Room）实体，代表整个环境空间，其他实体作为子节点按照空间或逻辑关系进行组织。这种设计直观地反映了现实世界的组织结构。

**实体类型系统**
  系统定义了多种实体类型以适应不同的建模需求：

  - ``GENERIC`` - 通用实体，适用于一般对象
  - ``CONTROLLABLE`` - 可控实体，通常代表机器人或可操作设备
  - ``COMPUTING`` - 计算实体，代表具有计算能力的组件
  - ``SYSTEM`` - 系统实体，代表系统级组件
  - ``HUMAN`` - 人类实体，代表人类参与者
  - ``ROOM`` - 房间实体，代表空间容器

**关系建模机制**
  实体间通过关系类型进行连接，目前主要支持父子关系（``PARENT_OF``/``CHILD_OF``），未来可扩展支持更多关系类型如邻接关系、包含关系等。

核心组件
-------

Entity
~~~~~~~~~

Entity类是实体图的基础构建块，每个实体包含以下核心属性：

**基础属性**
  - ``entity_id`` - 唯一标识符，使用UUID生成
  - ``entity_type`` - 实体类型，决定实体的基本行为特征
  - ``entity_name`` - 实体名称，用于路径寻址和用户识别
  - ``metadata`` - 元数据，存储描述信息和标签

**关系管理**
  每个实体维护一个关系字典，记录与其他实体的连接关系。关系是双向的，添加子实体时会自动建立相互的父子关系。

**技能绑定**
  实体通过 ``skills`` 列表和 ``skill_bindings`` 字典管理绑定的技能。技能绑定时会进行规范验证，确保技能的正确性。

EntityPath
~~~~~~~~~~~

实体图提供了基于路径的寻址机制，类似文件系统的路径结构：

.. code-block:: python

   # Path examples
   root = create_root_room()           # Path: /
   robot = create_controllable_entity("robot")  # Path: /robot
   camera = create_controllable_entity("camera") # Path: /robot/camera
   
   root.add_child(robot)
   robot.add_child(camera)

路径寻址支持相对路径和绝对路径查找，通过 ``get_entity_by_path()`` 方法可以快速定位任意实体。

Skill Binding 机制
-----------

**绑定验证**
  实体绑定技能时，系统会根据技能规范进行严格的验证：

  - 检查技能是否在标准规范中定义
  - 验证绑定函数的签名是否符合要求
  - 确保依赖的其他技能已正确绑定

**动态调用**
  通过Python的 ``__getattr__`` 方法，实体支持动态技能调用。当访问实体的技能属性时，系统会自动创建调用包装器，处理参数验证、类型转换和结果检查。

**参数类型处理**
  系统支持复杂的参数类型验证和自动转换：

  - 基础类型（int, float, str, bool）的自动转换
  - 复合类型（dict, list, tuple）的递归验证
  - 数据类和枚举类型的严格检查
  - 多选类型的灵活匹配

Entity Factory Functions
-----------

为了简化实体创建过程，系统提供了一系列工厂函数：

.. code-block:: python

   # Create different types of entities
   generic_entity = create_generic_entity("object1")
   controllable_entity = create_controllable_entity("robot1")
   computing_entity = create_computing_entity("computer1")
   human_entity = create_human_entity("user1")
   room_entity = create_room_entity("living_room", room_type="residential")
   root_room = create_root_room()

这些工厂函数自动处理ID生成、类型设置等细节，让用户专注于业务逻辑的实现。

Example
-------

以下是一个典型的实体图构建示例：

.. code-block:: python

   from uapi.graph.entity import create_root_room, create_controllable_entity
   
   # Create root room
   root_room = create_root_room()
   
   # Create robot entity
   robot = create_controllable_entity("robot")
   root_room.add_child(robot)
   
   # Create camera entity
   camera = create_controllable_entity("camera")
   robot.add_child(camera)
   
   # Bind skill (Skill Binding)
   def get_pose_impl():
       return {"x": 0.0, "y": 0.0, "z": 0.0}
   
   robot.bind_skill("cap_space_getpos", get_pose_impl)
   
   # Use skill (Skill Usage)
   position = robot.cap_space_getpos()
   print(f"Robot position: {position}")