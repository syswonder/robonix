Cap-Skill-Action 系统
========

技能系统是UAPI的行为建模核心，通过标准化的技能规范和类型安全的绑定机制，为具身应用提供了丰富的行为能力。该系统将复杂的行为抽象为可复用的技能组件，支持组合式的能力构建。

设计原理
-------

**能力与技能分层**
  系统将具身应用的行为能力分为两个层次：

  - **能力（Capability）** - 原子级的基础操作，如获取位置、移动、拍照等。能力通常对应硬件接口或底层服务，是不可再分解的最小行为单元。
  - **技能（Skill）** - 由多个能力组合而成的复合行为，如导航到目标、检测物体等。技能封装了业务逻辑，可以调用其他技能和能力。

**声明式规范定义**
  所有技能都通过声明式规范进行定义，包含描述、类型、输入输出规格和依赖关系。这种方式确保了技能接口的一致性和可验证性。

**类型安全保障**
  系统实现了完整的类型检查机制，支持复杂类型的验证和自动转换，包括基础类型、复合类型、数据类和枚举等。

技能规范系统
-----------

EOS_SKILL_SPECS
~~~~~~~~~~~~~~~

关于技能规范的更多信息，请参见 :doc:`../api/skill_specs_table` 章节。

技能规范是一个全局字典，定义了所有标准技能的接口规格：

.. code-block:: python

   EOS_SKILL_SPECS = {
       "cap_space_getpos": {
           "description": "Get the position of the entity",
           "type": EOS_SkillType.CAPABILITY,
           "input": None,
           "output": {"x": float, "y": float, "z": float},
       },
       "skl_detect_objs": {
           "description": "Detect objects in the current view",
           "type": EOS_SkillType.SKILL,
           "input": {"camera_name": str},
           "output": Dict[str, Tuple[float, float, float]],
           "dependencies": ["cap_camera_dep_rgb", "cap_camera_info"],
       }
   }

**规范字段说明**
  - ``description`` - 技能的功能描述
  - ``type`` - 技能类型（CAPABILITY 或 SKILL）
  - ``input`` - 输入参数规格，支持None、字典或列表（多选类型）
  - ``output`` - 输出结果规格
  - ``dependencies`` - 依赖的其他技能列表（仅技能类型需要）

类型系统
-------

**基础类型支持**
  系统支持Python的所有基础类型，包括int、float、str、bool等，并提供自动类型转换功能。

**复合类型处理**
  支持复杂的数据结构：

  - **字典类型** - 定义结构化数据的字段和类型
  - **列表类型** - 支持同质元素的集合
  - **元组类型** - 支持异构元素的有序组合
  - **数据类** - 支持用户定义的数据结构
  - **枚举类型** - 支持有限选项的类型安全

**多选类型机制**
  通过列表定义多种可接受的输入格式：

.. code-block:: python

   "input": [
       None,  # 无参数调用
       {"timeout_sec": float}  # 带超时参数调用
   ]

技能绑定机制
-----------

**绑定验证流程**
  实体绑定技能时经过严格的验证：

  1. 检查技能是否在标准规范中定义
  2. 验证绑定函数的存在性和可调用性
  3. 将技能添加到实体的技能列表中
  4. 建立技能名称到函数的映射关系

**动态调用包装**
  通过 ``__getattr__`` 方法实现动态调用：

.. code-block:: python

   def __getattr__(self, name):
       if name in self.skill_bindings:
           def wrapper(**kwargs):
               # 参数验证
               self._check_skill_args(name, kwargs)
               # 函数调用
               result = self.skill_bindings[name](**kwargs)
               # 结果验证
               self._check_skill_returns(name, result)
               return result
           return wrapper

**自实体注入机制**
  对于需要访问实体上下文的技能，系统支持自动注入 ``self_entity`` 参数，使技能函数能够访问调用实体的状态和其他技能。

使用示例
-------

以下展示了技能系统的典型使用方式：

.. code-block:: python

   from uapi.graph.entity import create_controllable_entity
   
   # Create entity
   robot = create_controllable_entity("robot")
   
   # Define skill implementations
   def get_position():
       # Actual position retrieval logic
       return {"x": 1.0, "y": 2.0, "z": 0.0}
   
   def move_to_position(x, y, z):
       # Actual movement logic
       print(f"Moving to ({x}, {y}, {z})")
       return {"success": True}
   
   # Bind skills
   robot.bind_skill("cap_space_getpos", get_position)
   robot.bind_skill("cap_space_move", move_to_position)
   
   # Use skills
   current_pos = robot.cap_space_getpos()
   print(f"Current position: {current_pos}")
   
   result = robot.cap_space_move(x=5.0, y=3.0, z=0.0)
   print(f"Move result: {result}")