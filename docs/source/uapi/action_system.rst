Action 编程模型
========

Action 是使用 Python 语法的用户编程语言，只需使用 ``@action`` 装饰器标记函数，即可将函数转换为可管理的 Action。

  每个 Action 程序在独立的线程中执行，避免阻塞主程序流程。系统提供完整的并发控制机制，支持 Action 的并行执行和结果同步。
  系统内置了完整的异常捕获和处理机制，包括彩色日志输出、堆栈跟踪和错误状态管理，确保系统的健壮性。
  通过 ``EOS_TYPE_ActionResult`` 枚举定义标准的Action执行结果，包括成功（SUCCESS）、失败（FAILURE）和中止（ABORT）三种状态。

Action 装饰器机制
-------------

@action 装饰器
~~~~~~~~~~~~~

``@action`` 装饰器是 Action 系统的核心组件，负责将普通函数转换为 Action 函数：

.. code-block:: python

   @action
   def my_action(param1: str, param2: int) -> EOS_TYPE_ActionResult:
       # Implement business logic
       action_print(f"Executing with {param1} and {param2}")
       return EOS_TYPE_ActionResult.SUCCESS

装饰器会在函数上添加 ``_is_action`` 属性，用于运行时识别Action函数。同时保持原函数的名称和其他属性。

执行模型
-----------

运行时系统为每个Action创建独立的守护线程：

.. code-block:: python

   def action_worker():
       try:
           result = action_func(*args, **kwargs)
           self.action_results[action_name] = result
       except Exception as e:
           self.action_results[action_name] = None
           logger.error(f"action {action_name} failed: {str(e)}")

系统维护全局的结果字典，支持多种结果查询方式：

- ``wait_for_action()`` - 等待单个Action完成
- ``wait_for_all_actions()`` - 等待所有Action完成
- ``get_action_status()`` - 查询Action执行状态

线程被标记为守护线程，确保主程序退出时不会被阻塞。系统提供完整的线程状态监控和清理机制。

Action 日志
-----------

action_print 函数
~~~~~~~~~~~~~~~~

``action_print`` 是专为Action系统设计的日志函数，提供了丰富的上下文信息：

**自动上下文识别**
  通过堆栈检查自动识别调用的Action函数名称，无需手动指定上下文。

**多级日志支持**
  支持DEBUG、INFO、WARN、ERROR、CRITICAL五个日志级别，每个级别都有对应的颜色编码。

**双重输出机制**
  - 控制台输出：彩色格式化，便于实时监控
  - 文件输出：完整信息记录，便于后续分析，输出到 action 代码同目录的 action 名称 .log 文件中。

**文件自动管理**
  根据Action名称自动创建日志文件，文件位置基于Action程序的路径确定。

Runtime 系统
-------------

**运行时实例管理**
  系统维护全局的运行时实例，通过 ``get_runtime()`` 和 ``set_runtime()`` 函数进行管理：

.. code-block:: python

   # Get current runtime instance
   runtime = get_runtime()
   
   # Set new runtime instance
   set_runtime(new_runtime)

**Action程序加载**
  运行时支持动态加载Action程序文件：

  1. 读取程序文件内容
  2. 创建独立的模块命名空间
  3. 执行代码并识别Action函数
  4. 建立函数名称到实现的映射


Type Safety
-----------

  虽然 Action 函数本身不强制类型检查，但通过实体的技能调用会进行严格的类型验证。Action函数应返回 ``EOS_TYPE_ActionResult`` 枚举值，系统会在异常情况下自动返回FAILURE状态。
  
.. important::
   Action 执行时会设置当前实体上下文，使 cap/skill 的具体实现函数能够访问 ``self_entity`` 参数，以实现对自身绑定的其他 skill 和 cap 的调用。

使用示例
-------

以下展示了Action系统的完整使用流程：

**定义Action程序** (example.action)

.. code-block:: python

   from uapi.runtime.action import action, EOS_TYPE_ActionResult, get_runtime, action_print
   from uapi.specs.types import EntityPath
   
   @action
   def move_and_capture(robot_path: EntityPath) -> EOS_TYPE_ActionResult:
       runtime = get_runtime()
       robot = runtime.get_graph().get_entity_by_path(robot_path)
       
       if robot is None:
           action_print(f"Robot not found at path: {robot_path}", "ERROR")
           return EOS_TYPE_ActionResult.FAILURE
       
       # Get current position
       current_pos = robot.cap_space_getpos()
       action_print(f"Current position: {current_pos}")
       
       # Move to target position
       move_result = robot.cap_space_move(x=5.0, y=3.0, z=0.0)
       if not move_result["success"]:
           action_print(f"Move failed: {move_result}", "ERROR")
           return EOS_TYPE_ActionResult.FAILURE
       
       # Capture image
       robot.cap_save_rgb_image(filename="captured.jpg", camera_name="camera0")
       action_print("Image captured successfully")
       
       return EOS_TYPE_ActionResult.SUCCESS

**加载和执行Action**

.. code-block:: python

   from robonix.uapi import get_runtime, set_runtime
   
   # Get runtime instance
   runtime = get_runtime()
   
   # Build entity graph and set runtime
   runtime.build_entity_graph("my_scene")
   set_runtime(runtime)
   
   # Load action program
   action_names = runtime.load_action_program("example.action")
   print(f"Loaded actions: {action_names}")
   
   # Configure action parameters
   runtime.configure_action("move_and_capture", robot_path="/robot")
   
   # Execute action
   thread = runtime.execute_action("move_and_capture")
   
   # Wait for execution completion
   result = runtime.wait_for_action("move_and_capture")
   print(f"Action result: {result}")