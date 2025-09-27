Runtime
==========

运行时系统是UAPI的执行引擎，负责管理实体图的生命周期、动作程序的加载执行以及技能提供者的注册管理。该系统采用单例模式设计，提供了完整的运行时环境和管理接口。

数据结构
-------

Runtime 类（单例）
~~~~~~~~~~~~~~~~~~

Runtime类是运行时系统的核心，采用单例模式确保全局唯一性：

**实体图管理**
  Runtime维护着全局的实体图引用，提供图的设置、获取和钩子管理功能。钩子机制允许在图初始化完成后自动执行自定义逻辑，为系统扩展提供了便利。

**动作程序管理**
  支持多个动作程序的并存和切换。每个程序都有独立的模块命名空间，包含程序路径、动作函数列表和加载时间等元信息。这种设计支持大型项目的模块化开发。

**并发执行控制**
  内置多线程支持，每个动作在独立线程中执行。系统维护线程池和结果集合，提供完整的并发控制和状态监控能力。

**实体构建器注册**
  通过构建器模式支持不同的实体图构建策略。用户可以注册多个构建器函数，根据不同场景选择合适的构建方式。

**统一管理接口**
  提供了程序加载、动作配置、场景导出等统一的管理接口，简化了复杂系统的操作流程。

Skill Registry System
-----------

Registry 和 SkillProvider
~~~~~~~~~~~~~~~~~~~~~~~~

技能注册系统采用提供者模式设计：

**SkillProvider**
  技能提供者封装了一组相关的技能实现，包含提供者名称、网络地址和技能列表。这种设计支持分布式的技能部署和管理。

**Registry**
  注册表管理所有的技能提供者，支持提供者的添加、查找和管理。通过注册表，系统可以动态发现和使用可用的技能。

Action Program Loading
-----------

系统使用Python的动态模块机制加载动作程序。每个程序文件被解析为独立的模块对象，拥有自己的命名空间和执行环境。通过反射机制扫描模块中的函数，识别带有 ``_is_action`` 属性的动作函数。这种设计确保了只有正确标记的函数才会被识别为动作。系统支持在运行时切换不同的动作程序，无需重启即可加载新的业务逻辑。这为开发和调试提供了极大的便利。

Hook Extension
-----------

  在实体图设置完成后自动执行的扩展点。钩子函数接收Runtime实例作为参数，可以对图进行进一步的配置和初始化。支持钩子的动态添加和移除。如果图已经初始化，新添加的钩子会立即执行，保证了系统状态的一致性。

State Export
-----------

  系统可以将当前的实体图结构导出为JSON格式，包含实体层次、绑定技能、关系信息等完整的图状态。支持将技能规范信息导出，便于文档生成和系统集成。
  提供完整的运行时状态导出，包括图信息、程序状态、动作参数等，支持系统的调试和监控。

Example
-------

以下展示了运行时系统的典型使用方式：

.. code-block:: python

   from robonix.uapi import get_runtime, set_runtime
   
   # Get globally unique runtime instance
   runtime = get_runtime()
   
   # Register entity builder
   def my_builder(runtime, **kwargs):
       from robonix.uapi.graph.entity import create_root_room, create_controllable_entity
       root = create_root_room()
       robot = create_controllable_entity("robot")
       root.add_child(robot)
       runtime.set_graph(root)
   
   runtime.register_entity_builder("my_scene", my_builder)
   
   # Build entity graph
   runtime.build_entity_graph("my_scene")
   
   # Set global runtime
   set_runtime(runtime)
   
   # Load action program
   action_names = runtime.load_action_program("my_actions.action")
   
   # Configure and execute action
   runtime.configure_action("my_action", param1="value1")
   runtime.execute_action("my_action")

