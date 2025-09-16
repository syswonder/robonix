UAPI 用户编程接口
=================

UAPI（User Application Programming Interface）是Robonix OS 的核心用户编程模块，提供了一套完整的具身应用开发接口。该模块实现了分层架构设计，通过实体图建模、运行时管理、技能系统和动作框架等组件。

UAPI模块的设计遵循以下核心原则：

1. 系统以实体（Entity）为核心构建具身应用的认知模型。每个实体代表环境中的一个对象或组件，通过层次化的树状结构组织，形成完整的世界状态表示。实体不仅承载状态信息，还绑定相应的技能和能力。

2. UAPI将具身应用的行为能力抽象为技能（Skill）和能力（Capability）。能力代表基础的原子操作，技能则是由多个能力组合而成的复合行为。

3. 通过 Python 装饰器提供声明式的动作编程接口。开发者只需使用 ``@action`` 装饰器标记函数，即可将其转换为可执行的动作程序，被 UAPI 加载器加载并管理。

.. raw:: html

   <div style="margin-top: 3em;"></div>

UAPI 子模块
==============

.. toctree::
   :maxdepth: 2

   graph_system
   runtime_system
   skill_system
   action_system