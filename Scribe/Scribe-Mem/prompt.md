参考 @services/memsearch/README.md   @Scribe/Scribe-Mem/十五、Scribe.md  @Scribe/Scribe-Mem/Robonix-ScribeMem.md  生成 @Scribe/Scribe-Mem/Scribe-Mem-struct.md 的框架结构 , 不要生成具体的代码，只基于已有的 Memsearch 结构，结合 Scribe 的设计，生成 Scribe Mem 的数据结构框架。
目标是从 Scribe Log 的日志记录出发，设计 Scribe Mem 的记忆节点结构，包含必要的字段和说明，形成一个清晰的框架结构文档。

----

基于 @Scribe/Scribe-Mem/十五、Scribe.md @Scribe/Scribe-Mem/Scribe-Mem-struct.md 中的 Mem 数据结构设计，结合 @services/memsearch/README.md 中的 MemSearch 结构，基于 Demo 1 2 生成 @Scribe/Scribe-Mem/TODO.md, 不要实现具体的代码
详细说明其需要实现的功能 api 与 需要修改的文件，并给出这个实现TODO进行排序，以实现核心功能为主，首先实现基础模块，被依赖的模块，扩展功能为辅的原则进行排序

其中TODO.md中内容的格式为

- TODO {功能名称} - {依赖项}
    {功能说明}
    需要修改的文件：
    需要实现的功能api：
    需要生成的测试：

----

基于 @Scribe/Scribe-Mem/TODO.md 中的功能需求，结合 @Scribe/Scribe-Mem/十五、Scribe.md @Scribe/Scribe-Mem/Robonix-ScribeMem.md 中的设计理念，生成相应代码和测试样例，完成 TODO 中每一个 Phase 后，进行测试，测试通过后，将 TODO.md 中的内容简化为已完成的功能列表，并更新完成的工作到 @Scribe/Scribe-Mem/step.md 中，当遇到了报错的情况，记录错误信息和解决方案到 step.md 中

----

实现加载到 robonix 中并提供相应的技能
包括 检索-查找历史信息 的技能
实现相应 记忆构造的 函数，实现记忆的存储与加载，尝试构造一批数据 （参考但不完全使用并不一定要解析现在的非规范化的日志 - 可以使用函数构造一批数据）
可以使用手工构造的一批记忆 以 .yaml 形式本地存储，然后加载到 robonix 中，基于此记忆，进行历史信息的查找

实现以下功能并集成到 Robonix 中，
1.提供检索技能：实现历史信息查找技能，支持基于记忆数据的检索查询。
2.实现记忆构造的函数：实现记忆构建函数，支持记忆的存储与加载。构造一批测试数据（可参考现有非规范化日志格式，但不强制解析；也可直接通过函数生成结构化数据）。
3.数据准备：手工构造一批记忆数据，以 YAML 格式本地存储，随后加载到 Robonix 中。
4.功能验证：基于已加载的记忆数据，执行历史信息查找，验证检索功能的正确性与可用性。

---

Scribe Mem - 参考
Scribe Mem - 设计实现
Scribe Mem - 数据结构框架设计