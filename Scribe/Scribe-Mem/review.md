
enkerewpo reviewed yesterday
capabilities/service/memgraph/compact.v1.toml
@@ -0,0 +1,8 @@
[contract]
id      = "robonix/service/memgraph/compact"
@enkerewpo
enkerewpo
yesterday
• 
Member
I suggest that we put everything (including the mem graph and the legacy memory) under robonix/service/memory namespace instead of separating them



enkerewpo reviewed yesterday
examples/webots/robonix_manifest.yaml
  # Memgraph (Scribe Mem) — structured CKG memory (BM25 + Embedding).
  # Contracts: robonix/service/memgraph/{remember,search,compact}
  # Embedding: all-MiniLM-L6-v2 (d=384), BM25 + Cosine hybrid retrieval.
  - name: memgraph
@enkerewpo
enkerewpo
yesterday
Member
please use the name memory like before



enkerewpo reviewed 2 hours ago
services/memory/memory_service/storage/graph_store.py
        self._children[new_id] = set()
        self._parents[new_id] = set()

        for pid in old_parents:
@enkerewpo
enkerewpo
2 hours ago
• 
Member
MemoryNode.causal_chain is the serialized copy of parent node ids (see MemoryNode.causal_chain in core/types.py), and add_edge() explicitly appends parent_id into that field on the child node.

The problem here is that promote_to_long_term() remaps the in-memory adjacency sets from old_id to new_id, but it does not rewrite the corresponding causal_chain entries stored inside affected child MemoryNodes.

So after compaction, these two representations can diverge:

graph structure says new_id -> child
serialized child node still says causal_chain=[old_id]
That leaves stale parent ids in persisted node data after compaction. Please rewrite affected child causal_chain values when remapping ids, and add a test that verifies both adjacency and serialized causal_chain stay consistent across promotion.

> **Reply (ohhhHwH):**
> Agreed — this is a genuine bug. `promote_to_long_term()` should iterate over
> all child nodes whose `causal_chain` references the old ID and rewrite those
> entries to the new ID.  Causal-chain integrity is fundamental to memgraph's
> value proposition: if a Skill is later refined from a causal chain, stale
> parent IDs would silently break the step-dependency template.  Added to the
> TODO list and will be fixed in the next iteration.
>
> ---
> 同意，这是一个确实的 bug。`promote_to_long_term()` 应该遍历所有 `causal_chain`
> 中引用旧 ID 的子节点，将其改写为新 ID。因果链的完整性是 memgraph 的核心价值：
> 如果后续从因果链提炼 Skill，过期的父节点 ID 会悄无声息地破坏步骤依赖模板。
> 已加入 TODO 列表，下个迭代修复。

@ohhhHwH	Reply...
@enkerewpo
enkerewpo
commented
2 hours ago
• 
Member
Thanks for your contribution. This is a pretty interesting direction. I do think a richer structured memory backend is worth doing, especially if we want memory to hold more than simple text notes.

I still have a few design questions though, and I think it would help to make them explicit in this PR.

Right now my understanding is:

memsearch is still the current simple memory implementation, and pilot still depends on it
this new graph-based memory is meant to provide richer structured memory
for now the two may coexist
later we may want to converge them under one memory abstraction
If that is the plan, I think the PR should say it more clearly.

At the moment I still cannot quite tell:

whether this is supposed to replace memsearch now, or just live alongside it
what exactly should still go to memsearch
what should go to this new graph memory
who is expected to write into it
whether the two stores are intentionally separate for now, or supposed to stay in sync somehow
If the intended model is basically “memsearch stays as the simple memo-style memory for now, and this graph backend adds a richer structured path”, then I think stating that directly would make the design much easier to understand and review.

> **Reply (ohhhHwH):**
> Yes — that is exactly the intended model, and thanks for calling it out.
> Let me explain the design rationale more explicitly.
>
> **memsearch** is a *declarative* note-pad: Pilot explicitly calls
> `save(“remember this fact”)` when the LLM decides something is worth
> keeping.  The stored content is free-form text — a user preference, a
> one-off observation, a reminder.  It stays as the default lightweight
> path for memo-style persistence.
>
> **memgraph** is an *observational* knowledge graph: it is **not** driven
> by Pilot's explicit save calls.  Instead, the robot's own execution
> pipeline — Executor (action results), Scene (perceived objects and their
> 3D positions), and downstream Scribe Log feeds — writes structured
> `MemoryNode`s that record **what the robot did, what it perceived, where
> it was, and how those events are causally linked**.  Each node carries:
>
> - **spatial tags** — scene type, object IDs + 3D coordinates
> - **behavioural tags** — action type (grasp / navigate / craft), success/failure, tool used
> - **cognitive tags** — task type, difficulty, intent
> - **causal edges** — “I navigated to the kitchen → I grasped the cup”
> - **embedding** — dense vector for hybrid BM25+semantic retrieval
>
> In short:
> | | memsearch | memgraph |
> |---|---|---|
> | **What is stored** | free-text facts & preferences | structured experience records |
> | **Who writes** | Pilot (LLM decides) | Executor + Scene + Scribe (automatic) |
> | **What is retrievable** | “what did the user say?” | “where was the cup? how did I grasp it? what step came before?” |
> | **Query dimensions** | text only | spatial + behavioural + cognitive + temporal + semantic |
>
> They are **intentionally separate** and **do not need to stay in sync** —
> they serve different layers of memory.  memsearch answers *”what did the
> user tell me?”*; memgraph answers *”what happened, where, why, and what
> did I learn from it?”*.  The latter is the foundation for skill refinement
> (N successful experiences → extract a reusable Skill template) and
> failure-lesson protection (failed actions are guarded from forgetting).
>
> This is in the PR description now.
>
> ---
> **中文回复 (ohhhHwH):**
> 是的，这正是预期的模型，感谢提出。
>
> **memsearch** 是一个*声明式*记事本：当 LLM 判定某事值得保存时，Pilot 显式调用
> `save(“记住这个事实”)`。存储内容是自由文本 — 用户偏好、一次性观察、提醒事项。
> 它作为备忘录风格的轻量路径保持不变。
>
> **memgraph** 是一个*观察式*知识图谱：它**不是**由 Pilot 显式保存调用驱动的。
> 而是，机器人自身的执行管线 — Executor（动作结果）、Scene（感知到的物体及其
> 3D 坐标）、以及下游 Scribe Log 馈送 — 写入结构化的 `MemoryNode`，记录
> **机器人做了什么、感知到了什么、在哪里、以及这些事件之间的因果关联**。每个节点包含：
>
> - **空间标签** — 场景类型、物体 ID + 3D 坐标
> - **行为标签** — 动作类型（抓取/导航/制作）、成功/失败、使用的工具
> - **认知标签** — 任务类型、难度、意图
> - **因果边** — “我导航到厨房 → 我抓取了杯子”
> - **嵌入向量** — 稠密向量用于 BM25+语义混合检索
>
> 简而言之：
> | | memsearch | memgraph |
> |---|---|---|
> | **存储什么** | 自由文本事实和偏好 | 结构化的经验记录 |
> | **谁写入** | Pilot（LLM 决策） | Executor + Scene + Scribe（自动） |
> | **可检索什么** | “用户说过什么？” | “杯子在哪？我是怎么抓取的？前一步是什么？” |
> | **查询维度** | 仅文本 | 空间 + 行为 + 认知 + 时间 + 语义 |
>
> 两者**有意分开**且**不需要保持同步** — 它们服务于不同的记忆层次。memsearch
> 回答*”用户告诉过我什么？”*；memgraph 回答*”发生了什么、在哪、为什么、
> 我从中学到了什么？”*。后者是技能提炼（N 次成功经验 → 提取可复用 Skill 模板）
> 和失败教训保护（失败动作免于遗忘）的基础。
>
> 此说明已加入 PR 描述。
>
> ---
> **补充说明 (ohhhHwH):**
> 诚然，在 **Phase 1 的当前实现**中，两者确实存在一些功能重叠：
> memsearch 的 `search` 和 memgraph 的 `hybrid_search` 表面上看都是
> "输入查询文本 → 返回相关记忆"，都注册了 MCP tool 供 Pilot/LLM 调用。
> 这是初期共存的妥协 — 在 memgraph 的因果过滤、可执行性校验、技能提炼
> 等高级特性尚未完成之前，两者都需要提供一个可用的检索入口。
>
> 但随着后续 Phase 的推进，两者将走向**截然不同的演化路径**：
>
> | Phase | memgraph 独有能力 | memsearch 无法做到 |
> |-------|------------------|-------------------|
> | Phase 2 `refine` | 从 N≥3 个同类成功经验中提炼 `SkillTemplate`，注册为 LLM function tool | — |
> | Phase 2 `forget` | `ForgetRisk` 分级遗忘（降权→归档→删除），保护教训/技能源节点 | 仅能手动删除文件 |
> | Phase 3 `search_history` | 时空三维回溯："上周在厨房抓取杯子的那次" | 仅文本相似度 |
> | Phase 3 `cron_trigger` | 定时任务记忆："明天提醒我去取快递" | — |
> | Phase 4 因果推理 | `causal_chain` 可执行性校验："当前机器人只有单臂，那个需要双臂的经验不可用" | — |
> | Phase 4 并行分析 | `CausalRelation::ParallelTo` — 识别可并行的子任务，释放双臂并行度 | — |
>
> 所以初期的功能重叠是**有意为之的阶段性妥协**，而非设计的模糊。
> memsearch 将保持其轻量事实存储的定位不变；memgraph 则向"经验 → 技能"
> 的完整生命周期管理演进。
>
> ---
> **Supplemental note (ohhhHwH):**
> Admittedly, in the **current Phase 1 implementation** there is some
> functional overlap: both memsearch's `search` and memgraph's
> `hybrid_search` appear to be "input query text → return relevant
> memories", both registered as MCP tools for Pilot/LLM consumption.
> This is a deliberate short-term compromise — while memgraph's causal
> filtering, executability checking, and skill refinement are still
> under development, both backends need a working retrieval entry point.
>
> As later phases land, the two will follow **fundamentally different
> evolution paths**:
>
> | Phase | memgraph-unique capability | beyond memsearch |
> |-------|---------------------------|-----------------|
> | Phase 2 `refine` | Extract `SkillTemplate` from N≥3 successful experiences, register as LLM function tool | — |
> | Phase 2 `forget` | `ForgetRisk` tiered forgetting (downgrade→archive→delete), protecting lesson/skill-source nodes | manual file deletion only |
> | Phase 3 `search_history` | Spatiotemporal 3D backtracking: "the time I grasped a cup in the kitchen last week" | text similarity only |
> | Phase 3 `cron_trigger` | Scheduled memory triggers: "remind me to pick up the package tomorrow" | — |
> | Phase 4 causal reasoning | `causal_chain` executability check: "my robot has only one arm — that dual-arm experience is not usable" | — |
> | Phase 4 parallelism | `CausalRelation::ParallelTo` — identify parallelizable subtasks, unlock dual-arm concurrency | — |
>
> So the initial overlap is an **intentional phased compromise**, not a
> design ambiguity.  memsearch stays as lightweight fact storage; memgraph
> grows toward the full "experience → skill" lifecycle.


