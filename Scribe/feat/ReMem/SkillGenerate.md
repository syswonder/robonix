
# 描述

## 概述

Skill生成模块（ScriptKnowledgeGraph, SKG）是记忆系统的中间层，负责技能的全生命周期管理：从经验中提取技能、对齐已有知识、驱动技能成熟度演化，并向上层LLM规划器提供可检索的技能库。

**核心流程**: 上层LLM根据任务目标进行规划 → SKG检索匹配的技能/事件链 → runtime调用底层执行节点的API执行动作 → 执行结果反馈回SKG更新技能状态（成功提升/失败降权）。

## 技能生命周期

技能在知识图谱中经历从候选到固定的演化过程：

```
经验输入 → Candidate Skill → Short Term Skill → Long Term Skill → (人工提升) → Fixed Skill
              (失败经验)        (成功但未成熟)     (成熟技能)                     (永久技能)
```

### 技能分级

| 级别 | 名称 | 对应记忆类型 | 描述 | 遗忘/衰减 |
|------|------|-------------|------|-----------|
| **FIXED Skill** | 固定技能 | 固定记忆 | 初始化时写入的基础模板，由人工标记提升，永不被遗忘 | 不参与 |
| **Long Term Skill** | 长期技能 | 长期记忆 | 多次成功验证且经过提纯的成熟技能，由短期记忆自动提升（hit_count ≥ 阈值） | 不被遗忘，持续提纯更新 |
| **Short Term Skill** | 短期技能 | 短期记忆 | 有成功经历但尚未成熟的技能，等待更多成功验证以提升 | 可被GC遗忘 |
| **Candidate Skill** | 候选技能 | 候选记忆 | 有失败经历的技能，权重被降低（weight *= decay_factor），检索排名靠后 | 权重持续衰减，可被GC回收 |

### 提升与衰减机制

- **自动提升**: 每次检索命中时 `hit_count++`，当 `hit_count ≥ promote_threshold`（默认3）时，SHORT_TERM → LONG_TERM
- **失败降权**: `feedback_failure()` 将事件链中所有节点的权重乘以衰减因子（默认0.5），下限0.1
- **提纯**: 同一技能的多条成功事件链，通过结构对齐（LCS匹配）提取共同核心步骤，合并为更纯粹的技能节点
- **遗忘**: 独立GC线程定期清理超时的SHORT_TERM节点（LONG_TERM/FIXED/SKG_EVENT不受影响）

## 对齐算法

新经验插入时，SKG通过两阶段对齐判断是否与已有技能匹配：

1. **语义粗筛**: Retriever向量编码 → 余弦相似度 ≥ 0.7 → 筛选候选
2. **结构精匹配**: 新旧动作序列 → LCS比率 ≥ 0.6 → 确认匹配
   - 匹配成功 → 合并到已有事件链（`_merge_event_chain`），增加根节点权重
   - 匹配失败 → 创建新事件链（`_create_event_chain`），作为Short Term Skill

## 三层架构

```
┌──────────────────────────────────────────────────┐
│         逻辑调用层 (LLM规划器)                      │
│  任务规划 → 调用 SKG 的 5 个公共接口                 │
│  接收 task_spec 用于指导执行                        │
└──────────────────────┬───────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────┐
│       SKG 中间层 (ScriptKnowledgeGraph)            │
│                                                    │
│  load()              - 初始化场景 + 加载已有技能     │
│  save()              - 持久化（MemGraph + skg.json） │
│  retrieve(query)     - 向量检索 + 权重排序 + 链追踪  │
│  insert_experience() - 成功经验插入（含对齐/合并）    │
│  update_short_term() - 短期空间记忆增量更新          │
│  feedback_failure()  - 失败经验降权                  │
│                                                    │
│  内部: Retriever(向量检索) + 对齐算法(语义+结构)     │
│  内部: 自动提升(SHORT→LONG) + 失败衰减 + GC遗忘     │
└──────────────────────┬───────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────┐
│       MemoryGraph 底层 (networkx DiGraph)          │
│  节点CRUD + 父子/标签边 + 链查询 + 多模态数据       │
│  序列化/反序列化 + 可视化                           │
│  底层保存执行模块的调用API（不对上层暴露）            │
│  仅在事件链被LLM规划命中时由runtime调用              │
└──────────────────────────────────────────────────┘
```

## MemoryGraph 节点结构

底层按需存储，以指针形式驻留内存（如空间物体仅保留依赖关系描述，功能调用仅保留API引用）。

| 字段 | 说明 |
|------|------|
| **TYPE** | NodeType × NodeClass = 4×3 种组合（SHORT_TERM/LONG_TERM/FIXED/SKG_EVENT × CONTEXT/TIME/SPACE） |
| **NAME/SUMMARY** | 节点名称与摘要描述 |
| **CHILDS/FATHER** | 父子层级关系（child_cnt / parents_cnt / parent_id） |
| **EDGES** | 标签边（NEXT因果链 / REQUIRES依赖 / PRODUCES产出） |
| **MODALITIES** | 多模态数据容器（TEXT/IMAGE/DEPTH/SPATIAL/INVENTORY/EVENT） |
| **weight** | 检索权重（score = cosine_sim × weight），受成功/失败反馈调节 |
| **hit_count** | 检索命中次数，达阈值触发提升 |
| **embedding** | 语义向量缓存（用于Retriever检索，不序列化） |
| **API** | （仅底层执行节点）调用执行模块执行动作/技能的接口，不对上层暴露 |

## 公共接口

| 接口 | 功能 | 说明 |
|------|------|------|
| `load(scene_info, skg_path)` | 初始化/加载 | 从场景信息构建空间节点，从skg.json加载事件链 |
| `save()` | 持久化 | 保存MemoryGraph + 导出skg.json |
| `retrieve(query, top_k)` | 检索 | 向量检索 → 权重排序 → 链追踪 → 自动提升；返回匹配的事件链列表 |
| `insert_experience(task, actions)` | 经验插入 | 两阶段对齐 → 合并或新建事件链；返回task_spec |
| `update_short_term(obj_list, entity)` | 空间更新 | 增量更新短期空间节点（新增/删除差异部分） |
| `feedback_failure(chain_ids)` | 失败反馈 | 降低事件链权重（weight *= decay_factor） |

## 内部方法

| 方法 | 功能 |
|------|------|
| `_align_and_merge()` | 对齐：语义粗筛 + 结构精匹配，返回匹配链根ID |
| `_semantic_match()` | 语义粗筛：Retriever编码 + 余弦相似度过滤 |
| `_structural_match()` | 结构精匹配：动作序列LCS比率计算 |
| `_create_event_chain()` | 创建新事件链（任务根 + 动作节点 + NEXT边） |
| `_merge_event_chain()` | 合并到已有事件链（增权重 + 追加新步骤） |
| `_try_promote()` | 检查hit_count并触发SHORT_TERM → LONG_TERM提升 |
| `_sync_to_skg_json()` | 导出SKG_EVENT节点为skg.json格式 |
| `_load_skg_json()` | 从skg.json加载事件链到MemoryGraph |


# 架构


# 接口细节


