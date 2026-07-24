# Scribe Mem — 数据结构框架

> **定位**：本文档定义 Scribe Mem 的数据结构框架，从 Scribe Log 的日志记录出发，设计记忆节点结构。基于现有 memsearch 的三合约（save / search / compact）扩展为完整的记忆生命周期。
>
> **原则**：
> - 只定义"是什么"（结构 + 字段语义），不定义"怎么做"（算法 / 存储引擎选型）。
> - 兼容 memsearch 现有 IDL（`std_msgs/String`），扩展合约保持同风格。
> - 与 Scribe Log 的 `LogRecord` 建立明确的上下游映射关系。

---

## 1. 从 LogRecord 到 MemoryNode 的映射

Scribe Log 产出 `LogRecord`，Scribe Mem 消费 `LogRecord` 并整合为 `MemoryNode`。两者的关系：

```
LogRecord                      MemoryNode
─────────                      ──────────
ts        ──────────────►      timestamp        (直接映射)
level     ─── 影响 ───►       weight            (Error → 教训节点加权保护)
tag       ─── 拆分 ───►       tags.source       (provider_id → 标签维度)
msg       ─── LLM提取 ►       summary           (LLM 一句话摘要)
          ─── VLM提取 ►       spatial_data      (从视觉描述中提取坐标)
          ─── LLM提取 ►       tags (四维度)      (从内容中分类标注)
```

### 1.1 LogRecord（Scribe Log，定义在 system/scribe/）

| 字段 | 类型 | 说明 |
|---|---|---|
| `ts` | `u64` | 纳秒时间戳，chronos `now()` 统一时钟 |
| `level` | `enum Level` | `Debug \| Info \| Warn \| Error` |
| `tag` | `String` | 来源组件名 / `provider_id`，按来源过滤维度 |
| `msg` | `String` | 日志正文，自由文本 |

### 1.2 扩展 LogRecord（Scribe Mem 写入入口需要的附加上下文）

当 LogRecord 用作 `remember` 的输入时，调用方需附加以下结构化上下文（并非 Log 子系统本身产生，而是组件在调用 `remember` 时额外传入）：

| 字段 | 类型 | 必须 | 说明 |
|---|---|---|---|
| `session_id` | `String` | 是 | 会话标识，关联同一任务的所有记忆 |
| `plan_id` | `String` | 是 | 规划标识，关联同一规划步的所有记忆 |
| `call_id` | `String` | 否 | 单次工具调用标识，用于精确回溯 |
| `kv` | `Map<String, String>` | 否 | 扩展键值对，如 `object_id`, `error_code` 等 |
| `spatial` | `SpatialContext` | 否 | 空间上下文（物体坐标列表），由 Scene 提供 |
| `parent_node_id` | `u32` | 否 | 因果父节点，本次动作的"前一步" |

---

## 2. 核心数据结构

### 2.1 MemoryNode — 记忆节点（CKG 基本单元）

`MemoryNode` 是 Scribe Mem 的核心存储单元。一个节点代表"机器人经历过的一个事件"——一次工具调用、一次观察、一次决策。

```
MemoryNode {
    // ── 标识 ──
    node_id:           u32,          // 全局唯一; 0~999 短期, 1000+ 长期,
                                     //   9000+ 技能, 9999 固定

    // ── 内容 ──
    summary:           String,       // LLM 生成的一句话摘要，检索精排依据
                                     //   示例: "在厨房用右手成功抓取红色杯子"
    raw_log:           LogRecord,    // 原始日志记录（内嵌，不分离存储）

    // ── 时空 ──
    timestamp:         u64,          // chronos 纳秒时间戳
    spatial_data:      SpatialContext, // 物体坐标列表 [{obj_id, x, y, z}]

    // ── 标签（四维度） ──
    tags:              TagSet,       // 标签集，驱动多维度过滤

    // ── 因果 ──
    causal_chain:      Vec<u32>,     // 因果依赖父节点 ID 列表
                                     //   例如: [采集木头, 采集石头] → 制作石斧
                                     //   空列表 = 独立事件

    // ── 检索权重 ──
    weight:            f32,          // 综合质量评分 0.0~1.0
                                     //   公式: 成功率 × 时效性衰减 × 访问频率归一化

    // ── 嵌入 ──
    embedding:         Vec<f32>,     // 多模态融合向量, d=384 (all-MiniLM-L6-v2 基准)
                                     //   当前阶段: 仅对 summary 做文本嵌入
                                     //   多模态扩展: VLM视觉描述 + 行为描述 分模态编码 → 融合层

    // ── 元数据 ──
    node_type:         NodeType,     // 节点类型，决定遗忘策略
    created_at:        u64,          // 创建时间戳
    last_access:       u64,          // 最近一次被检索命中的时间戳
    access_count:      u32,          // 累计被检索命中的次数
    version:           u32,          // 乐观锁版本号，并发更新用
}
```

#### 2.1.1 NodeType — 节点类型

| 值 | 类型 | ID 范围 | 说明 | 遗忘保护 |
|---|---|---|---|---|
| `ShortTerm` | 短期记忆 | 0–999 | 当前会话 / 最近任务的事件；容量有上限 (FIFO) | 无保护，正常遗忘评分 |
| `LongTerm` | 长期记忆 | 1000–8999 | 经短期记忆提炼/冷却后的持久化记忆 | 正常遗忘评分 |
| `Skill` | 技能节点 | 9000–9998 | 从 N 个相似成功经验提炼的技能模板 | **永久保护**（除非成功率连续下降） |
| `Fixed` | 固定记忆 | 9999 保留区 | 安全约束、教训、用户偏好等不可遗忘的信息 | **永久保护**（除非显式删除） |
| `Lesson` | 教训节点 | 归属于 LongTerm 区间 | 失败经验标记，权重被锁定 | **遗忘保护**（ForgetRisk 跳过，仅显式归档） |

#### 2.1.2 节点状态转换

```
LogRecord ──[remember]──► ShortTerm (0~999)
                              │
                              │ 会话结束 / 容量满
                              ▼
                          LongTerm (1000+)  ──[forget评分]──►  降权 / 归档 / 删除
                              │
                              │ N≥3 同类成功经验
                              ▼
                          Skill (9000+)     ──[成功率连续下降]──►  弃用 → 归档
                              │
                              │ 用户/系统显式标记
                              ▼
                          Fixed (9999)
```

---

### 2.2 TagSet — 四维度标签集

标签是检索的**第一入口**，目的不是替代向量检索，而是用 O(1) 倒排索引将候选集从 5000 条压缩到 2-10 条，再交给向量精排。

```
TagSet {
    // ── 空间维度 ──  回答 "在哪里"
    scene_type:        String,          // 场景类型: kitchen / living_room / workshop / outdoor
    objects_present:   Vec<String>,     // 场景中的物体: ["cup", "table", "sink"]
    region:            String,          // 区域标签: north / south / near_window

    // ── 行为维度 ──  回答 "做了什么"
    action_type:       String,          // 动作类型: grasp / place / navigate / craft / observe
    success:           bool,            // 执行是否成功
    tool_used:         Vec<String>,     // 使用的工具/能力 provider_id 列表

    // ── 认知维度 ──  回答 "什么任务"
    task_type:         String,          // 任务类型: fetch / build / explore / dialogue
    difficulty:        String,          // 难度: easy / medium / hard (按工具调用次数分级)
    intent:            String,          // 意图摘要（LLM 从指令中提取的短标签）

    // ── 优化维度 ──  用于遗忘评分, 不参与检索过滤
    frequency:         u32,             // 累计命中次数
    last_access:       u64,             // 最近命中时间戳
    quality_score:     f32,             // 执行质量评分 0.0~1.0
}
```

#### 2.2.1 标签索引结构（与存储引擎无关的逻辑定义）

| 维度 | 字段 | 索引类型 | 查询示例 |
|---|---|---|---|
| 空间 | `scene_type`, `objects_present` | 倒排索引 | `scene_type=kitchen AND objects_present⊇["cup"]` |
| 行为 | `action_type`, `success` | 倒排 + 位图 | `action_type=grasp AND success=true` |
| 认知 | `task_type`, `difficulty` | 倒排索引 | `task_type=craft AND difficulty<=medium` |
| 优化 | `frequency`, `last_access`, `quality_score` | B+ 树（排序扫描） | `ORDER BY quality_score DESC LIMIT 10` |

---

### 2.3 SpatialContext — 空间上下文

从 Scene 的 `list_objects` 获取，存入记忆节点供空间过滤检索。

```
SpatialContext {
    objects:    Vec<ObjectCoord>,   // 物体坐标列表
    origin:     String,             // 坐标系参考: "world" / "robot_base" / "camera"
}

ObjectCoord {
    obj_id:     String,             // 物体 ID: "scene.object.cup_003"
    label:      String,             // 语义标签: "red cup"
    x:          f32,                // 世界坐标 x
    y:          f32,                // 世界坐标 y
    z:          f32,                // 世界坐标 z
}
```

---

### 2.4 CausalEdge — 因果边（CKG 图的边）

`MemoryNode.causal_chain` 的展平形式。用于在检索阶段做因果过滤：检索到的记忆节点所依赖的前置节点也必须在当前环境下可复现。

```
CausalEdge {
    from_node_id:   u32,      // 前置节点（原因）
    to_node_id:     u32,      // 后置节点（结果）
    relation:       CausalRelation,  // 因果关系类型
}

CausalRelation = enum {
    Enables,        // A 完成后 B 才可能开始（采集→制作）
    Triggers,       // A 的结果触发 B（检测到障碍→绕行）
    DependsOn,      // B 的执行依赖 A 的输出（感知→抓取）
    ParallelTo,     // A 与 B 可并行执行（左手拿杯 + 右手拿壶）
}
```

---

### 2.5 SkillTemplate — 技能模板

从 N≥3 个同类成功 `MemoryNode` 中提炼而成的**可执行模式**，注册为 Atlas MCP function tool，LLM 在规划时可直接调用。

```
SkillTemplate {
    // ── 标识 ──
    skill_id:           u32,             // 全局唯一, 9000+ 区间
    skill_name:         String,          // 技能名: "craft_plank", "pour_water"
    version:            u32,             // 版本号, 每次提炼更新递增

    // ── 目标模板 ──
    goal_template:      String,          // 目标描述模板: "制作 {count} 个 {item}"
    goal_params:        Vec<String>,     // 模板参数: ["count", "item"]

    // ── 因果步骤模板 ──
    causal_template:    Vec<StepTemplate>, // 步骤因果链，每步是一个可执行动作

    // ── 条件 ──
    pre_conditions:     Vec<Condition>,  // 前置条件, 全部满足方可执行
    post_conditions:    Vec<Condition>,  // 后置条件, 执行后的世界状态断言

    // ── 约束 ──
    constraints:        Vec<Constraint>, // 执行约束（能力/环境/安全）

    // ── 统计 ──
    success_rate:       f32,             // 基于历史执行成功率 0.0~1.0
    sample_count:       u32,             // 提炼来源的经验数量
    source_nodes:       Vec<u32>,        // 来源 MemoryNode ID 列表（可追溯）

    // ── 元数据 ──
    created_at:         u64,
    last_updated:       u64,
    deprecated:         bool,            // 是否已弃用（成功率连续下降）
}

StepTemplate {
    order:              u32,             // 步骤序号 (1-based)
    capability_id:      String,          // 需要的 capability contract_id
    action_desc:        String,          // 动作描述: "使用右手抓取物体"
    parallel_group:     Option<u32>,     // 并行组号, 同组步骤可并行执行
    depends_on_step:    Vec<u32>,        // 依赖的前置步骤序号
}

Condition {
    type:               ConditionType,   // 条件类型
    predicate:          String,          // 条件断言: "robot.has(item=log, count>=1)"
}

ConditionType = enum {
    Object,             // 物体存在性: "附近有木头"
    Capability,         // 能力可用性: "已注册 grasp capability"
    Environment,        // 环境约束: "在 crafting_table 3m 范围内"
    State,              // 机器人状态: "机械臂空闲"
}

Constraint {
    constraint_type:    String,          // "safety" / "capability" / "environment"
    description:        String,          // 约束描述
    check_expr:         String,          // 可在 Executor 侧求值的布尔表达式
}
```

---

### 2.6 ForgetRisk — 遗忘评分

```
ForgetRisk {
    node_id:            u32,

    // ── 评分因子 ──
    frequency_score:    f32,             // 访问频率衰减 (0~1, 越不访问越高)
    recency_score:      f32,             // 时效性衰减 (0~1, 越旧越高)
    quality_score:      f32,             // 执行质量 (0~1, 质量越低越高)
    redundancy_score:   f32,             // 与同类节点的重复度 (0~1, 越重复越高)

    // ── 综合评分 ──
    total_risk:         f32,             // 加权综合: w1×freq + w2×recency + w3×quality + w4×redundancy

    // ── 分级阈值 ──
    //   total_risk < 0.3  → 保留
    //   0.3 ≤ total_risk < 0.7 → 降权 (weight × 0.5)
    //   total_risk ≥ 0.7  → 归档 (移除索引, 30天后删除)

    // ── 保护标记（满足任一则跳过遗忘） ──
    is_protected:       bool,            // 固定/教训/Skill源 节点
    recent_hit:         bool,            // 近30天被检索命中过
    in_causal_chain:    bool,            // 被其他节点引用为因果父节点
}
```

---

### 2.7 嵌入模型配置 & 检索策略

#### 2.7.1 嵌入模型

当前阶段使用 **all-MiniLM-L6-v2** 进行文本嵌入，向量维度 d=384。模型存放于本地路径，服务启动时加载。

```
EmbeddingModelConfig {
    model_name:         String,        // "all-MiniLM-L6-v2"
    model_path:         String,        // "~/EmbodyMemory/memory/all-MiniLM-L6-v2"
    dim:                u32,           // 384
    batch_size:         u32,           // 编码批大小 (默认 32)
    device:             String,        // "cpu" / "cuda"
    max_seq_length:     u32,           // 最大输入 token 数 (默认 256)
}
```

#### 2.7.2 检索策略：BM25 + Embedding 混合检索

检索的向量精排阶段采用 **BM25（稀疏）+ Embedding（稠密）混合检索**：

```
HybridScore = α × BM25_norm(query, summary) + (1-α) × Cosine(query_embedding, node_embedding)

α: 稀疏权重, 默认 0.3 (偏重语义, 保留关键词匹配兜底)
```

| 阶段 | 方法 | 输入 | 作用 |
|---|---|---|---|
| **稀疏召回** | BM25 | `query` vs `MemoryNode.summary` | 关键词精确匹配，召回语义相似但向量可能遗漏的结果 |
| **稠密召回** | Cosine Similarity | `query_embedding` vs `node_embedding` | 语义相似度排序，召回近义词/改写表达的匹配 |
| **混合融合** | 加权求和 | BM25 归一化分 + Cosine 分 | 综合排序，返回最终 Top-K |

混合检索只在 TagStore 过滤后的候选集上执行，候选集规模 ≤ 100，无需全量暴搜。

#### 2.7.3 多模态 Embedding 接口（Phase1 占位，Phase2+ 实现）

当前阶段仅对 `MemoryNode.summary`（文本）做嵌入。但 `VectorStore` 和 `MemoryNode.embedding` 的结构预留了多模态扩展空间：

```
# 当前（Phase1）：单模态文本嵌入
embedding = text_encoder.encode(node.summary)              # d=384

# 未来（Phase2+）：多模态融合嵌入
embedding = MultiModalEncoder {
    text_branch:    TextEncoder.encode(node.summary),       # d_text=384
    visual_branch:  VisualEncoder.encode(image_descriptions), # d_vis=512
    action_branch:  ActionEncoder.encode(action_sequence),   # d_act=256
    fusion:         Concat + Linear(d_text+d_vis+d_act → d_fused=768)
}
```

**多模态接口预留原则**：

| 层级 | 当前设计 | 多模态预留 |
|---|---|---|
| `VectorStore.encode()` | 接收 `str` → 返回 `List[float]` | 预留 `modality: str = "text"` 参数，支持 `("text", "visual", "fused")` |
| `VectorStore.insert()` | `(node_id, embedding)` | embedding 维度不写死，从 `EmbeddingModelConfig.dim` 读取 |
| `MemoryNode.embedding` | `Vec<f32>` 无模态标记 | 节点元数据预留 `embedding_modality: str = "text"` 字段 |

---

## 3. 存储后端映射

每个字段按访问模式与优先级映射到不同存储引擎。以下为逻辑划分，具体引擎选型不在本文档范围。

| 存储层 | 存储内容 | 索引模式 | 定位 |
|---|---|---|---|
| **Tag Store**（标签索引） | `TagSet` 全量字段 + `node_id` | 倒排索引（空间/行为/认知）+ B+ 树（优化维度排序） | 检索第一入口 O(1) 过滤 |
| **Vector Store**（向量存储） | `embedding: Vec<f32>` d=384 + `node_id` + BM25 词频统计 | 向量相似度索引 (Cosine ANN) + BM25 稀疏索引 | 标签过滤后的候选集上混合检索（BM25 + Embedding） |
| **Graph Store**（图存储） | `MemoryNode` 完整内容（除 embedding + TagSet 的热索引部分） + `CausalEdge` + `SkillTemplate` | 节点 ID 主键 + 因果边邻接表 | 因果过滤 + 持久化 |
| **Skill Store**（技能存储） | `SkillTemplate` 完整内容 | skill_name 唯一索引 | 技能注册/列表/版本管理 |

## 4. 合约接口定义

基于 memsearch 现有三合约（`save`/`search`/`compact`），扩展为完整的记忆生命周期合约。Phase1 实现基础合约，Phase2 实现核心合约，Phase3 实现拓展合约。

### 4.1 基础合约（Phase1 — 对应 memsearch 现有能力）


#### `robonix/service/memory/remember`（对标 save，语义升级）

写入一条记忆节点。输入从 `String` 升级为结构化的 `RememberRequest`。

```
# Request
session_id:     String          # 会话 ID
plan_id:        String          # 规划 ID
log_record:     LogRecord       # 原始日志记录
spatial:        Option<SpatialContext>  # 空间上下文（Scene 提供）
parent_node_id: Option<u32>     # 因果父节点
kv:             Map<String, String>    # 扩展键值对

# Response
node_id:        u32             # 新创建的记忆节点 ID
```

#### `robonix/service/memory/search`（对标 search，维度扩展）

三阶段检索：标签过滤 → BM25 + Embedding 混合精排 → 因果过滤。

```
# Request
query:          String              # 语义查询文本
tags:           Option<TagFilter>   # 标签过滤条件
top_k:          u32                 # 返回 Top-K (默认 5)
alpha:          Option<f32>         # BM25 混合权重; None 则用默认 0.3
                                    #   α=1.0 纯关键词, α=0.0 纯语义
time_range:     Option<TimeRange>   # 时间范围过滤
require_executable: bool            # 是否只返回当前机器人可执行的记忆

# Response
nodes:          Vec<MemoryNode>     # 检索结果, 按相关性降序

TagFilter {
    scene_type:     Option<String>
    objects:        Option<Vec<String>>      # 交集匹配
    action_type:    Option<String>
    success:        Option<bool>
    task_type:      Option<String>
    difficulty_max: Option<String>           # easy / medium
}

TimeRange {
    start_ts:       u64
    end_ts:         u64
}
```

#### `robonix/service/memory/compact`（保持，语义不变）

压缩/总结旧记忆。保持 memsearch 现有接口风格。

```
# Request: empty trigger
# Response
summary:        String              # 压缩结果描述
nodes_compacted: u32                # 被压缩的节点数
```

### 4.2 核心合约（Phase2）

#### `robonix/service/memory/refine` — 技能提炼

从 N≥3 个同类成功经验中提炼 SkillTemplate。

```
# Request
task_type:      String              # 要提炼的任务类型
min_samples:    u32                 # 最少成功样本数 (默认 3)
min_success_rate: f32               # 最低成功率阈值 (默认 0.8)

# Response
skill:          Option<SkillTemplate>  # 提炼出的技能, 样本不足则 None
message:        String              # 提炼过程描述
```

#### `robonix/service/memory/forget` — 遗忘执行

对符合条件的节点执行分级遗忘。

```
# Request
scan_before:    Option<u64>         # 只扫描此时间戳之前的节点, 不传则全量
dry_run:        bool                # true = 只返回评分, 不执行遗忘

# Response
evaluated:      u32                 # 评分的节点总数
retained:       u32                 # 保留
downgraded:     u32                 # 降权 (weight × 0.5)
archived:       u32                 # 归档 (移除索引)
forgotten:      u32                 # 已归档节点中超过30天被物理删除的
```

#### `robonix/service/memory/list_skills` — 获取技能列表

返回可用技能列表，供 Pilot 注册为 LLM function tools。

```
# Request: empty trigger

# Response
skills:         Vec<SkillSummary>   # 技能摘要列表

SkillSummary {
    skill_id:       u32
    skill_name:     String
    version:        u32
    success_rate:   f32
    sample_count:   u32
    deprecated:     bool
}
```

### 4.3 拓展合约（Phase3）

#### `robonix/service/memory/search_history` — 历史回溯

时间 + 空间 + 任务三维历史查询。

```
# Request
time_range:     Option<TimeRange>
spatial_filter: Option<SpatialFilter>
task_type:      Option<String>
session_id:     Option<String>

# Response
nodes:          Vec<MemoryNode>

SpatialFilter {
    region:         Option<String>       # 区域标签
    near_object:    Option<String>       # 靠近某物体
    radius_m:       Option<f32>          # 半径范围
}
```

#### `robonix/service/memory/cron_trigger` — 定时任务触发

```
# Request
cron_id:        String              # 定时任务 ID
action:         CronAction          # trigger / list / cancel

# Response
status:         String

CronAction = enum { Trigger, List, Cancel }
```

---

## 5. 内部管线数据结构

以下结构不暴露为 contract，仅在 service 内部管线间传递。

### 5.1 Refine 管线

```
ClusterResult {
    task_type:      String,              // 聚类维度
    clusters:       Vec<MemoryCluster>,  // 聚类结果
}

MemoryCluster {
    centroid_node:  MemoryNode,          // 聚类中心（最具代表性的节点）
    members:        Vec<u32>,            // 成员 node_id 列表
    intra_distance: f32,                 // 簇内平均距离
}

AbstractionResult {
    goal_template:      String,         // 目标模板
    causal_template:    Vec<StepTemplate>, // 步骤模板
    pre_conditions:     Vec<Condition>, // 前置条件（成员交集）
    post_conditions:    Vec<Condition>, // 后置条件（成员交集）
    lcs_steps:          Vec<u32>,       // 最长公共子序列步骤序号
}

ValidationResult {
    skill:              SkillTemplate,
    replay_success:     u32,            // 模拟回放成功次数
    replay_total:       u32,            // 模拟回放总次数
    pass:               bool,           // success_rate ≥ 80%
}
```

### 5.2 Forget 管线

```
ForgetEvaluation {
    node_id:        u32,
    risk:           ForgetRisk,
    decision:       ForgetDecision,
    reason:         String,             // 决策原因
}

ForgetDecision = enum {
    Retain,                             // 保留
    Downgrade,                          // weight × 0.5
    Archive,                            // 移除索引, 标记待删除
    Delete,                             // 物理删除
}

ForgetAuditEntry {
    timestamp:      u64,
    node_id:        u32,
    decision:       ForgetDecision,
    risk_total:     f32,
    operator:       String,             // "system" / "pilot" / "user"
}
```

---

## 6. 与 memsearch 现有结构的对照迁移

| memsearch 现有 | Scribe Mem 对应 | 迁移说明 |
|---|---|---|
| `memsearch_service/service.py` | `services/memory/memory_service/service.py` | 重命名目录，增加 contract handler |
| `save` contract | `remember` contract | 语义升级：纯文本 → 结构化 MemoryNode |
| `search` contract (String in/out) | `search` contract (结构化 in/out) | 请求增加 TagFilter + TimeRange；响应返回 MemoryNode[] |
| `compact` contract | `compact` contract | 保持触发式; 内部增加因果边压缩 |
| Milvus-lite 向量存储 | FAISS / Vector Store 层 | 向量维度从动态升级为固定 d=768 |
| Markdown 文件存储 | Graph Store 层 (CKG) | 从文件系统升级为图数据库 |
| 无 | Tag Store 层 (Redis) | 新增标签倒排索引 |
| 无 | Skill Store 层 | 新增技能模板持久化 |
| ONNX embedding | all-MiniLM-L6-v2 d=384 + BM25 混合检索 | 从纯文本升级为 BM25+Embedding 混合, 预留多模态融合接口 |
| 纯向量检索 | BM25 + Cosine 加权混合 | 增加关键词召回补齐语义遗漏 |

---

## 7. 附录：字段命名约定

| 层级 | 命名风格 | 示例 |
|---|---|---|
| Rust 内部类型 | `CamelCase` | `MemoryNode`, `TagSet`, `SpatialContext` |
| Rust 字段 | `snake_case` | `node_id`, `causal_chain`, `last_access` |
| Python 内部类型 | `CamelCase` | `MemoryNode`, `TagSet` |
| Python 字段 | `snake_case` | `node_id`, `causal_chain` |
| Contract IDL (.srv) | `snake_case` | `node_id`, `top_k`, `min_samples` |
| Contract ID | `robonix/service/memory/<verb>` | `robonix/service/memory/remember` |
