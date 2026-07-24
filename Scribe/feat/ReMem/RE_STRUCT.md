# 记忆模块三层架构重构计划

## Context




事理知识图谱:
    (skill生成器)

    上层给LLM
    ----
    KG(描述底层还没有定型的skill)->成熟(对齐其他成功经验)->生成skill
        -FIXED Skill (固定Skill) 这些模板是初始化写入的，永不被遗忘，且不参与提升/衰减机制 -> 固定记忆（人为提为固定记忆）
        -Long Term Skills (有成功经历且成熟的Skill) - 这些模板成功很多次 -> 长期记忆（由短期记忆转化，且不断成功不断提纯更新）
            -提纯：KG中同一技能的多个事件链，提纯出共同的核心步骤形成更纯粹的技能节点 - 知识图谱对其

        -Short Term Skill (有成功经历但没成熟的Skill) -> 短期记忆（成功经验）


        -FUNC:
            - init_SKG()
            - add_node()
            - delete_node()
            - update_node()
            - find_nodes() # 找到节点
            - find_chain() # 找到事件链
            - forget_node() # 遗忘节点 独立GC线程
            - RETRIEVAL(query) # 检索接口，返回匹配的事件链/空间技能节点

        - MemoryGraph(底层存储)
            -NODE:
            - TYPE:3*3
            - NAME/SUMMARY:
            - DESC:
            - CHILDS:
            - FATHER:
            - EDGES:
            - ...
    ----
    Map底层保存对知识
    +底层执行节点



## 架构总览

```
┌─────────────────────────────────────────────────┐
│          逻辑调用层 (run-llm-mem.py)             │
│  只调用 SKG 的 5 个公共接口                       │
└──────────────────────┬──────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────┐
│       SKG 中间层 (skg.py 重写)                    │
│                                                   │
│  load()              - 初始化/加载记忆             │
│  save()              - 持久化（MemGraph + skg.json）│
│  retrieve()          - 记忆检索                    │
│     内部: 命中节点 hit_count++，达阈值自动提升长期  │
│  insert_experience() - 成功经验插入（含对齐）       │
│  update_short_term() - 短期空间记忆更新            │
│  feedback_failure()  - 失败经验反馈，降权事件链     │
│                                                   │
│  内部: Retriever(向量检索) + 对齐算法              │
│  内部: 短期→长期提升（retrieve时自动触发）          │
│  内部: 失败降权（feedback后降低检索概率）           │
└──────────────────────┬──────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────┐
│       MemoryGraph 底层 (memory_module.py)         │
│  networkx DiGraph + 节点CRUD + 边标签 + 序列化     │
└─────────────────────────────────────────────────┘
```

## 实施步骤

### Phase 1: 扩展 MemoryGraph 底层（含多模态节点支持）

**文件**: `memory/memory_module.py`

---

#### 1.1 枚举扩展

```python
class NodeType(IntEnum):
    SHORT_TERM = 0   # 短期记忆（可被遗忘/可提升）
    LONG_TERM  = 1   # 长期记忆（不被遗忘，检索提升而来）
    FIXED      = 2   # 固定记忆（初始化写入，永不遗忘）
    SKG_EVENT  = 3   # 事理知识图谱事件节点

class NodeClass(IntEnum):
    CONTEXT = 0      # 情景/技能节点
    TIME    = 1      # 时间有序节点
    SPACE   = 2      # 空间节点

class EdgeType(str, Enum):     # 新增 - 边关系类型
    PARENT_CHILD = "child"     # 层级从属（原有默认边）
    NEXT         = "next"      # SKG 因果/时序后继
    REQUIRES     = "requires"  # 前置依赖
    PRODUCES     = "produces"  # 产出关系

class ModalityType(str, Enum): # 新增 - 多模态数据类型标识
    TEXT       = "text"        # 纯文本描述
    IMAGE      = "image"       # 图像（RGB截图/VLM检测结果）
    DEPTH      = "depth"       # 深度图
    SPATIAL    = "spatial"     # 3D坐标 (x, y, z)
    INVENTORY  = "inventory"   # 背包状态（结构化dict）
    EVENT      = "event"       # 事件要素 (A/O/T/V/P/L)
```

---

#### 1.2 多模态记忆节点数据结构

现有 `MemoryNode` 仅存储 `name/summary` 文本和基础属性。扩展为支持多模态 payload：

```python
@dataclass
class MemoryNode:
    # === 基础属性（保持不变） ===
    node_id: int
    node_type: NodeType
    node_class: NodeClass
    name: str = ""
    summary: str = ""
    parent_id: int = -1
    weight: float = 1.0
    child_cnt: int = 0
    parents_cnt: int = 0

    # === 新增：检索/提升相关 ===
    hit_count: int = 0              # 被检索命中次数（用于短期→长期提升）

    # === 新增：多模态数据容器 ===
    modalities: Dict[str, Any] = field(default_factory=dict)
    # 按 ModalityType key 存储，示例：
    # {
    #   "text":      {"description": "chop oak_log", "language": "en"},
    #   "image":     {"path": "log/malmo_rgb.png", "bbox": [...], "vlm_labels": [...]},
    #   "depth":     {"path": "log/malmo_depth.png"},
    #   "spatial":   {"x": 100.5, "y": 64.0, "z": -200.3, "yaw": 90.0, "pitch": 0.0},
    #   "inventory": {"input_items": {"oak_log": 1}, "output_items": {"planks": 4}},
    #   "event":     {"A": "attack", "O": "oak_log", "T": 3, "V": "forest", "P": "get_wood", "L": "chop the tree"}
    # }

    # === 新增：向量嵌入缓存 ===
    embedding: Optional[np.ndarray] = None  # 语义向量（由 Retriever 编码后缓存）
```

**事件要素 (A/O/T/V/P/L)** 遵循 TODO 中定义的事理图谱要素模型：

| 要素 | 字段 | 说明 | 数据来源 |
|------|------|------|----------|
| A (Action) | `event.A` | 动作触发词 | `action` 参数（attack/craft/move/look/use） |
| O (Object) | `event.O` | 操作客体 | `aimed_object` / `hotbar_item` / `craft_target` |
| T (Time) | `event.T` | 时间步序号 | 动作在序列中的顺序 |
| V (Environment) | `event.V` | 环境上下文 | `obj_list`（周围物体）/ biome 信息 |
| P (Assertion) | `event.P` | 断言/意图 | `sub_mission` 任务描述 |
| L (Language) | `event.L` | 自然语言表述 | LLM 生成的动作描述 |

---

#### 1.3 MemoryGraph 完整接口规范

##### 节点 CRUD

```python
class MemoryGraph:

    # ---------- 构造 ----------
    def __init__(self, id_start: int = 0, max_id: int = 1000):
        """
        初始化记忆图
        - id_start: 节点ID起始值
        - max_id: 短期记忆ID上限，溢出触发遗忘
        """

    # ---------- 节点操作 ----------
    def add_node() -> int:
        """
        添加节点到图中
        返回: 新节点的 node_id
        """

    def delete_node(self, node_id: int):
        """递归删除节点及其孤儿子节点,引用计数，使用GC线程定期清理孤儿节点"""

    def update_node(self, node_id: int, **kwargs):
        """
        更新节点任意属性
        """

    def get_node(self, node_id: int) -> Optional[MemoryNode]:
        """获取 MemoryNode dataclass（基础属性）"""

    # ---------- 节点查询 ----------
    def find_nodes(self, **filters) -> Dict[int, MemoryNode]:
        """
        按 MemoryNode 属性过滤查询
        """
```

##### 边（关系）操作

```python
    # ---------- 层级边（父子关系）----------
    def add_child(self, parent_id: int, child_id: int):
        """添加父子边（EdgeType.PARENT_CHILD），更新双方计数"""

    def delete_child(self, parent_id: int, child_id: int):
        """删除父子边，孤儿子节点自动删除"""

    def get_childs(self, node_id: int) -> List[int]:
        """获取所有子节点ID列表"""

    def get_child_num(self, node_id: int) -> int:
        """获取子节点数量"""

    # ---------- 标签边（SKG因果/关联关系）---------- 新增
    def add_edge_labeled(self, source_id: int, target_id: int,
                         relation: EdgeType = EdgeType.NEXT,
                         weight: float = 1.0):
        """
        添加带标签的有向边（不影响父子计数）
        """

    def delete_edge_labeled(self, source_id: int, target_id: int,
                            relation: EdgeType = None):
        """
        删除标签边
        relation=None 时删除 source→target 之间所有标签边
        指定 relation 时仅删除匹配类型的边
        """

    def get_edges_from(self, node_id: int,
                       relation: EdgeType = None) -> List[Tuple[int, EdgeType, float]]:
        """
        获取从 node_id 出发的标签边
        返回: [(target_id, relation, edge_weight), ...]
        relation=None 返回所有, 指定则过滤
        """

    def get_edges_to(self, node_id: int,
                     relation: EdgeType = None) -> List[Tuple[int, EdgeType, float]]:
        """
        获取指向 node_id 的标签边（反向查询）
        用于查找 "谁 NEXT 到我" / "谁 REQUIRES 我"
        """

    def find_chain(self, start_id: int, relation: EdgeType = EdgeType.NEXT) -> List[int]:
        """
        从 start_id 沿指定关系类型遍历，返回完整链的节点ID列表
        用于 SKG 事件链追踪（替代原 ScriptKnowledgeGraph.find_event_chain）
        """

    def find_chain_roots(self, relation: EdgeType = EdgeType.NEXT) -> List[int]:
        """
        查找所有链头节点（有 relation 类型出边但无同类型入边的节点）
        用于获取所有 SKG 事件链的起始节点
        """
```

##### 遗忘机制

```python
    # ---------- 遗忘 ----------
    def forget_node(self) -> bool:
        """
        短期记忆遗忘策略
        遍历所有 SHORT_TERM 节点:
        - 超过 time_threshold（默认3600s）的节点删除
        - ID 溢出时触发
        注意: LONG_TERM/FIXED/SKG_EVENT 不受影响

        删除引用数0的孤儿节点
        """
```

##### 多模态数据操作

```python
    # ---------- 多模态数据操作 ---------- 新增
    def set_modality(self, node_id: int, modality: ModalityType, data: dict):
        """
        设置/更新节点的特定模态数据
        示例:
          set_modality(nid, ModalityType.IMAGE,
                       {"path": "log/frame.png", "vlm_labels": ["oak_log", "grass"]})
          set_modality(nid, ModalityType.EVENT,
                       {"A": "attack", "O": "oak_log", "T": 1, "V": "forest"})
          set_modality(nid, ModalityType.INVENTORY,
                       {"input_items": {"oak_log": 1}, "output_items": {"planks": 4}})
        """

    def get_modality(self, node_id: int, modality: ModalityType) -> Optional[dict]:
        """
        获取节点的特定模态数据
        返回 None 如果该节点没有此模态
        """

    def get_all_modalities(self, node_id: int) -> Dict[str, dict]:
        """获取节点的全部模态数据字典"""

    def set_embedding(self, node_id: int, embedding: np.ndarray):
        """
        缓存节点的语义向量嵌入
        由 SKG 层的 Retriever 编码后回写
        避免重复编码同一节点
        """

    def get_embedding(self, node_id: int) -> Optional[np.ndarray]:
        """获取缓存的向量嵌入，None 表示未编码"""
```

##### 序列化

```python
    # ---------- 持久化 ----------
    def save_to_file(self, file_path: str):
        """
        序列化整个图到 JSON 文件
        包含:
        - nodes: 所有节点（基础属性 + 图属性 + modalities + hit_count）
        - edges: 所有边（source, target, relation, weight）
        - meta:  id_counter 当前值、max_id 等

        注意: embedding (numpy array) 不序列化到 JSON
              image/depth 的 path 序列化，实际文件不复制
        """

    def load_from_file(self, file_path: str):
        """
        从 JSON 反序列化图
        - 重建所有节点（按 node_class 区分空间/时间/情景属性）
        - 重建所有边（含 relation 标签和 weight）
        - 恢复 modalities 字典到图属性
        - 重置 _id_counters = max(已有ID) + 1，避免 ID 冲突
        - embedding 不恢复（需要时由 Retriever 重新编码）
        """
```

##### 可视化

```python
    # ---------- 可视化 ----------
    def visualize(self, file_path: str):
        """
        渲染图为 PNG
        - 不同 NodeType 用不同颜色区分:
          SHORT_TERM=浅黄, LONG_TERM=浅蓝, FIXED=浅绿, SKG_EVENT=浅紫
        - 不同 EdgeType 用不同线型:
          PARENT_CHILD=实线, NEXT=虚线箭头, REQUIRES=点线, PRODUCES=双线
        - 节点标签显示 name，边标签显示 relation
        """
```

---

#### 1.4 多模态数据在各层级的流转

```
观测层 (run-llm-mem.py)
  │
  ├─ RGB截图 ──► VLM检测 ──► vlm_labels + bbox
  ├─ 深度图 ──► depth_path
  ├─ 3D网格 ──► obj_list (name, RELx, RELy, RELz, size)
  ├─ 背包  ──► inventory dict
  ├─ 实体  ──► x, y, z, yaw
  │
  ▼
SKG 中间层
  │ update_short_term(obj_list, entity):
  │   对每个物体创建/更新 SPACE 节点:
  │     modalities.spatial = {x, y, z, yaw, pitch}
  │     modalities.image   = {path, vlm_labels, bbox}  (如果有VLM结果)
  │
  │ insert_experience(task, records):
  │   对每步动作创建 SKG_EVENT 节点:
  │     modalities.event     = {A, O, T, V, P, L}
  │     modalities.inventory = {input_items, output_items}
  │     modalities.spatial   = {x, y, z}  (动作后位置)
  │     modalities.image     = {path}     (动作时截图, 可选)
  │
  ▼
MemoryGraph 底层
  存储到 networkx 节点属性中
  序列化到 JSON（image/depth 只存路径引用）
```

---

#### 1.5 `MemoryNode.modalities` 各模态详细 Schema

| 模态 | Key | Schema | 来源 |
|------|-----|--------|------|
| `text` | `ModalityType.TEXT` | `{"description": str, "language": "en"\|"zh"}` | `summary` 字段的结构化版本 |
| `image` | `ModalityType.IMAGE` | `{"path": str, "bbox": List[dict], "vlm_labels": List[str], "frame_id": int}` | `vlm_detect` 输出 + RGB截图路径 |
| `depth` | `ModalityType.DEPTH` | `{"path": str, "range": [float, float]}` | 深度通道截图路径 |
| `spatial` | `ModalityType.SPATIAL` | `{"x": float, "y": float, "z": float, "yaw": float, "pitch": float}` | `entity` 状态 / `obj_list` 相对坐标 |
| `inventory` | `ModalityType.INVENTORY` | `{"input_items": Dict[str,int], "output_items": Dict[str,int], "hotbar_id": int, "hotbar_item": str}` | `craft_diff_get` / `mem_generation` 输出 |
| `event` | `ModalityType.EVENT` | `{"A": str, "O": str, "T": int, "V": str\|list, "P": str, "L": str}` | 从 `mem_generation` 结构化提取 |

**event 要素映射规则**（`mem_generation` → event modality）：

```python
def _extract_event_elements(action, mem_record, sub_mission, step_idx) -> dict:
    """从 mem_generation 的输出提取 A/O/T/V/P/L 事件要素"""
    event = {}
    event["A"] = action                                    # 动作触发词
    # 客体：根据动作类型从不同字段提取
    if "craft" in action:
        event["O"] = list(mem_record["output"].get("output_items", {}).keys())
    elif action == "attack":
        event["O"] = mem_record["input"].get("aimed_obj", "")
    elif action == "use":
        event["O"] = mem_record["input"].get("hotbar_item", "")
    else:
        event["O"] = mem_record["input"].get("aimed_obj", "")
    event["T"] = step_idx                                  # 序列时间步
    event["V"] = mem_record["input"].get("obj_list", [])   # 环境上下文
    event["P"] = sub_mission                               # 任务断言/意图
    event["L"] = f"{action} {event['O']}"                  # 自然语言表述
    return event
```

### Phase 2: 重写 SKG 为中间层门面

**文件**: `memory/modules/skg/skg.py`（完全重写）

新类结构：

```python
class ScriptKnowledgeGraph:
    def __init__(self, memory_graph: MemoryGraph, skg_path: str = None):
        self.mg = memory_graph               # 底层图引用
        self.retriever = None                 # 惰性初始化
        self._skg_path = skg_path or "memory/data/skg.json"
        self._retriever_dirty = True          # 标记是否需要重建索引
        self._skg_id_map = {}                 # skg string ID <-> mg int ID
        self._promote_threshold = 3           # 短期→长期提升的命中次数阈值
        self._failure_decay = 0.5             # 失败反馈的权重衰减因子

    # === 5 个公共接口 ===
    def load(self, scene_info: dict, skg_path: str = None): ...
    def save(self): ...
    def retrieve(self, query: str, top_k: int = 5) -> List[dict]: ...
        # 内部自动: 命中节点 hit_count++，达阈值自动 SHORT_TERM→LONG_TERM
        # 内部自动: 检索时用 weight 加权相似度分数，低权重节点排名靠后
    def insert_experience(self, task_describe: str, record_actions: list) -> dict: ...
    def update_short_term(self, obj_list: list, entity: dict): ...
    def feedback_failure(self, chain_node_ids: List[int]): ...
        # 对失败的事件链所有节点执行权重衰减

    # === 内部方法 ===
    def _align_and_merge(self, task_describe, action_names) -> Optional[int]: ...
    def _semantic_match(self, query, candidates, threshold=0.7): ...
    def _structural_match(self, new_actions, existing_chain_id, threshold=0.6) -> float: ...
    def _create_event_chain(self, task_describe, record_actions) -> int: ...
    def _merge_event_chain(self, chain_id, task_describe, record_actions): ...
    def _try_promote(self, node_id: int): ...
        # 检查 hit_count >= threshold，若是则 node_type: SHORT_TERM→LONG_TERM
    def _sync_to_skg_json(self): ...
    def _load_skg_json(self, path): ...
```

### Phase 3: 实现各接口

#### 3.1 `load(scene_info, skg_path)`
- 复用现有 `CurrentState.init_Scene` 的逻辑构建空间/技能节点到 MemoryGraph
- 调用 `_load_skg_json()` 读取 skg.json，将每个事件创建为 `SKG_EVENT` 节点，用 `add_edge_labeled` 建立因果链
- 维护 `_skg_id_map`（字符串ID↔整数ID映射）

#### 3.2 `save()`
- 调用 `self.mg.save_to_file("memory/data/current_memory_graph.json")`
- 调用 `_sync_to_skg_json()` 将所有 `SKG_EVENT` 节点导出为 skg.json 格式（整数ID→"eN"字符串）

#### 3.3 `retrieve(query, top_k)`（替代 `retrieval_Request` + `retrieval_memory`）
- 从 `self.mg._node_map` 收集所有节点的 summary/name
- 惰性初始化/更新 `Retriever`（检查 `_retriever_dirty` 标记）
- 调用 `retriever.retrieve(query, top_k)` 获取候选匹配
- **权重加权排序**：最终得分 = `cosine_similarity * node.weight`，weight 低的节点（被 `feedback_failure` 降权过的）自然排名靠后
- 对匹配到的 `SKG_EVENT` 节点，沿 `"next"` 边追踪完整事件链
- 对匹配到的空间/技能节点，直接返回节点信息
- **命中计数 + 短期→长期自动提升**：每个被返回的节点 `hit_count += 1`，然后调用 `_try_promote(node_id)` 检查是否达到提升阈值
- 返回结果中携带 `chain_node_ids` 字段，供调用层在执行失败时传给 `feedback_failure()`
- 返回 `[{"name": ..., "event": ..., "chain": [...], "chain_node_ids": [...]}]`
- **不再生成中间文件 retrieval_memory.json**

**`_try_promote(node_id)` 内部逻辑**：
```python
def _try_promote(self, node_id):
    node = self.mg.get_graph_node(node_id)
    node['hit_count'] = node.get('hit_count', 0) + 1
    if (node['hit_count'] >= self._promote_threshold
        and self.mg._node_map[node_id].node_type == NodeType.SHORT_TERM):
        self.mg.update_node(node_id, node_type=NodeType.LONG_TERM)
        # 长期节点不会被 forget_node() 淘汰
```

#### 3.3b `feedback_failure(chain_node_ids)`（失败经验反馈）
- 调用层在执行某个检索出的事件链失败后调用
- 对链中所有节点执行权重衰减：`node.weight *= self._failure_decay`（默认 0.5）
- 权重有下限（如 0.1），防止完全不可检索
- 衰减后的 weight 在下次 `retrieve()` 中通过加权排序生效，降低该链被优先返回的概率
- 调用 `_sync_to_skg_json()` 持久化权重变化

```python
def feedback_failure(self, chain_node_ids: List[int]):
    """失败经验反馈：降低事件链的检索权重"""
    for nid in chain_node_ids:
        node = self.mg.get_graph_node(nid)
        old_weight = node.get('weight', 1.0)
        new_weight = max(old_weight * self._failure_decay, 0.1)
        self.mg.update_node(nid, weight=new_weight)
    self._sync_to_skg_json()
```

#### 3.4 `insert_experience(task_describe, record_actions)`（替代 `skill2FIXED_mem` + `append_to_skg`）

**两阶段对齐流程**：

```
新经验 → 语义粗筛(Retriever余弦相似度 ≥ 0.7)
           ↓ 候选列表
       → 结构精匹配(动作序列LCS比率 ≥ 0.6)
           ↓
       匹配成功 → _merge_event_chain(): 更新权重、补充描述、追加新步骤
       匹配失败 → _create_event_chain(): 创建新 SKG_EVENT 链
```

- `_semantic_match`：用 Retriever 编码 `task_describe`，与所有已有事件链根节点（无入边的 SKG_EVENT）比较余弦相似度
- `_structural_match`：提取已有链的动作名序列，与新经验的动作名序列计算 LCS 比率：`2*len(LCS)/(len(a)+len(b))`
- `_create_event_chain`：创建任务根节点（SKG_EVENT/CONTEXT）+ 每步动作节点（SKG_EVENT/TIME），用 `"next"` 边串联。同时在 MemoryGraph 中添加为 FIXED 技能节点
- `_merge_event_chain`：增加已有链根节点权重，如果新经验有额外步骤则追加节点
- 最后调用 `_sync_to_skg_json()` 持久化
- 返回 task_spec 字典（`name, description, input, output, actions`）

#### 3.5 `update_short_term(obj_list, entity)`（替代 `record_short_space_memory`）
- 找到 `/temp` 空间节点
- **增量更新**而非清除重建：比较新旧物体集合，仅增删差异部分
- 新增节点初始化 `hit_count = 0`、`weight = 1.0`
- 标记 `_retriever_dirty = True`

> **注意**：原 `commit_short_to_long` 不再作为独立接口。短期→长期提升统一在 `retrieve()` 中通过 `_try_promote()` 自动完成——当空间节点被检索命中次数达到 `_promote_threshold`（默认3次）时自动提升为 LONG_TERM。这比显式调用更自然：真正有用的记忆会因为被频繁检索到而自动变为长期记忆。

### Phase 4: 重构 CurrentState 为薄代理

**文件**: `memory/memory_module.py`

```python
class CurrentState:
    def __init__(self):
        self.memory_graph = MemoryGraph(...)
        self.skg = ScriptKnowledgeGraph(self.memory_graph)

    def init_Scene(self, scene_info):
        self.skg.load(scene_info)

    def update_Scene(self, scene_info):
        # 保留向后兼容，内部委托给 skg
        self.skg.update_short_term(...)  # 或 self.skg.load(scene_info) for full reload

    def retrieval_Request(self, query, top_k=5):
        return self.skg.retrieve(query, top_k)
```

- 删除 `update_retrieval()` 方法
- 新代码应直接使用 `cs.skg.xxx()`

### Phase 5: 重构 run-llm-mem.py 调用层

**文件**: `simulator/MalmoEnv/run-llm-mem.py`

替换映射：

| 现有代码 | 替换为 |
|---------|--------|
| `scene_info = record_short_space_memory(scene_info, obj_list, entity)` | `cs.skg.update_short_term(obj_list, entity)` |
| `scene_info = short2long_space_memory(entity, around, scene_info)` | **删除**（短期→长期在 retrieve 内自动触发） |
| `cs.update_Scene(scene_info)` | 移除（已在 SKG 内部处理） |
| `rel_info, jud_info = retrieval_memory(cs, q, scene_info, log)` | `results = cs.skg.retrieve(q); rel_info, jud_info = format_results(results)` |
| `scene_info = skill2FIXED_mem(sub, records, scene_info)` + `cs.update_Scene(scene_info)` | `cs.skg.insert_experience(sub, records)` |
| `submission2MEM()` | `cs.skg.save()` |
| *(新增)* 子任务执行失败时 | `cs.skg.feedback_failure(results[i]["chain_node_ids"])` |

涉及调用点：~1841, ~1843, ~1846, ~1883, ~1885, ~1887, ~2036, ~2038, ~2082-2084, ~2131

新增辅助函数 `format_retrieval_results(results, log_file) -> (str, dict)` 处理返回值格式化。

**失败反馈调用时机**（新增逻辑）：
```python
# 在 sub-mission 执行结束后判断
results = cs.skg.retrieve(sub_mission)
rel_info, jud_info = format_retrieval_results(results, log_file)
# ... 执行 sub_mission ...
if not success:  # 子任务执行失败
    # 降低本次检索命中的事件链权重
    for r in results:
        if r.get("chain_node_ids"):
            cs.skg.feedback_failure(r["chain_node_ids"])
else:  # 子任务执行成功
    cs.skg.insert_experience(sub_mission, record_actions)
```

### Phase 6: 清理废弃代码

- **删除** `memory/modules/memory/memory.py` 中的 `append_to_skg`（被 `insert_experience` 替代）
- **删除** `memory_module.py` 中的 `update_retrieval` 方法
- **删除** `run-llm-mem.py` 中的独立函数：`skill2FIXED_mem`, `record_short_space_memory`, `short2long_space_memory`（已集成到retrieve自动提升）, `retrieval_memory`, `submission2MEM`
- **删除** 运行时生成的 `memory/data/retrieval_memory.json`（不再需要中间文件）
- **移除** `memory_module.py` 第15行 `from memory.modules.memory.memory import *`

## Retriever 扩展

**文件**: `memory/modules/retriever/retriever.py`

1. **增量编码方法**，避免每次重编码所有事件：
```python
def add_events(self, new_events: list):
    """增量编码新事件，追加到嵌入矩阵"""
    new_embeddings = self.model.encode(new_events, ...)
    self.embeddings = torch.cat([self.embeddings, new_embeddings])
    self.events.extend(new_events)
```

2. **权重加权检索**，支持失败降权后的排序调整：
```python
def retrieve(self, query, top_k=3, weights=None):
    """
    weights: 与 events 等长的权重数组，默认全1.0
    最终得分 = cosine_similarity * weight
    """
    query_embedding = self.model.encode([query], ...)
    scores = util.cos_sim(query_embedding, self.embeddings)[0]
    if weights is not None:
        scores = scores * torch.tensor(weights)
    top_results = torch.topk(scores, min(top_k, len(self.events)))
    return [(self.events[i], scores[i].item()) for i in top_results.indices]
```

SKG 在调用 `retriever.retrieve()` 时，从 MemoryGraph 中提取各节点的 `weight` 属性组成 weights 数组传入。

## 关键文件清单

| 文件 | 变更类型 |
|------|---------|
| `memory/memory_module.py` | 修改（扩展 MemoryGraph + 重构 CurrentState） |
| `memory/modules/skg/skg.py` | **完全重写**（36行→~300行） |
| `memory/modules/retriever/retriever.py` | 小修改（增量编码） |
| `memory/modules/memory/memory.py` | 删除或清空 |
| `simulator/MalmoEnv/run-llm-mem.py` | 修改（~10个调用点替换 + 删除5个函数） |

## 验证方案

1. 单元测试 MemoryGraph 新增的边标签方法和 SKG_EVENT 节点类型
2. 构造 mock scene_info + skg.json，验证 `SKG.load()` → `SKG.save()` 的双向同步
3. 验证 `insert_experience` 的对齐逻辑：相同任务二次插入应触发合并而非新增
4. 验证 `retrieve` 返回结果与原 `retrieval_memory` 一致
5. 在 Minecraft 仿真环境中端到端运行，对比 MEM_MODE 开启前后的行为一致性
