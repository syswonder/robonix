# Scribe Mem — TODO

## 已完成 Phase（✅ Phase 1–3）

| Phase | 模块 | 状态 | 文件 |
|---|---|---|---|
| Phase 1 | 核心类型定义 | ✅ | `services/memory/memory_service/core/types.py` |
| Phase 1 | Graph Store | ✅ | `services/memory/memory_service/storage/graph_store.py` |
| Phase 1 | Tag Index | ✅ | `services/memory/memory_service/storage/tag_index.py` |
| Phase 1 | Vector Store + BM25 + Embedding | ✅ | `services/memory/memory_service/storage/{embedding_config,vector_store}.py` |
| Phase 2 | Remember 管线 | ✅ | `services/memory/memory_service/core/remember.py` |
| Phase 2 | Search 管线 | ✅ | `services/memory/memory_service/core/retrieve.py` |
| Phase 2 | Compact 管线 | ✅ | `services/memory/memory_service/core/compact.py` |
| Phase 3 | 服务注册 + Contract IDL | ✅ | `services/memory/memory_service/service.py`<br>`capabilities/lib/memory/srv/Remember.srv`<br>`capabilities/service/memory/remember.v1.toml` |

**测试覆盖**：113 个测试全部通过（8 个测试文件）

**Demo 验证**：
- ✅ Demo 1（物体时空回溯）：写入含空间坐标的记忆 → scene_type 过滤 → 命中正确节点
- ✅ Demo 2（任务历史回溯）：写入含 task_type 的记忆 → task_type 过滤 → 命中正确节点

---

## 待实现（Phase 4 扩展 — 后续迭代）

- TODO 9: Refine 合约 — 技能提炼（依赖 TODO 6）
- TODO 10: Forget 合约 — 遗忘执行（依赖 TODO 1,2）
- TODO 11: list_skills 合约 — 技能列表（依赖 TODO 9）
- TODO 12: search_history 合约 — 历史回溯（依赖 TODO 6）
- TODO 13: cron_trigger 合约 — 定时任务管理（依赖 TODO 12）

---

## Code Review 修复（2026-07-04 — 全部已修复 ✅）

基于 `review.md` 的 9 条意见：

### 🔴 High（已修复）

| # | 文件 | 问题 | 修复 |
|---|------|------|------|
| R1 | `vector_store.py` | `_try_load_model()` 调用 `os.path.isdir()` 但未 `import os`，导致 `NameError` | 添加 `import os` |
| R2 | `vector_store.py` | `search()` 的 `np.dot(query_vec, vec)` 假设同维度；旧数据/手动插入的错维度向量会导致 shape mismatch crash | 添加 `vec.shape != query_vec.shape` 检查，跳过不兼容向量 |
| R3 | `embedding_config.py` | `_hash_embedding()` 用 `struct.unpack('>f', ...)` 解析任意 hash 字节为 float，可能产生 NaN/Inf；`min/max` 无法消除 NaN，导致 norm=NaN | 改为 `struct.unpack('>I', ...)` 无符号整数除法映射到 [-1, 1]，保证有限值和确定性单位向量 |
| R4 | `memory/srv/Search.srv` | 将 memsearch 的 `query → results` 改为 `request_json → response_json`，破坏了并行运行的 memsearch 线约签名 | 恢复为 `query → results`，JSON schema 文档移至 `memgraph/srv/Search.srv` |
| R5 | `memory/srv/Compact.srv` | 将 memsearch 的 `summary` 改为 `response_json`，同样破坏 memsearch 兼容性 | 恢复为 `summary`，JSON schema 文档移至 `memgraph/srv/Compact.srv` |

### 🟡 Medium（已修复）

| # | 文件 | 问题 | 修复 |
|---|------|------|------|
| R6 | `graph_store.py` | `_next_id()` 用 `< 999` 使 ID 999 不可达（off-by-one），短 term 区间变为 0–998 | 改为 `<= 999`，与 `add_node()` 的 `< 1000` / `promote` 的 `>= 1000` 一致 |
| R7 | `retrieve.py` | `search()` 更新 `node.last_access` / `access_count` 但不持久化回 `GraphStore`；重启后 access 元数据丢失，影响 forget/compact 评分 | 调用 `self._graph.update_node(nid, node)` 持久化更新 |
| R8 | `service.py` | `MEMORY_DIR` 默认 `./memory` 从 import 时的 CWD 解析；被 test/tool 从其他目录 import 时会落在意夕之外的位置 | 用 `Path(__file__).resolve().parent.parent / "memory"` 锚定到 package 根目录 |
| R9 | `builder.py` | `import_yaml_to_service(..., clear_existing=True)` 只调 `service.init()`（重建索引），不清除已有节点，导致重复 import 追加/重复数据 | 改为先遍历 `graph.all_ids()` 删除所有节点，再 `tags.rebuild([])` + `vectors.rebuild([])` |

### 测试回归

```
139 tests passed, 0 failed（全部 9 项修复后无回归）
```

---

## ✅ 已完成 — Demo 巡检基础设施（2026-07-06）

| TODO | 模块 | 状态 | 文件 |
|------|------|------|------|
| D1 | MemoryNode.image_refs 字段 | ✅ | `core/types.py` |
| D2 | ImageStore 本地图片存储 | ✅ | `storage/image_store.py` |
| D3 | ObservePipeline — Scene→ 图片→ 记忆 | ✅ | `core/observe.py` |
| D4 | VLM 图片描述 (复用 Pilot VLM 配置) | ✅ | `core/observe.py` (同 D3) |
| D5 | MCP observe tool + contract IDL | ✅ | `service.py`, `capabilities/.../Observe.srv`, `observe.v1.toml` |

**测试覆盖**: 177 tests（+7 image_store, +16 observe）

---

## 🔥 待完成 — rbnx chat 联调演示

| TODO | 说明 | 状态 |
|------|------|------|
| D6 | rbnx chat 端到端演示 (Webots + Tiago + Scene + Memgraph + Pilot) | 待实现 |
| D7 | 功能验证脚本 `scripts/verify_observe_demo.py` | 待实现 |

---

## 🔥 优先任务：Demo — 巡检保存图片 → 基于图片问问题（Webots + rbnx chat）

### 目标

```
rbnx boot (Webots Tiago)
  → rbnx chat: "patrol the house and remember what you see"
    → 机器人巡检各房间，检测物体 → 调用 memgraph.remember
      (保存 MemoryNode + 图片到 data/images/{node_id}/)
  → rbnx chat: "what was on the kitchen counter?"
    → Pilot 调 hybrid_search(scene_type=kitchen, ...) → 拿到图片引用
    → VLM 看图回答
```

### 设计决策

| 决策 | 选择 | 理由 |
|------|------|------|
| 图片存储 | `services/memory/data/images/{node_id}/` | 与 memory/ 数据目录同级，按 node_id 隔离 |
| 触发方式 | Scene 检测到物体后自动触发 `remember` | 不在 Pilot 显式调用，而是执行管线自动写入 |
| VLM 配置 | 复用 Pilot 的 `VLM_BASE_URL` / `VLM_API_KEY` / `VLM_MODEL` | 零额外配置 |
| 巡检路线 | Pilot 通过 LLM 规划 waypoints → Executor 调 navigation | 不需要硬编码路线 |

---

### TODO D1: MemoryNode 图片字段扩展（memgraph 侧）

**依赖**: Phase 1 types

**说明**: 在 `MemoryNode` 中增加 `image_refs` 字段，使记忆节点能关联巡检时拍摄的图片。

**需要修改的文件**:
- `services/memory/memory_service/core/types.py`
  - `MemoryNode` 新增字段 `image_refs: List[str]`（图片相对路径列表）
  - 更新 `to_dict()` / `from_dict()` 序列化

**需要实现的功能 API**:
- `MemoryNode.image_refs` — 关联图片路径列表（如 `["data/images/42/frame_0001.png", "data/images/42/frame_0002.png"]`）

**需要生成的测试**:
- `test_image_refs_roundtrip` — `MemoryNode` 含 `image_refs` 的 `to_dict` → `from_dict` 往返

---

### TODO D2: 图片本地存储模块

**依赖**: TODO D1

**说明**: 在 `services/memory/data/images/` 下按 `node_id` 创建子目录，保存 Camera 原始帧。

**需要修改的文件**:
- `services/memory/memory_service/storage/image_store.py`（**新增**）
  - `ImageStore` 类：`save(node_id, image_bytes)` → 写入 `data/images/{node_id}/frame_{seq}.png`
  - `list(node_id)` → 返回该节点的所有图片路径
  - `get_path(node_id)` → 返回 `data/images/{node_id}/` 目录路径

**需要实现的功能 API**:
- `image_store.save(node_id, image_bytes)` → `str`（保存一张图片，返回相对路径）
- `image_store.save_batch(node_id, images: List[bytes])` → `List[str]`
- `image_store.list(node_id)` → `List[str]`

**需要生成的测试**:
- `test_save_and_list_images`
- `test_empty_node_no_images`

---

### TODO D3: Scene 物体检测 → Memgraph 自动写入

**依赖**: TODO D2, Phase 2 remember

**说明**: 这是核心桥接模块。当 Scene service 检测到新物体时（通过 `list_objects` MCP 或事件回调），自动：
1. 获取当前 Camera 帧（RGB snapshot）
2. 保存图片到 `data/images/{node_id}/`
3. 构建 `SpatialContext`（从 Scene 的 `list_objects` 结果）
4. 调用 `remember` 写入 MemoryNode（含 spatial 坐标 + image_refs）

触发时机：Robot 到达新位置后，Scene 检测到 ≥1 个物体。

**需要修改的文件**:
- `services/memory/memory_service/core/observe.py`（**新增**）
  - `ObservePipeline` 类：
    - `observe_and_remember(session_id, plan_id, camera_frame, scene_objects) → node_id`
    - 管线：保存图片 → 构建 SpatialContext → 提取 TagSet → 生成 summary → 写 MemoryNode

**需要实现的功能 API**:
- `observe_pipe.observe_and_remember(session_id, plan_id, image_bytes, objects) → int`

**需要生成的测试**:
- `test_observe_saves_image` — 观察后 `data/images/{node_id}/` 存在图片
- `test_observe_node_has_image_refs` — MemoryNode 中的 `image_refs` 非空
- `test_observe_spatial_match` — 写入的 SpatialContext 与 scene 物体一致

---

### TODO D4: VLM 图片理解 — 增强 summary

**依赖**: TODO D3

**说明**: 复用 Pilot 的 VLM 配置（`VLM_BASE_URL` / `VLM_API_KEY` / `VLM_MODEL`），
在 `observe_and_remember` 管线中，将图片发给 VLM 生成自然语言场景描述作为
`MemoryNode.summary`，替换当前基于关键词的模板摘要。

**需要修改的文件**:
- `services/memory/memory_service/core/observe.py`（同上）
  - `_vlm_describe(image_bytes, objects)` → VLM 生成描述
  - 使用 OpenAI-compatible `/chat/completions` + `image_url` content part

**需要实现的功能 API**:
- `_vlm_describe(image_bytes, detected_objects)` → `str`（如 "a red cup and a blue plate sit on the kitchen counter"）

**需要生成的测试**:
- `test_vlm_summary_no_creds_fallback` — 无 VLM 配置时，回退到模板摘要
- `test_vlm_summary_format` — VLM 生成的 summary 存入 MemoryNode

---

### TODO D5: MCP 接口 — `robonix/service/memory/observe`

**依赖**: TODO D4

**说明**: 将 `observe_and_remember` 暴露为 MCP tool，供 Pilot/Executor 调用。
Contract ID: `robonix/service/memory/observe`

**需要修改的文件**:
- `capabilities/lib/memgraph/srv/Observe.srv`（**新增**）
- `capabilities/service/memory/observe.v1.toml`（**新增**）
- `services/memory/memory_service/service.py`
  - 注册 `@_memory_svc.mcp("robonix/service/memory/observe")`
- `services/memory/package_manifest.yaml`
  - 添加 capability: `robonix/service/memory/observe`

**需要实现的 API（JSON-over-String wire format）**:
```json
Request: {
  "session_id": "...",
  "plan_id": "...",
  "image_base64": "...",         // Camera RGB frame as base64
  "objects": [                   // from Scene.list_objects
    {"obj_id": "scene.obj.cup_001", "label": "red cup", "x": 1.0, "y": 2.0, "z": 0.8}
  ]
}
Response: {"node_id": 42, "image_path": "data/images/42/frame_0001.png", "summary": "..."}
```

**需要生成的测试**:
- `test_observe_mcp_roundtrip` — MCP JSON → observe → MemoryNode 含图片引用

---

### TODO D6: rbnx chat 端到端演示

**依赖**: TODO D5

**说明**: 在 Webots 环境中通过 `rbnx chat` 完成完整的巡检-问答闭环。

**演示步骤**:
1. `rbnx boot` — 启动 Webots + Tiago + Scene + Memgraph + Pilot
2. `rbnx chat` → 用户: "patrol the kitchen and living room, remember what you see"
3. Pilot 规划路线 → Executor 执行 navigation
4. 到达 kitchen: Scene 检测物体 → 调 `observe` → 保存图片 + MemoryNode
5. 到达 living room: 同上
6. 用户: "what was on the kitchen counter?" → Pilot 调 `hybrid_search(scene_type=kitchen)` → 取图片 → VLM 看图回答

**需要修改的文件**:
- 无代码改动，仅需验证 MCP tool 注册正确、Pilot 能发现 `observe` / `hybrid_search`

**验证清单**:
- [ ] `observe` MCP tool 出现在 Pilot tool list
- [ ] Scene object detection 能触发 observe
- [ ] hybrid_search 返回含 `image_refs` 的 MemoryNode
- [ ] VLM 基于图片正确回答用户问题
- [ ] demo 在 Webots 上完整跑通

---

### TODO D7: 功能验证脚本

**依赖**: TODO D6

**说明**: 编写 `scripts/verify_observe_demo.py`，自动化验证 observe 管线各环节。

**验证步骤**:
1. Service Setup — MemoryService 初始化
2. Simulated Observe — 用人造图片 + 人造物体列表调 `observe`
3. Image Storage — 验证 `data/images/{node_id}/` 存在图片文件
4. Memory Node — 验证 node 含 `image_refs`、spatial 坐标、tags
5. Hybrid Search — `hybrid_search(scene_type, objects_present)` 能检索到观察结果
6. VLM QA — （可选）VLM 看图片后回答物体相关问题

**需要生成的脚本**:
- `services/memory/scripts/verify_observe_demo.py`

---

### 文件变更汇总

| 文件 | 类型 | 所属 TODO |
|------|------|----------|
| `memory_service/core/types.py` | 修改 | D1 |
| `memory_service/storage/image_store.py` | **新增** | D2 |
| `memory_service/core/observe.py` | **新增** | D3, D4 |
| `memory_service/service.py` | 修改 | D5 |
| `capabilities/lib/memgraph/srv/Observe.srv` | **新增** | D5 |
| `capabilities/service/memory/observe.v1.toml` | **新增** | D5 |
| `package_manifest.yaml` | 修改 | D5 |
| `tests/test_image_store.py` | **新增** | D2 |
| `tests/test_observe.py` | **新增** | D3, D4, D5 |
| `scripts/verify_observe_demo.py` | **新增** | D7 |

### 依赖顺序

```
D1 (image_refs 字段)
  └─► D2 (ImageStore)
        └─► D3 (ObservePipeline)
              ├─► D4 (VLM summary)
              │     └─► D5 (MCP observe tool)
              │           └─► D6 (rbnx chat demo)
              │                 └─► D7 (verify script)
              └─► 可并行：D4 和 D5 部分逻辑可同时开发
```
