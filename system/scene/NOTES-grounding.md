# scene：自然语言指代 → 对的那个物体

2026-09-20 立。这份文档记录需求、已查到的先例、以及和现有代码的接口。
问题本身一句话说完：**用户说「去那盆绿植」，scene 要找到对的那一盆；
找不准时要问，而不是猜。**

## 一、要求（来自 2026-09-19 夜 ~ 20 日凌晨的讨论）

### 能力
1. **scene 直接提供「导航到某个东西」**，入参是名字/描述，不是位姿。
   大模型不该再搬运导航参数。**仅在导航能力在线时生效。**
2. **`goal_near` 不改名，弃用**。它是已发布 contract（`.v1.toml` + `.srv`
   + docs 三处 + CI 场景 + manifest），改名会打断消费者。新的高层能力拿
   好名字，它降级成底层逃生口。若仍要改名，诚实的名字是 `approach_pose`
   ——它返回的就是一个接近位姿，而 `goal_near` 从没说这件事。
3. **定名**：检索 = `robonix/system/scene/find`，导航 =
   `robonix/system/scene/go_to`。
4. **`list_objects` 费 token**。物体一多，把整个注册表倒给模型不可行。
   它保留，但不该再是默认路径。

### 交互
5. **不确定时让 pilot 去问用户**，并支持**二次确认**，而不是随便选一个。
6. **确认要带证据**：文字 id 对人没有意义，得给图。
7. 目前客户端**收不到图**——见「四、协议缺口」，缺的比图片深。

### UI
8. webui 要有**独立的 objects 视图和 regions 视图**（不是只有右侧 dock）。
9. 每个物体**留图**，**多角度**。
10. objects 视图里要能看到每个物体的**点云**和**图片**。
11. 3D 视图里放**真的 URDF 模型**（参考 robonix client 的 vitals 界面）。

### 方法
12. **先查先例，不重复造轮子。**
13. **优先用现成数据集**做校准和评测，不要从零攒。
14. **复用已有代码。**
15. **空间关系 scene 一直在维护**——不要在查询时重算已经有的东西。
16. **先测现状**：RTDL + scene 现在能做到什么程度，没测过。
    先量出基线再动手，否则不知道在改进什么。

## 二、先例（已查，2026-09-20）

| 工作 | 解决我们哪个问题 | 拿它的什么 |
| --- | --- | --- |
| **OVSG** (CoRL'23) | 自由文本 → 场景图里的实体 | **星形查询图**（中心=目标，外围=锚点）；**按类型分别编码、跨类型距离为 ∞**；**两段式：先提候选，再用局部子图重排** |
| **Transcrib3D** (2024) | 同上，且有公开代码 | **把场景转写成结构化文本行**再给 LLM；**先筛候选**再推理。Sr3D 98.4% / Nr3D 70.2% |
| **EmbodiedRAG** (2024) | token 成本 | 只检索任务相关子图：**token 降约 90%**、规划时间降至多 70% |
| **SayPlan** (CoRL'23) | token 成本的另一半 | 利用**层级**：在折叠图上语义搜索，只展开相关子图。对应我们的 region → object |
| **KnowNo** (CoRL'23) | 「什么叫不确定」 | **保形预测**：校准出一个预测集，**非单元素就问人**。阈值是推导的，不是拍的 |
| **ReferIt3D** (ECCV'20) | 评测与校准数据 | **Nr3D** 41,503 条人写指代 / **Sr3D** 83,572 条模板指代；按**同类干扰物数**分 Easy(≤1)/Hard(≥2)，另分 ViewDep/ViewIndep |

**OVSG 的仓库不能直接用**：绑死 OVIR-3D/Detic，27 个 commit，README 写着
"完整版正在整理"，**没有 license**。这里的"不重复造轮子"是**照着已发表的
设计做**，不是找个库装上——这个方向目前没有可复用的库。贵的那部分
（开放词表感知）我们已经有了：ConceptGraphs 和 OVIR-3D 同族。

## 三、和现有代码的接口

### 已经有的
| 东西 | 在哪 | 备注 |
| --- | --- | --- |
| 物体当前态 + **caption embedding** | `persistence.py`（milvus-lite） | `cls` / `confidence` / `last_seen` / `x,y,z` 都是**可过滤标量字段**，和向量同表，一次 search 就能 filter + rank |
| **场景图** | `scene_graph/store.py`、`types.py` | `SceneGraphEdge{source_id, target_id, relation, confidence, method, stale_rounds}` |
| 语义边（LLM，30s 一轮） | `scene_graph/prompts.py` | `near` / `on_top_of` / `under` / `inside` / `contains` |
| 几何边（3 Hz） | `scene_graph/geometric_loop.py` | `reachable_by` |
| region 多边形 | `annotations.py` | 「在客厅」= 点在多边形内，**精确** |
| 每物体点云 | `export_3d_snapshot()` | 已带 `points` + `point_colors`，`_rerun_tick` 在用 |
| 2D 裁剪 | `ingest/perception_concept_graphs.py` | YOLO 出 `xyxy`、**已为 CLIP 裁过**，存一张不需要新管线 |

### 对着 Sr3D 五类的缺口

| Sr3D 类 | 占比 | 状态 |
| --- | --- | --- |
| horizontal（closest/farthest/within reach/around） | **81%** | 一半有：`near`≈around，`reachable_by`=within reach；**closest/farthest 是最高级，不可能是边** |
| between | 8% | ❌ **三元**，我们的边是二元 |
| allocentric（left/right/front/back） | 5% | ❌ 需参照物朝向；`yaw` 存了但没做成边 |
| vertical（on/above/below） | 4% | ✅ |
| support | 2% | ✅（≈`on_top_of`） |

**注定不能是边的原因**：最高级依赖候选集（由查询决定）；`between` 是三元；
allocentric 预先算等于 O(n²) 条方向边而只有被提到的那对有用；视角相关依赖
机器人**此刻**位姿，每 tick 都在变。

**所以不需要代码生成。** 查询时要算的是一个**小的固定集合**，给 LLM 一个
**谓词库让它选**，而不是一个解释器让它写码——后者在机器人服务里既是可靠性
面也是安全面，而我们本来不必付。

```
维护的边   → near / on_top_of / under / inside / contains / reachable_by
查询时谓词 → closest / farthest / between
             / allocentric(left,right,front,back)   ← 以参照物 yaw 为基准
             / view_relative(left,right,front,back) ← 以机器人位姿为基准
```

### 维护带来的一个义务
图是持续维护的，所以能回答**当前看不见的东西**（边还在，带 `last_seen` /
`stale_rounds`）。Transcrib3D 面对静态扫描，没有这个问题也没有这个能力。
义务是：**`find` 必须把陈旧度摆进候选里**，而不是假装一切都是当下的。
「20 分钟前在卧室见过」正是需要二次确认的典型情形。

## 四、协议缺口 —— TODO，本轮不做

**2026-09-20 决定：用户交互/选择、以及图片回传这两块先挂起。**
原因是它跨 pilot + liaison + client 三边，不在 scene 内，也不在这两个 PR 的
解耦范围里。scene 侧先把能自洽的部分做完——`find` 照常返回
`needs_clarification` + 候选 + 图 URL，在协议补上之前，这个结果**至少能在
webui 里被人看见和回答**。下面是缺口的现状记录，供补协议时用。

`PilotEvent` 七种：`text_chunk` / `plan` / `batch_result` / `status` /
`final_text` / `node_state` / `task_state`。
`VoiceEvent` 十一种：会话/录音/ASR/说话人/TTS/结束/错误 + 包装的 PilotEvent。

**没有任何字段能装图片；更要命的是没有「我在问你、这是选项、我在等」这种
事件**，而且 `SESSION_DONE` 意味着一回合就结束，没有把答案送回同一轮的路。

要做二次确认，按深度缺三样：

1. **问答协议**——「本轮未完、需要答复」的事件 + 把答复送回同 session 的路。
2. **候选的结构化表达**——选项，不是散文。
3. **图片**——最简单的一层：**不要把字节塞进事件**。scene 本来就有 HTTP
   （已在服务 `/api/maps/{id}/preview`），事件只带 URL + 短说明，客户端自取。
   代价：客户端要能连到 scene 的 HTTP 口；而 #245 合并后 scene **默认只绑
   回环**，这需要显式设 `SCENE_WEB_HOST`——那应是主动决定，不是意外。

**没有屏幕的客户端**（纯语音）看不了缩略图，所以 `needs_clarification`
要同时带**一句能念出来的问题**（= 算出的区分度最高属性，「客厅那盆还是
卧室那盆？」）和**可选的可视候选**。同一个载荷，客户端能渲染什么渲染什么。

## 五、计划

先量基线，再动手：

0. **测现状**——RTDL + scene 现在对自然语言指代能做到什么程度。（要求 16）
1. **多角度截图落盘** + HTTP 路由。（要求 9，解锁 6）
2. **objects 视图**：网格 + 详情（胶片条 / 点云 / 属性与关系）。（要求 8、10）
3. **`find`**：星形查询图 + 分类型匹配 + 两段式重排 + 预测集。（要求 1、4）
4. **`go_to`**：resolve → 接近位姿 → navigate → 失败换候选 → 回写记忆。
5. URDF。（要求 11）

**挂起（TODO）**：要求 5、6、7 所依赖的问答协议与图片回传——见第四节。
`find` 该返回的 `needs_clarification` 照常返回，只是暂时只有 webui 能消费。

**评测**：ReferIt3D 的标注是开放的，底下的 ScanNet 扫描**要签使用条款**
——那一步得由本人走。在数据到位前，先按 Sr3D 的模板关系在 Webots 场景里
造小样本自测逻辑，同时把加载和评测协议按 ReferIt3D 实现，数据一到直接跑。

**两条必须说清的话**：
- ScanNet 是干净分割，我们的感知不是。**在它上面测出来的是定位逻辑，不是
  端到端**，而且会系统性高估真实表现。端到端还得在 Webots 里单独测。
- Nr3D 评测必须只取 `mentions_target_class=True` 且 `correct_guess=True`
  的语句，否则是在测人类自己都猜错的题。

## 六、分支归属

截图、objects 视图、URDF 是 UI，走 `feat/scene-rerun-viewer` (#248)。
**`find` / `go_to` 两者都不是**——它们是新的 scene 能力，既不是 dualmap 也
不是 UI。按"严格解耦"的约定，塞进任何一个现有 PR 都是不对的，**应当单开
分支**。待确认。
