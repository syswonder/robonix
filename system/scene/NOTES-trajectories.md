# scene 的执行轨迹：现状与改造后

2026-09-20。**上半部分是实测**（Webots，TIAGo，34 个感知物体的房间）；
**下半部分是设计**，还没实现。两者严格分开，不要混着讲。

测量口径：`rbnx ask --json <中文指令>` → pilot → executor(RTDL) → 各能力，
记录每一轮的 plan、能力调用、返回、墙钟时间。原始记录在
`/tmp/baseline-grounding/*.jsonl`。

---

## 一、现状（实测）

### A. 无歧义 —— 现在就能走通

**指令**：「去那盆绿植旁边」（场景里当时只有 1 盆）

```
r0  list_objects   {}                                    ← 921 B
r1  goal_near      {"object_id":"scene.object.potted_plant_001"}
r2  navigate       {goal: pose(1.05, -0.21, yaw 0.77)}
```
**3 轮，23 秒，6 次能力调用 → 到达，距规划靠近点 0.16 m。**

这条链是好的。要做的不是替换它。

### B. 有歧义 —— 它不问，它挑

**指令**：「去那把椅子那里」（场景里当时有 **8 把椅子**）

```
r0  list_objects   {}                                    ← 13,933 B
r1  goal_near      {"object_id":"scene.object.chair_014"}   ← 8 选 1，无依据
r2  navigate       {goal: pose(-1.50, -1.13, yaw -2.38)}
```
**3 轮，35 秒 → 报告「已到达椅子 scene.object.chair_014 附近」。**

机器人真的开过去了。**全程没有任何一步表达过"我不确定是哪一把"。**

### C. 找不到 —— 它会去找，但很贵，最后还是猜

**指令**：「去那幅画那里」（当时视野内没有，全场景有 6 个画框）

```
r0  list_objects        {}                               ← 11,459 B
r1  snapshot            {}                               ← 拍一张看看
r2  read_capability_doc {"provider_id":"explore"}        ← 现读 explore 的文档
r3  explore  {"area_hint":"this room", max_speed 0.15, timeout_s 180}
                                                          ← 探索，撞到 180 s 上限
r4  list_objects        {}
r7  goal_near           {"object_id":"scene.object.picture_frame_001"}
r8  navigate            {goal: pose(-0.05, 0.52, yaw 0.82)}
```
**7 轮，探针 240 秒超时被切断。**

**这条要讲清楚**：它不是卡死，它在干实事——自己找到 explore 能力、读了文档、
去探索了。这是好的自主性。问题是 ① 代价是 3 分钟；② 探索完之后，面对 6 个
画框，**仍然是无依据地挑了一个**。

### 三条实测结论

| | 数字 |
| --- | --- |
| `list_objects` 载荷随物体数 | 2 个物体 **921 B** → 34 个物体 **13,933 B**（**15×**），一个小房间 |
| 有歧义时询问用户的次数 | **0**（接口里没有这个返回值，模型只能挑或继续找） |
| object_id 在移动中的稳定性 | 静止 60 s 零变化；探索期间 `potted_plant_001` 消失，计数器走到 `_005+` |

---

## 二、改造后（设计，未实现）

### 同样三条指令

**A. 无歧义**——保持 3 轮，但第一轮换掉：

```
r0  find   {text:"绿植", k:5}
           → [{id:potted_plant_001, score:.81, region:客厅, views:2}]
             margin=0.44  → unique
r1  go_to  {query_id:"q-7f3a", chosen:"scene.object.potted_plant_001"}
           → 内部：接近位姿 → navigate → arrived
```
载荷从 921 B 降到**一条候选**。轮数不变，但 LLM 不再读全表。

**B. 有歧义**——从「挑一个」变成「问一句」：

```
r0  find   {text:"椅子", k:5}
           → 8 命中，前 2 名 margin=0.03（低于校准阈值）
           → needs_clarification
              question: "餐桌旁那把还是窗边那把？"       ← 按区分度最高的属性算出来的
              candidates: [{id:chair_003, region:餐厅, view:/api/objects/.../0.jpg},
                           {id:chair_011, region:客厅, view:...}]
r1  (用户回答 / webui 点选)
r2  go_to  {query_id:"q-91c2", chosen:"scene.object.chair_003"}
           → arrived
```
关键不是"多了一轮"，是**那一轮是和人确认，而不是掷骰子**。

**C. 找不到**——从「先探索 3 分钟再猜」变成「先说清楚现状」：

```
r0  find   {text:"画", k:5}
           → 6 命中，但全部 last_seen > 5 min 且不在当前视野
           → needs_clarification
              question: "有 6 幅画，都不在眼前。要我去客厅那两幅，还是先
                        整屋找一遍？"
              candidates: [... 带缩略图和"20 分钟前在卧室见过"]
```
探索仍然可以发生——但**是用户选的，不是模型替用户决定花 3 分钟**。

### 对照

| | 现在（实测） | 改造后（设计） |
| --- | --- | --- |
| 给 LLM 的场景载荷 | 全表，34 物体 13.9 KB | ≤k 条候选，约 300 B |
| 歧义处理 | 无依据挑一个 | 校准过的预测集；非单元素就问 |
| 「不确定」能否表达 | **接口里没有** | `needs_clarification` 是一等返回值 |
| 问题内容 | — | 按候选间**区分度最高的属性**算出来 |
| 二次确认 | — | 带该物体的多角度实拍图 |
| 陈旧物体 | 和当前可见的混在一起 | 候选里显式带 `last_seen` |
| 确认过的 id 失效 | 无处理 | 别名表解析到合并后的 id |

### 数字上的预期（有文献依据，但**我们自己还没测**）

- EmbodiedRAG：只检索任务相关子图，**token 降约 90%**、规划时间降至多 70%。
- Transcrib3D：结构化文本 + LLM，**Sr3D 98.4% / Nr3D 70.2%**。
- 我们的场景（「沙发旁那盆绿植」这类）离 Sr3D 比离 Nr3D 近。

**这些是别人的数，不是我们的。** 我们自己的数要在 ReferIt3D 上跑出来才算。

---

## 三、讲的时候建议说清楚的边界

1. 第一部分每个数字都是这台机器上跑出来的；第二部分一行代码都还没有。
2. C 那条不是"模型卡住了"，是"模型很努力但方向没人管"。这个区别重要——
   它说明问题不在模型能力，在**接口没给它表达不确定的方式**。
3. 「改造后」的预期数字引自文献，不是我们的实测。
