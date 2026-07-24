# 十五、Scribe

# 十五、Scribe

Github Issue：https://github\.com/syswonder/robonix/issues/63

https://robonix\.syswonder\.org/interface\-catalog/service/memory\.html

https://github\.com/syswonder/robonix/tree/dev/services/memsearch

## 1\. 功能描述

Scribe = Log \+ Mem \+ 基于记忆形成的技能

Log 作为 System 接收信息 \- Scribe 的基础模块
Mem 作为 Service 对信息整合，对其他模块提供信息 \- Scribe 的拓展模块

Scribe Log 是全系统 **统一日志入口**：各组件不再各自 `print` 或 `logging`。一条日志由时间、级别、来源（tag）、内容构成；同时输出到 console（人读）与文件（按组件分文件）。`tag` 直接取组件名或 `provider_id`，便于按来源过滤。设计借鉴 Android 日志系统的统一入口加 tag 加级别（参考 logcat / logd 思路），不照搬其实现。

Scribe Mem 是系统 **统一记忆系统**：将日志信息整合成知识图谱形式的记忆，提供给其他模块查询使用。设计借鉴人类记忆系统，包含记住、提炼、遗忘等功能，支持多模态信息（文本、图像、视频等），并且支持标签化记忆以便于检索过滤。与语义地图不同，Scribe Mem 更侧重于长期记忆和知识积累，而语义地图更侧重于环境感知和即时决策。

## 2\. 维护的数据、对应的解释和描述

### Log 数据结构

```Rust
enum Level { Debug, Info, Warn, Error }

struct LogRecord {
    ts:    u64,        // 纳秒时间戳，后续接 chronos now() 统一时钟
    level: Level,
    tag:   String,     // 组件名 / provider_id
    msg:   String,     // 日志正文
}
```

|字段|说明|
|---|---|
|`ts`|产生时刻；过渡期用 `time.time()`，后续切 chronos `now()`|
|`level`|Debug / Info / Warn / Error|
|`tag`|来源标识，初步规定填写 `provider_id`，作过滤维度|
|`msg`|日志正文|

### Mem 数据结构

Scribe Mem 构建在 Log 之上，将分散的日志记录整合为结构化的**因果知识图谱 \(CKG\)**。

**记忆节点 MemoryNode** — CKG 基本单元，按检索优先级组织：

|优先级|字段|说明|存储|
|---|---|---|---|
|★★★|tags|标签集，检索第一入口，驱动倒排索引 O\(1\) 过滤|Redis|
|★★☆|embedding|多模态融合向量 d=768，标签过滤后精排|FAISS|
|★☆☆|summary<br>|LLM 生成的事件一句话摘要|图DB|
|—|node\_id|u32, 短期0\~999 / 长期1000\+ / 技能 / 固定|图DB|
|—|spatial\_data|物体坐标 \[\{obj, x, y, z\}\]|图DB|
|—|causal\_chain|u32\[\], 因果依赖父节点|图DB|
|—|weight|综合质量评分\(成功率×时效性×频率\)|图DB|
|—|timestamp|u64, chronos 规范时间戳|图DB|

**标签集 TagSet** — 四维度标签，独立倒排索引：

|维度|字段|索引结构|示例|
|---|---|---|---|
|空间|scene\_type, objects\_present|倒排索引|kitchen, \["cup","table"\]|
|行为|action\_type, success|倒排\+位图|grasp, true|
|认知|task\_type, difficulty|倒排索引|craft, medium/工具调用次数|
|优化|frequency, last\_access, quality\_score|B\+树|命中计数/时间戳/评分|

**技能模板 SkillTemplate** — 从 N 个相似成功经验提炼的可执行模式：goal\_template \+ causal\_template\(步骤因果链\) \+ pre/post\_condition \+ constraints \+ success\_rate \+ version。

## 3\. 功能接口（输入输出、给谁来调用）

### Log 对外接口

Scribe 不是 capability，无 `contract_id` 与 ROS IDL：`log()` 是进程内 library（Rust trait，Python 侧 robonix\-api 暴露同语义 logger），`rbnx logs` 是 CLI；接口由 trait 签名与 CLI 定义。若日后需跨进程远程查询日志，可另设计 contract `robonix/system/scribe/query`（gRPC，`scribe/srv/Query.srv`）。

```Rust
trait Scribe {
    fn log(&self, level: Level, tag: String, msg: &str);
    // 另提供便捷封装 debug / info / warn / error(tag, msg)
}
```

|接口|输入|输出|transport|谁调用|
|---|---|---|---|---|
|`log(level, tag, msg)`|级别、tag、内容|—|进程内 library|所有组件（Rust 与 Python）|
|`rbnx logs`|过滤条件（tag 或级别）|日志流|CLI|开发者 / 运维|

### Log 依赖的能力接口

无能力依赖。时间戳取 chronos `now()`（进程内 library）。

### Mem 对外接口

https://robonix\.syswonder\.org/interface\-catalog/service/memory\.html

Mem 是 system service，以 `contract_id` \+ ROS IDL \+ gRPC 暴露。

|contract\_id|用途|调用者|分层|
|---|---|---|---|
|`robonix/service/memory/search`|标签过滤→向量精排→因果过滤三阶段检索|Pilot \(规划前显式调用\)|基础<br>|
|`robonix/service/memory/remember`|写入记忆节点（标签先行入 数据库，再写入 CKG\+FAISS）|Pilot \(规划后默认调用\)|基础<br>|
|`robonix/service/memory/refine`<br>|聚类→抽象公共模式→验证→存储 Skill|条件触发|核心|
|`robonix/service/memory/forget `|ForgetRisk 评分→分级处理\(降权/归档/删除\)|条件触发 / Pilot显式触发<br>|核心|
|`robonix/service/memory/list_skills`|获取已提炼技能列表，注册为 LLM function tools|Pilot \(启动时\)|核心|
|`robonix/service/memory/search_history`|时间\+空间\+任务三维历史回溯查询|Pilot调用|拓展|
|`robonix/service/memory/cron_*`|定时任务管理\(触发/列表\)|Pilot调用|拓展|

**关键设计**：search 返回 MemoryNode\[\] 注入 LLM prompt；list\_skills 返回 SkillTemplate\[\] 注册到 Atlas 为 MCP tools；remember 标签先于向量写入确保检索立即可用。

### Mem 依赖的能力接口

|contract\_id|用途|
|---|---|
|`robonix/system/scene/list_objects`|获取物体坐标\+标签辅助记忆标签提取|
|atlas \(Query\)|发现可用能力，供 Skill 模板引用|
|chronos `now()` \(进程内\)|统一时间戳|
|scribe log \(进程内\)|记忆写入同时产生日志记录|

## 4\. 执行图

### Log 执行图

```Plaintext
flowchart TB
    subgraph scribe["scribe"]
        direction TB
        LogRecord["LogRecord"]
        console["console"]
        files["files (per tag)"]
        
        LogRecord --> console
        LogRecord --> files
    end

    components["components"] -->|"log(level, tag, msg)"| LogRecord
    developer["developer"] -->|"rbnx logs"| files
```

各组件经 `log(level, tag, msg)` 调用 scribe，scribe 组装 `LogRecord` 后同时格式化输出到 console 并按 tag 写入 `logs/<tag>.log`。开发者经 `rbnx logs` 按 tag 与级别读文件过滤查看。

### Mem 执行图

**基础链路 — 记住\+检索**：

```Plaintext
flowchart TB
    subgraph Pilot["Pilot"]
        plan["LLM 规划"]
        done["执行完成"]
    end

    subgraph Mem["Scribe Mem"]
        search["search()<br/>标签过滤→向量精排<br/>→因果过滤"]
        remember["remember()<br/>标签提取→标签先行入Redis<br/>→嵌入入FAISS→写入CKG"]
        skills["list_skills()<br/>返回已提炼技能"]
    end

    Scene["Scene<br/>list_objects()"]

    Atlas["Atlas<br/>注册 MCP tools"]

    plan -->|"query + tags"| search
    search -->|"MemoryNode[] 注入 prompt"| plan
    done -->|"call + result"| remember
    Scene -->|"物体坐标+VLM描述"| remember
    remember -->|"node_id"| done
    skills -->|"SkillTemplate[]"| Atlas
    Atlas -->|"可用工具列表"| plan

    style Mem fill:#e1f5fe
    style Pilot fill:#fff3e0
    style Scene fill:#e8f5e9
    style Atlas fill:#fce4ec
```

**核心链路 — 提炼\+遗忘**：

```Plaintext
flowchart TB
    Mem["Scribe Mem"]

    subgraph Refine["提炼 (refine)"]
        r1["同 task_type 聚类"]
        r2["抽象公共模式<br/>前置交集+最长公共子序列+后置交集"]
        r3["模拟回放验证<br/>成功率≥80%"]
        r1 --> r2 --> r3
    end

    subgraph Forget["遗忘 (forget)"]
        f1["ForgetRisk 评分<br/>频率+时效+质量+重复度"]
        f2["低<0.3 保留"]
        f3["中0.3~0.7 降权<br/>weight×0.5"]
        f4["高≥0.7 归档<br/>移除索引,30天后删除"]
        f1 --> f2
        f1 --> f3
        f1 --> f4
    end

    long["长期记忆节点<br/>(CKG 1000+)"]
    skill["技能节点<br/>(SkillStore)"]
    Atlas["Atlas<br/>注册/弃用 MCP tool"]
    audit["审计日志<br/>(scribe log)"]

    long -->|"N≥3 同类成功经验"| r1
    r3 -->|"写入 Skill + 版本管理"| skill
    skill -->|"新 Skill 注册"| Atlas
    skill -->|"成功率连续下降"| Atlas

    long -->|"定时扫描"| f1
    f3 -->|"检索排序下降"| long
    f4 --> audit
    f1 -.->|"保护: 固定/教训/Skill源/30天命中"| long

    style Mem fill:#e1f5fe
    style Refine fill:#c8e6c9
    style Forget fill:#ffecb3
    style Atlas fill:#fce4ec
```

详细执行图见 \[\[具身记忆\-设计\]\] §4。

## 5\. 源代码组织

### Log 相关代码组织

`system/scribe/`（rust crate）加 Python 侧 logger（组件含 Python primitive 与 service）。

```Bash
system/scribe/
  Cargo.toml
  src/
    lib.rs      # trait Scribe，Level / LogRecord，便捷封装 debug / info / warn / error
    sink.rs     # 输出后端：console formatter 与 per-tag 文件写入，轮转可选
    format.rs   # 单行格式：时间 级别 [tag] msg
# Python primitive / service 侧：robonix-api 暴露同语义 logger
# CLI：rbnx logs，位于 tools/rbnx，按 tag 与级别读文件过滤
```

### Mem 相关代码组织

Mem 作为 scribe 的拓展模块，代码放在 `services/memory/`（Python service）：

https://github\.com/syswonder/robonix/tree/dev/services/memsearch

```Plaintext
services/memory/
  memory_service/
    ├── service.py            # atlas 注册, gRPC server
    ├── mcp_tools.py          # contract handlers
    ├── core/
    │   ├── remember.py       # 记住管线
    │   ├── retrieve.py       # 检索管线(标签→向量→因果)
    │   ├── refine.py         # 提炼管线(聚类→抽象→验证)
    │   ├── forget.py         # 遗忘管线(评分→分级→审计)
    │   ├── history.py        # 历史回溯查询
    │   └── cron.py           # 定时任务调度
    ├── storage/
    │   ├── graph_store.py    # CKG (NetworkX/Neo4j)
    │   ├── vector_store.py   # FAISS
    │   ├── tag_index.py      # Redis 倒排+位图+B+树
    │   └── skill_store.py    # Skill 模板持久化
    ├── background/
    │   ├── refine_loop.py    # 后台提炼 cron
    │   └── forget_loop.py    # 后台遗忘 cron
    └── dashboard/            # Web 管理面板
        ├── web_server.py     # FastAPI
        └── graph_viz.py      # CKG 可视化
```

**分层开发**：Phase1 基础\(Remember\+Search\) → Phase2 核心\(Refine\+Forget\+Skills\) → Phase3 拓展\(Cron\+History\+Dashboard\)。

## 6\. 日志格式

磁盘格式（每行一个 JSON 对象）：

```JSON
{"ts":1765432100123456789,"lvl":"I","tag":"scene_svc","msg":"object registered","sid":"sess-42","pid":"plan-7","cid":"plan-7:3","kv":{"object_id":"scene.object.cup_003"}}
```

console 端\(以及 rbnx logs 的默认渲染\)是 logcat 风格单行文本:

```JSON
06-11 10:23:45.123  I scene_svc   object registered  object_id=scene.object.cup_003 plan=plan-7
```

