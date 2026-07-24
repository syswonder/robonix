# Android 日志系统设计要点

> 参考来源：Android AOSP `system/logging` / `system/core`（logd、liblog、logcat），
> [liblog README.protocol.md](https://android.googlesource.com/platform/system/logging/+show/refs/heads/main/liblog/README.protocol.md)，
> [logd README.property](https://android.googlesource.com/platform/system/core/+/a3465e2/logd/README.property)，
> [Logcat 命令行工具](https://developer.android.google.cn/tools/logcat)。

Robonix Scribe 借鉴 Android 日志系统的"统一入口 + tag + 级别"思路（不照搬实现）。
本文梳理 Android logd / logcat 中与 Scribe 设计相关的架构要点。

---

## 1. 架构总览

Android 日志系统以 **logd**（userspace daemon，Android 5.0+）为核心，
替代早期内核环形缓冲区方案。

```
App (android.util.Log) / Native (ALOGD)
        │
        ▼
    liblog.so  (__android_log_buf_write)
        │
        ▼
  /dev/socket/logdw  ──►  LogListener  ──►  LogBuffer (环形缓冲区, RAM)
                                                    │
  /dev/socket/logdr  ◄──  LogReader  ◄──────────────┘
        │
        ▼
     logcat / logcatd
```

| 组件 | 角色 |
|---|---|
| **logd** | 日志守护进程，管理所有缓冲区与 socket |
| **liblog** | 共享库，提供 C/C++ 读写接口，所有日志调用最终经 `__android_log_buf_write()` 发出 |
| **logcat** | 命令行工具，读取并展示日志 |
| **logcatd** | 持久化守护进程，将日志落盘到 `/data/misc/logd/` |

三个关键 socket（由 `logd.rc` 在启动时创建）：

| Socket | 类型 | 用途 |
|---|---|---|
| `/dev/socket/logdw` | dgram + passcred | LogListener — 所有进程写日志入口 |
| `/dev/socket/logdr` | seqpacket | LogReader — logcat 从此读取缓冲区内容 |
| `/dev/socket/logd` | stream | CommandListener — 接收控制命令（清空、改大小等） |

---

## 2. 统一入口

Android 中所有语言层（Java `Log.d()`、Native `ALOGD()`、SLog、EventLog）最终
都收敛到 **`liblog` → `__android_log_buf_write()` → 通过 `/dev/socket/logdw`
发往 logd** 这一条路径。不存在各组件各自 `print` / `logging` 的情况。

**Scribe 对应**：Scribe 同样定位为进程内 library 统一入口（trait Scribe +
Python robonix-api），但 v1 不做 daemon，直接内存调用分发到 sink。

**设计启示**：
- 统一入口保证格式一致、元数据完整、可全局过滤
- 单一路径使得日志的级别控制、过滤规则、输出目标可以在一处集中配置
- logd 作为独立进程可以存活于应用崩溃之后，收集 crash 上下文 — Scribe v1 不追求此特性，但预留了今后拆出 daemon 的空间

---

## 3. Tag（来源标识）

Android 中 **tag** 是一条日志的核心索引字段，直接标识消息来源组件：

- **文本日志**（main/system/radio/crash）：tag 为 null-terminated 字符串，如
  `"ActivityManager"`、`"MyApp"`
- **二进制/事件日志**（events）：tag 为 `int32_t` 整型 ID，由
  `/system/etc/event-log-tags` 映射到可读名称
- 显示格式：`<priority>/<tag>`，如 `D/MyApp`

**Scribe 对应**：Scribe 的 `tag` 取 `provider_id`（组件名），与 Android
语义一致，用于按来源过滤。磁盘格式中 tag 为 LogRecord 的核心字段。

**设计启示**：
- tag 必须是稳定、有意义、可过滤的短标识符
- 同一组件所有日志用同一 tag，不准用类名或临时字符串（Android 有此惯例）
- tag 本身不承载层级信息，扁平命名如 `scene_svc`、`atlas` 即可

---

## 4. 日志级别（Priority）

Android 定义 **7 个级别**，从低到高：

| 字符 | 级别 | 数值 | 含义 |
|---|---|---|---|
| V | Verbose | 2 | 最详细 |
| D | Debug | 3 | 调试信息 |
| I | Info | 4 | 一般信息（默认最低输出级别） |
| W | Warning | 5 | 警告 |
| E | Error | 6 | 错误 |
| F | Fatal | 7 | 致命错误 |
| S | Silent | — | 最高优先级，不输出任何日志（仅用于过滤规则"禁止一切"） |

**Scribe 对应**：Scribe v1 定义 Debug / Info / Warn / Error 四级，
对应 Android 的 D / I / W / E。未引入 Verbose（可后续追加）。无 Silent —
Scribe 的过滤由 `rbnx logs` 处理。

**开放问题**：四级别是否足够？（Verbose 对机器人实时调试可能有用）

### 多层级过滤

Android 的级别过滤在 **四个层次** 依次生效：

| 层次 | 机制 | 说明 |
|---|---|---|
| 编译期 | ProGuard / R8 剔除 | 发布版本可自动移除 `Log.d()` / `Log.v()` |
| 系统属性 | `log.tag.<TagName>` / `persist.log.tag.<TagName>` | 按 tag 设最低级别，系统级控制 |
| 应用进程 | `__android_log_set_minimum_priority()` | 进程内设定，默认 ≥ Info |
| 展示层 | `logcat` 过滤表达式 | 临时按需过滤 |

**设计启示**：Scribe v1 不需要全部四层，但至少需要：
- 编译期剔除（Rust `cfg(debug_assertions)` 或 feature flag）
- `rbnx logs` 运行时按 tag + 级别过滤

---

## 5. 多缓冲区设计

Android 将日志按**类别**分入多个独立的**环形缓冲区**，而非全部写入单一文件：

| Buffer ID | 名称 | 内容 | 说明 |
|---|---|---|---|
| 0 | main | 大多数应用日志 | 通用 |
| 1 | radio | 无线/电话 | 避免高频冲刷其他日志 |
| 2 | events | 系统诊断事件 | 二进制格式 |
| 3 | system | 系统级消息 | OS framework |
| 4 | crash | 崩溃日志 | 低容量、高价值 |
| 5 | stats | 日志统计 | 内部使用 |
| 6 | security | 安全事件 | ADB 审计等 |
| 7 | kernel | 内核日志 | 来自 `/proc/kmsg` |

**设计要点**：
- **为什么分 buffer**：防止高频日志冲掉关键低容量日志（如 radio 不倒灌 crash）
- **每个 buffer 独立大小**：默认 256 KB，可通过 `persist.logd.size.<buffer>` 分别配置
- **环形覆盖策略**：buffer 满时自动淘汰最旧条目，内存边界确定

**Scribe 对应**：Scribe v1 不做多 buffer — 按 tag 分文件是另一维度的隔离。
如果需要防止某一组件刷屏淹没其他组件，可以利用文件不交叉写入的特性，
但无法防止同一文件内旧日志被覆盖（文件无限追加，不做轮转）。

**设计启示**：
- "不同类别日志应有独立容量和生命周期" 是 Android 多 buffer 的核心思想
- 如果 Scribe 后续遇到容量问题，可按 tag 分组设置文件大小上限 + 轮转

---

## 6. 环形缓冲区存储

Android logd 在用户空间维护**结构化环形缓冲区**：

- 内部结构：`LogBuffer` 内是一个**双向链表容器**，每个节点是 `LogBufferElement`
- **固定总大小**：默认 256 KB/缓冲区，开发者选项可调至最大 16 MB
- **满时覆盖最旧条目** — 经典环形缓冲区逐出策略
- **纯内存存储（默认）**：断电丢失；可选持久化经 `logcatd` 写入
  `/data/misc/logd/`

**Scribe 对应**：Scribe v1 走磁盘文件（JSONL），不走内存缓冲区。
磁盘持久化为默认行为，适合机器人离线场景的事后分析。

**设计启示**：
- 内存环 + 可选磁盘持久化的分层思路值得参考
- 环形缓冲即"固定写入开销、固定内存占用"的保障 — Scribe 若不加轮转，
  文件无限增长会成为问题

---

## 7. Wire Protocol（传输协议）

Android liblog → logd 的 wire format（`README.protocol.md`）：

```
struct {
    android_log_header_t header;    // 固定头部
    union {
        struct {                    // 文本日志 (main/system/radio/crash)
            char prio;              // 1 字节优先级
            char tag[...];          // null-terminated tag
            char message[...];      // null-terminated 消息体
        } string;
        struct {                    // 二进制事件日志 (events/security)
            android_event_header_t event_header;
            android_event_*_t payload[...];
        } binary;
    };
};
```

**Header 字段**：
- `id` (uint8) — buffer ID
- `tid` (uint16) — 线程 ID
- `realtime` (log_time) — 启动以来的秒+纳秒

**最大 payload**：`LOGGER_ENTRY_MAX_PAYLOAD`（通常 4068 字节），
单条日志总长上限 `LOGGER_ENTRY_MAX_LEN`。

**Scribe 对应**：Scribe 无 wire protocol（进程内调用），
但磁盘格式为 JSONL（每行一个 JSON 对象），单行可容纳任意长文本
（由写文件性能和人类可读性约束）。

**设计启示**：
- Android 二进制 header 的设计目的：极小固定成本（header 仅 ~11 bytes），
  避免格式化的 CPU 开销；Scribe 用 JSONL 则侧重人类可解析和工具链兼容。
- Wire protocol 中 `tid` 是必要上下文 — Scribe 的 JSONL 亦可附加 `pid`/`cid`
  /`sid` 等扩展字段

---

## 8. 日志输出格式

Android logcat 通过 `-v <format>` 支持多种渲染格式：

| 格式 | 字段 | 示例 |
|---|---|---|
| **brief**（旧默认） | 优先级/tag(PID) | `D/MyApp(12345): msg` |
| **threadtime**（当前默认） | 日期 时间 PID TID 优先级 Tag: msg | `01-30 10:30:45.123 12345 6789 D MyApp: msg` |
| **time** | 日期 时间 优先级/tag(PID) | `01-30 10:30:45.123 D/MyApp(12345): msg` |
| **long** | 所有元数据字段，空行分隔 | 完整展开 |
| **raw** | 仅消息体 | `msg` |
| **thread** | 优先级(PID:TID) | `D(12345:6789) msg` |
| **process** | 优先级(PID) | `D(12345) msg` |
| **tag** | 优先级/tag | `D/MyApp: msg` |

还支持格式修饰符：`color`（按级别着色）、`usec`（微秒精度）、`epoch`（Unix
秒）、`UTC`、`year`、`zone` 等，可逗号组合：`-v threadtime,color,usec`。

**Scribe 对应**：
- Console 输出：logcat 风格单行文本 `MM-DD hh:mm:ss.xxx  L tag   msg  kv...`
- 磁盘输出：JSONL（每行一个 JSON 对象）
- `rbnx logs` 可类似 logcat 的 `-v` 参数支持多输出格式

---

## 9. 过滤表达式

logcat 过滤语法：

```
tag:priority tag:priority ...
```

- `adb logcat ActivityManager:I MyApp:D *:S`
  → 只看 ActivityManager（≥Info）和 MyApp（≥Debug），其余全部静默
- `*:W` → 所有 tag ≥ Warning
- `*:S` 作为"默认为禁止"的锚点，仅显式列出的 tag 有输出

**Scribe 对应**：`rbnx logs` 需要按 tag 和 level 过滤；logcat 的白名单 +
默认静默模式是一个优秀的 CLI 设计范式。

---

## 10. 持久化策略

Android 的日志持久化采用**分层策略**：

| 层级 | 存储 | 容量 | 生命周期 |
|---|---|---|---|
| 内核环形缓冲 | 内核内存 | 固定 | 重启丢失 |
| logd 环形缓冲 | 用户空间内存 | 256 KB～16 MB | 重启丢失 |
| logcatd 磁盘 | `/data/misc/logd/` | 受磁盘容量限制 | 持久 |

logcatd 由属性控制：
- `logd.logpersistd.enable` — 是否开启持久化
- `persist.logd.logpersistd` — 持久化开关（开发者选项）
- `persist.logd.logpersistd.buffer` — 持久化哪些 buffer

**设计启示**：
- Android 默认纯内存、可选磁盘的模式适合移动设备（Flash 写入寿命）。
- 机器人场景磁盘容量充裕，Scribe 默认磁盘持久化合理；
  后续可考虑引入内存环作为热缓冲区 + 异步批量刷盘，减少写放大。

---

## 11. 关键设计要点汇总（与 Scribe 对照）

| 设计要素 | Android logd/logcat | Robonix Scribe (v1) | 启示 |
|---|---|---|---|
| **架构模型** | 独立 daemon + library + CLI | 进程内 library + CLI | v1 免 daemon 复杂度，后续可拆分 |
| **统一入口** | liblog 单一路径收敛所有语言层 | trait Scribe 单一面部 | 核心思想一致 |
| **Tag** | 组件标识字符串，扁平命名 | provider_id，扁平命名 | 一致 |
| **级别** | 7 级 (V/D/I/W/E/F/S) | 4 级 (D/I/W/E) | 是否需要 Verbose 待定 |
| **级别过滤** | 编译期 + 系统属性 + 进程 + 展示 四层 | v1：编译期 + CLI 展示两层 | 可之后加系统属性层 |
| **缓冲区** | 7 个独立环形 buffer，各可独立设大小 | 按 tag 分文件，无容量上限 | 需要轮转策略防止文件无限增长 |
| **存储介质** | 默认 RAM 环形缓冲，可选磁盘持久化 | 默认磁盘 JSONL 持久化 | 场景差异决定策略 |
| **传输协议** | 二进制 header + typed payload，紧凑高效 | 无 wire（进程内），磁盘 JSONL | Scribe 用 JSONL 侧重可读性 |
| **输出格式** | 8 种格式动词 + 多种修饰符 | console logcat 风格 / 磁盘 JSONL | logcat `-v` 参数是 CLI 设计参考 |
| **过滤语法** | `tag:priority` 白名单 + `*:S` 静默锚点 | `rbnx logs` 按 tag/level 过滤 | logcat 语法可作为 CLI 参考 |
| **持久化控制** | 系统属性开关，按 buffer 独立控制 | 无开关（默认持久化） | — |

---

## 12. 参考

- [liblog README.protocol.md — Android 日志传输协议](https://android.googlesource.com/platform/system/logging/+show/refs/heads/main/liblog/README.protocol.md)
- [logd README.property — logd 系统属性说明](https://android.googlesource.com/platform/system/core/+/a3465e2/logd/README.property)
- [Logcat command-line tool — Android Developers](https://developer.android.google.cn/tools/logcat)
- [Robonix Scribe 功能描述 (Issue #63)](https://github.com/syswonder/robonix/issues/63)
- [Robonix chronos — 统一时钟 (Issue #62)](https://github.com/syswonder/robonix/issues/62)
