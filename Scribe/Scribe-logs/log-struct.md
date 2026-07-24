# Scribe 内部结构设计

## 1. 约束

Scribe 对外只暴露**唯一接口** `log(level, tag, msg)`。调用方不能、也不需要
调用任何初始化函数 —— 首条 `log()` 触发内部懒初始化，后续调用直接写入。

此外，Scribe 通过 `#[ctor]` 在 `main()` 之前自动注册为 Rust 标准 `log` facade
的 backend —— 即使组件完全不调用 `robe::info(...)`，仅使用 `log::info!("msg")`
也能自动写入 `_default.log`。

```
调用方代码中只出现：
  // 方式 1: Scribe 原生 API（显式 tag）
  scribe::log(Level::Info, "atlas", "control plane ready");
  scribe::info("atlas", "control plane ready");

  // 方式 2: 标准 log 宏（tag = module_path!()，零改动迁移）
  log::info!("robonix-atlas starting (control plane)");

不存在:
  scribe::init(...)       // 禁止
  scribe::set_log_dir(...) // 禁止
  scribe::flush()          // 禁止
  scribe::rotate()         // 禁止
```

## 2. 对外接口（两条等价路径）

### 路径 A：Scribe 原生 API（显式 tag）

```rust
// Rust — 自由函数，非 trait 方法（避免调用方持有 handle）
fn log(level: Level, tag: &str, msg: &str);

// 便捷封装（内部调 log()）
fn debug(tag: &str, msg: &str);
fn info(tag: &str, msg: &str);
fn warn(tag: &str, msg: &str);
fn error(tag: &str, msg: &str);
```

### 路径 B：标准 `log` 宏（tag 自动推导）

Scribe 实现了 `log::Log` trait，注册为 Rust 标准 `log` facade 的 backend。
调用方只需使用已存在的 `log::info!("msg")` 宏，tag 自动取 `record.target()`
（即 `module_path!()`，如 `"robonix_atlas"`）。

```rust
// 无需任何 scribe 调用，log 宏直接路由到 Scribe
// (backed by #[ctor] auto-registration before main())
log::info!("robonix-atlas starting");
log::warn!("no capabilities dir configured");
log::error!("port bind failed: {e:#}");
```

Backend 注册由 `#[ctor]` 在 `main()` 之前自动完成，无需任何显式调用。
`log::info!("msg")` 的 tag = `"_default"`，所有此类日志聚合到 `_default.log`。

### tag 取值

| 路径 | tag 来源 | 目标文件 |
|------|---------|------|
| `scribe::info("atlas", ...)` | 显式传参 | `atlas.log` |
| `log::info!("msg")` | auto: `"_default"` | `_default.log`（所有模块聚合） |

**`ts` 格式**：JSON 中序列化为可读字符串 `"YYYY-MM-DD HH:MM:SS.nnnnnnnnn"`（29 字符，本地时间），
反序列化兼容旧整数格式。内部存储仍为 `u64` 纳秒，保证排序效率。

**`tag` 取值**：直接使用 `provider_id`（如 `atlas`、`primitive_tiago_chassis`、
`service_memory`），与现有日志文件命名完全对应。

Python 侧 `robonix_api` 暴露同语义的模块级函数：

```python
from robonix_api import scribe_logger

scribe_logger.log(Level.INFO, "atlas", "control plane ready")
scribe_logger.info("atlas", "control plane ready")
```

## 3. 内部状态（调用方不可见）

所有状态为 crate 内部 `static`，对调用方透明。

```
┌─────────────────────────────────────────────────┐
│                  static 内部状态                  │
│                                                   │
│  LOG_DIR: LazyLock<PathBuf>                       │
│    │  env SCRIBE_LOG_DIR  ?  else  "./logs"       │
│    │                                              │
│  CONSOLE_MIN: LazyLock<Level>                     │
│    │  $SCRIBE_CONSOLE_LEVEL, default Warn         │
│    │  debug < info < warn < error                 │
│    │                                              │
│  FILE_MIN: LazyLock<Level>                        │
│    │  $SCRIBE_FILE_LEVEL, default Info            │
│    │                                              │
│  FILE_SINK: LazyLock<FileSink>                    │
│    │  key = tag → {tag}.log 追加句柄               │
│    │  Mutex<HashMap<String, File>> 保护            │
│    │                                              │
│  #[ctor::ctor] auto_init()                        │
│    │  log::set_logger(&BACKEND)                   │
│    │  在 main() 前执行，best-effort               │
│    │  log::info!("msg") → _default.log            │
│    │                                              │
│  CONSOLE: stderr (不缓存，每次 write_all)          │
└─────────────────────────────────────────────────┘
```

**不存在任何** `AtomicBool initialized` 或显式两段式 init 标记 ——
`LazyLock` 的首次解引用即为初始化点，调用方无感知。

## 4. 数据流

两条入口，汇入同一 pipeline：

```
┌─ 路径 A: scribe::info("atlas", "msg")
│       │ tag 由调用方显式传入 → atlas.log
│       │
├─ 路径 B: log::info!("msg")
│       │ #[ctor] 已注册 backend，tag = "_default"
│       │ log 宏 → LogBackend::log() → enabled() 预检查 → log()
│       │ → _default.log（所有模块聚合）
│       │
└───────┬─────────────────────────────────────
        ▼
  log(level, tag, msg)
  │
  ├─ 0. 级别过滤（per-sink 阈值）
  │      console_ok = level >= CONSOLE_MIN      // 默认: Warn+
  │      file_ok    = level >= FILE_MIN         // 默认: Info+
  │      if !console_ok && !file_ok → return    // Debug 默认被两端丢弃
  │
  ├─ 1. 取当前时间戳 ts = system_time_ns()
  │
  ├─ 2. 构造 LogRecord { ts, level, tag, msg }
  │
  ├─ 3. 若 console_ok: write_all(stderr, logcat_line)
  │
  └─ 4. 若 file_ok: write_all({tag}.log, json_line)
        └─ flush(file)
```

单次 `log()` 调用内完成 console 写 + 文件写 + flush，不存在后台异步线程或缓冲队列。

## 5. 文件命名

文件位于 `LOG_DIR/{tag}.log`，`tag` 即为文件名 stem。

| 来源 | tag | 日志文件 |
|------|-----|---------|
| `log` 宏（默认路由） | `_default` | `logs/_default.log` |
| atlas (scribe::info) | `atlas` | `logs/atlas.log` |
| executor | `executor` | `logs/executor.log` |
| pilot | `pilot` | `logs/pilot.log` |
| liaison | `liaison` | `logs/liaison.log` |
| scene (system pkg) | `system_scene` | `logs/system_scene.log` |
| tiago_chassis (primitive) | `primitive_tiago_chassis` | `logs/primitive_tiago_chassis.log` |
| memory (service) | `service_memory` | `logs/service_memory.log` |
| explore (skill) | `skill_explore` | `logs/skill_explore.log` |

这与 `deploy.rs` 当前 `log_path()` 的命名规则一致：
- 系统内置：`{name}.log`
- 包：`{component_kind}_{name}.log`

**`rbnx boot` 适配**：`rbnx boot` spawn 子进程前设置环境变量
`SCRIBE_LOG_DIR=<manifest_dir>/rbnx-boot/logs`，Scribe 内部读该变量决定
`LOG_DIR`。这样 Scribe 本身无部署概念，`rbnx boot` 保持对日志目录的控制权。

## 6. `rbnx boot` 侧的变化

`rbnx boot` 涉及两层进程 spawn：

| 层级 | 文件 | 责任 |
|------|------|------|
| boot → rbnx start | `deploy.rs` | 设置 `SCRIBE_LOG_DIR` 环境变量（不再创建 log 文件或重定向 stdout/stderr） |
| rbnx start → 包脚本 | `process.rs` | `Stdio::piped()` 捕获子进程 stdout/stderr，逐行转发到终端 + Scribe |

**完整数据流**：

```
rbnx boot (deploy.rs)
  │  .env("SCRIBE_LOG_DIR", "rbnx-boot/logs")
  ▼
rbnx start -p <pkg> (process.rs)
  │  .env("SCRIBE_LOG_DIR", &self.log_dir)
  │  Stdio::piped()
  │  tokio::spawn: 每行 → println! + scribe::info(tag, line)
  ▼
bash -c "export SCRIBE_LOG_DIR=...; ./start.sh"
  │  stdout/stderr 被管道捕获
  ▼
docker run ... entrypoint.sh ... start_engine.sh
  │  ROS 2 节点输出
  ▼
$SCRIBE_LOG_DIR/{tag}.log   (JSON-lines)
  msg: "[engine] [start_engine] rtabmap scan2d=/scanner_normalized ..."
```

**职责变更**：

| 旧（fd 继承链） | 新（Scribe 管道） |
|------|--------------|
| `deploy.rs` 创建 log 文件 + fd 重定向 | `deploy.rs` 仅设 `$SCRIBE_LOG_DIR` |
| `process.rs` `Stdio::inherit()` 透传 | `process.rs` `Stdio::piped()` + tokio task |
| 原始文本行写入文件 | 每行封装为 JSON `{"ts":...,"level":"info","tag":"...","msg":"..."}` |
| 子进程 fd 必须继承到底 | 子进程只需读 `$SCRIBE_LOG_DIR`，管道由 process.rs 处理 |

`process.rs` 的 tag 取包的反向域名（`std_name`），如
`"com.robonix.service.mapping"` → 日志写入 `service_mapping.log` 等价文件。

## 7. 线程安全

- `FILE_SINK.writers` 由 `Mutex<HashMap<..>>` 保护：同一 tag 串行写入，行完整；不同 tag 仅争用 mutex 查找/插入
- `CONSOLE_MIN` / `FILE_MIN` 为 `LazyLock`，首次求值后不可变读，无锁争用
- Console 写入为 `write_all` 单次系统调用，行级原子
- `LOG_BACKEND` 由 `OnceLock` 保证 `log::set_logger` 只调用一次

## 8. 写入端级别过滤

Scribe 在**写入端**按 per-sink 阈值过滤，不同级别走不同 sink：

| Level | Console 默认 | File 默认 | 说明 |
|:-----:|:--------:|:-----:|------|
| Debug | ✗ | ✗ | 需 `SCRIBE_FILE_LEVEL=debug` / `SCRIBE_CONSOLE_LEVEL=debug` 显式开启 |
| Info  | ✗ | ✓ | 终端不刷屏，文件可追溯 |
| Warn  | ✓ | ✓ | |
| Error | ✓ | ✓ | |

环境变量覆盖：

```bash
SCRIBE_CONSOLE_LEVEL=debug   # 终端也看所有级别
SCRIBE_FILE_LEVEL=error      # 仅 Error 落盘
```

设计意图：
- **Debug 默认关闭**：开发调试用，不污染生产日志文件，需主动开启
- **Info 仅文件**：`rbnx logs` 可事后查阅，不干扰终端操作
- **Warn/Error 双写**：运维关注的异常级别，终端+文件都可见

过滤在时间戳生成**之前**判定——若两端都不接受，直接 return，避免浪费
`SystemTime::now()` 系统调用。

## 9. `log` facade 兼容

Scribe 通过 `#[ctor]` 在 `main()` 前自动注册为 Rust 标准 `log` facade 的
backend。注册失败（已有其他 backend 抢占）不 panic。

- **tag**：`log::info!("msg")` → `"_default"`，聚合到 `_default.log`
- **msg 格式**：`"module_path|原消息"`（如 `"robonix_executor|dispatched 5 contracts"`）
- **级别过滤**：`Log::enabled()` 与写入端阈值一致
- **显式 tag**：`scribe::info("executor", "msg")` → `executor.log`（独立文件）

## 10. 不做什么（v0.1 非目标）

- **不做** log rotation / 按大小切分
- **不做** 网络聚合 / 远程上报
- **不做** 结构化查询引擎（那是 `rbnx logs` CLI 的职责，不在 library 内）
- **不做** 异步 / ring buffer / 后台 flush 线程
- **不做** 调用方传自定义 `kv` 字段（LogRecord 预留扩展点，但 v0.1 不收）
