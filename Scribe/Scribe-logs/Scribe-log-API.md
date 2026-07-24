# Scribe 日志使用指南

Scribe 是 Robonix 全系统**统一日志入口**。每条日志由时间、级别、来源（tag）、内容组成，
同时输出到 stderr（人读）和按 tag 分文件的 JSON-lines 文件。

## 1. 快速开始

### Rust

```rust
// 方式 A：Scribe 原生 API — 显式 tag → 独立日志文件
use robonix_scribe;
robonix_scribe::info("my_component", "ready on 0.0.0.0:50051");

// 方式 B：标准 log 宏 — tag 自动为 "_default" → _default.log
log::info!("connecting to atlas at {}", addr);
log::warn!("retry {}/{}", n, max);
```

### Python

```python
from robonix_api import scribe_logger
scribe_logger.info("my_component", "ready on 0.0.0.0:50051")
scribe_logger.warn("my_component", "retry 3/10")
```

## 2. 日志级别

| Level | 含义 | Console 默认 | File 默认 |
|:-----:|------|:------------:|:---------:|
| DEBUG | 开发调试 | ✗ | ✗ |
| INFO | 正常运行时信息 | ✗ | ✓ |
| WARN | 异常但不阻断 | ✓ | ✓ |
| ERROR | 需立即处理的故障 | ✓ | ✓ |

DEBUG 默认两端关闭。开启：

```bash
SCRIBE_FILE_LEVEL=debug         # DEBUG 落盘
SCRIBE_CONSOLE_LEVEL=debug      # DEBUG 也输出到终端
SCRIBE_CONSOLE_LEVEL=info       # INFO+ 输出到终端
SCRIBE_FILE_LEVEL=error         # 仅 ERROR 落盘
```

## 3. 两条调用路径

### 路径 A：Scribe 原生 API（推荐：需要独立日志文件时）

```rust
use robonix_scribe;
robonix_scribe::info("executor", "robonix-executor starting");
robonix_scribe::warn("atlas", "retry connect {}/10", n);
robonix_scribe::error("pilot", "VLM call failed: {e:#}");
robonix_scribe::debug("my_driver", "sensor raw: {buf:?}");

// 等价写法
use robonix_scribe::{log, Level};
log(Level::Info, "executor", "robonix-executor starting");
```

- `tag` 由调用方显式传入
- 写入 `$SCRIBE_LOG_DIR/{tag}.log`
- 适合系统组件（executor / pilot / liaison / atlas 等），每个有独立文件

### 路径 B：标准 `log` 宏（零改动迁移）

```rust
log::info!("connecting to atlas at {}", addr);
log::warn!("no capabilities dir configured");
log::error!("port bind failed: {e:#}");
```

- 无需 `use robonix_scribe`
- tag 自动为 `"_default"`
- 所有 `log` 宏写入同一个 `_default.log`
- Scribe 在 `main()` 之前通过 `#[ctor]` 自动注册为 `log` backend

> 具体实现：`log::LogBackend` 将 `record.target()`（即模块路径）和 `record.args()`
> 拼接为 `"模块路径|消息"`，例如 `log::info!("hello")` 在 `_default.log` 中写为
> `{"msg": "robonix_executor::dispatcher|hello", ...}`。

### 路径对比

| | 路径 A (scribe::info) | 路径 B (log::info!) |
|---|---|---|
| tag | 调用方显式传入 | `"_default"` |
| 目标文件 | `{tag}.log` | `_default.log` |
| 消息格式 | 原文 | `"模块路径|原文"` |
| import | 需要 `use robonix_scribe` | 不需要 |
| 适用场景 | 系统核心组件 | 库 / 临时调试 |

## 4. API 参考

### Rust

```rust
use robonix_scribe;

// 核心函数
robonix_scribe::log(Level::Info, "tag", "message");

// 便捷封装
robonix_scribe::debug("tag", "msg");
robonix_scribe::info( "tag", "msg");
robonix_scribe::warn( "tag", "msg");
robonix_scribe::error("tag", "msg");

// 时间戳解析（用于外部读取日志）
let ns: u64 = robonix_scribe::ts_parse("2026-06-18 14:03:47.793259332")?;

// 日志记录结构体（用于外部程序解析日志）
use robonix_scribe::LogRecord;
let rec: LogRecord = serde_json::from_str(&json_line)?;
```

### Python

```python
from robonix_api import scribe_logger
from robonix_api.scribe_logger import Level

# 核心函数
scribe_logger.log(Level.INFO, "tag", "message")
scribe_logger.log(Level.WARN, "tag", "message")

# 便捷封装
scribe_logger.debug("tag", "msg")
scribe_logger.info("tag", "msg")
scribe_logger.warn("tag", "msg")
scribe_logger.error("tag", "msg")
```

## 5. 日志目录

日志文件的存放路径由以下规则决出（Scribe 内部 `LazyLock` 一次性求值）：

1. 环境变量 `SCRIBE_LOG_DIR` → 有则用
2. 否则 → `./logs`（相对于进程当前工作目录）

```bash
# 指定日志目录
SCRIBE_LOG_DIR=/var/log/robonix rbnx boot

# rbnx boot 会为子进程自动设置此变量指向部署目录
# deploy.rs: .env("SCRIBE_LOG_DIR", "<manifest>/rbnx-boot/logs")
```

**重要**：`rbnx` CLI 父进程自身在进入 `deploy` 子命令前就打印了第一条日志
（`main.rs:24`），因此 `rbnx.log` 落在启动时的 CWD 的 `./logs/` 下，而非
deploy 目录的 `rbnx-boot/logs/`。子进程（executor / pilot / liaison 等）
则由 `deploy.rs` 设置 `SCRIBE_LOG_DIR`，日志写入 deploy 目录。

### 典型日志文件布局

```
rbnx-boot/logs/
├── executor.log        # robonix_scribe::info("executor", ...)
├── pilot.log           # robonix_scribe::info("pilot", ...)
├── liaison.log         # robonix_scribe::info("liaison", ...)
├── rbnx.log            # robonix_scribe::info("rbnx", ...)
├── _default.log        # log::info!(...) 聚合
└── com.robonix.system.scene.log  # package 子进程日志（按 deploy 中 package name）
```

## 6. 输出格式

### Console（stderr）

logcat 风格单行文本，格式：`MM-DD HH:MM:SS.sss  L tag（24字符左对齐） msg`

```
06-18 14:03:47.793  I executor                  robonix-executor starting
06-18 14:03:48.112  W atlas                     retry connect 3/10
06-18 14:03:50.004  E pilot                     VLM call failed: timeout
```

- 时间：本地时间，月-日 时:分:秒.毫秒
- 级别：`D` / `I` / `W` / `E` 单字符
- tag：左对齐、空格填充至 24 字符；超过 24 字符则截断末尾加 `…`

### 文件（JSON-lines）

每行一个 JSON 对象，写入 `{tag}.log`：

```json
{"ts":"2026-06-18 14:03:47.793259332","level":"info","tag":"executor","msg":"robonix-executor starting"}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `ts` | string | 本地时间 `"YYYY-MM-DD HH:MM:SS.nnnnnnnnn"`（29字符） |
| `level` | string | `"debug"` / `"info"` / `"warn"` / `"error"` |
| `tag` | string | 来源标识 |
| `msg` | string | 日志正文 |

> 兼容旧格式：反序列化时 `ts` 也接受原始整数纳秒时间戳。

## 7. `rbnx logs` CLI

```bash
# 读当前部署的全部日志（按时间排序）
rbnx logs

# 指定日志目录
rbnx logs -d /path/to/rbnx-boot/logs

# 按 tag 过滤（OR 语义）
rbnx logs -t executor -t pilot

# 按级别过滤
rbnx logs -l warn       # 只看 WARN + ERROR
rbnx logs -l error      # 只看 ERROR

# 实时跟踪（tail -f）
rbnx logs -f
rbnx logs -f -t executor -l info

# 输出原始 JSON（供 jq 等工具处理）
rbnx logs --json | jq 'select(.level=="error")'
rbnx logs --json -t pilot -l info
```

### 常见查询示例

```bash
# 看 executor 的所有日志
rbnx logs -t executor

# 看 executor + pilot 的 WARN 以上
rbnx logs -t executor -t pilot -l warn

# 实时跟踪全部 error
rbnx logs -f -l error

# 导出 JSON 用 jq 过滤特定消息
rbnx logs --json | jq 'select(.msg | contains("timeout"))'
```

## 8. Tag 命名约定

| 来源 | tag | 示例 |
|------|-----|------|
| 系统核心组件 | 组件名 | `executor`, `pilot`, `liaison`, `atlas` |
| `rbnx` CLI 自身 | `rbnx` | `"rbnx starting"` |
| 标准 `log` 宏 | `_default` | 所有 `log::info!()` 聚合 |
| deploy 启动的 package | 反向域名 | `com.robonix.system.scene`, `com.robonix.service.mapping` |

> 建议：新增系统组件时使用组件名作为 tag（如 `"executor"`），和已有的
> executor/pilot/liaison 保持一致。

## 9. 真实用法示例

### Rust 系统组件

```rust
// executor/src/main.rs — 启动日志用显式 tag
robonix_scribe::info("executor", "robonix-executor starting");
// 业务逻辑中用 log 宏
log::info!("connecting to atlas at {}", cfg.atlas_endpoint);

// pilot/src/planner.rs — 关键事件用 log 宏
log::info!("[pilot] session_end: invoking compact_memory if available");

// pilot/src/main.rs
robonix_scribe::info("pilot", "robonix-pilot starting");
robonix_scribe::info("pilot", &format!("robonix-pilot ready on {listen_addr}"));
```

### Python 组件

```python
# 服务 / primitive 启动时
from robonix_api import scribe_logger
scribe_logger.info("my_service", "starting")
scribe_logger.info("my_service", f"ready on {addr}")

# 运行时事件
scribe_logger.warn("my_service", f"sensor timeout after {t}s")
scribe_logger.error("my_service", f"gRPC call failed: {e}")
```

## 10. 为系统新增日志

### 新增一个系统组件（Rust）

1. 在 `Cargo.toml` 添加依赖：
   ```toml
   [dependencies]
   robonix_scribe = { path = "../system/scribe" }
   ```

2. 在 `main.rs` 添加启动日志：
   ```rust
   robonix_scribe::info("my_component", "robonix-my_component starting");
   ```

3. 运行时打印日志：
   ```rust
   // 关键状态变更 — 用显式 tag 写入独立文件
   robonix_scribe::info("my_component", "config loaded: {cfg:?}");
   robonix_scribe::warn("my_component", "slow query took {ms}ms");

   // 一般调试信息 — 用 log 宏写入 _default.log
   log::debug!("processing item {}/{}", i, total);
   log::info!("batch completed: {} items", n);
   ```

### 新增一个 Python 组件

```python
from robonix_api import scribe_logger

scribe_logger.info("my_component", "starting")
scribe_logger.info("my_component", "ready")
```

## 11. 常见问题

### Q: 为什么 `rbnx logs` 看不到排查日志？

1. 确认日志目录存在：`ls rbnx-boot/logs/`
2. 确认级别过滤：DEBUG 默认不落盘，需 `SCRIBE_FILE_LEVEL=debug`
3. 确认 tag 过滤：`rbnx logs` 无 `-t` 时显示所有 tag

### Q: 为什么有两个 `rbnx.log`？

因为 `rbnx` CLI 自身的启动日志落在运行时的 CWD `./logs/` 下，而子进程
的日志落在 deploy 目录的 `rbnx-boot/logs/` 下。执行 `rbnx logs` 时，
默认读 `./rbnx-boot/logs/`。

### Q: 如何让第三方容器（如 rtabmap）的日志进入 Scribe？

第三方容器（mapping/explore）的 stdout/stderr 由 `process.rs`
通过管道捕获，每行以 package 的反向域名作为 tag 转发到 Scribe：
```rust
// process.rs
robonix_scribe::info(&tag, &line);  // tag = "com.robonix.service.mapping"
```

但注意：当前 `process.rs` 使用 `Stdio::inherit()`（非 `Stdio::piped()`），
因此容器日志直接透传到终端，不经过 Scribe 文件。如需持久化，需改为
`Stdio::piped()` 并起 tokio 任务转发行。

### Q: 如何改 Console 显示的级别？

```bash
# 想让终端也看到 INFO（默认只有 WARN+）
SCRIBE_CONSOLE_LEVEL=info rbnx boot

# 想让终端也看到 DEBUG（默认只有 WARN+）
SCRIBE_CONSOLE_LEVEL=debug rbnx boot
```

### Q: 日志文件会轮转吗？

v0.1 不做轮转。`rbnx boot` 每次启动前会清空 deploy 目录下的旧 `*.log` 文件
（[deploy.rs:483-491](tools/rbnx/src/cmd/deploy.rs#L483-L491)）。

## 12. 设计约束（禁止事项）

- **不要** 自己调用 `scribe::init()` — Scribe 自动懒初始化
- **不要** 自己 `open()` 写 `logs/` 目录 — 统一走 `log()` 入口
- **不要** 在 tag 中嵌入时间戳或随机数 — tag 决定文件名，变化会导致日志碎片
- **不要** 在热点路径记录 DEBUG 级别日志 — 提前用条件判断包裹（或在生产环境设置 `SCRIBE_FILE_LEVEL=info` 屏蔽）
