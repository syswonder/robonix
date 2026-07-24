# Scribe 实现步骤记录

## TODO 1 — 基础数据模型 Level / LogRecord ✅

**完成时间**：2026-06-17

**实现内容**：
- `system/scribe/Cargo.toml`：新建 library crate `robonix-scribe`，依赖 `serde` / `serde_json`
- `system/scribe/src/lib.rs`：
  - `Level` 枚举（Debug / Info / Warn / Error），含 `code()` 单字符缩写方法和 `Display` 实现
  - `LogRecord` 结构体 `{ ts, level, tag, msg }`，含 `now()` 构造器取当前系统时间纳秒戳
  - `system_time_ns()` 内部函数（过渡期用 `SystemTime`，后续切 chronos）
- 根 `Cargo.toml`：将 `system/scribe` 加入 workspace members

**测试结果**：6 项单元测试全部通过
- `level_codes`：单字符缩写正确性
- `level_display_is_code`：Display 输出 = code
- `level_ordering`：Debug < Info < Warn < Error
- `log_record_serialization`：序列化为 `{"ts":...,"level":"info","tag":"scene_svc","msg":"object registered"}`
- `log_record_deserialization`：反序列化往返
- `log_record_now_has_reasonable_ts`：时间戳在合理范围内

**审查**：类型定义与 describe.md 第 2 节完全一致；JSON 序列化格式与 describe.md 第 6 节一致。

---

## TODO 2 — Console 格式化器 ✅

**完成时间**：2026-06-17

**实现内容**：
- `system/scribe/src/format.rs`：
  - `format_console(&LogRecord) -> String`：logcat 风格单行，格式 `MM-DD HH:MM:SS.sss  L tag（24 列左对齐） msg\n`
  - `write_console(&LogRecord) -> io::Result<()>`：直接 write_all 到 stderr
  - `decompose_ts(u64) -> (month, day, hour, min, sec, ms)`：纳秒时间戳转本地时间
  - Unix 下通过 `libc::localtime_r` 取本地时间；非 Unix 回退 UTC
- `system/scribe/Cargo.toml`：添加 `[target.'cfg(unix)'.dependencies] libc`
- `system/scribe/src/lib.rs`：注册 `pub mod format`

**测试结果**：4 项测试通过
- `format_includes_level_code_and_tag`：输出行包含级别码和 tag
- `tag_padded_to_fixed_width`：短 tag 补齐到 24 列，长 tag 截断带 `…`
- `level_codes_in_output`：四个级别各输出 D/I/W/E
- `write_console_does_not_panic`：stderr 写入不 panic

---

## TODO 3 — Per-tag 文件 Sink ✅

**完成时间**：2026-06-17

**实现内容**：
- `system/scribe/src/sink.rs`：
  - `FileSink { dir, writers: Mutex<HashMap<String, File>> }`
  - `FileSink::new(log_dir) -> io::Result<Self>`：create_dir_all + 初始化空 writers map
  - `FileSink::write(&LogRecord) -> io::Result<()>`：序列化 JSON 行 → 按 tag 取/创建文件 → write + flush
  - 每行写入后立即 flush，保证崩溃不丢最后一条
  - `Mutex` 保护 HashMap 保证线程安全
- `system/scribe/Cargo.toml`：添加 `tempfile` dev-dependency
- `system/scribe/src/lib.rs`：注册 `pub mod sink`

**测试结果**：4 项测试通过
- `first_write_creates_file`：首次写入某 tag 创建文件
- `same_tag_appends`：同一 tag 多次写入为追加
- `different_tags_different_files`：不同 tag 写入各自文件
- `concurrent_writes_no_data_loss`：8 线程 × 100 条并发写入，无数据丢失，每条可解析

---

## TODO 4 — log() 核心函数 + 懒初始化 + 便捷封装 ✅

**完成时间**：2026-06-17

**实现内容**：
- `system/scribe/src/lib.rs` 新增：
  - `LOG_DIR: LazyLock<PathBuf>`：首条 `log()` 时读 `$SCRIBE_LOG_DIR` 或默认 `"./logs"`
  - `FILE_SINK: LazyLock<FileSink>`：首条 `log()` 时 `create_dir_all` + 初始化全局 sink
  - `pub fn log(level, tag, msg)`：组装 LogRecord → write_console → FILE_SINK.write（best-effort，不 panic）
  - `pub fn debug/info/warn/error(tag, msg)`：便捷封装，内部调 `log()`

**对外接口**（唯一）：
```rust
pub fn log(level: Level, tag: &str, msg: &str);
pub fn debug(tag: &str, msg: &str);
pub fn info(tag: &str, msg: &str);
pub fn warn(tag: &str, msg: &str);
pub fn error(tag: &str, msg: &str);
```
无 `init()`、无 `setup()`、无 `flush()`。

**测试结果**：3 项新增测试通过（共 17 项）
- `log_does_not_panic`：调用 `log()` 及各便捷封装不 panic
- `convenience_wrappers_use_correct_levels`：各封装使用正确 Level
- `default_log_dir_is_dot_logs`：未设 SCRIBE_LOG_DIR 时默认 `./logs`

**审查**：接口与 describe.md 第 3 节一致；无 init 函数；懒初始化由 LazyLock 保证。

---

## TODO 5 — rbnx boot 适配 ✅

**完成时间**：2026-06-17

**实现内容**：
- 根 `Cargo.toml`：添加 `robonix-scribe = { path = "system/scribe" }` workspace dependency
- `tools/rbnx/Cargo.toml`：添加 `robonix-scribe.workspace = true`
- `tools/rbnx/src/cmd/deploy.rs`：
  - `spawn_system_binary()`：删除 `File::create` + `Stdio::from(log)` + `Stdio::from(err)` fd 重定向；改为 `.env("SCRIBE_LOG_DIR", log_dir)` 传给子进程
  - `spawn_package()`：同上；子进程 stdout/stderr 不再被重定向
  - 保留 `log_path()` 辅助函数（其他地方仍使用）
  - 新增 `use robonix_scribe as scribe`
  - 新增 bootstrap 日志：boot 开始、组件全部就绪、收到 shutdown 信号
- 子进程的日志由 Scribe library 自行写入（读取 `$SCRIBE_LOG_DIR`）

**审查**：
- `cargo build -p robonix-cli` 通过
- `cargo clippy -p robonix-scribe -p robonix-cli -- -D warnings` 通过
- `cargo test -p robonix-scribe` 17 项全部通过
- 不再需要 `rbnx boot` 父进程手动管理 fd 重定向；子进程的 console 输出回到终端

---

## TODO 6 — Python 侧 scribe_logger ✅

**完成时间**：2026-06-17

**实现内容**：
- `pylib/robonix-api/robonix_api/scribe_logger.py`（新建）：
  - `Level` 枚举（DEBUG/INFO/WARN/ERROR，value 为 D/I/W/E）
  - `log(level, tag, msg)`：模块级函数，唯一入口
  - `debug/info/warn/error(tag, msg)`：便捷封装
  - 懒初始化：首条 `log()` 时 `mkdir -p $SCRIBE_LOG_DIR`（默认 `./logs`）
  - console 输出：`_format_console()` 产生与 Rust `format.rs` 一致的 logcat 行 → stderr
  - 文件输出：`_format_json()` 产生 `{"ts":...,"level":"info","tag":...,"msg":...}` → `{tag}.log`
  - 线程安全：`threading.Lock` 保护 per-tag writer dict
  - 时间戳：`time.time_ns()`（过渡期；后续切 chronos Python binding）
- `pylib/robonix-api/robonix_api/__init__.py`：导出 `scribe_logger` 到 `__all__`

**测试结果**：smoke test 通过
- 日志同时输出到 stderr（logcat 格式）和 `{tag}.log`（JSON 格式）
- JSON 格式与 Rust 侧一致（4 字段，`level` 小写）
- Console 格式 tag 补齐到 24 列

**审查**：API 与 describe.md 第 3 节一致；与 Rust `lib.rs` 同语义；线程安全；无 init 函数

---

## TODO 7 — rbnx logs CLI ✅

**完成时间**：2026-06-17

**实现内容**：
- `tools/rbnx/src/cmd/mod.rs`：
  - 新增 `Commands::Logs` 变体：`-d/--log-dir`、`-t/--tag`（可多次）、`-l/--level`、`-f/--follow`、`--json`
  - 新增 `mod logs;` 声明和 dispatch
- `tools/rbnx/src/cmd/logs.rs`（新建）：
  - `read_all()`：读目录下所有 `*.log`，解析 JSON → 按 ts 排序 → tag/level 过滤 → 输出
  - `follow()`：tail -f 模式，poll 文件增量
  - `render_console()`：复用 `robonix_scribe::format::format_console()`

**测试结果**：smoke test 通过
- `rbnx logs --help` 正常输出
- 无过滤：所有日志按时间排序输出
- `--tag atlas`：只显示 atlas
- `--level warn`：只显示 Warn + Error
- `--json`：原始 JSON 行输出
- console 渲染：logcat 风格，tag 补齐 24 列

**审查**：clippy 通过；复用 Scribe 的 `format_console()` 和 `LogRecord` 反序列化

---

## TODO 8 — 首批组件迁移 ✅

**完成时间**：2026-06-17（更新 2026-06-18）

**实现内容**：

**Rust — atlas**：
- `system/atlas/Cargo.toml`：添加 `robonix-scribe.workspace = true`，移除 `env_logger`，保留 `log`
- `system/atlas/src/main.rs`：
  - 首行 `robonix_scribe::info("atlas", "robonix-atlas starting")` — 注册 Scribe 为 `log` backend
  - 后续全部使用标准 `log::info!("...")` / `log::warn!("...")` 宏（零改动）
  - 日志 level CLI arg 保留（forward compat），值不再被消费
- 库代码（service.rs / client.rs / contract_registry.rs）的 `log::info!()` / `log::warn!()` 自动路由到 Scribe，tag = module path（如 `"robonix_atlas::service"`）

**Python — memsearch**：
- `services/memsearch/memsearch_service/service.py`：
  - 添加 `from robonix_api import scribe_logger`
  - 在 startup 处添加 `scribe_logger.info("service_memory", "memory service starting")`
  - 保留现有 `logging` 调用（过渡阶段，双写不冲突）

**审查**：`cargo clippy` 通过；`cargo test` 17 项通过

---

## 追加 — `log` facade 兼容层 ✅

**完成时间**：2026-06-18

**实现内容**：
- `system/scribe/Cargo.toml`：添加 `log.workspace = true`
- `system/scribe/src/lib.rs`：
  - `LogBackend` 结构体实现 `log::Log` trait
  - `Log::log()`：`log::Record` → Scribe `LogRecord`（`tag = record.target()`，`msg = format!("{}", record.args())`）
  - `LOG_BACKEND: OnceLock<()>`：首次 `log()` 调用时 `log::set_logger(&BACKEND)` 注册
  - 注册策略改为 **best-effort**：`SetLoggerError` 不 panic（`let _ = log::set_logger(...)`）
  - `Log::enabled()` 始终返回 `true`（写入全量，读时过滤）
- `system/atlas/src/main.rs`：回归标准 `log::info!()` / `log::warn!()` 宏

**设计要点**：
| 维度 | 决策 |
|------|------|
| tag 来源 | `record.target()` = `module_path!()`（如 `"robonix_atlas"`） |
| 注册时机 | 与 Scribe `log()` 懒初始化同一点，无额外 `init()` |
| 注册失败 | 不 panic — Scribe 原生 API 始终工作，仅 log facade bridge 失效 |
| 写入端过滤 | 无 — `enabled()` 始终 true，全量写入 |

**审查**：`cargo test` 17 项通过；`cargo clippy` 干净。

---

## TODO 9 — chronos 时间戳接入 ✅

**状态**：deferred（chronos #62 尚未实现；代码已预留，落地后单行切换）

**迁移路径（chronos 就绪后）**：

| 项目 | 当前 | 切换到 chronos |
|------|------|---------------|
| Rust `system_time_ns()` | `SystemTime::now().duration_since(...)` | `chronos::now()` |
| Python `time.time_ns()` | `time.time_ns()` | `chronos.now()` (Python binding) |

两处均为单行替换，无 API 变化。chronos 落地后立即可切。

---

## 追加 5 — process.rs pipe-through-scribe：包 stdout/stderr 接入 Scribe ✅

**完成时间**：2026-06-18

**问题**：之前 `process.rs::start_process` 使用 `Stdio::inherit()`，包脚本的
stdout/stderr（bash、Docker、ROS 节点输出）直接继承父进程 fd。`deploy.rs` 在
TODO 5 中移除了 fd 重定向后，包输出仅到终端，**不写入任何日志文件**。

**实现内容**：

- `tools/rbnx/src/process.rs`：
  - `Stdio::inherit()` → `Stdio::piped()`：捕获子进程 stdout/stderr
  - 起两个 tokio task，逐行读取管道：
    - `println!/eprintln!` → 终端保持实时可见
    - `robonix_scribe::info/warn!(tag, &line)` → Scribe 结构化日志
  - tag = `std_name`（包的反向域名，如 `"com.robonix.service.mapping"`）
  - 日志目标：`$SCRIBE_LOG_DIR/{tag}.log`
  - 设置 `.env("SCRIBE_LOG_DIR", &self.log_dir)` 使子进程继承
  - 重命名 `_log_dir` → `log_dir`（实际使用）

- `tools/rbnx/src/cmd/run_package.rs`：
  - log_dir 解析：优先 `$SCRIBE_LOG_DIR`（由 `rbnx boot` 设置），fallback 到
    `<pkg>/rbnx-build/logs`
  - 将 `SCRIBE_LOG_DIR` 加入 shell export，使 package 脚本及 Docker 容器继承

- `system/scribe/src/lib.rs`：
  - 修复 pre-existing clippy lint：`y % 4 == 0` → `y.is_multiple_of(4)`

**数据流**（新）：

```
包脚本 stdout
  │
  ▼
bash -c "export SCRIBE_LOG_DIR=...; ./start.sh"
  │  Stdio::piped()
  ▼
tokio::spawn(async { BufReader::lines() })
  │
  ├─ println!("{line}")          → 终端实时可见
  └─ scribe::info(tag, &line)    → $SCRIBE_LOG_DIR/{tag}.log (JSON-lines)
```

**与旧继承链的对比**：

| 旧的继承链 | 新的 Scribe 管道 |
|-----------|-----------------|
| `deploy.rs` 创建 log 文件 + fd 重定向 | `deploy.rs` 仅设 `$SCRIBE_LOG_DIR` |
| `process.rs` `Stdio::inherit()` 透传 | `process.rs` `Stdio::piped()` + tokio task |
| 原始文本行写入文件 | 每行封装为 `{"ts":"...","level":"info","tag":"...","msg":"..."}` |
| `rbnx logs` 需在特定目录运行 | `rbnx logs` 读取 JSON-lines，支持 tag/level 过滤 + follow |

**终端静默**（同日后续修改）：
- `process.rs`：去掉 `println!`/`eprintln!` 转发 —— 包 stdout/stderr 只写入 Scribe 文件，不刷终端
- `deploy.rs`：`spawn_system_binary` 和 `spawn_package` 增加 `.stdout(Stdio::null()).stderr(Stdio::null())` —— 系统二进制（executor/pilot/liaison）和 `rbnx start` 的原始 `println!`/`eprintln!` 不再漏到终端。结构化日志通过 `$SCRIBE_LOG_DIR` 正常写入

**审查**：`cargo check` + `cargo clippy` 干净；`cargo test -p robonix-scribe` 19 项通过；
`cargo test -p robonix-cli` 通过。

---

## 追加 2 — 写入端级别过滤 + 全仓库 env_logger → Scribe ✅

**完成时间**：2026-06-18

### 级别过滤

**实现内容**：
- `system/scribe/src/lib.rs`：
  - `CONSOLE_MIN: LazyLock<Level>`：`$SCRIBE_CONSOLE_LEVEL` 或默认 `Warn`
  - `FILE_MIN: LazyLock<Level>`：`$SCRIBE_FILE_LEVEL` 或默认 `Info`
  - `parse_level_env(key, fallback)`：解析 `"debug"`/`"info"`/`"warn"`/`"error"` 字符串
  - `log()`：每 sink 写入前检查 `level >= THRESHOLD`，两边都不满足则提前 return
  - `Log::enabled()`：level >= min(CONSOLE_MIN, FILE_MIN)（保守，避免误丢弃）

**默认行为**：

| Level | Console | File | 说明 |
|:-----:|:-------:|:----:|------|
| Debug | ✗ | ✗ | 需 `SCRIBE_FILE_LEVEL=debug` 显式开启 |
| Info  | ✗ | ✓ | 文件保留，终端不刷屏 |
| Warn  | ✓ | ✓ | |
| Error | ✓ | ✓ | |

**环境变量覆盖**：
```bash
SCRIBE_CONSOLE_LEVEL=debug   # 终端也看 Debug
SCRIBE_FILE_LEVEL=error      # 仅 Error 落盘
SCRIBE_FILE_LEVEL=debug      # Debug 也落盘
```

### 全仓库 env_logger 替换

| 文件 | 变更 |
|------|------|
| `system/executor/src/main.rs` | `env_logger::init()` → `robonix_scribe::info("executor", ...)` |
| `system/executor/Cargo.toml` | `env_logger` → `robonix-scribe` |
| `system/pilot/src/main.rs` | `env_logger::init()` → `robonix_scribe::info("pilot", ...)` |
| `system/pilot/Cargo.toml` | `env_logger` → `robonix-scribe` |
| `system/liaison/src/main.rs` | `env_logger::init()` → `robonix_scribe::info("liaison", ...)` |
| `system/liaison/Cargo.toml` | `env_logger` → `robonix-scribe` |
| `system/liaison/examples/mock_pilot.rs` | `env_logger::init()` → `robonix_scribe::info("mock_pilot", ...)` |
| `system/liaison/examples/mock_audio.rs` | `env_logger::init()` → `robonix_scribe::info("mock_audio", ...)` |
| `tools/codegen/src/main.rs` | `env_logger::init()` → `robonix_scribe::info("codegen", ...)` |
| `tools/codegen/Cargo.toml` | `env_logger` → `robonix-scribe` |
| `tools/rbnx/src/main.rs` | `env_logger::init()` → `robonix_scribe::info("rbnx", ...)` |
| `tools/rbnx/Cargo.toml` | 移除 `env_logger`（scribe 已存在） |
| 根 `Cargo.toml` | 移除 `env_logger = "0.11"` workspace dep |

**审查**：`cargo build --workspace` + `cargo clippy --workspace -- -D warnings` 通过；`cargo test` 17 项通过。

---

## 追加 3 — ctor 自动注册 + `_default.log` 聚合 ✅

**完成时间**：2026-06-18

**问题**：`log::info!("msg")` 在无 backend 时被 `log` crate 静默丢弃。此前要求
每个组件首行调用 `robonix_scribe::info("tag", "starting")` 来激活。

**解决**：`ctor` 在 `main()` 前自动注册 Scribe 为 `log` backend。

**实现内容**：
- `system/scribe/Cargo.toml`：添加 `ctor = "0.4"`
- `system/scribe/src/lib.rs`：
  - `#[ctor::ctor] fn auto_init()`：`log::set_logger(&BACKEND)` + `log::set_max_level(Trace)`
  - `LogBackend::log()`：tag 固定为 `"_default"`，msg = `"module_path|原消息"`
  - 删除 `LOG_BACKEND: OnceLock<()>` 和 `ensure_log_backend()`（ctor 替代）
  - `log()` 函数不再调用 `ensure_log_backend()`

**行为变化**：

| 场景 | 之前 | 之后 |
|------|------|------|
| 仅有 `log::info!("msg")`，无 scribe 调用 | 消息丢弃 | → `_default.log` |
| `scribe::info("executor", "msg")` | → `executor.log` | → `executor.log`（不变） |
| 两者混用 | 各自独立 | 各自独立 |
| 组件无需任何 scribe 导入 | 日志丢失 | 自动工作 |

**文件影响**：
- `_default.log`：所有 `log::info!/warn!/error!` 宏的聚合文件，msg 前缀为 module path
- `{tag}.log`：`scribe::info("tag", ...)` 显式调用，tag 精确

**审查**：`cargo test` 17 项通过，`cargo build --workspace` + `cargo clippy --workspace` 干净。

---

## 追加 4 — ts 序列化为可读时间字符串 ✅

**完成时间**：2026-06-18

**实现内容**：
- `system/scribe/src/lib.rs`：
  - `ts_fmt(u64) -> String`：纳秒时间戳 → `"YYYY-MM-DD HH:MM:SS.nnnnnnnnn"`（29 字符，本地时间）
  - `ts_parse(&str) -> Option<u64>`：反向解析（支持 `mktime` + UTC fallback）
  - `LogRecord`：手动 `impl Serialize`，ts 序列化为可读字符串
  - `deserialize_ts`：兼容新字符串格式 + 旧整数格式（backward compat）
- `pylib/robonix-api/robonix_api/scribe_logger.py`：
  - `_ts_fmt(ts_ns) -> str`：与 Rust `ts_fmt` 同格式
  - `_format_json()`：ts 字段改为字符串

**格式示例**：
```json
{"ts":"2026-06-18 11:30:15.123456789","level":"info","tag":"atlas","msg":"control plane ready"}
```

- 旧格式 `{"ts":1765432100123456789,...}` 反序列化仍接受（backward compat）
- `rbnx logs` console 渲染不受影响（内部用 `decompose_ts`）
- `rbnx logs --json` 输出新格式

**测试**：19 项通过（新增 2 项：`deserialization_readable_string`、`roundtrip`）

---

## 追加 6 — P0 修复：I/O 路径 panic + Tag 路径遍历 ✅

**完成时间**：2026-06-25

### 修复 P0.1 — I/O 路径去掉所有 panic

| 位置 | 修复前 | 修复后 |
|------|--------|--------|
| `lib.rs` FILE_SINK LazyLock | `.expect("failed to create scribe log dir")` | `.ok()` → `Option<FileSink>`，失败退化为 console-only |
| `sink.rs` write() | `.expect("failed to open log file")` | `?` 传播 io::Error，调用方 `let _ = sink.write(...)` 静默丢弃 |
| `lib.rs` 写路径 | `FILE_SINK.write(&record)` | `if let Some(sink) = FILE_SINK.as_ref() { let _ = sink.write(&record); }` |
| Python `scribe_logger.py` | `open(path, "a")` 无保护 | `try/except OSError` + `_get_writer` 返回 `Optional[TextIO]` |

### 修复 P0.2 — Tag 路径遍历

- `sink.rs` `tag_path()`：非 `[A-Za-z0-9._-]` 字符替换为 `_`
- `scribe_logger.py` `_sanitize_tag()`：同规则
- `"a/../b c"` → `"a_.._b_c"`（`/`→`_`，`.`保留，` `→`_`）

### 新增测试（3 项，共 24 项）
- `sanitize_tag_replaces_dangerous_chars`：路径遍历字符被替换
- `write_does_not_panic_on_bad_path`：非目录路径失败而非 panic
- `sink_creation_fails_gracefully`：`/dev/null/logs` 失败而非 panic

**审查**：24 项测试通过，clippy 干净，workspace build 通过。

---

## 最终状态总结

| 模块 | 文件 | 功能 |
|------|------|------|
| `system/scribe/src/lib.rs` | ~260 行 | `Level`, `LogRecord`, `log()` + 便捷封装, `log::Log`, 懒初始化, per-sink 级别过滤 |
| `system/scribe/src/format.rs` | 142 行 | Console logcat 格式化 |
| `system/scribe/src/sink.rs` | 150 行 | Per-tag JSON 行文件写入 |
| `system/scribe/Cargo.toml` | 18 行 | 依赖 serde, serde_json, log, libc(unix) |
| `pylib/robonix-api/robonix_api/scribe_logger.py` | 126 行 | Python 侧同语义 logger |
| `tools/rbnx/src/cmd/logs.rs` | 160 行 | `rbnx logs` CLI |
| `tools/rbnx/src/cmd/deploy.rs` | 修改 | 移除 fd 重定向, 添加 SCRIBE_LOG_DIR, bootstrap 日志 |
| `tools/rbnx/src/cmd/run_package.rs` | 修改 | 解析 SCRIBE_LOG_DIR，加入 shell export |
| `tools/rbnx/src/process.rs` | 修改 | Stdio::piped + tokio task 管道转发 → Scribe |
| `tools/rbnx/src/cmd/mod.rs` | 修改 | 添加 Logs 子命令 |
| `system/atlas`, `executor`, `pilot`, `liaison`, `codegen` main.rs | 修改 | env_logger → scribe（7 文件） |
| `services/memsearch/.../service.py` | 修改 | 添加 scribe_logger 调用 |
| 根 `Cargo.toml` | 修改 | 移除 `env_logger` workspace dep |

**测试**：`cargo test -p robonix-scribe` 17 项通过，`cargo build --workspace` + `cargo clippy --workspace -- -D warnings` 干净。
**外部接口**：
  - 原生 API：`log(level, tag, msg)` + `debug/info/warn/error(tag, msg)`，无 `init()`
  - `log` facade：`log::info!("msg")`（零改动迁移，tag = module path，best-effort 注册）
**默认级别**：Console=Warn, File=Info, Debug 需显式开启（`SCRIBE_FILE_LEVEL=debug`）

---

## TODO 10 — Scribe 宏化：移除 `log` crate，引入 `init()` + 宏 ✅

**完成时间**：2026-06-22

### 问题（来自 修改点.md）

1. **两种 info**：`robonix_scribe::info("executor", ...)`（Scribe 原生）和 `log::info!("msg")`（标准 log 宏）并存，调用方困惑。
2. **前缀太长**：`robonix_scribe::info` 每次都要写完整路径，不如 `log::info!` 短。
3. **tag 重复**：每个组件每次打日志都要写 `"liaison"` 之类的 tag，应提供 `init()` 一次设置进程级默认 tag。

### 设计方案

三合一解决：

| 维度 | 旧 | 新 |
|------|----|----|
| 依赖 | `log` crate（`log::info!`）+ `ctor` + `LogBackend` | 全部移除 |
| 默认 tag | 无；每次传 | `robonix_scribe::init("executor")` 一次设置 |
| 日志调用 | `robonix_scribe::info("tag", ...)` 或 `log::info!(...)` | `robonix_scribe::info!("msg")` 宏 |
| tag 来源 | 显式传参或 `"_default"` | `init()` 设定的值 |
| 模块路径 | `_default.log` 的 msg 前缀 `module_path\|` | 不再拼接（tag 即来源） |

### 实现内容

**`system/scribe/Cargo.toml`**：
- 移除 `log.workspace = true`
- 移除 `ctor = "0.4"`

**`system/scribe/src/lib.rs`**：
- 新增 `DEFAULT_TAG: OnceLock<String>` 静态变量
- 新增 `pub fn init(tag: &str)` — 设置进程级默认 tag（重复调用 panic）
- 新增 `#[doc(hidden)] pub fn default_tag() -> &'static str` — 供宏内部使用，`init()` 未调用时返回 `"_default"`
- 新增 4 个 `#[macro_export]` 宏：
  ```rust
  info!("msg {}", arg)   // → log(Level::Info, default_tag(), &format!(...))
  warn!("msg {}", arg)
  error!("msg {}", arg)
  debug!("msg {}", arg)
  ```
- 移除 `LogBackend` 结构体及其 `log::Log` trait 实现
- 移除 `#[ctor::ctor] fn auto_init()`
- 移除 `BACKEND` 静态变量
- 移除 `use log::Log` 等相关引用
- 更新模块文档（Quick start 示例）

**根 `Cargo.toml`**：
- 移除 `log = "0.4"` workspace dependency

**迁移范围**（22 文件，~70 处调用点）：

| 文件 | 变更 |
|------|------|
| `system/scribe/src/lib.rs` | 新增 `init()` + 宏，删除 `LogBackend` / `ctor` |
| `system/scribe/Cargo.toml` | 移除 `log`, `ctor` |
| `system/executor/src/main.rs` | `robonix_scribe::info("executor", ...)` → `init("executor")` + `info!(...)`；`log::*!` → `robonix_scribe::*!` |
| `system/executor/src/service.rs` | `log::*!` → `robonix_scribe::*!` |
| `system/executor/src/dispatch/mod.rs` | 同上 |
| `system/executor/src/dispatch/grpc.rs` | 同上 |
| `system/executor/Cargo.toml` | 移除 `log` |
| `system/pilot/src/main.rs` | `robonix_scribe::info("pilot", ...)` → `init("pilot")` + `info!(...)`；`log::*!` → `robonix_scribe::*!` |
| `system/pilot/src/planner.rs` | `log::*!` → `robonix_scribe::*!` |
| `system/pilot/src/service.rs` | 同上 |
| `system/pilot/src/memory.rs` | 同上 |
| `system/pilot/src/discovery.rs` | 同上 |
| `system/pilot/Cargo.toml` | 移除 `log` |
| `system/liaison/src/main.rs` | `robonix_scribe::info("liaison", ...)` → `init("liaison")` + `info!(...)`；`log::*!` → `robonix_scribe::*!` |
| `system/liaison/src/voice.rs` | `log::*!` → `robonix_scribe::*!` |
| `system/liaison/examples/mock_pilot.rs` | `robonix_scribe::info("mock_pilot", ...)` → `init("mock_pilot")` + `info!(...)`；`log::*!` → `robonix_scribe::*!` |
| `system/liaison/examples/mock_audio.rs` | `robonix_scribe::info("mock_audio", ...)` → `init("mock_audio")` + `info!(...)`；`log::*!` → `robonix_scribe::*!` |
| `system/liaison/Cargo.toml` | 移除 `log` |
| `system/atlas/src/main.rs` | 添加 `init("atlas")`；`log::*!` → `robonix_scribe::*!` |
| `system/atlas/src/service.rs` | `use log::{info, warn}` → `use robonix_scribe::{info, warn}`；其余 `log::*!` 已由 sed 转换 |
| `system/atlas/src/contract_registry.rs` | 同上 |
| `system/atlas/src/client.rs` | `log::*!` → `robonix_scribe::*!` |
| `system/atlas/Cargo.toml` | 移除 `log` |
| `tools/codegen/src/main.rs` | `robonix_scribe::info("codegen", ...)` → `init("codegen")` + `info!(...)` |
| `tools/codegen/Cargo.toml` | 移除 `log` |
| `tools/rbnx/src/main.rs` | `robonix_scribe::info("rbnx", ...)` → `init("rbnx")` + `info!(...)`；`log::*!` → `robonix_scribe::*!` |
| `tools/rbnx/src/process.rs` | `log::*!` → `robonix_scribe::*!`；（保留 `robonix_scribe::info(&tag, &line)` 函数调用 — 动态 tag 场景） |
| `tools/rbnx/src/database.rs` | `log::*!` → `robonix_scribe::*!` |
| `tools/rbnx/src/manifest.rs` | 同上 |
| `tools/rbnx/src/cmd/codegen.rs` | 同上 |
| `tools/rbnx/src/cmd/chat.rs` | 同上 |
| `tools/rbnx/Cargo.toml` | 移除 `log` |
| 根 `Cargo.toml` | 移除 `log = "0.4"` |

### 新的调用模式

```rust
// 组件 main.rs — 一次 init 设置进程级 tag
robonix_scribe::init("executor");
robonix_scribe::info!("robonix-executor starting");
robonix_scribe::info!("connecting to atlas at {}", addr);
robonix_scribe::warn!("heartbeat failed: {e:#}");

// 动态 tag 场景（如 process.rs 按包名打标签）— 继续用函数
robonix_scribe::info(&package_tag, &line);
robonix_scribe::log(Level::Info, &custom_tag, "custom message");
```

### 行为变化

| 场景 | 之前 | 之后 |
|------|------|------|
| 组件启动日志 | `robonix_scribe::info("executor", "starting")` | `init("executor")` + `info!("starting")` |
| 组件业务日志 | `log::info!("msg")` → `_default.log` | `robonix_scribe::info!("msg")` → `executor.log` |
| `log` facade backend | `#[ctor]` 自动注册 | 移除 — Scribe 宏直接替代 |
| 无 `init()` 时的 tag | `"_default"` | `"_default"`（向后兼容） |
| tag 重复指定 | 每次调用传 tag | `init()` 一次，宏自动取 |

### 审查

- `cargo build --workspace` 通过
- `cargo clippy --workspace --all-targets -- -D warnings` 干净
- `cargo fmt --all` 已执行
- `cargo test -p robonix-scribe` 21 项全部通过（新增 2 项：`default_tag_is_default_until_init`、`macro_info_uses_default_tag`）
- 全仓库无残留 `log::` 导入或调用（`grep -rn "log::" system/ tools/` 返回空）
- 全仓库无残留 `use log` 导入
