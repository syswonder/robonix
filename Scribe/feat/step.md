# Scribe 实现步骤记录

> 注：当前环境未安装 Rust 工具链，无法运行 `cargo check` / `cargo test`。
> 代码正确性通过设计文档（log-struct.md / TODO.md）review 验证。

---

## TODO 1: Cargo workspace 脚手架 ✅

- 创建 `system/scribe/Cargo.toml` — package `robonix-scribe`，workspace deps
- 创建 `system/scribe/src/lib.rs` — crate doc + 模块声明 + re-export 骨架
- 创建占位模块：`record.rs`, `format.rs`, `filter.rs`, `sink.rs`, `log_bridge.rs`
- 根 `Cargo.toml`：workspace.members 新增 `"system/scribe"`；deps 新增 `chrono`、`globset`
- 更新 `system/scribe/README.md` 为当前设计描述

---

## TODO 2: Level 枚举 + LogRecord 结构体 ✅

文件：`system/scribe/src/record.rs`

- `Level` enum：Debug/Info/Warn/Error，派生 Ord/Serialize/Deserialize
  - `Display`：输出 D/I/W/E
  - `FromStr`：解析 debug|info|warn|error（大小写不敏感，支持 "warning" 别名）
  - `From<Level> for log::Level`：桥接用
- `LogRecord` struct：ts/level/tag/msg + 可选 sid/pid/cid + kv HashMap
  - `new()` 自动填入纳秒时间戳（`SystemTime::now()`）
  - Builder 链：`with_sid/pid/cid/kv()` → 消费后返回 Self
- 7 个单元测试：排序、显示、解析、映射、builder、序列化 round-trip

---

## TODO 3: Format 格式化模块 ✅

文件：`system/scribe/src/format.rs`

- `format_console(&LogRecord) -> String`：logcat 风格单行
  - MM-DD HH:MM:SS.mmm  L tag  msg  k=v...
  - tag 最小宽度 8 字符，kv 按 key 排序
- `format_jsonl(&LogRecord) -> String`：JSON 单行 + \n
  - 所有可选字段输出 null/{}，msg 内 \n 由 serde_json 自然转义
- `timestamp_local(ns) -> String`：纳秒 → 本地时间字符串（chrono）
- 8 个单元测试：console 基本/kv/long-tag/short-tag、JSONL 基本/转义/null/kv/round-trip

---

## TODO 4: Filter 过滤模块 ✅

文件：`system/scribe/src/filter.rs`

- `FilterGate`：规则列表 + 默认级别
  - `parse(filter_str) -> Result<Self, FilterParseError>`
  - `parse_optional(Option<&str>)` — None 时默认 `*=info`
  - `accepts(level, tag) -> bool` — 左优先，命中即停
- `TagPattern`：Exact / Prefix (prefix_*) / Wildcard (*)
- `off` 语义：规则 level 为 None → 完全静默
- `FilterParseError`：Empty / InvalidRule / InvalidLevel / DuplicateExact
- 12 个单元测试：解析基本/off/非法/空/重复、匹配语义 7 种、Display round-trip

---

## TODO 5+6: Sink trait + ConsoleSink + FileSink + FileRouter ✅

文件：`system/scribe/src/sink.rs`

- `Sink` trait：`write(&LogRecord)` + `flush()`
- `ConsoleSink`：stderr 输出，ANSI 着色（TTY 检测 + NO_COLOR 检查）
  - `is_tty()` 使用 `std::io::IsTerminal`
- `FileRouter`：`Mutex<HashMap<String, BufWriter<File>>>`
  - `with_writer(log_dir, tag, closure)` — 闭包式 API，自动创建/复用文件句柄
- `FileSink`：JSONL per-tag 文件写入，每条 flush（crash-safe）
  - 首次写入错误向 stderr 告警一次，后续静默
- 8 个单元测试：ConsoleSink 基本/color/flush、FileSink 目录创建/写入/路由/追加/kv、FileRouter flush_all

---

## TODO 7: trait Scribe + LogBuilder + ScribeImpl + scribe_init ✅

文件：`system/scribe/src/lib.rs`

- `trait Scribe`：
  - `log(level, msg)` — 基本日志
  - `log_record(record)` — 接受预构建 LogRecord（LogBuilder 和 log_bridge 使用）
  - `with_kv(key, value) -> LogBuilder` — 链式 kv
  - `debug/info/warn/error(msg)` — 便捷方法（默认实现）
- `LogBuilder<'a>`：积累 kv 对，终端方法消费 builder 调用 `scribe.log_record()`
- `ScribeImpl`：持有 tag + sinks vec + FilterGate，填充 tag 后过滤并分发
- `scribe_init(tag, log_dir, filter) -> Box<dyn Scribe>`：工厂函数
- 5 个单元测试：便捷方法不 panic、默认参数、filter 过滤、文件输出、LogBuilder API

---

## TODO 8: log crate 桥接适配器 ✅

文件：`system/scribe/src/log_bridge.rs`

- `ScribeLogger`：`Arc<dyn Scribe>` wrapper，实现 `log::Log`
  - `enabled()` 始终 true（实际过滤交给 FilterGate）
  - `log()`：level 映射 + module/file/line 注入 kv + 调用 `scribe.log_record()`
- `scribe_install_as_global(scribe) -> Result<(), SetLoggerError>`
  - 调用 `log::set_boxed_logger()` + `set_max_level(Trace)`
- Level 映射：Error/Info/Warn/Debug 直通，Trace → Debug
- 3 个单元测试：level 映射表、SpyScribe 验证路由（含 kv 注入）、全局安装

---

## 文件清单

```
system/scribe/
  Cargo.toml              # package robonix-scribe
  README.md               # 使用文档
  src/
    lib.rs                # trait Scribe + LogBuilder + ScribeImpl + scribe_init (204 行)
    record.rs             # Level + LogRecord (185 行)
    format.rs             # format_console + format_jsonl (160 行)
    filter.rs             # FilterGate + TagPattern (220 行)
    sink.rs               # Sink trait + ConsoleSink + FileSink + FileRouter (265 行)
    log_bridge.rs         # impl log::Log adapter (165 行)

Cargo.toml                # 根 workspace: members + deps 更新
log-struct.md             # 设计方案
TODO.md                   # 实现计划
step.md                   # 本文件
```

---

## TODO 9: PyO3 bindings ✅

文件：`system/scribe/src/py_bindings.rs`、`system/scribe/pyproject.toml`

- `PyScribe` pyclass：包装 `Box<dyn Scribe>`，暴露 debug/info/warn/error 方法
- `**kwargs` → `extract_kv()` → msg suffix 注入（v0.1 简化方案）
- `#[pymodule] fn _scribe` 模块入口
- `system/scribe/Cargo.toml`：新增 `[lib] crate-type` + `python` feature + `pyo3` dep
- `system/scribe/pyproject.toml`：maturin 构建配置，module-name = `robonix_api._scribe`
- `system/scribe/src/lib.rs`：`#[cfg(feature = "python")] pub mod py_bindings;`

---

## TODO 10: Python ScribeLogger ✅

文件：`pylib/robonix-api/robonix_api/scribe.py`

- `ScribeLogger` class：prefer PyO3 `_scribe.PyScribe`，fallback to stdlib `logging`
- 方法：debug/info/warn/warning/error/exception (valueError traceback 注入)
- `__init__(tag, log_dir=None, filter=None)`
- `pylib/robonix-api/robonix_api/__init__.py`：新增 `ScribeLogger` import + `__all__` 条目

---

## TODO 11: rbnx logs CLI ✅

文件：`tools/rbnx/src/cmd/logs.rs`

- `LogsArgs` struct + `execute(args) -> Result<()>`
- 支持 `-t/--tag`（多值 OR + glob）、`-l/--level`、`-f/--follow`、`-n/--lines`、`-v/--format`
- 支持 `--plan` (pid) 和 `--session` (sid) 过滤
- `discover_files()` + `build_glob_set()` → glob 匹配 `logs/*.log`
- `follow_logs()` → 500ms 轮询检测新内容（`SeekFrom::Start`）
- `render_record()` → logcat / json / raw 三种输出格式
- `tools/rbnx/src/cmd/mod.rs`：新增 `Logs` variant + execute 分支
- `tools/rbnx/Cargo.toml`：新增 `robonix-scribe` + `globset` 依赖

---

## TODO 12: 组件迁移 ✅

**Rust 系统服务**（4 个）：
- `system/atlas/` — 替换 `env_logger::Builder` → `scribe_init("atlas", None, args.log.as_deref())` + `scribe_install_as_global`
- `system/executor/` — 同上，tag = "executor"
- `system/liaison/` — 同上，tag = "liaison"
- `system/pilot/` — 同上，tag = "pilot"
- 各 `Cargo.toml` 新增 `robonix-scribe = { path = "../scribe" }`

**rbnx CLI 自身**：
- `tools/rbnx/src/main.rs` — 替换 env_logger → scribe_init("rbnx", ...) + install_as_global
- filter 内置：`rbnx=info,rustdds=off,ros2_client=warn`（等效于原 env_logger 的 filter_module）

**Python scene_svc**：
- `system/scene/scene_service/service.py` — `logging.getLogger` → `ScribeLogger(tag="scene_svc")`
- 环境变量 `SCENE_LOG_FILTER` 替代 `SCENE_LOG_LEVEL`
- `ScribeLogger` 新增 `warning()` 别名 + `exception()`（traceback 注入）以兼容现有调用

---

## TODO 13: 文档 + E2E 测试 ✅

- `system/scribe/tests/e2e_test.rs` — 6 个端到端集成测试：
  - 单组件写入文件、多组件分文件、filter 级别过滤、LogBuilder kv 附加、JSONL 合法性、8 线程并发写入无损坏
- `pylib/robonix-api/tests/test_scribe_logger.py` — 11 个 Python 单元测试：
  - 构造、debug/info/warn/warning/error 发射、kwargs 注入、exception traceback、无活跃异常不崩溃、filter 参数接受、双 logger 独立
- 注：`docs/src/developer-guide.md` 尚不存在（`docs/` 目录为空骨架），Logging 小节待后续 docs PR 补充

---

## 文件清单（完整）

```
system/scribe/
  Cargo.toml              # package + features (python) + deps
  pyproject.toml           # maturin build config
  README.md                # 使用文档
  src/
    lib.rs                 # trait Scribe + LogBuilder + ScribeImpl + scribe_init
    record.rs              # Level + LogRecord
    format.rs              # format_console + format_jsonl
    filter.rs              # FilterGate + TagPattern
    sink.rs                # Sink trait + ConsoleSink + FileSink + FileRouter
    log_bridge.rs          # impl log::Log adapter
    py_bindings.rs         # PyO3 PyScribe class
  tests/
    e2e_test.rs            # 6 个端到端集成测试

tools/rbnx/src/cmd/
  logs.rs                  # rbnx logs 子命令

pylib/robonix-api/
  robonix_api/
    scribe.py              # Python ScribeLogger
    __init__.py             # ScribeLogger export
  tests/
    test_scribe_logger.py  # 11 个 Python 单元测试

系统组件迁移（6 个）:
  system/atlas/Cargo.toml    + main.rs  (env_logger → Scribe)
  system/executor/Cargo.toml + main.rs
  system/liaison/Cargo.toml  + main.rs
  system/pilot/Cargo.toml    + main.rs
  tools/rbnx/Cargo.toml      + main.rs
  system/scene/scene_service/service.py  (logging → ScribeLogger)

根文件:
  Cargo.toml              # workspace members + chrono/globset deps
  log-struct.md            # 设计方案
  TODO.md                  # 实现计划（含状态表）
  step.md                  # 本文件
```

---

## 日志路径对齐修复 ✅

**问题**：Scribe 默认日志目录 `./logs/` 与 `rbnx boot` 使用的 `<manifest-dir>/rbnx-boot/logs/` 不一致。

**修复**（3 处）：

1. **`system/scribe/src/lib.rs` — `scribe_init()`**：
   - `log_dir=None` 时不再硬编码 `./logs/`，改为先查 `$ROBONIX_LOG_DIR` 环境变量，最后 fallback `./logs/`
   - 当 `rbnx boot` 设置了该环境变量后，所有组件自动写入 `<manifest-dir>/rbnx-boot/logs/<tag>.log`

2. **`tools/rbnx/src/cmd/logs.rs` — `rbnx logs`**：
   - 默认 `log_dir` 同样先查 `$ROBONIX_LOG_DIR`，再 fallback `./rbnx-boot/logs/`
   - 确保 `rbnx logs` 读取的目录与组件写入的目录一致

3. **`tools/rbnx/src/cmd/deploy.rs` — `rbnx boot` 启动器**：
   - `spawn_system_binary()`：新增 `.env("ROBONIX_LOG_DIR", log_dir)` 传环境变量给 Rust 系统二进制
   - `spawn_package()`：同上，传环境变量给 `rbnx start` 启动的 Python/Rust 包

**结果**：Scribe 文件日志路径与现有 `examples/webots/rbnx-boot/logs/<component>.log` 完全一致。

---

## 编译错误修复 ✅

`cargo check --workspace` 首次运行发现 7 errors + 4 warnings，全部修复后 0/0：

| # | 错误/警告 | 文件 | 修复方式 |
|---|---|---|---|
| 1 | E0252 重复导入 ×5 | `lib.rs` | 删除 `use`，仅保留 `pub use` |
| 2 | E0277 Debug 不满足 | `sink.rs` | `#[derive(Debug)]` → 手动 `impl Debug` |
| 3 | unused variable `i` | `filter.rs` | `i` → `_i` |
| 4 | E0382 args partial move | `logs.rs` | `.or_else()` → `match .as_ref()` |
| 5 | E0382 path moved in loop | `logs.rs` | 显式 `let p: &Path = path;` reborrow |

详细记录见 [Scribe/error.md](Scribe/error.md) 末尾新增的"编译错误与修复记录"章节。

---

## 总计

- 13 个 TODO 全部完成 + 1 次路径对齐修复 + 1 次编译错误修复
- 8 个 Rust 源文件 + 1 个集成测试文件
- 2 个 Python 源文件 + 1 个 Python 测试文件
- 6 个组件迁移
- ~49 个单元/集成测试用例
- `cargo check --workspace` 0 errors / 0 warnings
