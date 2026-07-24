# Scribe 实现 TODO

> **状态：全部完成 ✅** (2026-06-17)
> 详细实现记录见 [step.md](step.md)

排序原则：核心数据模型 → 被依赖的基础模块 → 核心函数组装 → 集成适配 → 扩展工具与迁移。

---

## 1. ✅ 基础数据模型：Level / LogRecord

- **TODO** 定义 Level 枚举与 LogRecord 结构体 - **依赖项**：无
- **优先级**：高
- **功能说明**
  这是 Scribe 所有模块的共同数据类型依赖，必须先落地。`Level` 为四个变体的枚举，
  `LogRecord` 为四条字段的结构体，不含任何行为逻辑。时间戳来源在后续 TODO 中接入
  chronos；此处只定义类型与 `Display` / `Serialize` derive。
- **需要修改的文件**
  - `system/scribe/src/lib.rs`（新建 `Level` / `LogRecord`，添加 `#[derive(Debug, Clone, Serialize)]` 等）
  - `system/scribe/Cargo.toml`（如需加 `serde` 依赖）
- **需要实现的功能 api**
  ```rust
  enum Level { Debug, Info, Warn, Error }
  struct LogRecord { ts: u64, level: Level, tag: String, msg: String }
  ```
  以及 `Level` 的单字符缩写方法（`D`/`I`/`W`/`E`，供 console 格式化使用）和
  `Display` 实现。
- **需要生成的测试**
  - `Level` 单字符缩写正确性
  - `LogRecord` JSON 序列化/反序列化往返

---

## 2. ✅ Console 格式化器

- **TODO** 实现 logcat 风格 console 行格式化 - **依赖项**：TODO 1
- **优先级**：高
- **功能说明**
  将 `LogRecord` 渲染为单行 human-readable 文本写入 stderr。格式遵循 describe.md
  第 6 节的 logcat 风格：`MM-DD HH:MM:SS.sss  L tag   msg`，级别用单字母（`D`/`I`/`W`/`E`），
  tag 列宽固定以保持对齐。时间戳由纳秒 u64 换算为本地时间。
- **需要修改的文件**
  - `system/scribe/src/format.rs`（新建）
- **需要实现的功能 api**
  ```rust
  fn format_console(record: &LogRecord) -> String;
  fn write_console(record: &LogRecord);  // 直接 write_all 到 stderr
  ```
- **需要生成的测试**
  - 各级别单字母正确
  - 输出行宽对齐（不同长度 tag）
  - 时间戳格式正确性（给定已知纳秒值，验证输出的月-日 时:分:秒.毫秒）

---

## 3. ✅ Per-tag 文件 Sink

- **TODO** 实现按 tag 分文件的 JSON 行写入 - **依赖项**：TODO 1
- **优先级**：高
- **功能说明**
  维护一个 tag → `File` 的 HashMap。首次遇到某个 tag 时在 `LOG_DIR` 下创建
  `{tag}.log` 并缓存句柄；后续复用。每行写入一个 JSON 序列化的 `LogRecord` +
  换行符，行级 flush。Mutex 保护 HashMap 保证线程安全。
  注意：此模块只负责"已确定 LOG_DIR 后的文件写入"，LOG_DIR 的确定由 TODO 4 的
  懒初始化完成。
- **需要修改的文件**
  - `system/scribe/src/sink.rs`（新建）
- **需要实现的功能 api**
  ```rust
  struct FileSink { dir: PathBuf, writers: Mutex<HashMap<String, File>> }
  impl FileSink {
      fn new(log_dir: PathBuf) -> io::Result<Self>;  // create_dir_all
      fn write(&self, record: &LogRecord) -> io::Result<()>;
  }
  ```
- **需要生成的测试**
  - 首次写入某 tag 后文件存在且内容正确
  - 同一 tag 多次写入均为追加
  - 两个不同 tag 写入各自文件，内容不混淆
  - 并发写入同一 tag（多线程）不丢行、不行断裂

---

## 4. ✅ log() 核心函数 + 懒初始化 + 便捷封装

- **TODO** 组装 log() 自由函数、内部懒初始化、debug/info/warn/error 便捷封装 - **依赖项**：TODO 2, TODO 3
- **优先级**：高
- **功能说明**
  这是 Scribe 的**唯一对外入口**。内部使用 `LazyLock` / `OnceLock` 在首条 `log()`
  调用时完成：
  1. 读环境变量 `SCRIBE_LOG_DIR`，缺省则用 `"./logs"`；
  2. 创建目录；
  3. 初始化全局 `FileSink`。
  之后每条 `log()` 调用：取 chronos `now()` → 构造 `LogRecord` →
  `write_console()` → `file_sink.write()`。
  便捷封装 `debug(tag, msg)` / `info(tag, msg)` / `warn(tag, msg)` / `error(tag, msg)`
  内部直接调 `log()`。
- **需要修改的文件**
  - `system/scribe/src/lib.rs`（主入口：`log()` + 四个便捷函数 + 内部 `static` 状态）
- **需要实现的功能 api**
  ```rust
  // 对外唯一接口
  pub fn log(level: Level, tag: &str, msg: &str);

  // 便捷封装（内部调 log()）
  pub fn debug(tag: &str, msg: &str);
  pub fn info(tag: &str, msg: &str);
  pub fn warn(tag: &str, msg: &str);
  pub fn error(tag: &str, msg: &str);
  ```
  调用方不能也不需要调用任何 init / setup / flush 函数。不存在 `scribe::init()`。
  时间戳来源：调用 `chronos::now()` 取纳秒 u64。

- **需要生成的测试**
  - 首条 `log()` 触发懒初始化，`LOG_DIR` 目录被创建
  - 日志同时出现在 stderr（console 格式）和 `logs/{tag}.log`（JSON 格式）
  - `SCRIBE_LOG_DIR` 环境变量控制日志目录
  - 未设环境变量时默认写入 `./logs/`
  - 便捷封装 `info("atlas", "msg")` 等价于 `log(Level::Info, "atlas", "msg")`
  - 多线程并发调 `log()` 不 panic、不丢行

---

## 5. ✅ rbnx boot 适配

- **TODO** rbnx boot：不再重定向子进程 fd，改为设置 SCRIBE_LOG_DIR 环境变量 - **依赖项**：TODO 4
- **优先级**：高
- **功能说明**
  当前 `deploy.rs` 的 `spawn_system_binary()` 和 `spawn_package()` 手动创建 log 文件
  并把子进程 stdout/stderr 重定向进去。Scribe 落地后，`rbnx boot` 不再做 fd
  重定向；改为在 spawn 子进程前设置 `SCRIBE_LOG_DIR` 环境变量指向
  `<manifest_dir>/rbnx-boot/logs`。子进程内的 Scribe 读到该变量后自动写入正确路径。
  boot 启动时的 stale `*.log` 清理逻辑保留。子进程 stdout/stderr 可保留用于
  panic backtrace 等非日志输出。
  此外，需在 `Cargo.toml` 中将 `system/scribe` 添加为 `rbnx` 的依赖（`rbnx boot`
  自身也使用 Scribe 输出自身日志），但 `rbnx boot` 的日志 tag 固定为 `"bootstrap"`。
- **需要修改的文件**
  - `tools/rbnx/src/cmd/deploy.rs`：删除 `spawn_system_binary()` 和 `spawn_package()`
    中的 `File::create` + `Stdio::from(log)` 逻辑；在 `cmd` 构建时添加
    `.env("SCRIBE_LOG_DIR", log_dir)`；保留 stale log 清理
  - `tools/rbnx/Cargo.toml`：添加 `robonix-scribe` 依赖
  - `tools/rbnx/src/main.rs`：`rbnx` 自身启动时调用 `scribe::info("bootstrap", ...)`
    记录 boot 开始/结束
- **需要实现的功能 api**
  无新增 API。修改现有关键路径：
  - `spawn_system_binary()` 去掉 `File::create` + `Stdio::from(log)` + `Stdio::from(err)`
  - `spawn_package()` 同上
  - spawn 前统一 `.env("SCRIBE_LOG_DIR", log_dir)`
- **需要生成的测试**
  - 集成测试：`rbnx boot` 启动后确认日志目录无变化（仍为 `rbnx-boot/logs/`）
  - 确认子进程日志文件由 Scribe 写入（而非父进程 fd 重定向）
  - 确认 stale log 清理仍有效

---

## 6. ✅ Python 侧 scribe_logger

- **TODO** robonix-api 中暴露与 Rust Scribe 同语义的 Python logger - **依赖项**：TODO 4
- **优先级**：高
- **功能说明**
  Python primitive / service 组件需要与 Rust 组件一致的日志入口。
  `robonix_api` 新增 `scribe_logger` 模块，提供模块级函数 `log(level, tag, msg)`
  与便捷封装 `debug/info/warn/error(tag, msg)`。内部逻辑：
  - 读 `SCRIBE_LOG_DIR` 环境变量决定日志目录（与 Rust 侧一致）
  - 首次调用时懒创建目录并初始化
  - console 输出到 stderr（logcat 风格单行）
  - 文件输出到 `{SCRIBE_LOG_DIR}/{tag}.log`（JSON 行）
  - 线程安全：使用 `threading.Lock` 保护 per-tag 文件句柄 dict
  时间戳：过渡期用 `time.time_ns()`，后续切 chronos Python binding。
- **需要修改的文件**
  - `pylib/robonix-api/robonix_api/scribe_logger.py`（新建）
  - `pylib/robonix-api/robonix_api/__init__.py`（导出 scribe_logger）
- **需要实现的功能 api**
  ```python
  class Level(Enum):
      DEBUG = "D"
      INFO  = "I"
      WARN  = "W"
      ERROR = "E"

  def log(level: Level, tag: str, msg: str) -> None: ...
  def debug(tag: str, msg: str) -> None: ...
  def info(tag: str, msg: str) -> None: ...
  def warn(tag: str, msg: str) -> None: ...
  def error(tag: str, msg: str) -> None: ...
  ```
- **需要生成的测试**
  - `SCRIBE_LOG_DIR` 未设时默认 `./logs/`
  - `SCRIBE_LOG_DIR` 设定后文件写入正确路径
  - console 输出格式与 Rust 侧一致
  - JSON 行格式与 Rust 侧一致
  - 多线程并发写不丢行

---

## 7. ✅ rbnx logs CLI

- **TODO** 实现 `rbnx logs` 子命令，按 tag / 级别过滤查看日志 - **依赖项**：TODO 3（文件格式稳定后）
- **优先级**：中
- **功能说明**
  `rbnx logs` 读取 `rbnx-boot/logs/`（或 `--log-dir` 指定）下的 JSON 行日志文件，
  支持：
  - `--tag` / `-t`：按 tag 过滤（可多次指定，OR 语义）
  - `--level` / `-l`：最低级别过滤（如 `--level warn` 只显示 Warn + Error）
  - `--follow` / `-f`：tail -f 模式
  - 默认渲染为 logcat 风格 console 格式；`--json` 输出原始 JSON 行
  不指定 tag 时读取所有 `*.log` 文件，按时间戳合并排序输出。
- **需要修改的文件**
  - `tools/rbnx/src/cmd/mod.rs`（添加 `Logs` 子命令定义）
  - `tools/rbnx/src/cmd/logs.rs`（新建，核心实现）
- **需要实现的功能 api**
  ```rust
  // CLI 接口
  rbnx logs                    # 所有日志，时间排序
  rbnx logs --tag atlas        # 只看 atlas
  rbnx logs --level warn       # Warn + Error
  rbnx logs --follow           # tail -f
  rbnx logs --json             # 原始 JSON 输出
  ```
- **需要生成的测试**
  - 按 tag 过滤正确（只输出匹配 tag 的行）
  - 按 level 过滤正确（`--level warn` 不输出 Info/Debug）
  - 多文件时间戳合并排序正确
  - `--json` 输出为合法 JSON 行
  - 空目录不报错

---

## 8. ✅ 首批组件迁移

- **TODO** 将第一批 Rust 和 Python 组件从 env_logger / logging 迁移到 Scribe - **依赖项**：TODO 4, TODO 6
- **优先级**：中
- **功能说明**
  选取少量组件验证 Scribe 端到端可用，不追求全覆盖。首批建议：
  - Rust：`atlas`（系统内置，调用点少，日志语句明确）
  - Python：`tiago_chassis`（primitive driver，单文件，调用点清晰）
  迁移方式：删除 `env_logger` / `logging.basicConfig` 初始化，将 `info!()` /
  `logger.info()` 替换为 `scribe::info(tag, msg)` / `scribe_logger.info(tag, msg)`。
  验证 `rbnx boot` 启动后日志文件由 Scribe 写入，格式符合 describe.md 约定。
- **需要修改的文件**
  - `system/atlas/src/main.rs`：删 env_logger 初始化，调 `scribe::info("atlas", ...)`
  - `system/atlas/Cargo.toml`：添加 `robonix-scribe` 依赖
  - `examples/webots/primitives/tiago_chassis/chassis_driver/driver.py`：替换 logging 调用
- **需要实现的功能 api**
  无新增 API。替换现有日志调用点。
- **需要生成的测试**
  - 集成测试：`rbnx boot` 后检查 `rbnx-boot/logs/atlas.log` 和
    `rbnx-boot/logs/primitive_tiago_chassis.log` 格式正确
  - 确认旧日志框架的初始化代码已删除

---

## 9. ✅ chronos 时间戳接入

- **TODO** Scribe 时间戳正式切换为 chronos `now()` - **依赖项**：TODO 4, chronos 可用
- **优先级**：中
- **功能说明**
  当前过渡期 Rust 侧直接调 chronos `now()`（已在 TODO 4 中包含），Python 侧用
  `time.time_ns()`。chronos 完成（issue #62）后，Python 侧通过 chronos Python
  binding 获取统一时钟，确保 Rust / Python 日志时间戳同源、可直接按时间排序合并。
- **需要修改的文件**
  - `system/scribe/src/lib.rs`（若 chronos 调用方式有变）
  - `pylib/robonix-api/robonix_api/scribe_logger.py`（`time.time_ns()` → chronos binding）
- **需要实现的功能 api**
  无新增 Scribe API。仅替换时间戳获取调用。
- **需要生成的测试**
  - Rust 侧与 Python 侧同一秒内产生的日志，纳秒时间戳差值在合理范围内

---

## 实现顺序

```
Phase 1 ── 基础模块（无依赖，可并行）
  TODO 1  数据模型 Level / LogRecord
  TODO 2  Console 格式化器           ← 依赖 TODO 1
  TODO 3  Per-tag 文件 Sink          ← 依赖 TODO 1

Phase 2 ── 核心组装（依赖 Phase 1 全部完成）
  TODO 4  log() + 懒初始化 + 便捷封装  ← 依赖 TODO 2, 3

Phase 3 ── 集成适配（依赖 TODO 4，可并行）
  TODO 5  rbnx boot 适配             ← 依赖 TODO 4
  TODO 6  Python scribe_logger       ← 依赖 TODO 4

Phase 4 ── 扩展与迁移（依赖 Phase 3）
  TODO 7  rbnx logs CLI              ← 依赖 TODO 3（文件格式稳定）
  TODO 8  首批组件迁移               ← 依赖 TODO 4, 6
  TODO 9  chronos 接入               ← 依赖 TODO 4, chronos 就绪
```

**可并行点**：
- TODO 2 和 TODO 3 可在 TODO 1 完成后并行
- TODO 5 和 TODO 6 可在 TODO 4 完成后并行
- TODO 7 可在 TODO 3 完成后与 Phase 3 并行启动
