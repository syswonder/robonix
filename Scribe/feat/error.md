# error: failed to compile `robonix-cli` — OpenSSL 开发库缺失

## 错误现象

```
error: failed to compile `robonix-cli v0.1.0 (/home/hyl/robonix/tools/rbnx)`
make: *** [Makefile:68：install] 错误 101
```

`make install` 构建到 `robonix-cli` 时失败，`openssl-sys` crate 报错：

```
Could not find openssl via pkg-config:
  Package openssl was not found in the pkg-config search path.
The system library `openssl` required by crate `openssl-sys` was not found.
```

## 原因

Robonix Rust 工作区中某个 crate（被 `robonix-cli` 直接或间接依赖）依赖了 `openssl-sys` crate。`openssl-sys` 在编译时需要链接系统的 OpenSSL **开发库**（头文件 + 静态/动态链接所需 `.pc` 文件），通过 `pkg-config` 查找 `openssl.pc`。

当前系统状态：

| 包 | 状态 |
|----|------|
| `libssl3`（运行时 .so） | 已安装 |
| `openssl`（CLI 工具） | 已安装 |
| `libssl-dev`（C 头文件 + pkg-config `.pc`） | **缺失** ← 根因 |

没有 `libssl-dev`，`pkg-config --libs --cflags openssl` 找不到 `openssl.pc`，`openssl-sys` 的 build script 直接报错退出，导致整个 workspace 编译失败。

`Makefile:68` 对应的是 `cargo install` 命令，它依赖 workspace 中所有 crate 先编译成功，所以一个依赖的 sys crate 失败就会阻断全局。

## 解决方案

安装 OpenSSL 开发包：

```bash
sudo apt install libssl-dev
```

安装后验证：

```bash
pkg-config --libs --cflags openssl
# 预期输出类似: -lssl -lcrypto
```

然后重新运行：

```bash
make install
```

---

# Docker 拉取镜像超时 — registry-1.docker.io 不可达

## 错误现象

```text
=> ERROR [internal] load metadata for docker.io/osrf/ros:humble-desktop-full
------
failed to solve: DeadlineExceeded: osrf/ros:humble-desktop-full:
failed to resolve source metadata for docker.io/osrf/ros:humble-desktop-full:
failed to do request: Head "https://registry-1.docker.io/v2/osrf/ros/manifests/humble-desktop-full":
dial tcp 103.228.130.27:443: i/o timeout
```

`bash examples/webots/sim/start.sh` 在 `docker compose build` 阶段失败，Docker 无法从 Docker Hub 拉取基础镜像 `osrf/ros:humble-desktop-full`。

触发位置：[Dockerfile:8](examples/webots/sim/bridge/Dockerfile#L8)

## 原因

Docker 默认从 `registry-1.docker.io`（Docker Hub）拉取镜像。当前网络环境（GFW 后的国内网络）下，到 Docker Hub 的 TCP 443 连接被墙/严重丢包，`Head` 请求超时（默认 30 s），`docker compose build` 无法获取基础镜像的 manifest。

当前 `/etc/docker/daemon.json` 只配置了 NVIDIA runtime，**未配置 registry mirror**。

> 说明：Dockerfile 内部的 apt/pip 源已通过 [Dockerfile:11-28](examples/webots/sim/bridge/Dockerfile#L11) 切换到了 TUNA 镜像，但 **`FROM` 行的拉镜像动作发生在 Docker Engine 层面**，不受 Dockerfile 内 `RUN` 指令的影响——registry mirror 必须在 daemon 层配置。

## 解决方案

### 方案一：配置 Docker registry mirror（推荐，一劳永逸）

编辑 `/etc/docker/daemon.json`，添加国内可用镜像加速器：

```json
{
    "registry-mirrors": [
        "https://docker.1ms.run",
        "https://docker.xuanyuan.me"
    ],
    "runtimes": {
        "nvidia": {
            "args": [],
            "path": "nvidia-container-runtime"
        }
    }
}
```

然后重启 Docker：

```bash
sudo systemctl restart docker
```

验证 mirror 生效：

```bash
docker info | grep -A5 "Registry Mirrors"
```

重新运行：

```bash
bash examples/webots/sim/start.sh
```

### 方案二：手动 docker pull（单次绕过） - 不行

如果 mirror 不可用，可通过代理/VPN 手动拉取：

```bash
export HTTP_PROXY="http://127.0.0.1:7890"
export HTTPS_PROXY="http://127.0.0.1:7890"
# 开代理后
docker pull osrf/ros:humble-desktop-full
```

然后 `docker compose build` 会直接使用本地已有的镜像，不再走 registry。

### 验证

```bash

docker image ls osrf/ros:humble-desktop-full
# 预期存在该镜像
```

---

# Webots 启动崩溃 — X11 display `:0` 认证失败 (Qt xcb plugin 无法加载)

## 错误现象

```
[webots-1] Authorization required, but no authorization protocol specified
[webots-1] Warning: could not connect to display :0
[webots-1] Info: Could not load the Qt platform plugin "xcb" in "" even though it was found.
[webots-1] Fatal: This application failed to start because no Qt platform plugin could be initialized.
[webots-1] Available platform plugins are: xcb, wayland, wayland-egl.
[webots-1] /usr/local/webots/webots: line 105:   106 Aborted  (core dumped)
[ERROR] [webots-1]: process has died [pid 94, exit code 134]
```

容器内 Webots 进程启动后立即崩溃。Qt 找到了 xcb 平台插件但因为 X11 连接失败无法加载，Webots 退出码 134（SIGABRT）。

## 原因

**根因：容器内 `DISPLAY=:0`，但当前环境可用的 X display 是 `:10`。**

当前系统的 X11 布局：

| Display | 身份 | 状态 |
|---------|------|------|
| `:0` | GDM 登录管理器 | Xorg 进程在运行，但需要 GDM 的 auth cookie（`/run/user/127/gdm/Xauthority`），用户无权访问 |
| `:10` | 用户的 xrdp 远程桌面会话 | **可用** — `xset q` 正常、cookie 在 `~/.Xauthority` 中、`/tmp/.X11-unix/X10` 存在 |

`start.sh` 有一段自动检测逻辑（[start.sh:31-41](examples/webots/sim/start.sh#L31)），按 `:0 → :1 → :10` 顺序用 `xset q` 探测。但如果 `xset` 在运行环境中不可用，或 DISPLAY 已被设定为非空值但指向错误 display，检测会被跳过，最终 fallback 到 `: "${DISPLAY:=:0}"`。

本次运行中，`DISPLAY` 被设为 `:0`（GDM screen）。容器拿到这个 DISPLAY 后尝试连接，但 `~/.Xauthority` 里只有 `:10` / `:11`（xrdp）的 MIT-MAGIC-COOKIE，没有 `:0` 的 cookie，X 服务器返回 `Authorization required`。Qt xcb 插件初始化失败，Webots 调 `qFatal` abort。

触发位置：[entrypoint.sh](examples/webots/sim/bridge/entrypoint.sh) 启动 `webots` 时，Qt 尝试连接 `$DISPLAY`（由 [compose.yaml:40](examples/webots/sim/compose.yaml#L40) 注入：`DISPLAY: "${DISPLAY:-:0}"`）。

## 解决方案

### 方案一：显式指定正确的 DISPLAY（推荐）

```bash
export DISPLAY=:10
bash examples/webots/sim/start.sh
```

`start.sh` 检测到 DISPLAY 已非空，跳过探测直接使用 `:10`。

### 方案二：使用 Webots 流模式（适合远端 xrdp 等无 GPU X session 场景）

```bash
ROBONIX_SIM_STREAM=1 bash examples/webots/sim/start.sh
```

此模式启动 Xvfb 虚拟显示 + Webots 流式推送到浏览器，不依赖 host X session。

### 验证

```bash
# 确认容器内 DISPLAY 正确
docker exec robonix_tiago_sim bash -c 'echo $DISPLAY'
# 预期: :10

# 确认 Webots 进程存活
docker exec robonix_tiago_sim bash -c 'pgrep -a webots-bin'
```


---

# `rbnx boot` 构建失败 — `uv` 未安装

pip install grpcio-tools

## 错误现象

```
⚠ memory: not built — `rbnx build` should run before `rbnx boot`. building inline.
[Building] com.robonix.example.memsearch_service via manifest.build
[build] error: 'uv' not found on PATH. Install: https://docs.astral.sh/uv/
Error: inline build of memory at /home/hyl/robonix/services/memsearch failed

Caused by:
    Build exited with status Some(1)
```

`rbnx boot` 在处理 `memory`（`services/memsearch`）包的 inline build 时失败，因为构建脚本依赖 `uv`（Astral 出品的 Python 包管理器）但 `uv` 不在 PATH 中。

## 原因

Robonix 工作区使用 `uv` 管理 Python 依赖（根目录有 `uv.lock` 和 `pyproject.toml`）。`rbnx build` / `rbnx boot` 在构建 Python 包（如 `memsearch`）时会调用 `uv` 来解析和安装依赖。conda 环境 `env_robonix` 中未安装 `uv`。

## 解决方案

```bash
conda activate env_robonix
pip install uv
```

或直接：

```bash
conda run -n env_robonix pip install uv
```

验证：

```bash
conda run -n env_robonix uv --version
# 预期: uv 0.x.x
```

然后重新运行：

```bash
cd examples/webots
rbnx boot
```

---

# `rbnx boot` 3 个组件启动失败（scene / memory / audio_driver）

## 错误现象

```
[  66.091] [FAIL]  scene               registration timeout after 60s
[  73.407] [ ⠹ ]  audio_driver        registering with atlas…
[ 133.453] [FAIL]  memory              registration timeout after 60s

  3 of 13 packages failed to start; the rest are running.
```

---

## 错误 1：scene — Docker 镜像构建超时

### 原因

`scene` 服务的包清单（[package_manifest.yaml](system/scene/package_manifest.yaml)）声明 `build: bash scripts/build.sh`。该构建脚本做了 3 件事：
1. `rbnx codegen`（秒级）
2. 下载模型权重：`yolov8l-world.pt` (~400 MB) + `mobile_sam.pt` (~30 MB)
3. **`docker build` 构建 `robonix-scene` 镜像**，内含 PyTorch 2.7.1 (~1 GB)、torchvision、OpenCV 等——**首次构建约 8–12 分钟**

`rbnx boot` 在 `build` 完成后等待包向 atlas 注册 provider，超时 60 s。scene 的 Docker 镜像构建在 60 s 内远未完成，因此 `start:` 从未被调用，atlas 永远等不到注册。

quickstart 明确提示了这一点（[quickstart.md:80](quickstart.md#L80)）：
> scene 第一次跑要预热：第一次启 sim 之前先 `cd system/scene && bash scripts/build.sh` 把镜像建好（首次约 8–12 分钟）

### 解决方案

**先预建 scene 镜像，再运行 `rbnx boot`：**

```bash
cd system/scene
bash scripts/build.sh
# 等待 8–12 分钟完成
cd ../../examples/webots
rbnx boot
```

> 后续 `rbnx boot` 会复用已有镜像，增量构建只需要几秒。

---

## 错误 2：memory — `httpx` 不识别 `socks://` 代理 scheme

### 原因

memsearch 服务启动时，Python `httpx`（通过 `huggingface_hub` → `sentence-transformers` 链路）读取了环境变量中的代理配置：

```
ValueError: Unknown scheme for proxy URL URL('socks://127.0.0.1:7890/')
```

`httpx` 支持的代理 scheme 只有 `http://`、`https://`、`socks5://`（需额外安装 `socksio`）。`socks://` 不是合法 scheme，`httpx` 解析时直接抛 `ValueError`，导致进程退出（exit code 1）。

代理 `socks://127.0.0.1:7890` 很可能来自运行 `rbnx boot` 的 shell（如 Clash/V2Ray 设置的 `ALL_PROXY` 或 `all_proxy` 环境变量），`rbnx boot` 会将这些环境变量透传给子进程。

### 解决方案

**方案 A：运行 `rbnx boot` 时临时 unset 代理（推荐）**

```bash
unset ALL_PROXY all_proxy HTTPS_PROXY https_proxy HTTP_PROXY http_proxy
rbnx boot
```

（memsearch 内 pip/uv 的包下载发生在 build 阶段，已经走 TUNA 镜像；运行时不需要代理访问外网。）

**方案 B：改用 `http://` scheme**

如果代理支持 HTTP 协议（如 Clash 的 mixed port 通常同时支持 HTTP 和 SOCKS）：

```bash
export https_proxy=http://127.0.0.1:7890
export http_proxy=http://127.0.0.1:7890
rbnx boot
```

**方案 C：安装 `httpx[socks]` 支持 SOCKS5**

```bash
pip install "httpx[socks]"
# 然后把 socks:// 改为 socks5://
export all_proxy=socks5://127.0.0.1:7890
```

---

## 错误 3：audio_driver — 无音频硬件

### 原因

```
[audio_driver] state REGISTERED -> ERROR (no ALSA capture or playback device available)
```

机器没有麦克风和扬声器硬件（`/proc/asound/cards` 为空或只有 HDMI/虚拟设备）。audio_driver 在 `Driver(CMD_INIT)` 阶段检测不到可用音频设备，转入 ERROR 状态。

### 影响

**非关键**：audio_driver 只提供 `mic`、`speaker`、`list_devices` 等能力，失效不影响仿真、导航、探索、VLM 对话等其他功能。其余 10 个组件正常运行。

### 解决方案

- 如果不需要音频能力：**可忽略**，不影响核心功能。
- 如果需要音频：插入 USB 麦克风/音箱后重启 `rbnx boot`。

---

# Scribe 实现 — 编译错误与修复记录

> `cargo check --workspace` 发现 7 个 error + 4 个 warning，均位于新增的 `robonix-scribe` crate 和 `robonix-cli`（`rbnx logs`）。

---

## 错误 1：重复导入（E0252）— `ConsoleSink` / `FileSink` / `Sink` 等

### 错误现象

```
error[E0252]: the name `ConsoleSink` is defined multiple times
  --> system/scribe/src/lib.rs:47
   |
39 | use sink::{ConsoleSink, FileSink, Sink};
   |             ----------- previous import of the type `ConsoleSink` here
...
47 | pub use sink::{ConsoleSink, FileRouter, FileSink, Sink};
   |                ----------- `ConsoleSink` reimported here
```

`FilterGate`、`Level`、`LogRecord`、`Sink` 同样报 E0252。

### 原因

Rust edition 2024 对 `use` 和 `pub use` 同名导入检查更严格。`lib.rs` 中先 `use filter::FilterGate;` 再 `pub use filter::FilterGate;` 被视为重复定义。

### 修复

删除 `use` 语句，仅保留 `pub use`（`pub use` 同时将名称带入当前模块作用域，可满足内部使用和外部导出两个需求）。

修改文件：[system/scribe/src/lib.rs:37-47](system/scribe/src/lib.rs#L37)

```rust
// Before:
use filter::FilterGate;
use record::{Level, LogRecord};
use sink::{ConsoleSink, FileSink, Sink};
pub use filter::FilterGate;
pub use record::{Level, LogRecord};
pub use sink::{ConsoleSink, FileRouter, FileSink, Sink};

// After:
pub use filter::FilterGate;
pub use record::{Level, LogRecord};
pub use sink::{ConsoleSink, FileRouter, FileSink, Sink};
```

---

## 错误 2：`ConsoleSink` 无法 derive `Debug`（E0277）

### 错误现象

```
error[E0277]: `(dyn std::io::Write + Send + 'static)` doesn't implement `std::fmt::Debug`
  --> system/scribe/src/sink.rs:38
   |
38 | #[derive(Debug)]
   |          ----- in this derive macro expansion
39 | pub struct ConsoleSink {
40 |     writer: Mutex<Box<dyn Write + Send>>,
   |     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ `Debug` not implemented
```

### 原因

`#[derive(Debug)]` 要求所有字段实现 `Debug`，但 `Box<dyn Write + Send>` 中的 trait object 不满足 `Debug` bound。同时 `Sink` trait 有 `Debug` 作为 supertrait，所以不能去掉 Debug 实现。

### 修复

手动实现 `fmt::Debug`，将 writer 字段显示为占位描述字符串。

修改文件：[system/scribe/src/sink.rs:38-42](system/scribe/src/sink.rs#L38)

```rust
// Before:
#[derive(Debug)]
pub struct ConsoleSink { ... }

// After:
pub struct ConsoleSink { ... }
impl std::fmt::Debug for ConsoleSink {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ConsoleSink")
            .field("writer", &"<dyn Write + Send>")
            .field("color", &self.color)
            .finish()
    }
}
```

---

## 警告 3：未使用变量 `i` 在 filter.rs

### 错误现象

```
warning: unused variable: `i`
  --> system/scribe/src/filter.rs:75:14
   |
75 |         for (i, part) in filter.split(',').enumerate() {
```

### 修复

`i` → `_i`（[system/scribe/src/filter.rs:75](system/scribe/src/filter.rs#L75)）

---

## 错误 4：`args.log_dir` partial move — logs.rs

### 错误现象

```
error[E0382]: borrow of partially moved value: `args`
  --> tools/rbnx/src/cmd/logs.rs:44
   |
33 |     let log_dir = args.log_dir.or_else(...)
   |                   ------------ `args.log_dir` partially moved due to this method call
44 |     print_logs(&log_dir, &args, min_level)
   |                           ^^^^^ value borrowed here after partial move
```

### 原因

`Option::or_else(self, ...)` 获取了 `args.log_dir` 的所有权（partial move），后续 `&args` 借用失败。

### 修复

改用 `match args.log_dir.as_ref()` 先行 borrow，避免 move。

修改文件：[tools/rbnx/src/cmd/logs.rs:32-35](tools/rbnx/src/cmd/logs.rs#L32)

```rust
// Before:
let log_dir = args.log_dir
    .or_else(|| std::env::var("ROBONIX_LOG_DIR").ok().map(PathBuf::from))
    .unwrap_or_else(|| PathBuf::from("./rbnx-boot/logs"));

// After:
let log_dir = match args.log_dir.as_ref() {
    Some(d) => d.clone(),
    None => std::env::var("ROBONIX_LOG_DIR")
        .ok()
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("./rbnx-boot/logs")),
};
```

---

## 错误 5：`path` 变量被 moved — logs.rs follow 循环

### 错误现象

```
error[E0382]: use of moved value: `path`
   --> tools/rbnx/src/cmd/logs.rs:115
    |
104 |         for (path, last_size) in &mut positions {
    |              ---- move occurs because `path` has type `&mut PathBuf`
105 |             let current_size = match fs::metadata(path) {
    |                                                   ---- value moved here
115 |             let mut file = match File::open(path) {
    |                                             ^^^^ value used here after move
```

### 原因

`for (path, _) in &mut positions` 中 `path` 为 `&mut PathBuf`。`fs::metadata()` 接受 `AsRef<Path>`，对 `&mut PathBuf` 会自动 deref 然后 move。解决方案：显式 reborrow 为 `&Path`。

### 修复

在循环体内显式 `let p: &Path = path;` 然后统一使用 `p`。

修改文件：[tools/rbnx/src/cmd/logs.rs:107-118](tools/rbnx/src/cmd/logs.rs#L107)

```rust
// Before:
for (path, last_size) in &mut positions {
    let current_size = match fs::metadata(path) { ... };
    ...
    let mut file = match File::open(path) { ... };

// After:
for (path, last_size) in &mut positions {
    let p: &Path = path;
    let current_size = match fs::metadata(p) { ... };
    ...
    let mut file = match File::open(p) { ... };
```

---

## 验证

```
$ cargo check --workspace
    Checking robonix-scribe v0.1.0
    Checking robonix-executor v0.1.0
    Checking robonix-atlas v0.1.0
    Checking robonix-liaison v0.1.0
    Checking robonix-pilot v0.1.0
    Checking robonix-cli v0.1.0
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 3.45s
```

0 errors, 0 warnings，全部通过。
