# Examples

Robonix PoC 示例：调用 robonix-server 提供的 query。

## 前置条件

1. 构建 robonix-server：`cargo build --manifest-path robonix-server/Cargo.toml`
2. 启动 robonix-server：`../start_server`（另一终端）

## 示例列表

| 示例 | 说明 |
|------|------|
| **callquery** | Rust 客户端，调用任意已注册的 query |
| **list_interfaces** | 列出 runtime 节点与 channel（gRPC InspectRuntime） |
| **python_ping_client** | Python 客户端，含 manifest、调用 ping query |
| **stream_demo** | Stream 示例：stream_server（pose 发布）+ stream_client（订阅），经 robonix-server 解析连接 |
| **query_demo** | Query 示例：semantic_server（semantic_query 服务端）+ semantic_client（客户端），含示例语义地图实现 |
| **skill_demo** | Command 示例：skill_server（execute 服务端）+ skill_client（客户端），含示例 skill 实现 |

## 运行

```bash
# 列出节点与 channel（需先 build robonix-server）
./list_interfaces
./list_interfaces 127.0.0.1:50051

# 调用 ping（Rust）
./callquery robonix-server robonix/system/debug/ping '"hello"'

# 调用 ping（Python）：rbnx 构建并启动指定 node
rbnx build -p python_ping_client && rbnx start -p python_ping_client -n call_ping
```

## rbnx 控制 package

直接用 rbnx 构建与运行：`rbnx build -p <package>` → `rbnx start -p <package> -n <node>`（每次只启动一个 node，start 会阻塞直到进程退出）。

- `python_ping_client/robonix_manifest.yaml`：manifest 中 `nodes` 列出各 node 的 id、type、entry。

## 测试脚本

需先启动 robonix-server（`../start_server`），再运行：

```bash
# 单独测试
./test_query_demo.sh   # semantic_query server + client
./test_skill_demo.sh   # execute command server + client
./test_stream_demo.sh  # pose stream publisher + subscriber

# 一次性测试全部
./test_all_demos.sh
```
