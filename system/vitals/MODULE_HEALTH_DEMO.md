# Vitals Module Health Demo

本文档用于演示 Vitals 对系统模块健康状态的最小闭环：

```text
executor / pilot 自报 ModuleHealthReport
        -> Vitals 定期轮询 get_health
        -> Vitals 聚合为 ModuleHealthSnapshot
        -> 模块失联超过 ttl_ms 后，Vitals 合成 STALE / ERROR
        -> 模块恢复后，Vitals 记录 ERROR -> OK
```

这个 demo 不需要真实机器人硬件。硬件健康链路使用 mock Soma；模块健康链路使用
executor 和 pilot 的 `get_health`。

## 1. 准备

进入仓库并先编译：

```bash
cd /path/to/robonix

cargo build -p robonix-atlas
cargo build -p robonix-executor
cargo build -p robonix-pilot
cargo build -p robonix-vitals
```

如果需要查询 gRPC 结果，建议安装 `grpcurl` 和 `jq`。没有 `grpcurl` 时，也可以只
通过 Vitals 日志观察模块连接、stale 和恢复。

## 2. 启动系统

需要 5 个终端。

### Terminal 1: Atlas

```bash
cd /path/to/robonix

cargo run -p robonix-atlas -- \
  --listen 127.0.0.1:50051 \
  --capabilities capabilities
```

Atlas 是服务注册和 capability 发现中心。

### Terminal 2: Executor

```bash
cd /path/to/robonix

cargo run -p robonix-executor -- \
  --atlas 127.0.0.1:50051 \
  --listen 127.0.0.1:50061
```

Executor 会注册：

```text
robonix/system/executor/get_health
```

Vitals 会轮询这个接口。

### Terminal 3: Pilot

```bash
cd /path/to/robonix

cargo run -p robonix-pilot -- \
  --atlas 127.0.0.1:50051 \
  --listen 127.0.0.1:50071 \
  --vlm-upstream http://127.0.0.1:9/v1 \
  --vlm-api-key dummy \
  --vlm-model dummy
```

这里 VLM 参数可以用 dummy，因为本 demo 只测试 `get_health`，不提交任务给 Pilot。

Pilot 会注册：

```text
robonix/system/pilot/get_health
```

### Terminal 4: Mock Soma

```bash
cd /path/to/robonix

cargo run -p robonix-vitals -- \
  --atlas 127.0.0.1:50051 \
  --mock-soma \
  --mock-soma-listen 127.0.0.1:50092 \
  --mock-soma-scenario normal \
  --mock-soma-interval-ms 10000 \
  --log info
```

Mock Soma 会注册：

```text
robonix/system/soma/health
robonix/system/soma/get_health
```

这里 `--mock-soma-interval-ms 10000` 表示每 10 秒生成一帧硬件健康快照，录屏时不
会滚动太快。

### Terminal 5: Vitals

```bash
cd /path/to/robonix

cargo run -p robonix-vitals -- \
  --atlas 127.0.0.1:50051 \
  --listen 127.0.0.1:50091 \
  --thresholds-path system/vitals/thresholds/soma_mock.yaml \
  --log robonix_vitals=info
```

Vitals 会注册：

```text
robonix/system/vitals/get
robonix/system/vitals/stream
robonix/system/vitals/modules/get
```

## 3. 观察正常启动

Vitals 启动后应看到：

```text
declared GetVitals at 127.0.0.1:50091
declared StreamVitals at 127.0.0.1:50091
declared ModuleHealthSnapshot at 127.0.0.1:50091
connected to Soma provider 'mock-soma' through Atlas
connected to Soma StreamHealth
module health poll connected: executor (robonix/system/executor/get_health)
module health poll connected: pilot (robonix/system/pilot/get_health)
```

这说明两条链路都通了：

```text
mock Soma -> Vitals                       硬件健康链路
executor / pilot -> Vitals modules/get   模块健康链路
```

10 秒后，Vitals 还会打印 mock Soma 的 body 基线，例如：

```text
[vitals] body: NORMAL (computer_jetson/jetson_agx_orin)
[vitals] body: NORMAL (arm/mock_arm)
[vitals] body: NORMAL (battery_main/mock_bms)
```

这些是硬件健康数据，不是模块健康数据。

## 4. 查询模块健康快照

如果安装了 `grpcurl`，可以查询 Vitals 聚合后的模块健康快照：

```bash
cd /path/to/robonix

PROTO_DIR=$(ls -td target/debug/build/robonix-vitals-*/out | head -n1)

grpcurl -plaintext \
  -import-path "$PROTO_DIR" \
  -proto robonix_contracts.proto \
  -d '{}' \
  127.0.0.1:50091 \
  robonix.contracts.RobonixSystemVitalsModulesGet/GetModuleHealthSnapshot | jq
```

正常情况下应看到 executor 和 pilot：

```json
{
  "snapshot": {
    "schemaVersion": 1,
    "modules": [
      {
        "moduleKey": "executor",
        "moduleId": "executor",
        "providerId": "executor",
        "health": 0,
        "state": "active",
        "reasonCode": "OK",
        "detail": "executor serving",
        "source": "SELF_REPORTED",
        "ttlMs": 5000
      },
      {
        "moduleKey": "pilot",
        "moduleId": "pilot",
        "providerId": "pilot",
        "health": 0,
        "state": "active",
        "reasonCode": "OK",
        "detail": "pilot serving",
        "source": "SELF_REPORTED",
        "ttlMs": 5000
      }
    ]
  }
}
```

字段含义：

| 字段 | 含义 |
|---|---|
| `health = 0` | OK |
| `health = 1` | WARN |
| `health = 2` | ERROR |
| `source = SELF_REPORTED` | 来自模块自己的 `get_health` |
| `ttlMs = 5000` | 这条报告 5 秒内有效 |

## 5. 测试 executor 失联 stale

保持 Atlas、mock Soma、Vitals、Pilot 都运行。

在 Terminal 2 中停止 executor：

```bash
Ctrl+C
```

Vitals 先会发现 poll 失败：

```text
[vitals] module health poll lost executor: ...
```

如果 executor 曾经成功上报过，超过 `ttlMs = 5000` 后，Vitals 会合成 stale：

```text
[vitals] module executor health: OK -> ERROR (STALE)
[vitals] ALERT: module executor - no health report received within ttl
```

再次查询 `modules/get`，executor 应变为：

```json
{
  "moduleKey": "executor",
  "moduleId": "executor",
  "providerId": "executor",
  "health": 2,
  "state": "stale",
  "reasonCode": "STALE",
  "detail": "no health report received within ttl",
  "source": "VITALS_SYNTHESIZED_STALE",
  "ttlMs": 5000
}
```

注意：如果某个模块从来没有成功上报过，Vitals 不会立刻把它标成 ERROR。当前逻辑只
对“曾经成功上报过、后来失联”的模块做 stale 合成。

## 6. 测试 executor 恢复

重新启动 Terminal 2：

```bash
cd /path/to/robonix

cargo run -p robonix-executor -- \
  --atlas 127.0.0.1:50051 \
  --listen 127.0.0.1:50061
```

Vitals 应看到：

```text
[vitals] module health poll connected: executor (robonix/system/executor/get_health)
[vitals] module executor health: ERROR -> OK (OK)
```

再次查询 `modules/get`，executor 应恢复为：

```json
{
  "moduleKey": "executor",
  "health": 0,
  "state": "active",
  "reasonCode": "OK",
  "source": "SELF_REPORTED"
}
```

## 7. 测试 pilot stale

Pilot 的测试方式和 executor 一样：

1. 在 Terminal 3 停止 pilot。
2. 等待约 5 秒。
3. Vitals 应打印 `pilot OK -> ERROR (STALE)`。
4. 重启 pilot。
5. Vitals 应打印 `pilot ERROR -> OK (OK)`。

## 8. 常见问题

### Vitals 没有打印 executor / pilot connected

检查 executor 和 pilot 是否已经启动，并确认它们连接的是同一个 Atlas：

```text
--atlas 127.0.0.1:50051
```

### Pilot 启动失败，提示缺少 VLM 配置

即使只测 `get_health`，Pilot 当前启动也需要 VLM 参数。使用 demo 中的 dummy 参数即可。

### 端口占用

本 demo 使用：

| 服务 | 端口 |
|---|---|
| Atlas | `50051` |
| Executor | `50061` |
| Pilot | `50071` |
| Vitals | `50091` |
| Mock Soma | `50092` |

如果端口被占用，可以先停止旧进程，或者统一替换为其他端口。

### grpcurl 查询失败

先确认 Vitals 已经编译过，并且 `PROTO_DIR` 能找到生成的 proto：

```bash
PROTO_DIR=$(ls -td target/debug/build/robonix-vitals-*/out | head -n1)
ls "$PROTO_DIR/robonix_contracts.proto"
```

如果没有 `grpcurl`，可以先只通过 Vitals 日志验证。

## 9. 本 demo 证明了什么

完成以上步骤后，可以证明：

1. executor 和 pilot 已按 V1 协议提供 `ModuleHealthReport`。
2. Vitals 能发现并轮询它们的 `get_health`。
3. Vitals 能通过 `modules/get` 返回聚合后的 `ModuleHealthSnapshot`。
4. 模块失联超过 `ttl_ms` 后，Vitals 能合成 `STALE / ERROR`。
5. 模块恢复后，Vitals 能记录并展示 `ERROR -> OK`。
