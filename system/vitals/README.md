# robonix-vitals

监控机器人板载（CPU/GPU/NVMe 温度、电压）和本体（关节电机）健康状态，做阈值判定后通过 gRPC 上报。

## 运行

```bash
# 默认启动（scripts/collect.py 读 sysfs，自动检测 Piper SDK）
robonix-vitals --log info

# 指定 Python 环境（如 roboarm venv）
ROBONIX_VITALS_PYTHON=/path/to/roboarm/.venv/bin/python3 \
robonix-vitals --log info

# 用 mock 脚本测试（无需真实硬件）
ROBONIX_VITALS_SCRIPT=scripts/mock_collect.py \
robonix-vitals --log info
```

## CLI 参数

| Flag | 环境变量 | 默认值 |
|------|---------|--------|
| `--atlas` | `ROBONIX_ATLAS_ENDPOINT` | `127.0.0.1:50051` |
| `--listen` | `ROBONIX_VITALS_LISTEN` | `127.0.0.1:50091` |
| `--id` | `ROBONIX_VITALS_PROVIDER_ID` | `vitals` |
| `--collect-interval-ms` | `ROBONIX_VITALS_COLLECT_INTERVAL_MS` | `1000` |
| `--thresholds-path` | `ROBONIX_VITALS_THRESHOLDS_PATH` | `thresholds/jetson_agx_orin.yaml` |
| `--body-thresholds-path` | `ROBONIX_VITALS_BODY_THRESHOLDS_PATH` | `thresholds/body.yaml` |
| `--config` | `ROBONIX_CONFIG_PATH` | — |
| `--log` | `RUST_LOG` | `robonix_vitals=info` |

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ROBONIX_VITALS_SCRIPT` | `<crate>/scripts/collect.py` | 采集脚本路径 |
| `ROBONIX_VITALS_PYTHON` | `python3` | Python 二进制 |
| `MOCK_COLLECT_SCENARIO` | `normal` | mock 板载场景：`normal` / `ramp` / `low_voltage` |
| `MOCK_BODY_SCENARIO` | `none` | mock 本体场景：`none` / `normal` / `ramp` / `fault` / `toggle` / `mixed` |

## 测试

### 单元测试

```bash
cargo test -p robonix-vitals
```

### 无硬件端到端测试

```bash
# 仅板载
ROBONIX_VITALS_SCRIPT=scripts/mock_collect.py \
cargo run -p robonix-vitals -- --log info

# 板载 + 本体温度 ramp（joint_1 逐帧升温，跨 WARN 60°C / ERROR 75°C）
ROBONIX_VITALS_SCRIPT=scripts/mock_collect.py \
MOCK_BODY_SCENARIO=ramp \
cargo run -p robonix-vitals -- --log info

# 板载 + 故障注入
ROBONIX_VITALS_SCRIPT=scripts/mock_collect.py \
MOCK_BODY_SCENARIO=fault \
cargo run -p robonix-vitals -- --log info

# 板载 + 关节使能开关
ROBONIX_VITALS_SCRIPT=scripts/mock_collect.py \
MOCK_BODY_SCENARIO=toggle \
cargo run -p robonix-vitals -- --log info

# 低电压告警
ROBONIX_VITALS_SCRIPT=scripts/mock_collect.py \
MOCK_COLLECT_SCENARIO=low_voltage \
cargo run -p robonix-vitals -- --log info
```

### gRPC 验证

```bash
grpcurl -plaintext -d '{}' 127.0.0.1:50091 \
  robonix.contracts.RobonixServiceVitalsGet/GetVitals
```
