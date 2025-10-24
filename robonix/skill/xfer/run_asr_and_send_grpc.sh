#!/bin/bash
set -euo pipefail

# ========= 配置 =========
LANGUAGE_CODE="${LANGUAGE_CODE:-zh-CN}"
AUDIO_DEVICE="${AUDIO_DEVICE:-plughw:2,0}"     # 用 arecord -l 查到的 plughw:X,Y
SAVE_DIR="${SAVE_DIR:-/opt/riva/test}"         # 要与 docker -v 对应
TMP_FILE="${TMP_FILE:-/tmp/asr_buffer.txt}"

# Orin-B gRPC 接收端
ORIN_B_IP="${ORIN_B_IP:-}"                     # 例如 192.168.1.22
FILESINK_PORT="${FILESINK_PORT:-50111}"
ORIN_B_RENAME="${ORIN_B_RENAME:-}"             # 可留空

KEYWORDS=("机器人" "几奔" "机奔" "积奔" "机人" "几人" "极奔" "极人")

mkdir -p "$SAVE_DIR"
: > "$TMP_FILE"

# ========= 发送函数（要求容器里已安装 grpcio==1.71.0+）=========
PY_OK=1
python3 - <<'PY' >/dev/null 2>&1 || PY_OK=0
import grpc, sys
from packaging import version
assert version.parse(grpc.__version__) >= version.parse("1.71.0")
PY
if [[ $PY_OK -eq 0 ]]; then
  echo "[WARN] 未检测到匹配版本的 grpcio (>=1.71.0)，将跳过 gRPC 发送。"
fi

send_via_grpc() {
  local path="$1"
  if [[ -z "${ORIN_B_IP}" ]]; then
    echo "[SKIP] 未配置 ORIN_B_IP，跳过 gRPC 发送：$path"; return 0
  fi
  if [[ $PY_OK -eq 0 ]]; then
    echo "[SKIP] grpcio 版本不匹配，跳过 gRPC 发送：$path"; return 0
  fi
  local rename_arg=()
  [[ -n "${ORIN_B_RENAME}" ]] && rename_arg=("${ORIN_B_RENAME}")
  python3 /opt/riva/xfer/client/send_file_client.py "${ORIN_B_IP}" "${FILESINK_PORT}" "$path" "${rename_arg[@]}"
}

# ========= 只启动一次 Riva 客户端，并用 coproc 异步读取 =========
echo "[INFO] Using device: $AUDIO_DEVICE"
# -o0 -> 关闭 stdout 缓冲；确保逐行/即时输出
coproc RIVA { stdbuf -o0 riva_streaming_asr_client --language_code="${LANGUAGE_CODE}" --audio_device="${AUDIO_DEVICE}" 2>/dev/null; }
RIVA_PID=${RIVA_PID:-$COPROC_PID}   # bash 5: RIVA_PID；bash 4 用 COPROC_PID

if [[ -z "${RIVA_PID}" ]]; then
  echo "[ERR] 无法启动 riva_streaming_asr_client"; exit 1
fi
echo "[INFO] riva_streaming_asr_client pid=${RIVA_PID}"

cleanup() {
  exec {RIVA[0]}>&- 2>/dev/null || true
  exec {RIVA[1]}>&- 2>/dev/null || true
  [[ -n "${RIVA_PID}" ]] && kill "$RIVA_PID" 2>/dev/null || true
  rm -f "$TMP_FILE"
}
trap cleanup EXIT INT TERM

# ========= 关键词触发 + 静默 3 秒 =========
triggered=0
last_activity_time=$(date +%s)

# 说明：从 RIVA[0]（stdout 读端）读；没有新行时 read -t 0.2 会超时返回非0
while true; do
  line=""
  if IFS= read -r -t 0.2 -u "${RIVA[0]}" line; then
    # 收到一行识别文本
    [[ -z "$line" ]] && continue
    echo "$line"
    echo "$line" >> "$TMP_FILE"
    last_activity_time=$(date +%s)

    # 关键字检测
    for kw in "${KEYWORDS[@]}"; do
      [[ "$line" == *"$kw"* ]] && { echo "[Trigger] 检测到关键字：$kw"; triggered=1; }
    done
  else
    # 0.2s 内无新行：检查是否静默超时
    if [[ $triggered -eq 1 ]]; then
      current_time=$(date +%s)
      diff=$((current_time - last_activity_time))
      if (( diff >= 3 )); then
        timestamp=$(date +"%Y%m%d_%H%M%S")
        save_path="${SAVE_DIR}/asr_${timestamp}.txt"
        mv "$TMP_FILE" "$save_path"
        echo "[Saved] 无语音输入3秒，保存文件到 $save_path"
        send_via_grpc "$save_path" || true
        echo "[Exit] 程序即将退出。"
        exit 0
      fi
    fi
    # 如果 Riva 进程退出，则报错并退出
    if ! kill -0 "$RIVA_PID" 2>/dev/null; then
      echo "[ERR] riva_streaming_asr_client 已退出。"
      exit 1
    fi
  fi
done

