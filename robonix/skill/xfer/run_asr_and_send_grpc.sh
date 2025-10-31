#!/bin/bash
set -euo pipefail

LANGUAGE_CODE="${LANGUAGE_CODE:-zh-CN}"
AUDIO_DEVICE="${AUDIO_DEVICE:-plughw:2,0}"    
SAVE_DIR="${SAVE_DIR:-/opt/riva/test}"       
TMP_FILE="${TMP_FILE:-/tmp/asr_buffer.txt}"


ORIN_B_IP="${ORIN_B_IP:-}"                     
FILESINK_PORT="${FILESINK_PORT:-50111}"
ORIN_B_RENAME="${ORIN_B_RENAME:-}"             

KEYWORDS=("机器人" "几奔" "机奔" "积奔" "机人" "几人" "极奔" "极人")

mkdir -p "$SAVE_DIR"
: > "$TMP_FILE"


PY_OK=1
python3 - <<'PY' >/dev/null 2>&1 || PY_OK=0
import grpc, sys
from packaging import version
assert version.parse(grpc.__version__) >= version.parse("1.71.0")
PY
if [[ $PY_OK -eq 0 ]]; then
  echo "[WARN] No matching version of grpcio (>=1.71.0) was detected. gRPC sending will be skipped."
fi

send_via_grpc() {
  local path="$1"
  if [[ -z "${ORIN_B_IP}" ]]; then
    echo "[SKIP] ORIN_B_IP not configured, skip gRPC sending:$path"; return 0
  fi
  if [[ $PY_OK -eq 0 ]]; then
    echo "[SKIP] grpcio version mismatch, skip gRPC sending:$path"; return 0
  fi
  local rename_arg=()
  [[ -n "${ORIN_B_RENAME}" ]] && rename_arg=("${ORIN_B_RENAME}")
  python3 /opt/riva/xfer/client/send_file_client.py "${ORIN_B_IP}" "${FILESINK_PORT}" "$path" "${rename_arg[@]}"
}


echo "[INFO] Using device: $AUDIO_DEVICE"

coproc RIVA { stdbuf -o0 riva_streaming_asr_client --language_code="${LANGUAGE_CODE}" --audio_device="${AUDIO_DEVICE}" 2>/dev/null; }
RIVA_PID=${RIVA_PID:-$COPROC_PID}   

if [[ -z "${RIVA_PID}" ]]; then
  echo "[ERR] Unable to start riva_streaming_asr_client"; exit 1
fi
echo "[INFO] riva_streaming_asr_client pid=${RIVA_PID}"

cleanup() {
  exec {RIVA[0]}>&- 2>/dev/null || true
  exec {RIVA[1]}>&- 2>/dev/null || true
  [[ -n "${RIVA_PID}" ]] && kill "$RIVA_PID" 2>/dev/null || true
  rm -f "$TMP_FILE"
}
trap cleanup EXIT INT TERM


triggered=0
last_activity_time=$(date +%s)


while true; do
  line=""
  if IFS= read -r -t 0.2 -u "${RIVA[0]}" line; then

    [[ -z "$line" ]] && continue
    echo "$line"
    echo "$line" >> "$TMP_FILE"
    last_activity_time=$(date +%s)

    for kw in "${KEYWORDS[@]}"; do
      [[ "$line" == *"$kw"* ]] && { echo "[Trigger] Keyword detected:$kw"; triggered=1; }
    done
  else
    if [[ $triggered -eq 1 ]]; then
      current_time=$(date +%s)
      diff=$((current_time - last_activity_time))
      if (( diff >= 3 )); then
        timestamp=$(date +"%Y%m%d_%H%M%S")
        save_path="${SAVE_DIR}/asr_${timestamp}.txt"
        mv "$TMP_FILE" "$save_path"
        echo "[Saved] No voice input for 3 seconds, save file to $save_path"
        send_via_grpc "$save_path" || true
        echo "[Exit] The program is about to exit."
        exit 0
      fi
    fi
    if ! kill -0 "$RIVA_PID" 2>/dev/null; then
      echo "[ERR] riva_streaming_asr_client Exited."
      exit 1
    fi
  fi
done

