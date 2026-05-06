#!/usr/bin/env bash
# run_tui_test.sh — 一键启动语音+文本 TUI 测试环境
#
# 依赖已有的 dev 栈服务（Atlas、Pilot、speech_service），只启动本次新增的组件：
#   1. mock_audio   — 用 WAV 文件模拟麦克风和扬声器
#   2. Liaison      — 在独立端口上启动（避免与已有 Liaison 冲突）
#   3. rbnx chat    — TUI 客户端
#
# 前提条件：已通过 run.sh 启动了 Atlas + Pilot + speech_service
#
# Usage:
#   cd rust/examples
#   ./run_tui_test.sh
#
# Optional env:
#   MOCK_WAV_INPUT=<path>     — 自定义语音输入 WAV 文件
#   MOCK_WAV_TEXT="你好"      — 自动生成 WAV 的文本 (default: 我目前所在的位置是哪里)
#   MOCK_WAV_OUTPUT=<path>    — TTS 输出文件路径

set -euo pipefail

RUST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXAMPLES_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGES="${EXAMPLES_ROOT}/packages"

cd "$EXAMPLES_ROOT"

# Source .env for shared settings (VLM keys, Atlas addr)
_SAVED_START_PILOT="${START_PILOT:-}"
_SAVED_START_LIAISON="${START_LIAISON:-}"
if [ -f .env ]; then
  set -a; source .env; set +a
fi
export START_PILOT="${_SAVED_START_PILOT:-0}"
export START_LIAISON=""

export ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}"
export ROBONIX_META_GRPC_ENDPOINT="${ROBONIX_META_GRPC_ENDPOINT:-$ROBONIX_ATLAS}"
export ROBONIX_ATLAS_ENDPOINT="${ROBONIX_ATLAS_ENDPOINT:-http://$ROBONIX_ATLAS}"
export ROBONIX_PILOT_ENDPOINT="${ROBONIX_PILOT_ENDPOINT:-http://127.0.0.1:50071}"
export ROBONIX_LIAISON_PORT="${ROBONIX_LIAISON_PORT:-50082}"
export RUST_LOG="${RUST_LOG:-robonix_atlas=info,robonix_liaison=info,mock_audio=info}"
export MOCK_AUDIO_PORT="${MOCK_AUDIO_PORT:-50091}"
export MOCK_WAV_TEXT="${MOCK_WAV_TEXT:-我目前所在的位置是哪里}"
export MOCK_WAV_OUTPUT="${MOCK_WAV_OUTPUT:-/tmp/robonix_tts_output.wav}"
export ROBONIX_CHAT_VOICE_TTS="${ROBONIX_CHAT_VOICE_TTS:-1}"
export HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"

# TUI 直连我们的 Liaison（绕过 Atlas 发现旧 Liaison）
export ROBONIX_LIAISON_ENDPOINT="http://127.0.0.1:${ROBONIX_LIAISON_PORT}"

# 固定 mic/speaker 到 mock_audio（其他服务使用 Atlas 自动发现）
export ROBONIX_CHAT_MIC_NODE="com.robonix.demo.mock_audio"
export ROBONIX_CHAT_SPEAKER_NODE="com.robonix.demo.mock_audio"

rbnx() {
  (cd "$RUST_ROOT" && cargo run -p robonix-cli -- "$@")
}

# ── 清理旧进程 ───────────────────────────────────────────────────────────────
pkill -9 -u "$(id -u)" -f 'robonix-liaison' 2>/dev/null || true
pkill -9 -u "$(id -u)" -f 'mock_audio' 2>/dev/null || true
pkill -9 -u "$(id -u)" -f 'mock_pilot' 2>/dev/null || true
sleep 0.5

cleanup() {
  echo "[tui-test] shutting down..."
  kill $(jobs -p) 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# ── 0. 前置检查：确认 dev 栈已启动 ───────────────────────────────────────────
ATLAS_PORT="${ROBONIX_ATLAS##*:}"
PILOT_PORT="${ROBONIX_PILOT_ENDPOINT##*:}"

echo "[tui-test] checking dev stack..."
if ! ss -tlnp 2>/dev/null | grep -q ":${ATLAS_PORT} "; then
  echo "[tui-test] ERROR: Atlas not running on :${ATLAS_PORT}"
  echo "[tui-test] Please start the dev stack first: cd rust/examples && ./run.sh"
  exit 1
fi
echo "[tui-test]   Atlas:          :${ATLAS_PORT} ✓"

if ss -tlnp 2>/dev/null | grep -q ":${PILOT_PORT} "; then
  echo "[tui-test]   Pilot:          :${PILOT_PORT} ✓"
else
  echo "[tui-test]   Pilot:          :${PILOT_PORT} ✗ (will use fallback)"
fi

# 检查 speech_service（ASR + TTS）是否已在运行
SPEECH_RUNNING=0
if pgrep -f 'speech_service.service' >/dev/null 2>&1; then
  SPEECH_RUNNING=1
  echo "[tui-test]   speech_service: running ✓ (real ASR + TTS)"
else
  echo "[tui-test]   speech_service: not running ✗"
  echo "[tui-test]   WARNING: ASR/TTS will not work. Start speech_service via run.sh"
fi

# ── 1. 生成测试 WAV ──────────────────────────────────────────────────────────
if [ -z "${MOCK_WAV_INPUT:-}" ]; then
  MOCK_WAV_INPUT="/tmp/robonix_test_input.wav"
  echo "[tui-test] generating WAV: \"${MOCK_WAV_TEXT}\""
  python3 -c "
import asyncio, edge_tts, sys
async def main():
    c = edge_tts.Communicate(sys.argv[1], 'zh-CN-XiaoxiaoNeural')
    await c.save('/tmp/_robonix_tts_tmp.mp3')
asyncio.run(main())
" "${MOCK_WAV_TEXT}" 2>&1
  ffmpeg -y -loglevel error -i /tmp/_robonix_tts_tmp.mp3 \
    -ar 16000 -ac 1 -sample_fmt s16 "${MOCK_WAV_INPUT}" 2>&1
  rm -f /tmp/_robonix_tts_tmp.mp3
  echo "[tui-test]   WAV: ${MOCK_WAV_INPUT}"
fi
export MOCK_WAV_INPUT

# ── 2. mock_audio（WAV 模拟麦克风 + 扬声器保存文件）──────────────────────────
echo "[tui-test] starting mock_audio (:${MOCK_AUDIO_PORT})..."
(cd "$RUST_ROOT" && cargo run -p robonix-liaison --example mock_audio) &
sleep 2

# ── 3. Liaison（独立端口，使用真实 ASR/TTS/Pilot）────────────────────────────
echo "[tui-test] starting Liaison (:${ROBONIX_LIAISON_PORT})..."
(cd "$RUST_ROOT" && cargo run -p robonix-liaison) &
sleep 2

# ── 4. TUI ───────────────────────────────────────────────────────────────────
echo ""
echo "╔═══════════════════════════════════════════════════════════════════╗"
echo "║  TUI Ready                                                      ║"
echo "║                                                                  ║"
echo "║  Enter   → 文本输入 → Liaison → Pilot                           ║"
echo "║  Ctrl+V  → WAV → ASR(gRPC) → Pilot → TTS(gRPC) → 音频文件      ║"
echo "║  Esc     → 中断    Ctrl+C → 退出                                ║"
echo "║                                                                  ║"
echo "║  输入 WAV:  ${MOCK_WAV_INPUT}"
echo "║  TTS 输出:  ${MOCK_WAV_OUTPUT}"
echo "╚═══════════════════════════════════════════════════════════════════╝"
echo ""

rbnx chat --server "$ROBONIX_ATLAS"
