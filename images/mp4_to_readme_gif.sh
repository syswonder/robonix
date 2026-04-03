#!/usr/bin/env bash
# Convert MP4 to a README-friendly GIF (palette two-pass, Lanczos scale).
# Requires: ffmpeg (ffprobe optional, for printing input info)
#
# Usage:
#   ./mp4_to_readme_gif.sh [options]
#
# Defaults target a short, sharp clip; full-length 1080p60 → GIF is usually huge.
# Optional edge crop (CROP_* / CROP_INSET) runs on source frames before scaling.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INPUT="${INPUT:-$SCRIPT_DIR/demo_01.mp4}"
OUTPUT="${OUTPUT:-$SCRIPT_DIR/demo_01_readme.gif}"
START="${START:-0}"
# README GIFs: keep short unless you really need the whole video.
DURATION="${DURATION:-20}"
# Max width; height keeps aspect.
WIDTH="${WIDTH:-1280}"
FPS="${FPS:-14}"
# Playback speed-up (>1 = faster GIF, same source span feels snappier in README)
SPEED="${SPEED:-1.5}"
# palettegen quality: single | diff (diff often clearer on motion)
STATS_MODE="${STATS_MODE:-diff}"
# Crop (pixels from each edge, after fps/setpts, before scale). Use CROP_INSET for uniform inset.
if [[ -n "${CROP_INSET:-}" ]]; then
  CROP_LEFT="${CROP_INSET}"
  CROP_RIGHT="${CROP_INSET}"
  CROP_TOP="${CROP_INSET}"
  CROP_BOTTOM="${CROP_INSET}"
else
  CROP_LEFT="${CROP_LEFT:-0}"
  CROP_RIGHT="${CROP_RIGHT:-0}"
  CROP_TOP="${CROP_TOP:-0}"
  CROP_BOTTOM="${CROP_BOTTOM:-0}"
fi

usage() {
  sed -n '1,80p' "$0" | sed -n '/^# /s/^# //p'
  echo ""
  echo "Options (env vars):"
  echo "  INPUT=$INPUT"
  echo "  OUTPUT=$OUTPUT"
  echo "  START=$START          # seconds into the video"
  echo "  DURATION=$DURATION    # clip length (set empty for full length — not recommended)"
  echo "  WIDTH=$WIDTH"
  echo "  FPS=$FPS"
  echo "  SPEED=$SPEED         # setpts PTS/SPEED (e.g. 1.5 = 1.5× faster)"
  echo "  STATS_MODE=$STATS_MODE  # palette stats: single | diff"
  echo "  CROP_INSET=          # uniform px crop on all sides (overrides per-edge)"
  echo "  CROP_LEFT/RIGHT/TOP/BOTTOM  # per-edge px (defaults 0)"
  echo ""
  echo "Example — 25s clip, crop ~40px each side, ~2× speed:"
  echo "  DURATION=25 FPS=15 SPEED=2 CROP_INSET=40 $0"
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "error: ffmpeg not found; install ffmpeg." >&2
  exit 1
fi

if [[ ! -f "$INPUT" ]]; then
  echo "error: input not found: $INPUT" >&2
  exit 1
fi

echo "Input:  $INPUT"
if command -v ffprobe >/dev/null 2>&1; then
  ffprobe -hide_banner -v error -show_entries format=duration -show_entries stream=width,height,r_frame_rate -of default=noprint_wrappers=1 "$INPUT" || true
fi
echo "Output: $OUTPUT"
echo "Clip:   start=${START}s duration=${DURATION:-full}  speed=${SPEED}×"
echo "Crop:   L=${CROP_LEFT} R=${CROP_RIGHT} T=${CROP_TOP} B=${CROP_BOTTOM}"

VF_BASE="fps=${FPS},setpts=PTS/${SPEED},crop=iw-${CROP_LEFT}-${CROP_RIGHT}:ih-${CROP_TOP}-${CROP_BOTTOM}:${CROP_LEFT}:${CROP_TOP}"
if [[ -n "${DURATION}" ]]; then
  TIME_ARGS=(-ss "$START" -t "$DURATION")
else
  TIME_ARGS=(-ss "$START")
fi

# Two-pass GIF: generate palette then apply (clearer than single-pass rgb24→gif).
FILTER="\
${VF_BASE},\
scale=${WIDTH}:-1:flags=lanczos,\
format=rgb24,\
split[s0][s1];\
[s0]palettegen=reserve_transparent=0:stats_mode=${STATS_MODE}[p];\
[s1][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle\
"

ffmpeg -y "${TIME_ARGS[@]}" -i "$INPUT" -an -vf "$FILTER" -loop 0 "$OUTPUT"

ls -lh "$OUTPUT"
echo "Done. Markdown: ![demo](images/$(basename "$OUTPUT"))"
