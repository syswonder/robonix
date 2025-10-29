#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash

# defaults
PARAM_FILE="config/nav2_params.yml"
USE_SIM_TIME="true"

EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config-file|-c)
      if [[ ${2-} && ! "$2" =~ ^- ]]; then
        PARAM_FILE="$2"
        shift 2
      else
        echo "ERROR: --config-file requires a path argument" >&2
        exit 1
      fi
      ;;
    --use-sim-time|-s)
      # two forms:
      #   -s                -> true
      #   --use-sim-time    -> true
      #   --use-sim-time false / true
      if [[ ${2-} && ! "$2" =~ ^- ]]; then
        v="${2,,}" 
        if [[ "$v" == "true" || "$v" == "false" ]]; then
          USE_SIM_TIME="$v"
          shift 2
        else
          echo "ERROR: --use-sim-time requires true/false, received: $2" >&2
          exit 1
        fi
      else
        USE_SIM_TIME="true"
        shift 1
      fi
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

# check params file existence
if [[ ! -f "$PARAM_FILE" ]]; then
  echo "ERROR: params file does not exist: $PARAM_FILE" >&2
  exit 2
fi

echo "[run.sh] using params file: $PARAM_FILE"
echo "[run.sh] use_sim_time: $USE_SIM_TIME"

exec ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:="$USE_SIM_TIME" \
  params_file:="$PARAM_FILE" \
  "${EXTRA_ARGS[@]}"
