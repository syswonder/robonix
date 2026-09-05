#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Shared runtime-state helpers for the Webots simulator launchers.

robonix_sim_init_runtime_state() {
    export ROBONIX_SIM_CONTAINER="${ROBONIX_SIM_CONTAINER:-robonix_tiago_sim}"
    export ROBONIX_SIM_PROJECT="${ROBONIX_SIM_PROJECT:-robonix_tiago_sim}"

    local value
    for value in "$ROBONIX_SIM_CONTAINER" "$ROBONIX_SIM_PROJECT"; do
        if [[ ! "$value" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]]; then
            echo "[sim] invalid simulator identity '$value'" >&2
            echo "[sim] project and container names may contain only letters, digits, '.', '_' and '-'." >&2
            return 2
        fi
    done

    local runtime_user_id="${UID:-}"
    if [[ -z "$runtime_user_id" ]]; then
        runtime_user_id="$(id -u)"
    fi
    local runtime_root="${ROBONIX_SIM_STATE_DIR:-${XDG_RUNTIME_DIR:-${TMPDIR:-/tmp}}/robonix-webots-sim-${runtime_user_id}}"

    umask 077
    mkdir -p "$runtime_root"
    if [[ ! -d "$runtime_root" || ! -w "$runtime_root" ]]; then
        echo "[sim] runtime state directory is not writable: $runtime_root" >&2
        return 1
    fi

    ROBONIX_SIM_STATE_DIR="$runtime_root"
    ROBONIX_SIM_RVIZ_PID_FILE="$runtime_root/${ROBONIX_SIM_PROJECT}--${ROBONIX_SIM_CONTAINER}.rviz.pid"
    export ROBONIX_SIM_STATE_DIR ROBONIX_SIM_RVIZ_PID_FILE
}

robonix_sim_read_rviz_pid() {
    local pid_file="$1"
    [[ -f "$pid_file" && ! -L "$pid_file" ]] || return 1

    local pid
    IFS= read -r pid < "$pid_file" || return 1
    [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 1
    printf '%s\n' "$pid"
}

robonix_sim_rviz_pid_matches() {
    local pid="$1"
    local rviz_script="$2"
    [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 1

    local args
    args="$(ps -p "$pid" -o args= 2>/dev/null)" || return 1
    [[ -n "$args" && "$args" == *"$rviz_script"* ]]
}

robonix_sim_record_rviz_pid() {
    local pid="$1"
    local pid_file="$2"
    [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 2

    local temporary_file="${pid_file}.tmp.$$"
    printf '%s\n' "$pid" > "$temporary_file"
    mv -f -- "$temporary_file" "$pid_file"
}
