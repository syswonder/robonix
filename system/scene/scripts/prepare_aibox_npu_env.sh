#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Prepare the x86 conversion tools and the AIBOX-side RKNN3 Lite runtime.
set -euo pipefail

readonly RKNN3_API=https://api.github.com/repos/airockchip/rknn3-toolkit

AIBOX_SSH_TARGET="${AIBOX_SSH_TARGET:-aibox}"
AIBOX_NPU_WORKDIR="${AIBOX_NPU_WORKDIR:-$HOME/aibox-npu}"
RKNN3_TOOLKIT_WHEEL="${RKNN3_TOOLKIT_WHEEL:-}"
RKNN3_TOOLKIT_LITE_WHEEL="${RKNN3_TOOLKIT_LITE_WHEEL:-}"
# Both ends follow the AIBOX firmware's rknn3 API version; resolved at runtime.
RKNN3_VERSION=""
RKNN3_TAG=""
TOOLKIT_PYTHON=""
TOOLKIT_PYTHON_MINOR=""
BOARD_PYTHON_MINOR=""
TOOLKIT_URL=""
TOOLKIT_LITE_URL=""
RELEASES_JSON=""
BOARD_INFO=""

expand_home() {
    case "$1" in
        '~/'*) printf '%s/%s' "$HOME" "${1#~/}" ;;
        *) printf '%s' "$1" ;;
    esac
}

AIBOX_NPU_WORKDIR="$(expand_home "$AIBOX_NPU_WORKDIR")"
RKNN3_TOOLKIT_WHEEL="$(expand_home "$RKNN3_TOOLKIT_WHEEL")"
RKNN3_TOOLKIT_LITE_WHEEL="$(expand_home "$RKNN3_TOOLKIT_LITE_WHEEL")"

CHECK_ONLY=0
HOST_ONLY=0
BOARD_ONLY=0

usage() {
    cat <<'EOF'
Usage: prepare_aibox_npu_env.sh [--check-only] [--host-only | --board-only]

Without --check-only, missing host tools and Python modules are installed.
Both ends are aligned to the AIBOX firmware's rknn3 API version as reported
by rknn-smi -v on the board.

Environment variables:
  AIBOX_SSH_TARGET          SSH alias for the board (default: aibox)
  AIBOX_NPU_WORKDIR         x86 workspace (default: ~/aibox-npu)
  RKNN3_TOOLKIT_WHEEL       x86 Toolkit wheel download/cache path
  RKNN3_TOOLKIT_LITE_WHEEL  aarch64 Lite wheel download/cache path
EOF
}

die() {
    printf '[aibox-env] error: %s\n' "$*" >&2
    exit 1
}

info() {
    printf '[aibox-env] %s\n' "$*"
}

warn() {
    printf '[aibox-env] warning: %s\n' "$*" >&2
}

while (($#)); do
    case "$1" in
        --check-only) CHECK_ONLY=1 ;;
        --host-only) HOST_ONLY=1 ;;
        --board-only) BOARD_ONLY=1 ;;
        -h|--help) usage; exit 0 ;;
        *) die "unknown argument: $1" ;;
    esac
    shift
done
(( HOST_ONLY + BOARD_ONLY <= 1 )) || die '--host-only and --board-only cannot be combined'

apt_install_local() {
    local packages=("$@")
    ((${#packages[@]})) || return 0
    ((CHECK_ONLY == 0)) || die "host is missing apt packages: ${packages[*]}"
    command -v apt-get >/dev/null || die "host is missing packages ${packages[*]} and has no apt-get installer"

    if ((EUID == 0)); then
        apt-get update
        apt-get install -y --no-install-recommends "${packages[@]}"
    else
        command -v sudo >/dev/null || die 'sudo is needed to install host packages'
        sudo apt-get update
        sudo apt-get install -y --no-install-recommends "${packages[@]}"
    fi
}

ensure_host_commands() {
    local packages=()
    local pair command_name package_name
    for pair in git:git curl:curl ssh:openssh-client scp:openssh-client gcc:build-essential; do
        command_name="${pair%%:*}"
        package_name="${pair#*:}"
        command -v "$command_name" >/dev/null || packages+=("$package_name")
    done
    apt_install_local "${packages[@]}"
}

ensure_download() {
    local url="$1" path="$2" temp_path
    if [[ -s "$path" ]]; then
        verify_wheel_md5 "$url" "$path"
        return 0
    elif ((CHECK_ONLY)); then
        die "wheel is not cached: $path (run without --check-only to download it)"
    fi

    mkdir -p "$(dirname "$path")"
    temp_path="$(mktemp "${path}.part.XXXXXX")"
    if ! curl --fail --location --connect-timeout 15 --max-time 300 \
        --retry 3 --output "$temp_path" "$url"; then
        rm -f -- "$temp_path"
        die "download failed: $url"
    fi
    if [[ ! -s "$temp_path" ]]; then
        rm -f -- "$temp_path"
        die "downloaded wheel is empty: $url"
    fi
    verify_wheel_md5 "$url" "$temp_path"
    mv -f -- "$temp_path" "$path"
    info "downloaded $(basename "$path") from official RKNN3 Toolkit tag $RKNN3_TAG"
}

# Fetch the official md5sum.txt published next to the wheel and compare; a
# release without checksums degrades to a warning instead of failing.
verify_wheel_md5() {
    local url="$1" path="$2" name expected actual
    name="$(basename "$url")"
    expected="$(curl --silent --show-error --fail --location --connect-timeout 15 \
        "${url%/*}/md5sum.txt" | awk -v n="$name" '$2 == n {print $1; exit}')" || {
        info "cannot fetch md5sum.txt for $name; skipping verification"
        return 0
    }
    [[ -n "$expected" ]] || { info "no md5 entry for $name; skipping verification"; return 0; }
    actual="$(md5sum "$path" | awk '{print $1}')"
    [[ "$actual" == "$expected" ]] \
        || die "md5 mismatch for $name: got $actual, expected $expected (delete $path and retry)"
}

load_releases() {
    RELEASES_JSON="$(curl --silent --show-error --fail --location --connect-timeout 15 --max-time 60 \
        "$RKNN3_API/releases?per_page=100")" || die 'cannot list RKNN3 Toolkit releases'
}

release_version() {
    python3 -c 'import sys; print(sys.argv[1].lstrip("vV"))' "$1"
}

tag_for_version() {
    local wanted="$1"
    python3 -c 'import json,sys; wanted=sys.argv[1].lower(); print(next((r["tag_name"] for r in json.load(sys.stdin) if r["tag_name"].lstrip("vV").lower()==wanted), ""))' \
        "$wanted" <<<"$RELEASES_JSON"
}

wheel_url_for() {
    local tag="$1" kind="$2" python_minor="$3" path package version cp response
    version="$(release_version "$tag")"
    cp="cp${python_minor//./}"
    case "$kind" in
        toolkit) path=rknn3-toolkit/packages; package=rknn3_toolkit ;;
        lite) path=rknn3-toolkit-lite/packages; package=rknn3_toolkit_lite ;;
        *) die "unknown wheel kind: $kind" ;;
    esac
    response="$(curl --silent --show-error --fail --location --connect-timeout 15 --max-time 60 \
        "$RKNN3_API/contents/$path?ref=$tag")" || return 1
    python3 -c 'import json,sys; version,package,cp,kind=sys.argv[1:]; prefix=f"{package}-{version}-{cp}-{cp}-"; wheels=[x["download_url"] for x in json.load(sys.stdin) if x["name"].startswith(prefix) and x["name"].endswith(".whl") and ((kind=="toolkit" and "x86_64" in x["name"] and "manylinux" in x["name"]) or (kind=="lite" and "aarch64" in x["name"]))]; print(wheels[0] if wheels else "")' \
        "$version" "$package" "$cp" "$kind" <<<"$response"
}

toolkit_wheel_python_minors() {
    local tag="$1" response
    response="$(curl --silent --show-error --fail --location --connect-timeout 15 --max-time 60 \
        "$RKNN3_API/contents/rknn3-toolkit/packages?ref=$tag")" || return 1
    python3 -c 'import json,re,sys; names=[x["name"] for x in json.load(sys.stdin) if x["name"].endswith(".whl") and "x86_64" in x["name"] and "manylinux" in x["name"]]; tags=sorted({m.group(1) for n in names if (m:=re.search(r"-cp(\d+)-cp\d+-",n))}); print("\n".join("3."+t[1:] for t in tags))' \
        <<<"$response"
}

host_python_minors() {
    local system_minor=""
    if command -v python3 >/dev/null; then
        system_minor="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    fi
    {
        [[ -n "$system_minor" ]] && printf '%s\n' "$system_minor"
        if command -v uv >/dev/null; then
            uv python list --only-installed 2>/dev/null \
                | python3 -c 'import re,sys; print("\n".join(dict.fromkeys(re.findall(r"cpython-(\d+\.\d+)",sys.stdin.read()))))'
        fi
    } | awk 'NF && !seen[$0]++'
}

ensure_uv() {
    command -v uv >/dev/null ||
        die 'uv is missing; install it first: https://docs.astral.sh/uv/getting-started/installation/'
}

ensure_uv_python() {
    local version="$1"
    if uv python find "$version" >/dev/null 2>&1; then
        return
    fi
    ((CHECK_ONLY == 0)) || die "Python $version is missing from uv"
    uv python install "$version"
}

ensure_export_venv() {
    local path="$AIBOX_NPU_WORKDIR/venv-export"
    if [[ -x "$path/bin/python" ]]; then
        return
    fi
    ((CHECK_ONLY == 0)) || die "$path is missing"
    ensure_uv_python "$TOOLKIT_PYTHON_MINOR"
    mkdir -p "$(dirname "$path")"
    uv venv --python "$TOOLKIT_PYTHON_MINOR" "$path"
}

module_available() {
    local python="$1" module="$2"
    "$python" - "$module" <<'PY' >/dev/null 2>&1
import importlib
import sys

importlib.import_module(sys.argv[1])
PY
}

ensure_modules() {
    local python="$1"
    shift
    local pair module package
    local missing=()
    for pair in "$@"; do
        module="${pair%%=*}"
        package="${pair#*=}"
        if ! module_available "$python" "$module"; then
            missing+=("$package")
        fi
    done
    ((${#missing[@]} == 0)) && return 0
    ((CHECK_ONLY == 0)) || die "$(basename "$(dirname "$(dirname "$python")")") is missing Python packages: ${missing[*]}"
    uv pip install --python "$python" "${missing[@]}"
    for pair in "$@"; do
        module="${pair%%=*}"
        module_available "$python" "$module" || die "Python module $module is still missing after install"
    done
}

installed_dist_version() {
    local python="$1" distribution="$2"
    "$python" -c 'import importlib.metadata as m,sys; print(m.version(sys.argv[1]))' "$distribution" 2>/dev/null || true
}

ensure_toolkit_version() {
    local python="$1" distribution="$2" wheel="$3" label="$4"
    local version
    version="$(installed_dist_version "$python" "$distribution")"
    if [[ "$version" == "$RKNN3_VERSION" ]] && "$python" -c 'from rknn.api import RKNN' >/dev/null 2>&1; then
        info "$label version $version"
        return
    fi
    ((CHECK_ONLY == 0)) || die "$label reports '${version:-missing}' or cannot import rknn.api; expected $RKNN3_VERSION"
    ensure_download "$TOOLKIT_URL" "$wheel"
    [[ -f "$wheel" ]] || die "$label needs $RKNN3_VERSION but wheel is missing: $wheel"
    local cp="cp${TOOLKIT_PYTHON_MINOR//./}"
    [[ "$wheel" == *"-${RKNN3_VERSION}-${cp}-${cp}-"* ]] \
        || die "$wheel does not match Python $TOOLKIT_PYTHON_MINOR"
    info "installing $label $RKNN3_VERSION from $wheel"
    uv pip install --python "$python" --reinstall "$wheel"
    version="$(installed_dist_version "$python" "$distribution")"
    [[ "$version" == "$RKNN3_VERSION" ]] || die "$label reports '${version:-missing}' after installation"
    "$python" -c 'from rknn.api import RKNN' >/dev/null 2>&1 || die "$label still cannot import rknn.api after installation"
}

# The firmware's rknn3 API version is the source of truth for both ends: the
# Lite wheel and the host Toolkit must match the protocol the NPU firmware
# actually implements (the Lite-vs-firmware ABI mismatch the smoke test caught).
resolve_target_version() {
    local runtime host_version
    runtime="$(board_value runtime_version)"
    [[ -n "$runtime" && "$runtime" != missing ]] \
        || die 'cannot read the AIBOX firmware rknn3 API version via rknn-smi; fix the board first'
    RKNN3_VERSION="$(release_version "$runtime")"
    host_version=""
    if [[ -x "$AIBOX_NPU_WORKDIR/venv-toolkit/bin/python" ]]; then
        host_version="$(installed_dist_version "$AIBOX_NPU_WORKDIR/venv-toolkit/bin/python" rknn3-toolkit)"
    fi
    RKNN3_TAG="$(tag_for_version "$RKNN3_VERSION")"
    [[ -n "$RKNN3_TAG" ]] \
        || die "no official RKNN3 Toolkit release matches firmware API $RKNN3_VERSION (available: $(release_versions))"
    if [[ -n "$host_version" && "$host_version" != "$RKNN3_VERSION" ]]; then
        warn "aligning x86 Toolkit $host_version to firmware API version $RKNN3_VERSION"
    fi
    info "target RKNN3 version $RKNN3_VERSION from AIBOX firmware API ($RKNN3_TAG)"
}

select_toolkit_environment() {
    local path="$AIBOX_NPU_WORKDIR/venv-toolkit" candidate url available
    if [[ -x "$path/bin/python" ]]; then
        TOOLKIT_PYTHON="$path/bin/python"
        TOOLKIT_PYTHON_MINOR="$("$TOOLKIT_PYTHON" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
        TOOLKIT_URL="$(wheel_url_for "$RKNN3_TAG" toolkit "$TOOLKIT_PYTHON_MINOR")"
        [[ -n "$TOOLKIT_URL" ]] \
            || die "the official $RKNN3_TAG release has no x86 wheel for Python $TOOLKIT_PYTHON_MINOR"
        return
    fi
    ((CHECK_ONLY == 0)) || die "$path is missing"

    ensure_uv
    while IFS= read -r candidate; do
        [[ -n "$candidate" ]] || continue
        url="$(wheel_url_for "$RKNN3_TAG" toolkit "$candidate")"
        if [[ -n "$url" ]]; then
            TOOLKIT_PYTHON_MINOR="$candidate"
            TOOLKIT_URL="$url"
            break
        fi
    done < <(host_python_minors)

    if [[ -z "$TOOLKIT_PYTHON_MINOR" ]]; then
        available="$(toolkit_wheel_python_minors "$RKNN3_TAG" | head -n 1)"
        [[ -n "$available" ]] || die "the official $RKNN3_TAG release has no Linux x86_64 Toolkit wheel"
        TOOLKIT_PYTHON_MINOR="$available"
        TOOLKIT_URL="$(wheel_url_for "$RKNN3_TAG" toolkit "$TOOLKIT_PYTHON_MINOR")"
        ensure_uv_python "$TOOLKIT_PYTHON_MINOR"
    fi

    mkdir -p "$(dirname "$path")"
    uv venv --python "$TOOLKIT_PYTHON_MINOR" "$path"
    TOOLKIT_PYTHON="$path/bin/python"
}

select_wheel_paths() {
    local board_url
    if ((BOARD_ONLY == 0)); then
        select_toolkit_environment
        if [[ -z "$RKNN3_TOOLKIT_WHEEL" ]]; then
            RKNN3_TOOLKIT_WHEEL="$AIBOX_NPU_WORKDIR/downloads/$(basename "$TOOLKIT_URL")"
        else
            RKNN3_TOOLKIT_WHEEL="$(expand_home "$RKNN3_TOOLKIT_WHEEL")"
        fi
    fi
    if ((HOST_ONLY == 0)); then
        BOARD_PYTHON_MINOR="$(board_value python)"
        [[ "$BOARD_PYTHON_MINOR" =~ ^[0-9]+\.[0-9]+$ ]] \
            || die "cannot determine AIBOX Python version: $BOARD_PYTHON_MINOR"
        board_url="$(wheel_url_for "$RKNN3_TAG" lite "$BOARD_PYTHON_MINOR")"
        [[ -n "$board_url" ]] \
            || die "the official $RKNN3_TAG release has no AIBOX Lite wheel for Python $BOARD_PYTHON_MINOR"
        if [[ -z "$RKNN3_TOOLKIT_LITE_WHEEL" ]]; then
            RKNN3_TOOLKIT_LITE_WHEEL="$AIBOX_NPU_WORKDIR/downloads/$(basename "$board_url")"
        else
            RKNN3_TOOLKIT_LITE_WHEEL="$(expand_home "$RKNN3_TOOLKIT_LITE_WHEEL")"
        fi
        TOOLKIT_LITE_URL="$board_url"
    fi
}

prepare_host() {
    local arch os_name export_env export_python
    os_name="Linux"
    [[ ! -r /etc/os-release ]] || . /etc/os-release
    os_name="${PRETTY_NAME:-$os_name}"
    [[ "$(uname -s)" == Linux ]] || die "Linux host required; found $(uname -s)"
    arch="$(uname -m)"
    [[ "$arch" == x86_64 ]] || die "x86 host required; found $arch"

    ensure_host_commands
    ensure_uv
    if ((CHECK_ONLY == 0)); then
        mkdir -p "$AIBOX_NPU_WORKDIR"
    fi
    export_env="$AIBOX_NPU_WORKDIR/venv-export"
    ensure_export_venv
    export_python="$export_env/bin/python"

    ensure_modules "$export_python" \
        cv2=opencv-python numpy=numpy torch=torch torchvision=torchvision \
        ultralytics=ultralytics timm=timm \
        clip=git+https://github.com/openai/CLIP.git onnx=onnx onnxruntime=onnxruntime \
        onnxscript=onnxscript
    ensure_toolkit_version "$TOOLKIT_PYTHON" rknn3-toolkit "$RKNN3_TOOLKIT_WHEEL" 'x86 RKNN3 Toolkit'
    ensure_modules "$TOOLKIT_PYTHON" \
        cv2=opencv-python numpy=numpy onnx=onnx onnxruntime=onnxruntime
    info "x86 host ready ($os_name, $arch, Python $TOOLKIT_PYTHON_MINOR); workdir=$AIBOX_NPU_WORKDIR"
}

board_probe() {
    ssh -o BatchMode=yes -o ConnectTimeout=10 "$AIBOX_SSH_TARGET" 'bash -s' <<'REMOTE'
set -eu
if [[ -r /etc/os-release ]]; then . /etc/os-release; fi
printf 'kernel=%s\n' "$(uname -s)"
printf 'arch=%s\n' "$(uname -m)"
printf 'os=%s\n' "${ID:-unknown}"
printf 'version=%s\n' "${VERSION_ID:-unknown}"
if command -v apt-get >/dev/null; then printf 'apt_get=ready\n'; else printf 'apt_get=missing\n'; fi
printf 'python=%s\n' "$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")' 2>/dev/null || printf missing)"
printf 'lite=%s\n' "$(python3 -c 'import importlib.metadata as m; print(m.version("rknn3-toolkit-lite"))' 2>/dev/null || printf missing)"
# Older pips (Ubuntu 22.04 ships 22.0.2) reject --break-system-packages.
if python3 -m pip install --help 2>/dev/null | grep -q -- --break-system-packages; then
    printf 'pip_flag=--break-system-packages\n'
else
    printf 'pip_flag=\n'
fi
printf 'apt_missing='
missing=()
command -v clinfo >/dev/null || missing+=(clinfo)
python3 -m pip --version >/dev/null 2>&1 || missing+=(python3-pip)
python3 -c 'import numpy' >/dev/null 2>&1 || missing+=(python3-numpy)
python3 -c 'import cv2' >/dev/null 2>&1 || missing+=(python3-opencv)
if ((${#missing[@]})); then IFS=,; printf '%s' "${missing[*]}"; fi
printf '\n'
python3 - <<'PY'
import importlib
modules = ("rknn3lite", "numpy", "cv2", "transformers")
missing = []
for name in modules:
    try:
        importlib.import_module(name)
    except Exception:
        missing.append(name)
print("python_missing=" + ",".join(missing))
PY
if ! command -v rknn-smi >/dev/null; then
    printf 'rknn_device=unreported\n'
    printf 'runtime_version=\n'
elif rknn-smi info -l >/dev/null 2>&1; then
    printf 'rknn_device=ready\n'
    # rknn-smi -v intermittently omits the API line; retry until it shows up.
    runtime_version=""
    for _ in 1 2 3 4 5; do
        runtime_version="$(rknn-smi -v 2>&1 | awk -F: '/rknn3 API version/ {gsub(/[[:space:]]/, "", $2); print $2; exit}')"
        [[ -n "$runtime_version" ]] && break
        sleep 1
    done
    printf 'runtime_version=%s\n' "$runtime_version"
else
    printf 'rknn_device=error\n'
    printf 'runtime_version=\n'
fi
REMOTE
}

board_value() {
    local key="$1"
    awk -F= -v wanted="$key" '$1 == wanted {sub(/^[^=]*=/, ""); print; exit}' <<<"$BOARD_INFO"
}

prepare_board() {
    local kernel arch os_id version python lite apt_get apt_missing python_missing device runtime pip_flag
    [[ -n "$BOARD_INFO" ]] \
        || BOARD_INFO="$(board_probe)" \
        || die "cannot connect to board through SSH target '$AIBOX_SSH_TARGET'"
    kernel="$(board_value kernel)"
    arch="$(board_value arch)"
    os_id="$(board_value os)"
    version="$(board_value version)"
    python="$(board_value python)"
    lite="$(board_value lite)"
    apt_missing="$(board_value apt_missing)"
    python_missing="$(board_value python_missing)"
    device="$(board_value rknn_device)"
    pip_flag="$(board_value pip_flag)"

    apt_get="$(board_value apt_get)"
    [[ "$kernel" == Linux && "$arch" == aarch64 ]] \
        || die "AIBOX must be Linux/aarch64; found $kernel/$arch"
    [[ "$device" != error ]] || die 'rknn-smi cannot see the RK1828 device'
    local needs_install=0
    [[ "$lite" == "$RKNN3_VERSION" ]] || needs_install=1
    [[ -z "$apt_missing" ]] || needs_install=1
    [[ -z "$python_missing" ]] || needs_install=1
    if ((CHECK_ONLY)); then
        [[ "$needs_install" == 0 ]] || die "board needs preparation (Lite=${lite:-missing}, apt=${apt_missing:-none}, Python=${python_missing:-none})"
    elif ((needs_install)); then
        if [[ -n "$apt_missing" ]]; then
            [[ "$apt_get" == ready ]] || die "AIBOX needs packages ${apt_missing//,/ } but apt-get is unavailable"
            ssh -o BatchMode=yes "$AIBOX_SSH_TARGET" \
                "apt-get update && apt-get install -y --no-install-recommends ${apt_missing//,/ }"
        fi
        if [[ "$lite" != "$RKNN3_VERSION" || -n "$python_missing" ]]; then
            [[ -n "$lite" && "$lite" != missing && "$lite" != "$RKNN3_VERSION" ]] \
                && warn "replacing AIBOX Lite $lite with $RKNN3_VERSION"
            ensure_download "$TOOLKIT_LITE_URL" "$RKNN3_TOOLKIT_LITE_WHEEL"
            [[ -f "$RKNN3_TOOLKIT_LITE_WHEEL" ]] \
                || die "board Lite needs $RKNN3_VERSION but wheel is missing: $RKNN3_TOOLKIT_LITE_WHEEL"
            local board_cp="cp${BOARD_PYTHON_MINOR//./}"
            [[ "$RKNN3_TOOLKIT_LITE_WHEEL" == *"-${RKNN3_VERSION}-${board_cp}-${board_cp}-"* ]] \
                || die "$RKNN3_TOOLKIT_LITE_WHEEL does not match AIBOX Python $BOARD_PYTHON_MINOR"
            local wheel_name="$(basename "$RKNN3_TOOLKIT_LITE_WHEEL")"
            [[ "$wheel_name" =~ ^[A-Za-z0-9._+-]+\.whl$ ]] || die "invalid wheel filename: $wheel_name"
            local remote_wheel="/tmp/$wheel_name"
            scp -q "$RKNN3_TOOLKIT_LITE_WHEEL" "$AIBOX_SSH_TARGET:$remote_wheel"
            local reinstall=''
            [[ "$lite" == "$RKNN3_VERSION" ]] || reinstall='--force-reinstall'
            ssh -o BatchMode=yes "$AIBOX_SSH_TARGET" \
                "python3 -m pip install $pip_flag $reinstall '$remote_wheel' && rm -f -- '$remote_wheel'"
        fi
    fi

    if ((needs_install)); then
        BOARD_INFO="$(board_probe)" || die 'board probe failed after installation'
        [[ "$(board_value lite)" == "$RKNN3_VERSION" ]] || die 'board Lite version does not match the selected version after installation'
        [[ -z "$(board_value python_missing)" ]] || die "board Python modules still missing: $(board_value python_missing)"
        lite="$(board_value lite)"
    fi
    if [[ "$device" == unreported ]]; then
        info 'rknn-smi is unavailable; hardware validation remains for the board smoke-test step'
    fi
    runtime="$(board_value runtime_version)"
    if [[ -n "$runtime" && "$runtime" != missing && "$runtime" != "$RKNN3_VERSION" ]]; then
        warn "AIBOX firmware rknn3 API is $runtime while Lite is $RKNN3_VERSION; run the board smoke test before relying on the NPU"
    fi
    info "AIBOX ready ($os_id $version, $arch, Python $python, RKNN3 Lite $lite)"
}

ensure_host_commands
# The board is probed even for --host-only: the firmware API version decides
# which Toolkit release the host installs.
BOARD_INFO="$(board_probe)" || die "cannot connect to board through SSH target '$AIBOX_SSH_TARGET'"
load_releases
resolve_target_version
select_wheel_paths

if ((BOARD_ONLY == 0)); then
    prepare_host
fi
if ((HOST_ONLY == 0)); then
    prepare_board
fi

info "RKNN3 Toolkit and Lite match version $RKNN3_VERSION"
