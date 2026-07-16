#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail

TEST_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../scripts/container_network.sh
source "$TEST_DIR/../scripts/container_network.sh"

fail() {
    echo "test_container_network: $*" >&2
    exit 1
}

assert_eq() {
    local expected="$1" actual="$2" label="$3"
    [[ "$actual" == "$expected" ]] \
        || fail "$label: expected '$expected', got '$actual'"
}

# Mock the two docker-inspect templates used by the helper. Tests stay focused
# on endpoint policy and do not require a Docker daemon.
docker() {
    [[ "${MOCK_DOCKER_FORBIDDEN:-0}" != "1" ]] \
        || fail "docker inspect must not run for ROBONIX_SIM_ATLAS override"
    [[ "$1" == "inspect" && "$2" == "-f" ]] \
        || fail "unexpected docker invocation: $*"
    case "$3" in
        *HostConfig.NetworkMode*) printf '%s\n' "${MOCK_NETWORK_MODE:-bridge}" ;;
        *NetworkSettings.Networks*) printf '%s\n' "${MOCK_GATEWAY:-}" ;;
        *) fail "unexpected docker inspect template: $3" ;;
    esac
}

unset ROBONIX_SIM_ATLAS ROBONIX_ATLAS
MOCK_NETWORK_MODE=host
assert_eq "127.0.0.1:50051" \
    "$(resolve_container_atlas_endpoint sim)" \
    "host networking preserves loopback"

MOCK_NETWORK_MODE=bridge
MOCK_GATEWAY=172.31.0.1
assert_eq "172.31.0.1:50051" \
    "$(resolve_container_atlas_endpoint sim)" \
    "bridge networking translates loopback"

ROBONIX_ATLAS=atlas.internal:50123
MOCK_GATEWAY=
assert_eq "atlas.internal:50123" \
    "$(resolve_container_atlas_endpoint sim)" \
    "non-loopback Atlas endpoint needs no translation"
unset ROBONIX_ATLAS

ROBONIX_SIM_ATLAS=atlas-for-sim.internal:50234
MOCK_DOCKER_FORBIDDEN=1
assert_eq "atlas-for-sim.internal:50234" \
    "$(resolve_container_atlas_endpoint sim)" \
    "explicit simulator override bypasses discovery"
unset ROBONIX_SIM_ATLAS MOCK_DOCKER_FORBIDDEN

MOCK_GATEWAY=
error_file="$(mktemp)"
trap 'rm -f "$error_file"' EXIT
if endpoint="$(resolve_container_atlas_endpoint sim 2>"$error_file")"; then
    fail "bridge networking without a gateway returned success: '$endpoint'"
fi
grep -q "cannot resolve a Docker gateway" "$error_file" \
    || fail "missing fail-closed gateway diagnostic"
grep -q "ROBONIX_SIM_ATLAS" "$error_file" \
    || fail "missing explicit-override guidance"

assert_eq "localhost,127.0.0.1,172.31.0.1,172.31.0.23" \
    "$(append_no_proxy_hosts 'localhost,127.0.0.1' 172.31.0.1 172.31.0.23 172.31.0.1)" \
    "proxy bypass includes gateway and container IP once"

echo "test_container_network: PASS"
