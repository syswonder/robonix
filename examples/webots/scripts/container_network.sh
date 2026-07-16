#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0

# Resolve the Atlas endpoint seen by a driver launched with `docker exec`.
# In the default host-network simulator, host and container share localhost.
# In an isolated bridge-network simulator, localhost belongs to the container,
# so translate a host-loopback Atlas endpoint to that network's gateway.
#
# ROBONIX_SIM_ATLAS is an explicit container-reachable override and therefore
# bypasses Docker network discovery. A loopback ROBONIX_ATLAS value, however,
# must never leak into a bridge container: fail closed when Docker cannot give
# us a usable gateway instead of returning an endpoint that dials the container
# itself.
resolve_container_atlas_endpoint() {
    local container="$1"
    local requested network_mode gateway port

    if [[ -n "${ROBONIX_SIM_ATLAS:-}" ]]; then
        printf '%s\n' "$ROBONIX_SIM_ATLAS"
        return
    fi

    requested="${ROBONIX_ATLAS:-127.0.0.1:50051}"
    network_mode="$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$container" 2>/dev/null || true)"
    if [[ "$network_mode" == "host" ]]; then
        printf '%s\n' "$requested"
        return
    fi

    case "$requested" in
        127.0.0.1:*|localhost:*|0.0.0.0:*)
            gateway="$(
                { docker inspect \
                    -f '{{range .NetworkSettings.Networks}}{{if .Gateway}}{{println .Gateway}}{{end}}{{end}}' \
                    "$container" 2>/dev/null || true; } \
                    | awk '/^([0-9]{1,3}\.){3}[0-9]{1,3}$/ { print; exit }'
            )"
            port="${requested##*:}"
            if [[ "$gateway" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
                printf '%s:%s\n' "$gateway" "$port"
                return
            fi
            echo "[webots/network] error: cannot resolve a Docker gateway for bridge-network container '$container' (network mode: ${network_mode:-unknown})." >&2
            echo "[webots/network] Set ROBONIX_SIM_ATLAS=<container-reachable-host>:<port> to override discovery." >&2
            return 1
            ;;
    esac

    printf '%s\n' "$requested"
}

# Append hosts to an existing comma-separated proxy bypass list without
# duplicating entries. Callers pass the translated Atlas gateway and the
# container's advertised bridge IP so both directions of host/container RPC
# avoid an inherited HTTP(S) proxy.
append_no_proxy_hosts() {
    local value="${1:-}"
    local host
    shift || true

    for host in "$@"; do
        [[ -n "$host" ]] || continue
        case ",$value," in
            *",$host,"*) ;;
            *) value="${value:+$value,}$host" ;;
        esac
    done

    printf '%s\n' "$value"
}
