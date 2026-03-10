#!/bin/bash

set -e

make fmt

cleanup() {
    echo ""
    echo "Cleaning up: killing robonix-server, found pid(s): $(pgrep -x robonix-server | sort -n)"
    pgrep -x robonix-server | sort -n | xargs -r kill -9
    wait $ROBONIX_PID 2>/dev/null || true
    echo "Making sure robonix-server does not exist..."
    if pgrep -x robonix-server >/dev/null; then
        echo "robonix-server still exists"
        exit 1
    fi
    echo "Cleanup complete!"
    exit 0
}

trap cleanup SIGINT SIGTERM

make fmt

make build-sdk
eval $(make source-sdk)

echo "Killing any running robonix-server processes..."
pkill -9 -f "robonix-server" 2>/dev/null || true
sleep 1

echo "Freeing port 8000..."
PORT=8000

KILLED=false

if command -v fuser >/dev/null 2>&1; then
    if fuser -k ${PORT}/tcp 2>/dev/null; then
        KILLED=true
        sleep 1
    fi
fi

if [ "$KILLED" = false ] && command -v ss >/dev/null 2>&1; then
    PIDS=$(ss -lptn "sport = :${PORT}" 2>/dev/null | grep -oP 'pid=\K[0-9]+' | sort -u)
    if [ -n "$PIDS" ]; then
        for PID in $PIDS; do
            kill -9 "$PID" 2>/dev/null && KILLED=true
        done
        sleep 1
    fi
fi

if [ "$KILLED" = false ] && command -v netstat >/dev/null 2>&1; then
    PIDS=$(netstat -tlnp 2>/dev/null | grep ":${PORT}" | awk '{print $7}' | cut -d'/' -f1 | grep -E '^[0-9]+$' | sort -u)
    if [ -n "$PIDS" ]; then
        for PID in $PIDS; do
            kill -9 "$PID" 2>/dev/null && KILLED=true
        done
        sleep 1
    fi
fi

if [ "$KILLED" = false ] && [ -f /proc/net/tcp ]; then
    # Convert port to hex (8000 = 0x1f40)
    PORT_HEX=$(printf "%04x" ${PORT})
    # Find inode
    INODES=$(awk -v port=":${PORT_HEX}" '$2 ~ port {print $10}' /proc/net/tcp 2>/dev/null | sort -u)
    if [ -n "$INODES" ]; then
        for INODE in $INODES; do
            # Find process using this inode
            for PID in /proc/[0-9]*/fd/*; do
                if [ -L "$PID" ] && [ "$(readlink "$PID" 2>/dev/null)" = "socket:[$INODE]" ]; then
                    ACTUAL_PID=$(echo "$PID" | cut -d'/' -f3)
                    kill -9 "$ACTUAL_PID" 2>/dev/null && KILLED=true
                fi
            done
        done
        sleep 1
    fi
fi

sleep 2

if command -v ss >/dev/null 2>&1; then
    if ss -lptn "sport = :${PORT}" 2>/dev/null | grep -q ":${PORT}"; then
        echo "Warning: Port ${PORT} may still be in use. Trying one more time..."
        pkill -9 -f "robonix-server" 2>/dev/null || true
        sleep 2
    fi
fi

make install

set -m
ROBONIX_WEB_ASSETS_DIR="$(pwd)/robonix-core/web" \
ROBONIX_WEB_PORT=8000 \
RUST_LOG=robonix_server=info robonix-server &
ROBONIX_PID=$!
set +m

wait $ROBONIX_PID