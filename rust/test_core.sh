#!/bin/bash

set -e

# Cleanup function to kill robonix-core
cleanup() {
    if [ -n "$ROBONIX_PID" ]; then
        echo ""
        echo "Cleaning up: killing robonix-core (PID: $ROBONIX_PID)..."
        kill -9 "$ROBONIX_PID" 2>/dev/null || true
    else
        echo ""
        echo "Cleaning up: killing robonix-core..."
        pkill -9 -f "robonix-core" 2>/dev/null || true
    fi
    exit 0
}

# Set up signal handlers for Ctrl-C and script termination
trap cleanup SIGINT SIGTERM

make fmt

make build-sdk
eval $(make source-sdk)

# kill any running robonix-core process
echo "Killing any running robonix-core processes..."
pkill -9 -f "robonix-core" 2>/dev/null || true
sleep 1

# kill any process using port 8000
echo "Freeing port 8000..."
PORT=8000

# Try multiple methods to find and kill the process
KILLED=false

# Method 1: fuser (most reliable on Linux)
if command -v fuser >/dev/null 2>&1; then
    if fuser -k ${PORT}/tcp 2>/dev/null; then
        KILLED=true
        sleep 1
    fi
fi

# Method 2: ss (modern Linux)
if [ "$KILLED" = false ] && command -v ss >/dev/null 2>&1; then
    PIDS=$(ss -lptn "sport = :${PORT}" 2>/dev/null | grep -oP 'pid=\K[0-9]+' | sort -u)
    if [ -n "$PIDS" ]; then
        for PID in $PIDS; do
            kill -9 "$PID" 2>/dev/null && KILLED=true
        done
        sleep 1
    fi
fi

# Method 3: netstat (fallback)
if [ "$KILLED" = false ] && command -v netstat >/dev/null 2>&1; then
    PIDS=$(netstat -tlnp 2>/dev/null | grep ":${PORT}" | awk '{print $7}' | cut -d'/' -f1 | grep -E '^[0-9]+$' | sort -u)
    if [ -n "$PIDS" ]; then
        for PID in $PIDS; do
            kill -9 "$PID" 2>/dev/null && KILLED=true
        done
        sleep 1
    fi
fi

# Method 4: Parse /proc/net/tcp directly (last resort)
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

# Wait a moment for the port to be released
sleep 2

# Verify port is free
if command -v ss >/dev/null 2>&1; then
    if ss -lptn "sport = :${PORT}" 2>/dev/null | grep -q ":${PORT}"; then
        echo "Warning: Port ${PORT} may still be in use. Trying one more time..."
        pkill -9 -f "robonix-core" 2>/dev/null || true
        sleep 2
    fi
fi

make install

# Start robonix-core in background and save PID
ROBONIX_WEB_STATIC_DIR="$(pwd)/robonix-core/web_gui/static" \
ROBONIX_WEB_PORT=8000 \
RUST_LOG=robonix_core=debug robonix-core &
ROBONIX_PID=$!

# Wait for robonix-core, so we can catch Ctrl-C and clean up
wait $ROBONIX_PID