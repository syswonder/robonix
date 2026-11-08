#!/bin/sh
# SPDX-License-Identifier: MulanPSL-2.0
# ripgrep is optional; retain a usable POSIX search path when installation fails.
if ! command -v rg >/dev/null 2>&1 && [ "$(id -u)" = 0 ]; then
    if command -v apt-get >/dev/null 2>&1; then
        apt-get install -y ripgrep || printf '%s\n' 'ripgrep installation failed; using POSIX grep.' >&2
    elif command -v apk >/dev/null 2>&1; then
        apk add ripgrep || printf '%s\n' 'ripgrep installation failed; using POSIX grep.' >&2
    elif command -v dnf >/dev/null 2>&1; then
        dnf install -y ripgrep || printf '%s\n' 'ripgrep installation failed; using POSIX grep.' >&2
    fi
fi
if ! command -v rg >/dev/null 2>&1; then
    printf '%s\n' 'ripgrep unavailable; Executor will search retained outputs with grep and sed.' >&2
fi
for tool in bash grep sed head tail; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        printf '%s\n' "Missing Executor runtime dependency: $tool" >&2
        exit 1
    fi
done
