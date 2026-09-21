# SPDX-License-Identifier: MulanPSL-2.0
"""Resolve the Scene debug UI bind address.

The UI carries no authentication and its annotation endpoints read and write
map data, so reaching it is enough to alter what the robot believes about its
world. The default is therefore loopback: exposing that surface to a network
is a decision, and a decision should be made rather than inherited.

`web_host` in the Scene config, or SCENE_WEB_HOST, opens it deliberately —
a deployment whose operator UI is reached from another machine sets one of
them, and the service says at startup when it is bound wide.
"""

from __future__ import annotations

import ipaddress
import os
import re
from collections.abc import Mapping


def resolve_web_host(
    config: Mapping[str, object],
    environ: Mapping[str, str] | None = None,
) -> str:
    """Return the Uvicorn host from config, environment, or legacy default."""

    env = os.environ if environ is None else environ
    raw = (
        config["web_host"]
        if "web_host" in config
        else env.get("SCENE_WEB_HOST", "127.0.0.1")
    )
    if not isinstance(raw, str):
        raise ValueError("Scene web_host/SCENE_WEB_HOST must be a string")
    host = raw.strip()
    if not host:
        raise ValueError("Scene web_host/SCENE_WEB_HOST must not be blank")
    invalid = (
        "://" in host
        or "/" in host
        or any(char.isspace() for char in host)
    )
    if re.fullmatch(r"[0-9xX.]+", host):
        try:
            ipaddress.ip_address(host)
        except ValueError:
            invalid = True
    if invalid:
        raise ValueError(f"invalid Scene web bind host: {host!r}")
    return host
