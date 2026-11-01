# SPDX-License-Identifier: MulanPSL-2.0
"""Launch a Python consumer with only runtime settings and catalog credentials."""
from __future__ import annotations

import os
import sys
from pathlib import Path

from . import catalog_from_env


def main() -> None:
    if len(sys.argv) < 2:
        raise SystemExit("usage: python -m robonix_api.llm MODULE [ARGS...]")
    env = dict(os.environ)
    catalog = catalog_from_env()
    if catalog is not None:
        allowed = set(Path(__file__).with_name("runtime_env.txt").read_text().split())
        key_names = {model.api_key_env for model in catalog.models if model.api_key_env}
        allowed.update(key_names)
        env = {name: value for name, value in env.items() if name in allowed}
        env["ROBONIX_LLM_KEY_ENVS"] = ",".join(sorted(key_names))
    os.execve(sys.executable, [sys.executable, "-m", *sys.argv[1:]], env)


if __name__ == "__main__":
    main()
