# SPDX-License-Identifier: MulanPSL-2.0
"""Prepare self-contained JavaScript/CSS assets for the HTML CI report."""

from __future__ import annotations

import argparse
import os
import subprocess
from pathlib import Path

PACKAGES = [
    "jquery@3.7.1",
    "jstree@3.3.17",
    "ace-builds@1.44.0",
]

STYLE_FILES = [
    "jstree/dist/themes/default/style.min.css",
]

SCRIPT_FILES = [
    "jquery/dist/jquery.min.js",
    "jstree/dist/jstree.min.js",
    "ace-builds/src-min-noconflict/ace.js",
    "ace-builds/src-min-noconflict/ext-searchbox.js",
    "ace-builds/src-min-noconflict/theme-tomorrow_night_eighties.js",
    "ace-builds/src-min-noconflict/mode-json.js",
    "ace-builds/src-min-noconflict/mode-yaml.js",
    "ace-builds/src-min-noconflict/mode-sh.js",
    "ace-builds/src-min-noconflict/mode-python.js",
    "ace-builds/src-min-noconflict/mode-rust.js",
    "ace-builds/src-min-noconflict/mode-javascript.js",
    "ace-builds/src-min-noconflict/mode-typescript.js",
    "ace-builds/src-min-noconflict/mode-markdown.js",
]


def _write_github_multiline(env_file: Path, name: str, values: list[Path]) -> None:
    with env_file.open("a", encoding="utf-8") as f:
        f.write(f"{name}<<__ROBONIX_REPORT_ASSETS__\n")
        for value in values:
            f.write(f"{value}\n")
        f.write("__ROBONIX_REPORT_ASSETS__\n")


def main() -> int:
    ap = argparse.ArgumentParser(description="Prepare offline assets for testing/report.py")
    ap.add_argument("--asset-dir", type=Path, required=True)
    ap.add_argument("--env-file", type=Path, help="GitHub Actions env file to update; defaults to $GITHUB_ENV")
    args = ap.parse_args()

    args.asset_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["npm", "--prefix", str(args.asset_dir), "install", "--no-audit", "--no-fund", *PACKAGES],
        check=True,
    )
    node_modules = args.asset_dir / "node_modules"
    styles = [node_modules / rel for rel in STYLE_FILES]
    scripts = [node_modules / rel for rel in SCRIPT_FILES]
    missing = [str(path) for path in [*styles, *scripts] if not path.exists()]
    if missing:
        raise FileNotFoundError("missing report asset(s): " + ", ".join(missing))

    env_file = args.env_file or (Path(os.environ["GITHUB_ENV"]) if os.environ.get("GITHUB_ENV") else None)
    if env_file:
        _write_github_multiline(env_file, "ROBONIX_REPORT_INLINE_STYLES", styles)
        _write_github_multiline(env_file, "ROBONIX_REPORT_INLINE_SCRIPTS", scripts)
    else:
        print("ROBONIX_REPORT_INLINE_STYLES")
        print("\n".join(str(path) for path in styles))
        print("ROBONIX_REPORT_INLINE_SCRIPTS")
        print("\n".join(str(path) for path in scripts))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
