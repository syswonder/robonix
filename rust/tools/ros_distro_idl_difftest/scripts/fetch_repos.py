#!/usr/bin/env python3
"""
Fetch ROS 2 interface repos per distro branch into repos/<repo_name>/<distro>/.
Cloned repos/ should be listed in .gitignore and not committed.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None


def load_config(config_path: Path) -> dict:
    if not config_path.is_file():
        raise FileNotFoundError(f"Config not found: {config_path}")
    with open(config_path, "r", encoding="utf-8") as f:
        content = f.read()
    if yaml is None:
        # Minimal parser when PyYAML is not available
        config = {"distros": [], "repos": [], "repos_dir": "repos"}
        in_distros = in_repos = False
        for line in content.splitlines():
            line = line.strip()
            if line == "distros:":
                in_distros = True
                in_repos = False
                continue
            if line == "repos:":
                in_distros = False
                in_repos = True
                continue
            if in_distros and line.startswith("- ") and not line.startswith("- name:"):
                config["distros"].append(line[2:].strip())
            if in_repos and "name:" in line and "url:" not in line:
                name = line.split("name:", 1)[1].strip()
                config["repos"].append({"name": name, "url": ""})
            if in_repos and "url:" in line and config["repos"]:
                config["repos"][-1]["url"] = line.split("url:", 1)[1].strip()
            if "repos_dir:" in line:
                config["repos_dir"] = line.split("repos_dir:", 1)[1].strip()
    else:
        config = yaml.safe_load(content)
    return config


def main() -> int:
    parser = argparse.ArgumentParser(description="Fetch ROS 2 interface repos per distro branch.")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "config.yaml",
        help="Path to config.yaml",
    )
    parser.add_argument(
        "--refresh",
        action="store_true",
        help="If repo dir exists, fetch and checkout branch instead of skip",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would be done without cloning",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    distros = config.get("distros") or ["humble", "jazzy", "rolling"]
    repos = config.get("repos") or []
    repos_dir = config.get("repos_dir") or "repos"

    base = args.config.parent
    if not os.path.isabs(repos_dir):
        repos_root = base / repos_dir
    else:
        repos_root = Path(repos_dir)

    if not repos:
        print("No repos in config.", file=sys.stderr)
        return 1

    for repo in repos:
        name = repo.get("name")
        url = repo.get("url")
        if not name or not url:
            print(f"Skip repo entry missing name/url: {repo}", file=sys.stderr)
            continue
        for distro in distros:
            dest = repos_root / name / distro
            if args.dry_run:
                print(f"[dry-run] would clone -b {distro} {url} -> {dest}")
                continue
            if dest.exists():
                if not (dest / ".git").exists():
                    print(f"Skip (not a git repo): {dest}", file=sys.stderr)
                    continue
                if args.refresh:
                    try:
                        subprocess.run(
                            ["git", "fetch", "origin", distro],
                            cwd=dest,
                            check=True,
                            capture_output=True,
                        )
                        subprocess.run(
                            ["git", "checkout", distro],
                            cwd=dest,
                            check=True,
                            capture_output=True,
                        )
                        print(f"Updated {name}/{distro}")
                    except subprocess.CalledProcessError as e:
                        print(f"Refresh failed for {name}/{distro}: {e}", file=sys.stderr)
                else:
                    print(f"Skip (exists): {dest}")
                continue
            dest.parent.mkdir(parents=True, exist_ok=True)
            try:
                subprocess.run(
                    ["git", "clone", "--depth", "1", "-b", distro, url, str(dest)],
                    check=True,
                )
                print(f"Cloned {name} branch {distro} -> {dest}")
            except subprocess.CalledProcessError as e:
                print(f"Clone failed -b {distro} {url}: {e}", file=sys.stderr)
                # Some repos may not have that branch
                if dest.exists():
                    import shutil
                    shutil.rmtree(dest, ignore_errors=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())
