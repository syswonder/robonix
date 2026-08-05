#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time


DEMO_ROOT = Path(__file__).resolve().parent
DEFAULT_CONTROL_FILE = DEMO_ROOT / ".runtime" / "piper_health.json"
TARGETS = tuple([*(f"joint_{index}" for index in range(1, 7)), "gripper"])


def parse_args() -> argparse.Namespace:
    """Parse one fault, recovery, or status operation."""
    parser = argparse.ArgumentParser(
        description="Inject and recover Piper health faults during a Vitals demo."
    )
    parser.add_argument(
        "--control-file",
        type=Path,
        default=DEFAULT_CONTROL_FILE,
        help="runtime profile consumed by piper_health",
    )
    commands = parser.add_subparsers(dest="command", required=True)
    fault = commands.add_parser("fault", help="mark selected components unhealthy")
    fault.add_argument("targets", nargs="+", choices=TARGETS)
    fault.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="automatically recover after this many seconds",
    )
    commands.add_parser("recover", help="return all components to normal")
    commands.add_parser("status", help="show the current injected profile")
    return parser.parse_args()


def profile(mode: str, targets: list[str]) -> dict:
    """Build the small versioned document consumed by the health driver."""
    return {
        "schemaVersion": 1,
        "mode": mode,
        "targets": sorted(set(targets)),
        "updatedAt": datetime.now(timezone.utc).isoformat(),
    }


def write_profile(path: Path, document: dict) -> None:
    """Replace the runtime profile atomically on the local filesystem."""
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(f"{path.suffix}.tmp")
    temporary.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def print_status(path: Path) -> None:
    """Display the profile without requiring the Robonix Python environment."""
    path = path.expanduser().resolve()
    if not path.exists():
        print(f"normal (no control file at {path})")
        return
    document = json.loads(path.read_text(encoding="utf-8"))
    print(json.dumps(document, indent=2))


def main() -> int:
    """Apply the requested demo transition and optionally recover after a delay."""
    args = parse_args()
    control_file = args.control_file.expanduser().resolve()
    if args.command == "status":
        print_status(control_file)
        return 0
    if args.command == "recover":
        write_profile(control_file, profile("normal", []))
        print(f"Piper recovered: {control_file}")
        return 0

    if args.duration < 0:
        raise SystemExit("--duration must be zero or greater")
    write_profile(control_file, profile("fault", args.targets))
    print(f"Piper fault injected: {', '.join(sorted(set(args.targets)))}")
    print(f"Control file: {control_file}")
    if not args.duration:
        return 0
    try:
        print(f"Automatic recovery in {args.duration:g} seconds...")
        time.sleep(args.duration)
    finally:
        write_profile(control_file, profile("normal", []))
        print("Piper recovered")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
