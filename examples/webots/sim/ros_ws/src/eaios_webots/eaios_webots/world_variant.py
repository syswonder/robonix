#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Materialize a bundled Webots world for one supported TIAGo variant."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import re
import sys


@dataclass(frozen=True)
class TiagoModel:
    proto_path: str
    node_name: str


MODELS = {
    "lite": TiagoModel(
        "projects/robots/pal_robotics/tiago_lite/protos/TiagoLite.proto",
        "TiagoLite",
    ),
    "full": TiagoModel(
        "projects/robots/pal_robotics/tiago/protos/Tiago.proto",
        "Tiago",
    ),
}


def _node_count(world: str, node_name: str) -> int:
    """Count top-level robot declarations without matching comments or field text."""
    return len(re.findall(rf"(?m)^\s*{re.escape(node_name)}\s*\{{", world))


def detect_variant(world: str) -> str:
    """Return the unique TIAGo model used by a world or reject ambiguous input."""
    matches = []
    details = []
    for variant, model in MODELS.items():
        proto_count = world.count(model.proto_path)
        node_count = _node_count(world, model.node_name)
        details.append(f"{variant}: proto={proto_count}, node={node_count}")
        if proto_count == 1 and node_count == 1:
            matches.append(variant)
        elif proto_count != 0 or node_count != 0:
            raise ValueError(
                "TIAGo world has inconsistent model markers (" + "; ".join(details) + ")"
            )
    if len(matches) != 1:
        raise ValueError(
            "expected exactly one supported TIAGo model (" + "; ".join(details) + ")"
        )
    return matches[0]


def render_variant(world: str, target_variant: str) -> str:
    """Switch the unique robot import and node while preserving the rest verbatim."""
    if target_variant not in MODELS:
        raise ValueError(
            f"unsupported TIAGo variant '{target_variant}'; choose lite or full"
        )
    source_variant = detect_variant(world)
    if source_variant == target_variant:
        return world
    source = MODELS[source_variant]
    target = MODELS[target_variant]
    rendered = world.replace(source.proto_path, target.proto_path, 1)
    rendered, replacements = re.subn(
        rf"(?m)^(\s*){re.escape(source.node_name)}(?=\s*\{{)",
        rf"\1{target.node_name}",
        rendered,
        count=1,
    )
    if replacements != 1:
        raise ValueError(f"failed to replace the {source.node_name} robot node")
    if detect_variant(rendered) != target_variant:
        raise ValueError("generated world did not contain the requested TIAGo model")
    return rendered


def materialize_world(source_path: Path, target_variant: str) -> Path:
    """Write a generated sibling world only when the source model must change."""
    source_path = source_path.resolve()
    world = source_path.read_text(encoding="utf-8")
    rendered = render_variant(world, target_variant)
    if rendered == world:
        return source_path
    output_path = source_path.with_name(
        f".{source_path.stem}.{target_variant}.generated.wbt"
    )
    output_path.write_text(rendered, encoding="utf-8")
    return output_path


def main(argv: list[str] | None = None) -> int:
    """Parse the entrypoint request, materialize its world, and print the path."""
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("--variant", choices=sorted(MODELS), default="lite")
    args = parser.parse_args(argv)
    try:
        output_path = materialize_world(args.source, args.variant)
    except (OSError, ValueError) as exc:
        print(f"[world_variant] {exc}", file=sys.stderr)
        return 2
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
