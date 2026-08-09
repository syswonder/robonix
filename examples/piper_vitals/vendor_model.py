#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_PREFIX = "package://agx_arm_description/agx_arm_urdf/"
XACRO_INCLUDE = "{http://www.ros.org/wiki/xacro}include"
UPSTREAM_URL = "https://github.com/agilexrobotics/agx_arm_urdf"


def parse_args() -> argparse.Namespace:
    """Parse the pinned upstream checkout and generated output locations."""
    parser = argparse.ArgumentParser(
        description="Bundle the AgileX Piper and parallel gripper for Soma."
    )
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--revision", required=True)
    parser.add_argument("--output", type=Path, default=SCRIPT_DIR / "model")
    return parser.parse_args()


def sha256(path: Path) -> str:
    """Return the streaming SHA-256 digest of one generated artifact."""
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_robot(path: Path) -> ET.Element:
    """Load one upstream robot document and validate its root element."""
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError(f"expected robot root in {path}")
    return root


def merged_visual_robot(source: Path) -> ET.Element:
    """Expand the simple include and remove collision-only geometry."""
    base_path = source / "piper/urdf/piper_description.urdf"
    gripper_path = source / "piper/urdf/piper_with_gripper_description.xacro"
    root = ET.Element("robot", {"name": "piper_with_gripper"})
    for child in load_robot(base_path):
        root.append(copy.deepcopy(child))
    for child in load_robot(gripper_path):
        if child.tag != XACRO_INCLUDE:
            root.append(copy.deepcopy(child))
    for link in root.findall("link"):
        for child in list(link):
            if child.tag == "collision":
                link.remove(child)
    return root


def relative_asset_path(reference: str) -> tuple[PurePosixPath, PurePosixPath]:
    """Translate one package URI into a safe model-local relative path."""
    if not reference.startswith(PACKAGE_PREFIX):
        raise ValueError(f"unsupported Piper asset URI: {reference}")
    source_path = PurePosixPath(reference.removeprefix(PACKAGE_PREFIX))
    if source_path.is_absolute() or ".." in source_path.parts:
        raise ValueError(f"unsafe Piper asset URI: {reference}")
    if len(source_path.parts) < 3 or source_path.parts[:2] != ("piper", "meshes"):
        raise ValueError(f"asset is outside the Piper model: {reference}")
    output_path = PurePosixPath(*source_path.parts[1:])
    return source_path, output_path


def bundle(source: Path, output: Path, revision: str) -> None:
    """Write one display URDF, its visual assets, license, and hash manifest."""
    source = source.resolve()
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    mesh_output = output / "meshes"
    if mesh_output.exists():
        shutil.rmtree(mesh_output)

    root = merged_visual_robot(source)
    root.insert(0, ET.Comment(" SPDX-License-Identifier: MIT "))
    assets: dict[str, dict[str, str | int]] = {}
    for mesh in root.iter("mesh"):
        source_path, output_path = relative_asset_path(mesh.get("filename", ""))
        source_asset = source.joinpath(*source_path.parts).resolve()
        source_asset.relative_to(source)
        if not source_asset.is_file():
            raise FileNotFoundError(f"missing Piper asset: {source_asset}")
        output_asset = output.joinpath(*output_path.parts)
        output_asset.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_asset, output_asset)
        wire_path = output_path.as_posix()
        mesh.set("filename", wire_path)
        assets[wire_path] = {
            "bytes": output_asset.stat().st_size,
            "sha256": sha256(output_asset),
        }

    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    urdf_path = output / "piper_with_gripper.urdf"
    tree.write(urdf_path, encoding="utf-8", xml_declaration=True)
    shutil.copy2(source / "LICENSE", output / "AGX_ARM_URDF_LICENSE-MIT.txt")
    manifest = {
        "spdxLicense": "MIT",
        "upstream": UPSTREAM_URL,
        "revision": revision,
        "sourceFiles": [
            "piper/urdf/piper_description.urdf",
            "piper/urdf/piper_with_gripper_description.xacro",
        ],
        "model": {
            "path": urdf_path.name,
            "bytes": urdf_path.stat().st_size,
            "sha256": sha256(urdf_path),
            "visuals": len(root.findall(".//visual")),
            "links": len(root.findall("link")),
            "joints": len(root.findall("joint")),
        },
        "assets": dict(sorted(assets.items())),
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    total_bytes = sum(int(asset["bytes"]) for asset in assets.values())
    print(f"bundled {len(assets)} Piper visual assets ({total_bytes} bytes)")


def main() -> None:
    """Generate the checked-in Piper model from one pinned upstream checkout."""
    args = parse_args()
    bundle(args.source, args.output, args.revision)


if __name__ == "__main__":
    main()
