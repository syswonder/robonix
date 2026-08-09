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


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
MODEL_ROOT = (
    REPOSITORY_ROOT
    / "examples/webots/sim/ros_ws/src/eaios_webots/resource"
)
WEBOTS_TAGS = {"webots", "ros2_control"}


def parse_args() -> argparse.Namespace:
    """Parse source descriptions and Webots overlay locations."""
    parser = argparse.ArgumentParser(
        description="Bundle PAL TIAGo visuals into the URDFs served by Soma."
    )
    parser.add_argument("--lite-urdf", type=Path, required=True)
    parser.add_argument("--full-urdf", type=Path, required=True)
    parser.add_argument("--tiago-description", type=Path, required=True)
    parser.add_argument("--pmb2-description", type=Path, required=True)
    parser.add_argument("--pal-gripper-description", type=Path, required=True)
    parser.add_argument("--pal-urdf-utils", type=Path, required=True)
    parser.add_argument(
        "--lite-overlay",
        type=Path,
        default=MODEL_ROOT / "tiago_webots.overlay.urdf",
    )
    parser.add_argument(
        "--full-overlay",
        type=Path,
        default=MODEL_ROOT / "tiago_full_webots.overlay.urdf",
    )
    parser.add_argument("--output", type=Path, default=MODEL_ROOT)
    return parser.parse_args()


def sha256(path: Path) -> str:
    """Return a streaming SHA-256 digest for a generated file."""
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def package_reference(value: str) -> tuple[str, PurePosixPath]:
    """Split and validate a package URI without resolving it yet."""
    relative = value.removeprefix("package://")
    package, separator, package_path = relative.partition("/")
    path = PurePosixPath(package_path)
    if not separator or not package or not package_path:
        raise ValueError(f"invalid package URI: {value}")
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"unsafe package URI: {value}")
    return package, path


def relative_reference(value: str) -> PurePosixPath:
    """Validate a URDF-local asset path used by an already bundled model."""
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts or not path.parts:
        raise ValueError(f"unsafe relative asset URI: {value}")
    return path


def load_urdf(path: Path) -> ET.ElementTree:
    """Parse one robot document while preserving source comments."""
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(path, parser=parser)
    if tree.getroot().tag != "robot":
        raise ValueError(f"expected a robot root element in {path}")
    return tree


def upstream_reference(
    value: str,
    asset_name: str,
    package_roots: dict[str, Path],
) -> str:
    """Recover an official package URI when regenerating a bundled model."""
    for package in package_roots:
        prefix = f"{package}__"
        if asset_name.startswith(prefix):
            package_path = asset_name.removeprefix(prefix).replace("__", "/")
            return f"package://{package}/{package_path}"
    return value


def resolve_asset(
    value: str,
    source_urdf: Path,
    package_roots: dict[str, Path],
) -> tuple[Path, str, str]:
    """Resolve a package or URDF-local reference and choose its bundled name."""
    if value.startswith("package://"):
        package, package_path = package_reference(value)
        if package not in package_roots:
            raise ValueError(f"no source root configured for {package} in {source_urdf}")
        source_root = package_roots[package].resolve()
        source_asset = source_root.joinpath(*package_path.parts).resolve()
        source_asset.relative_to(source_root)
        asset_name = "__".join((package, *package_path.parts))
        return source_asset, asset_name, value

    relative_path = relative_reference(value)
    source_root = source_urdf.parent.resolve()
    source_asset = source_root.joinpath(*relative_path.parts).resolve()
    source_asset.relative_to(source_root)
    asset_name = relative_path.name
    return source_asset, asset_name, upstream_reference(value, asset_name, package_roots)


def merge_webots_overlay(tree: ET.ElementTree, overlay_path: Path) -> None:
    """Replace source control metadata with the deployment's Webots nodes."""
    root = tree.getroot()
    overlay_root = load_urdf(overlay_path).getroot()
    for child in list(root):
        if child.tag in WEBOTS_TAGS:
            root.remove(child)
    for child in overlay_root:
        if child.tag in WEBOTS_TAGS:
            root.append(copy.deepcopy(child))
    root.set("name", overlay_root.get("name", root.get("name", "tiago")))


def output_reference(
    element_tag: str,
    source_asset: Path,
    asset_name: str,
) -> PurePosixPath:
    """Place generated resources below the standard type and format path."""
    resource_type = "meshes" if element_tag == "mesh" else "textures"
    resource_format = source_asset.suffix.lower().removeprefix(".")
    if not resource_format:
        raise ValueError(f"resource has no file extension: {source_asset}")
    return PurePosixPath(resource_type, resource_format, asset_name)


def bundle_urdf(
    source_urdf: Path,
    overlay_urdf: Path,
    output_urdf: Path,
    package_roots: dict[str, Path],
    output_root: Path,
    assets: dict[str, dict[str, str | int]],
) -> dict[str, str | int]:
    """Bundle visual resources and merge Webots controls into one URDF."""
    tree = load_urdf(source_urdf)
    merge_webots_overlay(tree, overlay_urdf)
    root = tree.getroot()
    for child in list(root):
        if child.tag is ET.Comment and "SPDX-License-Identifier:" in (child.text or ""):
            root.remove(child)
    root.insert(0, ET.Comment(" SPDX-License-Identifier: Apache-2.0 "))
    reference_count = 0
    for element in root.iter():
        if element.tag not in {"mesh", "texture"}:
            continue
        value = element.get("filename", "")
        if not value:
            continue
        source_asset, asset_name, source_reference = resolve_asset(
            value, source_urdf, package_roots
        )
        if not source_asset.is_file():
            raise FileNotFoundError(f"missing asset {value}: {source_asset}")

        resource_path = output_reference(element.tag, source_asset, asset_name)
        resource_key = resource_path.as_posix()
        output_asset = output_root.joinpath(*resource_path.parts)
        output_asset.parent.mkdir(parents=True, exist_ok=True)
        asset_digest = sha256(source_asset)
        existing_asset = assets.get(resource_key)
        if existing_asset is not None and existing_asset["sha256"] != asset_digest:
            raise ValueError(
                f"asset name collision for {resource_key}: "
                f"{existing_asset['source']} and {source_reference}"
            )
        if existing_asset is None:
            if source_asset != output_asset.resolve():
                shutil.copy2(source_asset, output_asset)
            assets[resource_key] = {
                "source": source_reference,
                "bytes": output_asset.stat().st_size,
                "sha256": asset_digest,
            }
        element.set("filename", resource_key)
        reference_count += 1

    ET.indent(tree, space="  ")
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)
    return {
        "source": source_urdf.name,
        "overlay": overlay_urdf.name,
        "bytes": output_urdf.stat().st_size,
        "sha256": sha256(output_urdf),
        "assetReferences": reference_count,
    }


def main() -> None:
    """Generate the Lite and Full Soma/Webots descriptions and manifest."""
    args = parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    package_roots = {
        "tiago_description": args.tiago_description,
        "pmb2_description": args.pmb2_description,
        "pal_gripper_description": args.pal_gripper_description,
        "pal_urdf_utils": args.pal_urdf_utils,
    }
    assets: dict[str, dict[str, str | int]] = {}
    models = {
        "tiago_webots.urdf": bundle_urdf(
            args.lite_urdf,
            args.lite_overlay,
            output / "tiago_webots.urdf",
            package_roots,
            output,
            assets,
        ),
        "tiago_full_webots.urdf": bundle_urdf(
            args.full_urdf,
            args.full_overlay,
            output / "tiago_full_webots.urdf",
            package_roots,
            output,
            assets,
        ),
    }

    for resource_root_name in ("meshes", "textures"):
        resource_root = output / resource_root_name
        if not resource_root.exists():
            continue
        for stale_asset in resource_root.rglob("*"):
            resource_key = stale_asset.relative_to(output).as_posix()
            if stale_asset.is_file() and resource_key not in assets:
                stale_asset.unlink()
        for stale_dir in sorted(resource_root.rglob("*"), reverse=True):
            if stale_dir.is_dir() and not any(stale_dir.iterdir()):
                stale_dir.rmdir()

    manifest = {
        "spdxLicense": "Apache-2.0",
        "models": models,
        "assets": dict(sorted(assets.items())),
    }
    manifest_path = output / "tiago_visuals.manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    total_bytes = sum(int(asset["bytes"]) for asset in assets.values())
    print(f"bundled {len(assets)} assets ({total_bytes} bytes) into {output}")


if __name__ == "__main__":
    main()
