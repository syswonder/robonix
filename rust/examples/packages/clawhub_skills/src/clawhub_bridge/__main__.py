#!/usr/bin/env python3
"""Agent Skills (agentskills.io) / ClawHub bridge node for Robonix.

Registers with Atlas and imports all SKILL.md files from the skills/
directory (populated by build.sh from openclaw/skills or any Agent Skills
compatible source). The node stays alive with heartbeat so Pilot can
discover the skills.

Robonix natively supports the Agent Skills open standard (agentskills.io):
  ---
  name: skill-name                       # required (1-64 chars, lowercase)
  description: what it does and when      # required (1-1024 chars)
  license: Apache-2.0                    # optional
  compatibility: Requires Python 3.10+   # optional
  allowed-tools: Bash(git:*) Read        # optional (experimental)
  ---
  # Markdown body (instructions for the VLM agent)

This format is used by 30+ AI tools including Claude Code, Cursor, VS Code,
GitHub Copilot, OpenCode, Gemini CLI, and OpenClaw/ClawHub.
"""
import logging
import os
import signal
import sys
import threading
import time
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format="[clawhub_bridge] %(levelname)s %(message)s",
)
log = logging.getLogger("clawhub_bridge")


def _ensure_proto_gen() -> None:
    """Walk up from this file to find a package-local `proto_gen/` populated by
    `rbnx codegen` (or the legacy build.sh path). The shared
    `rust/examples/proto_gen/` is deprecated and no longer regenerated."""
    d = Path(__file__).resolve().parent
    while d.parent != d:
        pg = d / "proto_gen"
        if pg.is_dir() and (pg / "robonix_runtime_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent


_ensure_proto_gen()

try:
    import grpc
    import robonix_runtime_pb2 as pb
    import robonix_runtime_pb2_grpc as pb_grpc
except ImportError:
    log.error("Proto stubs not found. Run gen_proto_python.sh first.")
    sys.exit(1)


def _parse_skill_md(path: Path) -> dict:
    """Parse SKILL.md frontmatter (compatible with both ClawHub and Robonix)."""
    content = path.read_text(encoding="utf-8", errors="replace")
    name = path.parent.name
    description = ""
    metadata = {}

    if content.startswith("---"):
        parts = content.split("---", 2)
        if len(parts) >= 3:
            try:
                import yaml
                fm = yaml.safe_load(parts[1])
                if isinstance(fm, dict):
                    name = fm.get("name", name)
                    description = fm.get("description", "")
                    # Preserve ClawHub-specific metadata for transparency
                    if "metadata" in fm:
                        metadata = fm["metadata"]
            except Exception as e:
                log.warning("Failed to parse frontmatter in %s: %s", path, e)

    return {
        "name": name,
        "description": description,
        "path": str(path),
        "metadata": metadata,
    }


def _discover_skills(pkg_root: Path) -> list:
    """Find all SKILL.md files under skills/ directory."""
    skills_dir = pkg_root / "skills"
    if not skills_dir.exists():
        log.warning("No skills/ directory found at %s", skills_dir)
        return []

    skills = []
    for skill_md in sorted(skills_dir.rglob("SKILL.md")):
        info = _parse_skill_md(skill_md)
        skills.append(info)
        log.info("  found: %-25s — %s", info["name"], info["description"][:60])

    return skills


def _register_with_atlas(skills: list):
    """Register node + skills with Atlas control plane."""
    atlas = os.environ.get("ROBONIX_ATLAS", "localhost:50051")
    channel = grpc.insecure_channel(atlas)
    stub = pb_grpc.RobonixRuntimeStub(channel)
    node_id = "com.robonix.skills.clawhub"

    import json
    skill_infos = []
    for s in skills:
        meta = json.dumps({"source": "clawhub", **s.get("metadata", {})})
        skill_infos.append(pb.SkillInfo(
            name=s["name"],
            description=s["description"],
            path=s["path"],
            metadata_json=meta,
        ))

    try:
        stub.RegisterNode(pb.RegisterNodeRequest(
            node_id=node_id,
            namespace="robonix/skills/clawhub",
            kind="skill",
            skills=skill_infos,
        ))
        log.info("Registered with Atlas: %s — %d skills", node_id, len(skill_infos))
    except Exception as e:
        log.warning("Atlas registration failed: %s", e)
        return stub, node_id

    return stub, node_id


def main():
    # Find package root (where robonix_manifest.yaml lives)
    pkg_root = Path(__file__).resolve().parent.parent.parent
    if not (pkg_root / "robonix_manifest.yaml").exists():
        pkg_root = Path.cwd()

    log.info("Package root: %s", pkg_root)
    log.info("Discovering ClawHub skills...")

    skills = _discover_skills(pkg_root)
    if not skills:
        log.warning("No skills found — did you run 'rbnx build' first?")

    stub, node_id = _register_with_atlas(skills)

    # Heartbeat loop — keep the node alive so skills remain discoverable
    log.info("ClawHub bridge running (Ctrl+C to stop)")
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    while not stop.is_set():
        try:
            stub.NodeHeartbeat(pb.NodeHeartbeatRequest(node_id=node_id))
        except Exception:
            pass
        stop.wait(timeout=10)

    log.info("Shutting down")


if __name__ == "__main__":
    main()
