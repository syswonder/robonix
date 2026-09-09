"""Every dependency whose major version breaks us must carry a bound.

Two outages came from the same shape: a library published a new major, an
unpinned `pip install` picked it up, and a service exited at import time. The
first was protobuf; the second was `mcp`, whose 2.x renamed `FastMCP` to
`MCPServer` while three modules still imported the old path.

The rule this file enforces is narrow on purpose. It does not demand that every
dependency be pinned — most are fine floating. It demands a bound on the ones
whose API we are known to be coupled to, so that adding a new requirements file
without the bound fails here rather than in a Webots run an hour later.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]

# Distribution name -> why it needs a bound.
COUPLED: dict[str, str] = {
    "mcp": "2.x renamed FastMCP to MCPServer; callers import mcp.server.fastmcp",
    "fastmcp": "tracks mcp's major",
    "protobuf": "generated stubs refuse to load on an older runtime",
    "grpcio": "must match the grpcio-tools that generated the stubs",
    "grpcio-tools": "generates stubs a specific runtime must accept",
}

# A requirement line: name, optional extras, then whatever constrains it.
REQ = re.compile(r"^\s*([A-Za-z0-9][A-Za-z0-9._-]*)\s*(\[[^\]]*\])?\s*(.*)$")


def _requirements_files() -> list[Path]:
    out: list[Path] = []
    for path in ROOT.rglob("requirements*.txt"):
        if any(part in {".git", "node_modules", "target", ".venv"} for part in path.parts):
            continue
        out.append(path)
    for path in ROOT.rglob("requirements/*.txt"):
        if any(part in {".git", "node_modules", "target", ".venv"} for part in path.parts):
            continue
        out.append(path)
    return sorted(set(out))


def _pyproject_files() -> list[Path]:
    return sorted(
        p
        for p in ROOT.rglob("pyproject.toml")
        if not any(part in {".git", "node_modules", "target", ".venv"} for part in p.parts)
    )


def _unbounded(name: str, rest: str) -> bool:
    """True when a coupled package can still float onto a new major.

    A lower bound alone does not count. `mcp>=1.0` reads as pinned and is not:
    it admits 2.x, which is exactly the release that broke the import. What is
    required is a ceiling — `==`, `~=`, or an explicit `<`.
    """
    if name.lower() not in COUPLED:
        return False
    # A comment or an environment marker is not a version bound.
    rest = rest.split("#", 1)[0].split(";", 1)[0].strip()
    if "@" in rest:  # direct URL or VCS reference; the ref is the pin
        return False
    has_ceiling = "==" in rest or "~=" in rest or re.search(r"<=?\s*\d", rest)
    return not has_ceiling


@pytest.mark.parametrize("path", _requirements_files(), ids=lambda p: str(p.relative_to(ROOT)))
def test_requirements_bound_coupled_packages(path: Path) -> None:
    offenders = []
    for lineno, line in enumerate(path.read_text().splitlines(), start=1):
        stripped = line.strip()
        if not stripped or stripped.startswith(("#", "-")):
            continue
        m = REQ.match(stripped)
        if m and _unbounded(m.group(1), m.group(3)):
            offenders.append(f"{path.relative_to(ROOT)}:{lineno}: {stripped} — {COUPLED[m.group(1).lower()]}")
    assert not offenders, "unbounded dependency known to break on a major bump:\n" + "\n".join(offenders)


@pytest.mark.parametrize("path", _pyproject_files(), ids=lambda p: str(p.relative_to(ROOT)))
def test_pyproject_bounds_coupled_packages(path: Path) -> None:
    text = path.read_text()
    offenders = []
    for lineno, line in enumerate(text.splitlines(), start=1):
        m = re.match(r'^\s*"([^"]+)"\s*,?\s*$', line)
        if not m:
            continue
        spec = m.group(1)
        name = re.split(r"[<>=~!\[ ]", spec, maxsplit=1)[0]
        if _unbounded(name, spec[len(name):]):
            offenders.append(f"{path.relative_to(ROOT)}:{lineno}: {spec} — {COUPLED[name.lower()]}")
    assert not offenders, "unbounded dependency known to break on a major bump:\n" + "\n".join(offenders)


def test_the_guard_actually_catches_a_bare_requirement() -> None:
    # Without this the two tests above would keep passing if `_unbounded` were
    # ever broken to return False unconditionally.
    assert _unbounded("mcp", "")
    assert _unbounded("protobuf", "  # a comment is not a bound")
    # The case this file exists for: a lower bound that still admits 2.x.
    assert _unbounded("mcp", ">=1.0")
    assert _unbounded("fastmcp", ">=3")
    assert not _unbounded("mcp", ">=1.27,<2")
    assert not _unbounded("mcp", "==1.29.0")
    assert not _unbounded("fastmcp", "~=3.4")
    assert not _unbounded("scipy", "")  # not coupled, floating is fine
