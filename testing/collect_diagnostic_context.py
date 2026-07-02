# SPDX-License-Identifier: MulanPSL-2.0
"""Collect bounded CI diagnostic context for an LLM-assisted report note.

The collector is intentionally read-only. It sends PR metadata, PR diff hunks,
scenario summaries, and selected CI artifacts/logs; it never walks arbitrary
source files from the repository.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

ALLOWED_LOG_SUFFIXES = {
    ".json",
    ".jsonl",
    ".log",
    ".md",
    ".out",
    ".stderr",
    ".stdout",
    ".txt",
    ".yaml",
    ".yml",
}
ALLOWED_LOG_NAMES = {"Dockerfile"}
SECRET_PATTERNS = [
    re.compile(r"(?i)(authorization:\s*bearer\s+)[^\s]+"),
    re.compile(r"(?i)((?:github_)?token\s*[:=]\s*)[^\s]+"),
    re.compile(r"(?i)((?:api[_-]?key|password|secret)\s*[:=]\s*)[^\s]+"),
    re.compile(r"sk-[A-Za-z0-9_-]{16,}"),
]
IMPORTANT_LOG_RE = re.compile(
    r"(?i)(\[FAIL\]|\[Boot failed\]|error:|exception|traceback|timeout|failed|missing|required providers|RESULT:)"
)


def _read_json(path: Path | None) -> Any:
    if not path or not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def _redact(text: str) -> str:
    for pattern in SECRET_PATTERNS:
        if pattern.pattern.startswith("sk-"):
            text = pattern.sub("sk-[REDACTED]", text)
        else:
            text = pattern.sub(r"\1[REDACTED]", text)
    return text


def _run_git(args: list[str], cwd: Path, limit: int) -> str:
    try:
        proc = subprocess.run(
            ["git", *args],
            cwd=cwd,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=30,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return f"<git command failed: {exc}>"
    out = _redact(proc.stdout)
    if len(out) > limit:
        return out[:limit] + f"\n<truncated after {limit} chars>\n"
    return out


def _github_get_json(url: str, token: str | None) -> Any:
    headers = {
        "Accept": "application/vnd.github+json",
        "X-GitHub-Api-Version": "2022-11-28",
        "User-Agent": "robonix-ci-diagnostics",
    }
    if token:
        headers["Authorization"] = f"Bearer {token}"
    req = urllib.request.Request(url, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except (OSError, urllib.error.HTTPError, json.JSONDecodeError) as exc:
        return {"error": str(exc), "url": url}


def _collect_pr_files(metadata: dict[str, str], token: str | None, max_files: int, max_patch_chars: int) -> list[dict[str, Any]]:
    repo = metadata.get("repository") or os.environ.get("GITHUB_REPOSITORY", "")
    pr_number = metadata.get("pr_number", "")
    server = os.environ.get("GITHUB_API_URL", "https://api.github.com")
    if not repo or not pr_number:
        return []
    files: list[dict[str, Any]] = []
    page = 1
    while len(files) < max_files:
        url = f"{server}/repos/{repo}/pulls/{pr_number}/files?per_page=100&page={page}"
        data = _github_get_json(url, token)
        if not isinstance(data, list):
            files.append({"error": data})
            break
        if not data:
            break
        for item in data:
            patch = str(item.get("patch", ""))
            if len(patch) > max_patch_chars:
                patch = patch[:max_patch_chars] + f"\n<truncated after {max_patch_chars} chars>\n"
            files.append(
                {
                    "filename": item.get("filename", ""),
                    "status": item.get("status", ""),
                    "additions": item.get("additions", 0),
                    "deletions": item.get("deletions", 0),
                    "changes": item.get("changes", 0),
                    "patch": _redact(patch),
                }
            )
            if len(files) >= max_files:
                break
        page += 1
    return files


def _is_allowed(path: Path) -> bool:
    if path.is_symlink() or not path.is_file():
        return False
    return path.name in ALLOWED_LOG_NAMES or path.suffix.lower() in ALLOWED_LOG_SUFFIXES


def _read_text(path: Path, max_bytes: int) -> tuple[str, bool, int]:
    size = path.stat().st_size
    data = path.read_bytes()[:max_bytes]
    text = _redact(data.decode("utf-8", errors="replace"))
    return text, size > max_bytes, size


def _rank_log(path: Path) -> tuple[int, str]:
    name = path.name.lower()
    full = path.as_posix().lower()
    if name in {"rbnx-boot.log", "summary.json"}:
        return (0, full)
    if "soma" in name or "error" in name or "scenario" in full:
        return (1, full)
    if "provider" in full or "logs" in full:
        return (2, full)
    if "sim" in full or "webots" in full:
        return (3, full)
    return (4, full)


def _collect_logs(roots: list[Path], max_log_bytes: int, max_total_bytes: int) -> list[dict[str, Any]]:
    candidates: list[Path] = []
    seen: set[Path] = set()
    for root in roots:
        if not root.exists():
            continue
        if root.is_file() and _is_allowed(root):
            paths = [root]
        elif root.is_dir():
            paths = [p for p in root.rglob("*") if _is_allowed(p)]
        else:
            paths = []
        for path in paths:
            try:
                resolved = path.resolve(strict=True)
            except OSError:
                continue
            if resolved in seen:
                continue
            seen.add(resolved)
            candidates.append(path)
    candidates.sort(key=_rank_log)

    entries: list[dict[str, Any]] = []
    total = 0
    for path in candidates:
        if total >= max_total_bytes:
            break
        cap = min(max_log_bytes, max_total_bytes - total)
        try:
            text, truncated, size = _read_text(path, cap)
        except OSError as exc:
            text, truncated, size = f"<failed to read log: {exc}>", False, 0
        important = []
        for idx, line in enumerate(text.splitlines(), 1):
            if IMPORTANT_LOG_RE.search(line):
                important.append({"line": idx, "text": line[:1000]})
                if len(important) >= 80:
                    break
        entries.append(
            {
                "path": path.as_posix(),
                "name": path.name,
                "size": size,
                "truncated": truncated,
                "important_lines": important,
                "content": text,
            }
        )
        total += len(text.encode("utf-8", errors="replace"))
    return entries


def main() -> int:
    ap = argparse.ArgumentParser(description="Collect bounded Robonix CI diagnostic context")
    ap.add_argument("--artifact-root", action="append", type=Path, default=[])
    ap.add_argument("--summary-json", type=Path)
    ap.add_argument("--metadata-json", action="append", type=Path, default=[])
    ap.add_argument("--metadata", action="append", default=[])
    ap.add_argument("--repo", type=Path, default=Path("."))
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--max-files", type=int, default=80)
    ap.add_argument("--max-patch-chars", type=int, default=20000)
    ap.add_argument("--max-git-diff-chars", type=int, default=80000)
    ap.add_argument("--max-log-bytes", type=int, default=120000)
    ap.add_argument("--max-total-log-bytes", type=int, default=900000)
    args = ap.parse_args()

    metadata: dict[str, str] = {}
    for path in args.metadata_json:
        data = _read_json(path)
        if isinstance(data, dict):
            metadata.update({str(k): str(v) for k, v in data.items()})
    for item in args.metadata:
        if "=" in item:
            key, value = item.split("=", 1)
            if key.strip():
                metadata[key.strip()] = value.strip()

    summary = _read_json(args.summary_json)
    token = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
    pr_files = _collect_pr_files(metadata, token, args.max_files, args.max_patch_chars)

    tested_commit = metadata.get("tested_commit") or metadata.get("sha")
    git_diff = ""
    if not pr_files and tested_commit:
        git_diff = _run_git(["show", "--stat", "--patch", "--find-renames", tested_commit], args.repo, args.max_git_diff_chars)

    roots = args.artifact_root or [Path(".")]
    logs = _collect_logs(roots, args.max_log_bytes, args.max_total_log_bytes)

    context = {
        "schema_version": 1,
        "purpose": "LLM-assisted CI diagnostic note for Robonix Webots integration tests",
        "rules": [
            "Use only this context; do not infer from unavailable repository files.",
            "Do not decide pass/fail; status is determined by CI scripts.",
            "Mention uncertainty when evidence is incomplete.",
        ],
        "metadata": metadata,
        "summary": summary,
        "pr_files": pr_files,
        "git_diff": git_diff,
        "logs": logs,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(context, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
