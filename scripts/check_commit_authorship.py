#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Require human commit responsibility and reject bot or AI attribution."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


ZERO_SHA_RE = re.compile(r"^0+$")
TRAILER_RE = re.compile(r"^([A-Za-z][A-Za-z0-9-]*):\s*(.*?)\s*$")
IDENTITY_RE = re.compile(r"^(.*?)\s*<([^<>]+)>\s*$")

# Coding-agent identities are distinct from repository automation identities.
# Both are prohibited as authors or committers. GitHub's web-flow merge
# identity remains allowed because it records the hosting service that applied
# a human-reviewed merge; it is not a bot author or contributor identity.
AI_NAME_RE = re.compile(
    r"(?:^|[\s._+-])(?:"
    r"chatgpt|openai[\s._+-]*codex|codex|github[\s._+-]*copilot|copilot|"
    r"claude(?:[\s._+-]+code)?|anthropic[\s._+-]*claude|"
    r"gemini(?:[\s._+-]+cli)?|cursor(?:[\s._+-]+agent)?|"
    r"opencode|trae(?:[\s._+-]+(?:ai|agent))?|"
    r"windsurf(?:[\s._+-]+cascade)?|cline|"
    r"roo[\s._+-]*code|kilo[\s._+-]*code|qwen[\s._+-]*code|"
    r"amazon[\s._+-]*q[\s._+-]*developer|augment[\s._+-]*code|"
    r"qodo(?:[\s._+-]*merge)?|replit[\s._+-]*agent|"
    r"gitlab[\s._+-]*duo|sourcegraph[\s._+-]*cody|"
    r"jetbrains[\s._+-]*junie|google[\s._+-]*jules|"
    r"factory[\s._+-]*droid|warp[\s._+-]*agent|"
    r"codeium|codewhisperer|tabnine|"
    r"ai[\s._+-]*coding[\s._+-]*(?:agent|assistant)|"
    r"coding[\s._+-]*(?:agent|assistant)|"
    r"aider|devin|openhands|swe[\s._+-]*agent"
    r")(?:$|[\s._+-])",
    re.IGNORECASE,
)
AI_EMAIL_RE = re.compile(
    r"(?:"
    r"noreply@anthropic\.com|"
    r"(?:copilot|codex|chatgpt|claude|gemini|cursor|opencode|trae|"
    r"windsurf|cline|roo-code|kilo-code|qwen-code|augment-code|qodo|"
    r"replit-agent|gitlab-duo|aider|devin|openhands|swe-agent)"
    r"(?:[^@]*)@"
    r")",
    re.IGNORECASE,
)
AUTOMATION_NAME_RE = re.compile(
    r"(?:\[bot\]$|(?:^|[\s._+-])(?:"
    r"github[\s._+-]*actions|allcontributors|dependabot|renovate"
    r")(?:$|[\s._+-]))",
    re.IGNORECASE,
)
AUTOMATION_EMAIL_RE = re.compile(
    r"(?:"
    r"(?:github-actions|allcontributors|dependabot|renovate)(?:[^@]*)@|"
    r"bot@"
    r")",
    re.IGNORECASE,
)
MAX_FULL_HISTORY_COMMITS = 10_000

AUTHORSHIP_TRAILERS = {
    "co-authored-by",
    "co-developed-by",
    "signed-off-by",
    "reviewed-by",
    "tested-by",
    "acked-by",
    "suggested-by",
}


@dataclass(frozen=True)
class CommitRecord:
    sha: str
    author_name: str
    author_email: str
    committer_name: str
    committer_email: str
    message: str


def is_ai_identity(name: str, email: str) -> bool:
    """Return whether a Git identity represents a known AI coding agent."""

    return bool(AI_NAME_RE.search(name.strip()) or AI_EMAIL_RE.search(email.strip()))


def is_automation_identity(name: str, email: str) -> bool:
    """Return whether a Git identity is a repository automation account."""

    return bool(
        AUTOMATION_NAME_RE.search(name.strip())
        or AUTOMATION_EMAIL_RE.search(email.strip())
    )


def is_prohibited_identity(name: str, email: str) -> bool:
    """Return whether an identity cannot own Robonix commit responsibility."""

    return is_ai_identity(name, email) or is_automation_identity(name, email)


def validate_assisted_by(value: str) -> str | None:
    """Validate Robonix ``Assisted-by: AGENT:MODEL [TOOLS...]`` syntax."""

    if "<" in value or ">" in value:
        return "Assisted-by identifies a tool, so it must not use a person/email form"

    agent_model, _separator, _tools = value.partition(" ")
    agent, colon, model = agent_model.partition(":")
    if not colon or not agent.strip() or not model.strip():
        return "expected Assisted-by: AGENT_NAME:MODEL_VERSION [TOOL ...]"
    return None


def inspect_record(record: CommitRecord) -> list[str]:
    """Return policy violations for one parsed commit."""

    violations: list[str] = []
    if is_ai_identity(record.author_name, record.author_email):
        violations.append(
            f"Git author is an AI identity: {record.author_name} <{record.author_email}>"
        )
    if is_ai_identity(record.committer_name, record.committer_email):
        violations.append(
            "Git committer is an AI identity: "
            f"{record.committer_name} <{record.committer_email}>"
        )
    if is_automation_identity(record.author_name, record.author_email):
        violations.append(
            "Git author is an automation identity: "
            f"{record.author_name} <{record.author_email}>"
        )
    if is_automation_identity(record.committer_name, record.committer_email):
        violations.append(
            "Git committer is an automation identity: "
            f"{record.committer_name} <{record.committer_email}>"
        )

    for line_number, line in enumerate(record.message.splitlines(), 1):
        match = TRAILER_RE.match(line)
        if not match:
            continue
        key, value = match.group(1).lower(), match.group(2)
        if key == "assisted-by":
            error = validate_assisted_by(value)
            if error:
                violations.append(
                    "invalid Assisted-by trailer on message line "
                    f"{line_number}: {error}"
                )
            continue
        if key not in AUTHORSHIP_TRAILERS:
            continue
        identity = IDENTITY_RE.match(value)
        if identity and is_prohibited_identity(identity.group(1), identity.group(2)):
            violations.append(
                f"AI or automation identity used in {match.group(1)} trailer on "
                f"message line {line_number}; use Assisted-by instead for AI tools "
                "and keep responsibility with a human"
            )

    return violations


def git(*args: str) -> str:
    """Run Git from the repository root and return UTF-8 output."""

    result = subprocess.run(
        ["git", *args],
        cwd=Path(__file__).resolve().parents[1],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return result.stdout


def commit_exists(ref: str) -> bool:
    """Return whether ``ref`` resolves to a commit in the checkout."""

    try:
        git("cat-file", "-e", f"{ref}^{{commit}}")
    except subprocess.CalledProcessError:
        return False
    return True


def full_history_commits(head: str, reason: str) -> list[str]:
    """Audit reachable history when an event base is unavailable after a rewrite."""

    commits = git("rev-list", "--reverse", head).splitlines()
    if len(commits) > MAX_FULL_HISTORY_COMMITS:
        raise ValueError(
            f"full-history fallback contains {len(commits)} commits, exceeding "
            f"the safety limit of {MAX_FULL_HISTORY_COMMITS}"
        )
    print(
        "::warning title=Commit authorship full-history fallback::"
        f"{reason}; auditing all {len(commits)} commits reachable from {head}"
    )
    return commits


def introduced_commits(base: str, head: str) -> list[str]:
    """List commits introduced between event base and head."""

    if not head:
        raise ValueError("head commit is empty")
    if not commit_exists(head):
        raise ValueError(f"head commit is unavailable: {head}")
    if not base or ZERO_SHA_RE.fullmatch(base):
        return full_history_commits(head, "event base is empty")
    if not commit_exists(base):
        return full_history_commits(
            head,
            f"event base {base} is unavailable (for example after a force-push)",
        )
    return git("rev-list", "--reverse", f"{base}..{head}").splitlines()


def load_record(sha: str) -> CommitRecord:
    """Load one commit using NUL separators so message text is lossless."""

    fields = git("show", "-s", "--format=%H%x00%an%x00%ae%x00%cn%x00%ce%x00%B", sha).split(
        "\0", 5
    )
    if len(fields) != 6:
        raise ValueError(f"could not parse commit {sha}")
    return CommitRecord(*fields)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default="", help="exclusive base commit")
    parser.add_argument("--head", default="HEAD", help="inclusive head commit")
    args = parser.parse_args()

    try:
        commits = introduced_commits(args.base.strip(), args.head.strip())
    except (ValueError, subprocess.CalledProcessError) as error:
        print(f"error: cannot determine commit range: {error}", file=sys.stderr)
        return 2

    if not commits:
        print("Commit authorship check: no introduced commits.")
        return 0

    failed = False
    for sha in commits:
        try:
            record = load_record(sha)
        except (ValueError, subprocess.CalledProcessError) as error:
            print(f"error: cannot read commit {sha}: {error}", file=sys.stderr)
            return 2
        violations = inspect_record(record)
        if not violations:
            continue
        failed = True
        print(f"::error title=Commit authorship policy::{record.sha}")
        print(f"commit {record.sha}")
        for violation in violations:
            print(f"  - {violation}")

    if failed:
        print()
        print("Robonix commits must retain a human author and accountable submitter.")
        print("Disclose AI assistance only with: Assisted-by: AGENT_NAME:MODEL_VERSION [TOOL ...]")
        print("See CONTRIBUTING.md for the policy and examples.")
        return 1

    print(f"Commit authorship check: {len(commits)} commit(s) passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
