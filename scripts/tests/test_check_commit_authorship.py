# SPDX-License-Identifier: MulanPSL-2.0

import importlib.util
import sys
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "check_commit_authorship.py"
SPEC = importlib.util.spec_from_file_location("check_commit_authorship", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def record(
    *,
    author_name="Human Developer",
    author_email="human@example.com",
    committer_name="GitHub",
    committer_email="noreply@github.com",
    message="fix: keep responsibility human",
):
    """Build a deterministic commit record for checker unit tests."""

    return MODULE.CommitRecord(
        "a" * 40,
        author_name,
        author_email,
        committer_name,
        committer_email,
        message,
    )


class CommitAuthorshipTests(unittest.TestCase):
    def test_human_author_and_github_committer_are_allowed(self):
        self.assertEqual(MODULE.inspect_record(record()), [])

    def test_ai_git_author_is_rejected(self):
        violations = MODULE.inspect_record(
            record(author_name="OpenAI Codex", author_email="human@example.com")
        )
        self.assertTrue(any("Git author is an AI identity" in item for item in violations))

    def test_ai_git_committer_is_rejected(self):
        violations = MODULE.inspect_record(
            record(committer_name="Claude Code", committer_email="noreply@anthropic.com")
        )
        self.assertTrue(any("Git committer is an AI identity" in item for item in violations))

    def test_ai_coauthor_trailer_is_rejected(self):
        violations = MODULE.inspect_record(
            record(
                message=(
                    "fix: keep responsibility human\n\n"
                    "Co-authored-by: Claude Fable 5 <noreply@anthropic.com>"
                )
            )
        )
        self.assertTrue(any("use Assisted-by instead" in item for item in violations))

    def test_robonix_assisted_by_is_allowed(self):
        violations = MODULE.inspect_record(
            record(message="fix: disclose assistance\n\nAssisted-by: Codex:gpt-5.6 clang-tidy")
        )
        self.assertEqual(violations, [])

    def test_email_style_assisted_by_is_rejected(self):
        violations = MODULE.inspect_record(
            record(message="fix: disclose assistance\n\nAssisted-by: Codex <bot@example.com>")
        )
        self.assertTrue(any("must not use a person/email form" in item for item in violations))

    def test_unrelated_message_text_is_not_treated_as_attribution(self):
        violations = MODULE.inspect_record(
            record(message="docs: explain how Codex assistance is disclosed")
        )
        self.assertEqual(violations, [])


if __name__ == "__main__":
    unittest.main()
