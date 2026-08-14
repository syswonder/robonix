# SPDX-License-Identifier: MulanPSL-2.0

import importlib.util
import sys
import unittest
from pathlib import Path
from unittest import mock


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

    def test_explicit_coding_agent_identities_are_rejected(self):
        agent_identities = (
            "OpenCode",
            "Trae AI",
            "Windsurf Cascade",
            "Cline",
            "Roo Code",
            "Kilo Code",
            "Qwen Code",
            "Amazon Q Developer",
            "Augment Code",
            "Qodo Merge",
            "Replit Agent",
            "GitLab Duo",
            "Sourcegraph Cody",
            "JetBrains Junie",
            "Google Jules",
            "Factory Droid",
            "Warp Agent",
            "AI Coding Agent",
        )
        for identity in agent_identities:
            with self.subTest(identity=identity):
                violations = MODULE.inspect_record(record(author_name=identity))
                self.assertTrue(
                    any("Git author is an AI identity" in item for item in violations)
                )

    def test_github_actions_author_is_rejected(self):
        violations = MODULE.inspect_record(
            record(
                author_name="github-actions[bot]",
                author_email="41898282+github-actions[bot]@users.noreply.github.com",
            )
        )
        self.assertTrue(any("automation identity" in item for item in violations))

    def test_allcontributors_committer_is_rejected(self):
        violations = MODULE.inspect_record(
            record(
                committer_name="allcontributors[bot]",
                committer_email="allcontributors[bot]@users.noreply.github.com",
            )
        )
        self.assertTrue(any("automation identity" in item for item in violations))

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

    def test_missing_push_base_audits_full_reachable_history(self):
        with (
            mock.patch.object(MODULE, "commit_exists", side_effect=[True, False]),
            mock.patch.object(MODULE, "git", return_value="a\nb\n") as run_git,
        ):
            self.assertEqual(MODULE.introduced_commits("old", "new"), ["a", "b"])
        run_git.assert_called_once_with("rev-list", "--reverse", "new")

    def test_zero_push_base_audits_full_reachable_history(self):
        with (
            mock.patch.object(MODULE, "commit_exists", return_value=True),
            mock.patch.object(MODULE, "git", return_value="a\nb\n") as run_git,
        ):
            self.assertEqual(MODULE.introduced_commits("0" * 40, "new"), ["a", "b"])
        run_git.assert_called_once_with("rev-list", "--reverse", "new")

    def test_regular_push_audits_only_introduced_range(self):
        with (
            mock.patch.object(MODULE, "commit_exists", side_effect=[True, True]),
            mock.patch.object(MODULE, "git", return_value="b\nc\n") as run_git,
        ):
            self.assertEqual(MODULE.introduced_commits("a", "c"), ["b", "c"])
        run_git.assert_called_once_with("rev-list", "--reverse", "a..c")


if __name__ == "__main__":
    unittest.main()
