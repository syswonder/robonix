# SPDX-License-Identifier: MulanPSL-2.0
"""Ensure identifier-like IDL comments name registered Robonix contracts."""

from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_ID = re.compile(r'^id\s*=\s*"(robonix/[^"]+)"', re.MULTILINE)
COMMENT_ID = re.compile(r"robonix/[A-Za-z0-9_./-]+")


class ContractIdlCommentTest(unittest.TestCase):
    def test_identifier_like_comments_reference_registered_contracts(self):
        registered = {
            match.group(1)
            for path in (ROOT / "capabilities").rglob("*.v1.toml")
            for match in CONTRACT_ID.finditer(path.read_text())
        }
        stale = []
        for path in (ROOT / "capabilities" / "lib").rglob("*"):
            if path.suffix not in {".msg", ".srv", ".action"}:
                continue
            for line_number, line in enumerate(path.read_text().splitlines(), 1):
                comment = line.partition("#")[2]
                for identifier in COMMENT_ID.findall(comment):
                    identifier = identifier.rstrip(".,;:)`")
                    if identifier not in registered:
                        stale.append(
                            f"{path.relative_to(ROOT)}:{line_number}: {identifier}"
                        )
        self.assertEqual(stale, [], "stale contract identifiers:\n" + "\n".join(stale))


if __name__ == "__main__":
    unittest.main()
