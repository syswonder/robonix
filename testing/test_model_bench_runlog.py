# SPDX-License-Identifier: MulanPSL-2.0
"""Regression tests for benchmark console and component log capture."""

import contextlib
import io
import sys
import tempfile
import unittest
from pathlib import Path

from testing.model_bench import runlog


class RunLogTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.output = self.root / "capture"
        self.logs = self.root / "logs"
        self.logs.mkdir()

    def test_console_output_is_unchanged_and_empty_logs_exist(self):
        console_out, console_err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(console_out), contextlib.redirect_stderr(console_err):
            with runlog.capture(self.output, []):
                print("hello", end="")
                print("problem", file=sys.stderr)
        self.assertEqual(console_out.getvalue(), "hello")
        self.assertEqual(console_err.getvalue(), "problem\n")
        self.assertEqual((self.output / "bench.stdout.log").read_text(), "hello")
        self.assertEqual((self.output / "bench.stderr.log").read_text(), "problem\n")
        with runlog.capture(self.output, []):
            pass
        self.assertEqual((self.output / "bench.stdout.log").read_bytes(), b"")
        self.assertEqual((self.output / "bench.stderr.log").read_bytes(), b"")

    def test_exception_is_reraised_with_traceback_captured(self):
        exception = RuntimeError("failed benchmark")
        original_out, original_err = sys.stdout, sys.stderr
        with self.assertRaises(RuntimeError) as raised:
            with runlog.capture(self.output, [self.logs]):
                (self.logs / "pilot.log").write_text("failure\n")
                raise exception
        self.assertIs(raised.exception, exception)
        self.assertIs(sys.stdout, original_out)
        self.assertIs(sys.stderr, original_err)
        stderr = (self.output / "bench.stderr.log").read_text()
        self.assertIn("Traceback (most recent call last):", stderr)
        self.assertIn("RuntimeError: failed benchmark", stderr)
        self.assertEqual((self.output / "components/0/pilot.log").read_text(), "failure\n")

    def test_existing_suffix_new_files_and_rotation(self):
        existing = self.logs / "atlas.log"
        existing.write_text("older run\n")
        unchanged = self.logs / "unchanged.log"
        unchanged.write_text("older run\n")
        truncated = self.logs / "truncate.log"
        truncated.write_text("long old contents\n")
        later = self.root / "later"
        with runlog.capture(self.output, [self.logs, later]):
            with existing.open("a") as stream:
                stream.write("first new line\n")
            existing.rename(self.logs / "atlas.log.1")
            existing.write_text("second new line\n")
            truncated.write_text("short\n")
            (self.logs / "nested").mkdir()
            (self.logs / "nested/pilot.log").write_text("new pilot\n")
            later.mkdir()
            (later / "soma.log").write_text("new soma\n")
        expected = {"0/atlas.log": "second new line\n", "0/atlas.log.1": "first new line\n",
                    "0/unchanged.log": "", "0/truncate.log": "short\n",
                    "0/nested/pilot.log": "new pilot\n", "1/soma.log": "new soma\n"}
        actual = {path.relative_to(self.output / "components").as_posix(): path.read_text()
                  for path in (self.output / "components").rglob("*") if path.is_file()}
        self.assertEqual(actual, expected)

    def test_symlink_roots_and_files_are_rejected_and_directory_links_skipped(self):
        link = self.root / "linked"
        link.symlink_to(self.logs, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "symlink"):
            with runlog.capture(self.output, [link]):
                self.fail("symlink root accepted")
        other = self.root / "other"
        other.mkdir()
        (other / "private.log").write_text("outside\n")
        (self.logs / "directory-link").symlink_to(other, target_is_directory=True)
        with runlog.capture(self.output, [self.logs]):
            pass
        self.assertFalse((self.output / "components/0/directory-link").exists())
        (self.logs / "file-link").symlink_to(other / "private.log")
        with self.assertRaisesRegex(ValueError, "symlink"):
            with runlog.capture(self.output, [self.logs]):
                self.fail("symlink file accepted")

    def test_copytruncate_and_regrow_copies_all_new_contents(self):
        log = self.logs / "atlas.log"
        log.write_text("old\n")
        inode = log.stat().st_ino
        with runlog.capture(self.output, [self.logs]):
            log.write_text("new-first\nnew-second\n")
            self.assertEqual(log.stat().st_ino, inode)
        self.assertEqual((self.output / "components/0/atlas.log").read_text(),
                         "new-first\nnew-second\n")

    def test_existing_source_directory_cannot_disappear(self):
        with self.assertRaisesRegex(FileNotFoundError, "disappeared"):
            with runlog.capture(self.output, [self.logs]):
                self.logs.rmdir()


if __name__ == "__main__":
    unittest.main()
