# SPDX-License-Identifier: MulanPSL-2.0
"""Submission archives keep results useful without publishing local data."""

from __future__ import annotations

import gzip
import json
import os
import socket
import subprocess
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch

from testing.model_bench import bundle


class BundleTest(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tempdir.cleanup)
        self.root = Path(self.tempdir.name).resolve()

    def result(self, value=None, name="private-host.json"):
        path = self.root / name
        path.write_text(json.dumps(value if value is not None else {
            "meta": {"scoring_version": 2}, "results": [{"id": "old", "final": "done"}]
        }), encoding="utf-8")
        return path

    def test_all_archive_contents_are_scrubbed_and_results_remain_valid(self):
        secret = 'planted-quote-"-and-\\-credential-\u79d8\u5bc6'
        private = [secret, "10.23.42.8", "192.168.22.9", "127.0.0.1", "fd12:3456::7",
                   "::1", "100.100.5.9", "robot.internal", "robot.local", "robot.lan",
                   "robot.tailscale", "robot.example.ts.net", socket.gethostname(),
                   "/Users/bench-person/project", "/home/bench-person/run", "sk-examplecredential123"]
        text = "\n".join(private + ["ROBONIX_TEST_SECRET=private-route", "visible answer"])
        result = self.result({"meta": {"route": text}, "results": [{"final": "visible answer",
            "rounds": [{"reply": text, "escaped": json.dumps({"value": secret}),
                        "double_escaped": json.dumps(json.dumps({"value": secret}))}]}]})
        log = self.root / "pilot.log"
        log.write_text(text, encoding="utf-8")
        output = self.root / "submission.zip"
        with patch.dict(os.environ, {"BENCH_SECRET_VALUE": secret}):
            bundle.write_bundle(output, [result], {"pilot.log": log})
        with zipfile.ZipFile(output) as archive:
            self.assertEqual(archive.namelist(), ["logs/pilot.log", "result-001.json"])
            all_text = "\n".join(archive.read(name).decode() for name in archive.namelist())
            for value in private + ["private-route"]:
                with self.subTest(value=value):
                    self.assertNotIn(value, all_text)
                    self.assertNotIn(value.encode(), output.read_bytes())
            saved = json.loads(archive.read("result-001.json"))
            self.assertEqual(saved["results"][0]["final"], "visible answer")
            self.assertEqual(len(saved["results"][0]["rounds"]), 1)

    def test_credentials_patterns_and_sensitive_json_fields(self):
        value = {"api_key": "arbitrary-value", "Authorization": "anything", "prompt_tokens": 120,
                 "completion_tokens": 7, "nested": [True, None, 3, {
                     "reply": '\n'.join(['Authorization: Bearer header-value', 'Bearer loose-value',
                         'api_key="json-value"', "--api-key cli-value", "TOKEN=assignment-value",
                         "Authorization: Token alternate-scheme-value",
                         '{"access_token": "embedded-value"}', r'{\"api_key\": \"escaped-value\"}',
                         "password: hidden-value"]) }]}
        saved = bundle.scrub(value)
        text = json.dumps(saved)
        for value in ("arbitrary-value", "anything", "header-value", "loose-value", "json-value",
                      "cli-value", "assignment-value", "embedded-value", "escaped-value", "hidden-value",
                      "alternate-scheme-value"):
            self.assertNotIn(value, text)
        self.assertEqual(saved["prompt_tokens"], 120)
        self.assertEqual(saved["completion_tokens"], 7)
        self.assertEqual(saved["nested"][:3], [True, None, 3])

    def test_component_words_and_filenames_survive_unchanged(self):
        reply = "first\nbench proxy.py forwarded reply\nproxies completed\nlast"
        value = {"results": [{"final": "answer survives", "rounds": [{
            "reply": reply, "usage": {"completion_tokens": 5}}]}],
            "component": "proxy", "files": ["proxy.py", "proxies.log"],
            "encoded": json.dumps({"reply": reply})}
        self.assertEqual(bundle.scrub(value), value)
        self.assertEqual(bundle.scrub_text(reply), reply)
        log = self.root / "proxy.py"
        log.write_text(reply)
        output = self.root / "out.zip"
        bundle.write_bundle(output, [self.result(value)], {"proxy.py": log})
        with zipfile.ZipFile(output) as archive:
            self.assertEqual(archive.read("logs/proxy.py").decode(), reply)
            self.assertEqual(json.loads(archive.read("result-001.json")), value)

    def test_environment_assignments_preserve_context_and_remove_any_value(self):
        text = ('before ROBONIX_TEST_SECRET=private-setting after\n'
                'prefix BENCH_MODE: "harmless setting" suffix\n'
                "prefix BENCH_COUNT='987654' suffix\n"
                'prefix BENCH_EMPTY="" suffix\n'
                'prefix ROBONIX_TEST_SETTING: two word value\n'
                'prefix BENCH_VALUE=one\\ two suffix\n'
                'BENCH_EMPTY=\n'
                'bench proxy.py forwarded reply\n')
        expected = ('before ROBONIX_TEST_SECRET=[REDACTED] after\n'
                    'prefix BENCH_MODE: "[REDACTED]" suffix\n'
                    "prefix BENCH_COUNT='[REDACTED]' suffix\n"
                    'prefix BENCH_EMPTY="[REDACTED]" suffix\n'
                    'prefix ROBONIX_TEST_SETTING: [REDACTED]\n'
                    'prefix BENCH_VALUE=[REDACTED] suffix\n'
                    'BENCH_EMPTY=\n'
                    'bench proxy.py forwarded reply\n')
        value = {"reply": text, "BENCH_MODE": "harmless setting", "BENCH_COUNT": 987654,
                 "BENCH_NESTED": {"value": "private-setting"}, "number": 1,
                 "encoded": r'{\"BENCH_MODE\": \"harmless setting\"}'}
        with patch.dict(os.environ, {"SHLVL": "1", "BENCH_MODE": "harmless setting"}):
            saved = bundle.scrub(value)
            self.assertEqual(bundle.scrub_text(text), expected)
            self.assertEqual(bundle.scrub("reply 1 remains harmless setting"), "reply 1 remains harmless setting")
        self.assertEqual(saved["reply"], expected)
        for key in ("BENCH_MODE", "BENCH_COUNT", "BENCH_NESTED"):
            self.assertEqual(saved[key], bundle.REDACTED)
        self.assertEqual(saved["number"], 1)
        self.assertNotIn("harmless setting", saved["encoded"])

    def test_escaped_assignment_quotes_do_not_expose_the_rest_of_the_value(self):
        original = {"BENCH_VALUE": 'first "private-tail" last', "reply": "proxy.py forwarded reply"}
        encoded = json.dumps(original)
        for depth in range(1, 4):
            encoded = json.dumps(encoded)
            text = "DEBUG payload=" + encoded
            expected = original | {"BENCH_VALUE": bundle.REDACTED}
            for _ in range(depth + 1):
                expected = json.dumps(expected)
            expected = "DEBUG payload=" + expected
            with self.subTest(depth=depth):
                self.assertEqual(bundle.scrub_text(text), expected)
                self.assertEqual(bundle.scrub({"reply": text}), {"reply": expected})

    def test_jsonl_and_multiline_json_logs_preserve_interactions(self):
        records = [{"reply": "answer one\nROBONIX_TEST_SECRET=private-route\nanswer two", "request": {"id": 1}},
                   {"reply": "answer three", "completion_tokens": 4}]
        jsonl = self.root / "requests.jsonl"
        jsonl.write_text("".join(json.dumps(record) + "\n" for record in records))
        json_log = self.root / "case.json"
        json_log.write_text(json.dumps(records, indent=2))
        one = self.root / "single.jsonl"
        one.write_text(json.dumps(records[0]) + "\n")
        output = self.root / "out.zip"
        bundle.write_bundle(output, [self.result()], {"requests.jsonl": jsonl, "case.json": json_log,
                                                     "single.jsonl": one})
        with zipfile.ZipFile(output) as archive:
            saved = [json.loads(line) for line in archive.read("logs/requests.jsonl").splitlines()]
            self.assertEqual(saved[0]["reply"], "answer one\nROBONIX_TEST_SECRET=[REDACTED]\nanswer two")
            self.assertEqual(saved[1], records[1])
            self.assertEqual(saved[0]["request"], {"id": 1})
            self.assertEqual(json.loads(archive.read("logs/case.json")), saved)
            single = archive.read("logs/single.jsonl").splitlines()
            self.assertEqual(len(single), 1)
            self.assertEqual(json.loads(single[0]), saved[0])
            self.assertNotIn(b"private-route", output.read_bytes())

    def test_argv_and_unexpected_credential_names_are_scrubbed(self):
        argv = ["bench.py", "--api-key", "unknown-key", "--token", "unknown-token",
                "--client-secret", "unknown-secret", "--component-logs", "/path/withproxy",
                "--endpoint", "http://unknown-route", "--model", "public/model"]
        value = {"argv": argv, "UNEXPECTED_KEY": "json-key", "SECRETfoo": "json-secret",
                 "reply": "UNEXPECTED_KEY=assignment-key\nSECRETfoo=assignment-secret\nvisible env-secret",
                 "completion_tokens": 4}
        with patch.dict(os.environ, {"SECRETfoo": "env-secret"}):
            saved = bundle.scrub(value)
        self.assertEqual(saved["argv"], ["bench.py", "--api-key", bundle.REDACTED,
            "--token", bundle.REDACTED, "--client-secret", bundle.REDACTED,
            "--component-logs", "/path/withproxy", "--endpoint", bundle.REDACTED, "--model", "public/model"])
        self.assertEqual(saved["completion_tokens"], 4)
        text = json.dumps(saved)
        for secret in ("unknown-key", "unknown-token", "unknown-secret", "unknown-route",
                       "json-key", "json-secret", "assignment-key", "assignment-secret", "env-secret"):
            self.assertNotIn(secret, text)

    def test_credential_flag_does_not_consume_the_next_option(self):
        argv = ["bench.py", "--token", "--model", "public/model"]
        self.assertEqual(bundle.scrub(argv), argv)
        self.assertEqual(bundle.scrub(["--token", None, "proxy.py"]), ["--token", None, "proxy.py"])

    def test_legacy_results_and_deterministic_archive_metadata(self):
        first = self.result(name="a.json")
        second = self.result(name="b.json")
        log = self.root / "pilot.log"
        log.write_text("visible\n")
        one, two = self.root / "one.zip", self.root / "two.zip"
        bundle.write_bundle(one, [second, first], {"b.log": log, "a.log": log})
        os.utime(first, (10, 10))
        bundle.write_bundle(two, [first, second], {"a.log": log, "b.log": log})
        self.assertEqual(one.read_bytes(), two.read_bytes())
        with zipfile.ZipFile(one) as archive:
            self.assertEqual(archive.namelist(), sorted(archive.namelist()))
            for info in archive.infolist():
                self.assertEqual(info.date_time, (1980, 1, 1, 0, 0, 0))
                self.assertEqual(info.external_attr >> 16, 0o100644)
            saved = json.loads(archive.read("result-001.json"))
            self.assertNotIn("provenance", saved["meta"])
            self.assertNotIn("rounds", saved["results"][0])

    def test_images_binary_files_and_inline_image_data_are_omitted(self):
        image = self.root / "image.png"
        image.write_bytes(b"\x89PNG\r\n\x1a\nprivate-image-bytes")
        binary = self.root / "events.log"
        binary.write_bytes(b"binary\x00private-bytes")
        disguised = self.root / "disguised.log"
        disguised.write_bytes(b"\xff\xd8\xffprivate-image-bytes")
        result = self.result({"results": [{"final": "data:image/png;base64,cHJpdmF0ZQ==",
                                          "escaped": r"data:image\/png;base64,cHJpdmF0ZQ=="}]})
        output = self.root / "out.zip"
        bundle.write_bundle(output, [result], {"image.png": image, "events.log": binary,
                                               "disguised.log": disguised})
        with zipfile.ZipFile(output) as archive:
            self.assertEqual(archive.namelist(), ["result-001.json"])
            self.assertNotIn(b"cHJpdmF0ZQ==", archive.read("result-001.json"))

    def test_colored_component_output_and_compressed_rotations_are_included(self):
        component = self.root / "pilot.log"
        component.write_text("\x1b[32mready\x1b[0m\nAPI_KEY=terminal-secret\n")
        rotated = self.root / "pilot.log.1.gz"
        rotated.write_bytes(gzip.compress(b"older ready\nTOKEN=rotation-secret\n"))
        requests = self.root / "requests.jsonl.gz"
        requests.write_bytes(gzip.compress(b'{"reply":"older reply","api_key":"json-secret"}\n'))
        output = self.root / "out.zip"
        bundle.write_bundle(output, [self.result()], {"pilot.log": component,
            "pilot.log.1.gz": rotated, "requests.jsonl.gz": requests})
        with zipfile.ZipFile(output) as archive:
            self.assertIn("ready", archive.read("logs/pilot.log").decode())
            self.assertIn("older ready", archive.read("logs/pilot.log.1").decode())
            self.assertEqual(json.loads(archive.read("logs/requests.jsonl"))["reply"], "older reply")
            for member in archive.namelist():
                self.assertNotIn(b"\x1b", archive.read(member))
        for value in (b"terminal-secret", b"rotation-secret", b"json-secret"):
            self.assertNotIn(value, output.read_bytes())

    def test_private_log_names_get_neutral_names(self):
        log = self.root / "pilot.log"
        log.write_text("safe")
        output = self.root / "out.zip"
        bundle.write_bundle(output, [self.result()], {"robot.local/pilot.log": log})
        with zipfile.ZipFile(output) as archive:
            self.assertEqual(archive.namelist(), ["logs/log-001.txt", "result-001.json"])

    def test_url_hosts_host_ports_and_root_homes_are_scrubbed_without_dns(self):
        values = ["http://robot-box:5050/status", "grpc://planner.corp:5000",
                  "ROBONIX_ATLAS=work_pc:50051", "https://public.example/path",
                  "http://credential-user:credential-pass@public.example/path",
                  "localhost 169.254.12.3 fe80::abcd",
                  r"https:\/\/escaped.corp/path", "/root/private/run", "/root",
                  "~operator/private/run", "~/private/run"]
        with patch.object(socket, "getaddrinfo", side_effect=AssertionError("no DNS")):
            clean = bundle.scrub({"values": values})
        text = json.dumps(clean)
        for private in ("robot-box", "planner.corp", "work_pc", "public.example", "escaped.corp",
                        "/root", "~operator", "~/private", "credential-user", "credential-pass",
                        "localhost", "169.254.12.3", "fe80::abcd"):
            self.assertNotIn(private, text)
        self.assertIn("/status", text)

    def test_escaped_secrets_bare_host_fields_and_structured_image_payloads(self):
        escaped = r"abc\/def"
        values = [escaped, json.dumps(escaped), json.dumps(json.dumps(escaped)),
                  json.dumps(escaped).replace("/", r"\/"), "abc" + "\\" * 7 + "/def"]
        saved = bundle.scrub({"reply": values, "host": "robot-box", "hostname": "planner-corp",
            "server": "private-server", "endpoint": "private-endpoint", "model": "public-model",
            "log": "host=robot-box\nhostname=planner-corp\nserver=private-server\nendpoint=private-endpoint",
            "image": {"type": "base64", "media_type": "image/png", "data": "cHJpdmF0ZQ=="},
            "nested_uri": r"data:image\\/png;base64,cHJpdmF0ZQ=="}, ["abc/def"])
        text = json.dumps(saved)
        for secret in ("abc", "def", "robot-box", "planner-corp", "private-server", "private-endpoint",
                       "cHJpdmF0ZQ=="):
            self.assertNotIn(secret, text)
        self.assertEqual(saved["model"], "public-model")
        self.assertEqual(saved["image"]["data"], "[IMAGE]")

    def test_camera_and_depth_images_inside_json_encoded_leaf_outputs(self):
        output_values = [{"width": 640, "height": 480, "encoding": encoding, "data": payload}
                         for encoding, payload in (("jpeg", "Y2FtZXJhLXByaXZhdGU="),
                                                   ("16UC1", "ZGVwdGgtcHJpdmF0ZQ=="))]
        records = [{"batch_result": {"results": [{"leaf_result": {"call_id": str(index),
            "success": True, "output": json.dumps(value)}}]}} for index, value in enumerate(output_values)]
        log = self.root / "events.jsonl"
        log.write_text("".join(json.dumps(record) + "\n" for record in records))
        output = self.root / "out.zip"
        result = self.result({"results": [{"final": "unchanged model reply",
            "image_base64": "b3RoZXItcHJpdmF0ZQ==", "front_image_base64": "ZnJvbnQtcHJpdmF0ZQ=="}]})
        bundle.write_bundle(output, [result], {"events.jsonl": log})
        with zipfile.ZipFile(output) as archive:
            saved = [json.loads(line) for line in archive.read("logs/events.jsonl").splitlines()]
            for index, record in enumerate(saved):
                leaf = record["batch_result"]["results"][0]["leaf_result"]
                self.assertEqual(leaf["call_id"], str(index))
                self.assertTrue(leaf["success"])
                image = json.loads(leaf["output"])
                self.assertEqual(image, {**output_values[index], "data": "[IMAGE]"})
            saved_result = json.loads(archive.read("result-001.json"))
            self.assertEqual(saved_result["results"][0]["final"], "unchanged model reply")
        for payload in [value["data"] for value in output_values] + ["b3RoZXItcHJpdmF0ZQ==", "ZnJvbnQtcHJpdmF0ZQ=="]:
            self.assertNotIn(payload.encode(), output.read_bytes())
        plain_json = '{ "content": "model reply", "rtdl": [] }'
        self.assertEqual(bundle.scrub({"reply": plain_json})["reply"], plain_json)

    def test_plain_component_image_lines_are_dropped(self):
        text = ('ready\nDEBUG image_base64=cGxhaW4tcHJpdmF0ZQ==\n'
                'DEBUG front_image_base64=cHJlZml4LXByaXZhdGU=\n'
                'DEBUG camera {"width":640,"height":480,"encoding":"jpeg","data":"anBlZy1wcml2YXRl"}\n'
                'DEBUG depth width=640 height=480 encoding=16UC1 data=ZGVwdGgtcHJpdmF0ZQ==\n'
                'finished\n')
        self.assertEqual(bundle.scrub_text(text), "ready\nfinished\n")

    def test_symlinks_and_unsafe_archive_names_are_rejected(self):
        result = self.result()
        link = self.root / "link.json"
        link.symlink_to(result)
        with self.assertRaises(ValueError):
            bundle.write_bundle(self.root / "out.zip", [link], {})
        for name in ("../escape", "/absolute", "a/../b", "a//b", "a\\b", "C:drive", "./a", ""):
            with self.subTest(name=name), self.assertRaises(ValueError):
                bundle.write_bundle(self.root / "out.zip", [result], {name: result})

    def test_standard_temp_paths_work_without_resolving_aliases(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as directory:
            root = Path(directory)
            result = root / "result.json"
            result.write_text('{"results": []}')
            bundle.write_bundle(root / "out.zip", [result], {})
            with zipfile.ZipFile(root / "out.zip") as archive:
                self.assertEqual(json.loads(archive.read("result-001.json")), {"results": []})
        directory = self.root / "linked-logs"
        directory.symlink_to(self.root, target_is_directory=True)
        with patch.object(sys, "argv", ["bundle.py", str(self.result()), "--logs", str(directory),
                                        "--out", str(self.root / "out.zip")]):
            with self.assertRaises(ValueError):
                bundle.main()

    def test_cli_collects_only_supplied_directories_and_env_secrets(self):
        result = self.result({"results": [{"final": "standalone-secret"}]})
        logs = self.root / "logs"
        logs.mkdir()
        (logs / "pilot.log").write_text("answer standalone-secret")
        (self.root / "unselected.log").write_text("not selected")
        env = self.root / "api.env"
        env.write_text("VLM_API_KEY=standalone-secret\n")
        output = self.root / "out.zip"
        subprocess.run([sys.executable, bundle.__file__, str(result), "--logs", str(logs),
                        "--out", str(output), "--env-file", str(env)], check=True)
        self.assertNotIn(b"standalone-secret", output.read_bytes())
        with zipfile.ZipFile(output) as archive:
            self.assertEqual(archive.namelist(), ["logs/run-001/pilot.log", "result-001.json"])


if __name__ == "__main__":
    unittest.main()
