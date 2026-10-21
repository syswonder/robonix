# SPDX-License-Identifier: MulanPSL-2.0
"""Exercise model selection through the build script without network access."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


BUILD_SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "build.sh"
STUB = r'''#!/usr/bin/env python3
import json, os, pathlib, sys
name = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ["CALLS"], "a") as f:
    f.write(json.dumps([name, *args]) + "\n")
if name == "uv" and args[0] == "venv":
    target = pathlib.Path(args[-1]) / "bin"
    target.mkdir(parents=True)
    (target / "python").symlink_to(os.environ["STUB"])
elif name == "python" and args[:1] == ["-c"] and "sysconfig" in args[1]:
    print(pathlib.Path.cwd() / "site-packages")
elif name == "curl":
    pathlib.Path(args[args.index("-o") + 1]).touch()
elif name == "tar":
    archive = pathlib.Path(args[args.index("-xjf") + 1])
    dest = pathlib.Path(args[args.index("-C") + 1]) / archive.name.removesuffix(".tar.bz2")
    dest.mkdir(parents=True)
    for file in ["tokens.txt", "encoder.int8.onnx", "decoder.int8.onnx", "encoder.onnx", "decoder.onnx"]:
        (dest / file).touch()
'''


class BuildModelsTest(unittest.TestCase):
    def _build(self, backend, *, skip=False):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            tools = root / "tools"
            tools.mkdir()
            stub = tools / "python"
            stub.write_text(STUB)
            stub.chmod(0o755)
            for command in ("uv", "rbnx", "curl", "tar", "bzip2"):
                (tools / command).symlink_to(stub)
            calls = root / "calls.jsonl"
            env = {
                **os.environ,
                "PATH": f"{tools}:{os.environ['PATH']}",
                "RBNX_PACKAGE_ROOT": str(root),
                "SPEECH_BACKEND": backend,
                "SKIP_MODEL_DOWNLOAD": "1" if skip else "",
                "DOWNLOAD_WHISPER": "",
                "RBNX_BUILD_CLEAN": "",
                "SPEECH_SKIP_JETSON_TORCH": "1",
                "CALLS": str(calls),
                "STUB": str(stub),
            }
            result = subprocess.run(
                ["bash", str(BUILD_SCRIPT)], env=env, capture_output=True, text=True,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            records = [json.loads(line) for line in calls.read_text().splitlines()]
            urls = [args[-1] for name, *args in records if name == "curl"]
            asr = root / "rbnx-build/models/sherpa-onnx-streaming-paraformer-bilingual-zh-en"
            return urls, (asr / "encoder.int8.onnx").exists(), (asr / "encoder.onnx").exists()

    def test_local_downloads_both_models_and_keeps_only_int8_asr(self):
        urls, int8, fp32 = self._build("local")
        self.assertEqual(len(urls), 2)
        self.assertTrue(any("/kws-models/" in url for url in urls))
        self.assertTrue(any("/asr-models/" in url for url in urls))
        self.assertTrue(int8)
        self.assertFalse(fp32)

    def test_nonlocal_downloads_only_shared_wake_word_model(self):
        for backend in ("tencent", "custom", "mock"):
            with self.subTest(backend=backend):
                urls, int8, _ = self._build(backend)
                self.assertEqual(len(urls), 1)
                self.assertIn("/kws-models/", urls[0])
                self.assertFalse(int8)

    def test_skip_download_applies_to_both_models(self):
        for backend in ("local", "tencent"):
            with self.subTest(backend=backend):
                urls, int8, _ = self._build(backend, skip=True)
                self.assertEqual(urls, [])
                self.assertFalse(int8)
