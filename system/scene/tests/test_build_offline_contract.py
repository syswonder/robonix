# SPDX-License-Identifier: MulanPSL-2.0
"""Offline build contracts for Scene.

The behavioural tests replace ``docker``, ``rbnx``, and ``curl`` with small
recording commands. They never contact a daemon, ROS graph, or network.
"""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


SCENE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = SCENE_ROOT.parents[1]
OPEN_CLIP_REPO = "laion/CLIP-ViT-B-32-laion2B-s34B-b79K"
OPEN_CLIP_FILE = "open_clip_pytorch_model.bin"


class OfflineBuildContractTests(unittest.TestCase):
    def test_weight_downloads_use_a_stable_resumable_partial_file(self):
        build = (SCENE_ROOT / "scripts" / "build.sh").read_text(encoding="utf-8")
        self.assertIn('local tmp="$cached.part"', build)
        self.assertIn("-C -", build)
        self.assertNotIn('local tmp="$cached.tmp.$$"', build)

    def test_dockerfiles_default_to_bundled_frontend_and_local_clip(self):
        for name in ("Dockerfile", "Dockerfile.jetson"):
            text = (SCENE_ROOT / "docker" / name).read_text(encoding="utf-8")
            self.assertFalse(
                text.splitlines()[0].lstrip().startswith("# syntax="),
                f"{name} must not force a registry-backed frontend",
            )
            self.assertIn(
                "COPY _weights/open_clip_pytorch_model.bin "
                "/opt/models/open_clip_pytorch_model.bin",
                text,
            )
            self.assertIn(
                "pretrained='/opt/models/open_clip_pytorch_model.bin'", text
            )
            self.assertIn(
                "SCENE_CLIP_PRETRAINED=/opt/models/open_clip_pytorch_model.bin",
                text,
            )
            self.assertNotIn("pretrained='laion2b_s34b_b79k'", text)

    def test_native_start_defaults_to_staged_clip_checkpoint(self):
        start = (SCENE_ROOT / "scripts" / "start_native.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            'SCENE_CLIP_PRETRAINED:-$W/open_clip_pytorch_model.bin', start
        )
        self.assertIn('"$SCENE_CLIP_PRETRAINED"', start)

    def _fixture(self, root: Path, *, include_clip: bool) -> tuple[Path, dict[str, str]]:
        scene = root / "system" / "scene"
        (scene / "scripts").mkdir(parents=True)
        (scene / "docker" / "_weights").mkdir(parents=True)
        (root / "scripts").mkdir(parents=True)
        shutil.copy2(SCENE_ROOT / "scripts" / "build.sh", scene / "scripts")
        # The production build delegates exact-version Python generation to
        # this helper. These tests mock rbnx itself, so use a lightweight
        # fixture helper that preserves the delegation boundary without
        # creating a real uv environment.
        codegen_helper = scene / "scripts" / "run_python_codegen.sh"
        codegen_helper.write_text(
            "#!/usr/bin/env bash\n"
            "set -eu\n"
            "pkg=\"$1\"\n"
            "shift\n"
            "rbnx codegen -p \"$pkg\" \"$@\"\n",
            encoding="utf-8",
        )
        codegen_helper.chmod(0o755)
        shutil.copy2(
            REPO_ROOT / "scripts" / "docker_base_image.sh", root / "scripts"
        )

        weights = scene / "docker" / "_weights"
        (weights / "yolov8l-world.pt").write_bytes(b"yolo")
        (weights / "mobile_sam.pt").write_bytes(b"sam")
        if include_clip:
            (weights / OPEN_CLIP_FILE).write_bytes(b"clip")

        fake_bin = root / "fake-bin"
        fake_bin.mkdir()
        (fake_bin / "rbnx").write_text(
            "#!/usr/bin/env bash\nexit 0\n", encoding="utf-8"
        )
        (fake_bin / "docker").write_text(
            """#!/usr/bin/env bash
set -eu
if [[ "${1:-}" == image && "${2:-}" == inspect ]]; then
    exit 0
fi
if [[ "${1:-}" == build ]]; then
    printf '%s\n' "$@" > "$FAKE_DOCKER_LOG"
    exit 0
fi
echo "unexpected fake docker invocation: $*" >&2
exit 91
""",
            encoding="utf-8",
        )
        (fake_bin / "curl").write_text(
            """#!/usr/bin/env bash
set -eu
out=
previous=
for argument in "$@"; do
    if [[ "$previous" == -o ]]; then
        out="$argument"
    fi
    previous="$argument"
done
url="${!#}"
printf '%s\n' "$url" >> "$FAKE_CURL_LOG"
if [[ "$url" == "${RBNX_HF_MIRROR%/}/"* ]]; then
    exit 22
fi
[[ -n "$out" ]]
printf 'downloaded-clip' > "$out"
""",
            encoding="utf-8",
        )
        for command in ("rbnx", "docker", "curl"):
            (fake_bin / command).chmod(0o755)

        env = os.environ.copy()
        env.update(
            {
                "PATH": f"{fake_bin}{os.pathsep}{env['PATH']}",
                "RBNX_PACKAGE_ROOT": str(scene),
                "RBNX_BUILD_PROXY": "0",
                "ROBONIX_MODEL_CACHE_DIR": str(root / "model-cache"),
                "RBNX_HF_MIRROR": "https://hf.invalid",
                "FAKE_DOCKER_LOG": str(root / "docker.log"),
                "FAKE_CURL_LOG": str(root / "curl.log"),
            }
        )
        return scene, env

    def _run_build(self, scene: Path, env: dict[str, str]) -> list[str]:
        subprocess.run(
            ["bash", "scripts/build.sh"],
            cwd=scene,
            env=env,
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return Path(env["FAKE_DOCKER_LOG"]).read_text(encoding="utf-8").splitlines()

    def test_buildkit_syntax_override_is_optional_and_forwarded_verbatim(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scene, env = self._fixture(root, include_clip=True)
            args = self._run_build(scene, env)
            self.assertFalse(any(arg.startswith("BUILDKIT_SYNTAX=") for arg in args))
            self.assertFalse((root / "curl.log").exists())

            syntax = "registry.invalid/docker/dockerfile:1.7@sha256:012345"
            env["ROBONIX_SCENE_BUILDKIT_SYNTAX"] = syntax
            args = self._run_build(scene, env)
            self.assertIn("--build-arg", args)
            self.assertIn(f"BUILDKIT_SYNTAX={syntax}", args)

    def test_hf_mirror_failure_falls_back_to_direct_file_and_populates_cache(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scene, env = self._fixture(root, include_clip=False)
            self._run_build(scene, env)

            urls = (root / "curl.log").read_text(encoding="utf-8").splitlines()
            suffix = f"{OPEN_CLIP_REPO}/resolve/main/{OPEN_CLIP_FILE}"
            self.assertEqual(
                urls,
                [f"https://hf.invalid/{suffix}", f"https://huggingface.co/{suffix}"],
            )
            expected = b"downloaded-clip"
            self.assertEqual((root / "model-cache" / OPEN_CLIP_FILE).read_bytes(), expected)
            self.assertEqual(
                (scene / "docker" / "_weights" / OPEN_CLIP_FILE).read_bytes(),
                expected,
            )

    def test_empty_hf_mirror_uses_only_the_direct_file_url(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scene, env = self._fixture(root, include_clip=False)
            env["RBNX_HF_MIRROR"] = ""
            self._run_build(scene, env)

            suffix = f"{OPEN_CLIP_REPO}/resolve/main/{OPEN_CLIP_FILE}"
            urls = (root / "curl.log").read_text(encoding="utf-8").splitlines()
            self.assertEqual(urls, [f"https://huggingface.co/{suffix}"])


if __name__ == "__main__":
    unittest.main()
