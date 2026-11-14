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
        for name in ("Dockerfile",):
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
            self.assertNotIn("pretrained='laion2b_s34b_b79k'", text)

    def test_native_start_accepts_onnx_without_torch_weights(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            models = root / "onnx"
            models.mkdir()
            (models / "yolo-world-cut.onnx").write_bytes(b"onnx")
            fake_python = root / "python"
            fake_python.write_text(
                '#!/bin/bash\nprintf "%s\\n" "$*" >> "$NATIVE_LOG"\n'
                'if [[ "$1" == */scripts/prepare_models.py && "${NATIVE_PREFLIGHT_FAIL:-0}" == 1 ]]; then exit 1; fi\n',
                encoding="utf-8",
            )
            fake_python.chmod(0o755)
            env = os.environ.copy()
            env.update(RBNX_PACKAGE_ROOT=str(root), SCENE_ONNX_DIR=str(models),
                       SCENE_NATIVE_PYTHON=str(fake_python), NATIVE_LOG=str(root / "calls"),
                       SCENE_CG_INFERENCE="tensorrt")
            subprocess.run(
                ["bash", "-c", 'source() { :; }; export -f source; exec bash "$1"',
                 "native-test", str(SCENE_ROOT / "scripts" / "start_native.sh")],
                env=env, check=True, capture_output=True, text=True,
            )
            calls = (root / "calls").read_text().splitlines()
            self.assertTrue(any("prepare_models.py --check" in call for call in calls))
            self.assertIn("-m scene_service.service", calls)
            self.assertTrue(any(call.endswith("--check") for call in calls))
            (root / "calls").unlink()
            env["NATIVE_PREFLIGHT_FAIL"] = "1"
            failed = subprocess.run(
                ["bash", "-c", 'source() { :; }; export -f source; exec bash "$1"',
                 "native-test", str(SCENE_ROOT / "scripts" / "start_native.sh")],
                env=env, capture_output=True, text=True,
            )
            self.assertNotEqual(failed.returncode, 0)
            self.assertNotIn("-m scene_service.service", (root / "calls").read_text().splitlines())

    def _fixture(self, root: Path, *, include_clip: bool) -> tuple[Path, dict[str, str]]:
        scene = root / "system" / "scene"
        (scene / "scripts").mkdir(parents=True)
        (scene / "docker" / "_weights").mkdir(parents=True)
        (root / "scripts").mkdir(parents=True)
        shutil.copy2(SCENE_ROOT / "scripts" / "build.sh", scene / "scripts")
        (scene / "scene_service").mkdir()
        shutil.copy2(SCENE_ROOT / "scene_service/inference_config.py", scene / "scene_service")
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
if [[ "${1:-}" == run ]]; then
    printf '%s\n' "$@" > "$FAKE_PREP_LOG"
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

    def test_tensorrt_docker_build_prepares_models_with_selected_encoder(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scene, env = self._fixture(root, include_clip=True)
            env.update(SCENE_CG_INFERENCE="tensorrt", SCENE_CLIP_MODEL="ViT-B-32",
                       SCENE_PROFILE="lite", FAKE_PREP_LOG=str(root / "prepare.log"))
            args = self._run_build(scene, env)
            self.assertIn("SCENE_CG_INFERENCE=tensorrt", args)
            prep = (root / "prepare.log").read_text().splitlines()
            self.assertIn("SCENE_CLIP_MODEL=ViT-B-32", prep)
            self.assertIn("SCENE_PROFILE=lite", prep)
            self.assertIn("/scene/scripts/prepare_models.py", prep)
            self.assertIn("--gpus", prep)

    def test_native_build_only_prepares_models_when_tensorrt_is_selected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scene, env = self._fixture(root, include_clip=True)
            fake_python = root / "python"
            fake_python.write_text('#!/bin/bash\nprintf "%s\\n" "$*" >> "$NATIVE_LOG"\n')
            fake_python.chmod(0o755)
            (scene / "rbnx-build").mkdir()
            (scene / "rbnx-build/native-candidate.sh").write_text(
                f'export SCENE_NATIVE_PYTHON="{fake_python}"\nexport ROS_DISTRO=jazzy\n')
            env.update(RBNX_BUILD_TARGET="jetson-native", SCENE_NATIVE_PYTHON=str(fake_python),
                       NATIVE_LOG=str(root / "native.log"))
            for backend in ("torch", "tensorrt"):
                env["SCENE_CG_INFERENCE"] = backend
                subprocess.run(["bash", "scripts/build.sh"], cwd=scene, env=env,
                               check=True, capture_output=True, text=True)
                calls = (root / "native.log").read_text()
                self.assertEqual("prepare_models.py" in calls, backend == "tensorrt")
                self.assertEqual("scene-perception-export.txt" in calls, backend == "tensorrt")
                (root / "native.log").unlink()

    def test_full_downloads_selected_weights_and_reuses_the_cache(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scene, env = self._fixture(root, include_clip=True)
            env.update(SCENE_PROFILE="full", RBNX_GH_MIRROR="", SCENE_MODELS_DIR=str(root / "full"))
            self._run_build(scene, env)
            urls = (root / "curl.log").read_text().splitlines()
            self.assertIn("https://github.com/ultralytics/assets/releases/download/v8.3.0/sam_l.pt", urls)
            clip = "laion/CLIP-ViT-H-14-laion2B-s32B-b79K/resolve/main/open_clip_pytorch_model.bin"
            self.assertIn(f"https://hf.invalid/{clip}", urls)
            self.assertIn(f"https://huggingface.co/{clip}", urls)
            self.assertEqual((root / "full/sam_l.pt").read_bytes(), b"downloaded-clip")
            name = "open_clip_vit_h14_laion2b_s32b_b79k.bin"
            self.assertEqual((root / "full" / name).read_bytes(), b"downloaded-clip")
            self.assertEqual((scene / "docker/_build/full-models" / name).read_bytes(), b"downloaded-clip")
            shutil.rmtree(root / "full")
            (root / "curl.log").unlink()
            self._run_build(scene, env)
            self.assertFalse((root / "curl.log").exists())
            self.assertEqual((root / "full" / name).read_bytes(), b"downloaded-clip")

    def test_full_explicit_checkpoints_are_reused_without_download(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scene, env = self._fixture(root, include_clip=True)
            sam, clip = root / "sam_l.pt", root / "clip.bin"
            sam.write_bytes(b"local sam")
            clip.write_bytes(b"local clip")
            env.update(SCENE_PROFILE="full", SCENE_MOBILE_SAM_WEIGHTS=str(sam),
                       SCENE_CLIP_PRETRAINED=str(clip))
            self._run_build(scene, env)
            self.assertFalse((root / "curl.log").exists())
            sam.unlink()
            result = subprocess.run(["bash", "scripts/build.sh"], cwd=scene, env=env,
                                    capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("existing local checkpoint", result.stderr)
            self.assertFalse((root / "curl.log").exists())

    def test_x86_stages_external_lite_checkpoints_in_the_build_context(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scene, env = self._fixture(root, include_clip=False)
            for path in (scene / "docker/_weights").iterdir():
                path.unlink()
            overrides = {"SCENE_YOLO_WORLD_WEIGHTS": "yolov8l-world.pt",
                         "SCENE_MOBILE_SAM_WEIGHTS": "mobile_sam.pt",
                         "SCENE_CLIP_PRETRAINED": OPEN_CLIP_FILE}
            for variable, name in overrides.items():
                checkpoint = root / name
                checkpoint.write_bytes(name.encode())
                env[variable] = str(checkpoint)
            self._run_build(scene, env)
            self.assertFalse((root / "curl.log").exists())
            for name in overrides.values():
                self.assertEqual((scene / "docker/_weights" / name).read_bytes(), name.encode())
            self.assertTrue((scene / "docker/_build/full-models").is_dir())
            for variable, name in overrides.items():
                env[variable] = f"docker/_weights/{name}"
            self._run_build(scene, env)
            self.assertFalse((root / "curl.log").exists())

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


def test_tensorrt_preflight_failure_does_not_stop_existing_container(tmp_path):
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    docker = fake_bin / 'docker'
    docker.write_text('#!/bin/bash\nprintf "%s\\n" "$*" >> "$DOCKER_LOG"\nexit 23\n')
    docker.chmod(0o755)
    env = os.environ.copy()
    env.update(PATH=str(fake_bin) + os.pathsep + env['PATH'],
               RBNX_PACKAGE_ROOT=str(tmp_path), ROBONIX_SCENE_FORCE='docker',
               SCENE_CG_INFERENCE=' TensorRT ', SCENE_PERCEPTION_BACKEND='',
               DOCKER_LOG=str(tmp_path / 'docker.log'))
    result = subprocess.run(['bash', str(SCENE_ROOT / 'scripts/start.sh')],
                            env=env, capture_output=True, text=True)
    assert result.returncode == 23
    calls = (tmp_path / 'docker.log').read_text().splitlines()
    assert len(calls) == 1
    assert 'prepare_models.py --check' in calls[0]
    assert calls[0].startswith('run --rm ')


def test_native_start_selects_prepared_python_and_stops_on_stale_environment(tmp_path):
    (tmp_path / 'rbnx-build').mkdir()
    python = tmp_path / 'prepared-python'
    python.write_text(
        '#!/bin/bash\nprintf "%s\\n" "$*" >> "$NATIVE_LOG"\n'
        'if [[ "$1" == */prepare_native.py && "${STALE_ENV:-0}" == 1 ]]; then exit 18; fi\n')
    python.chmod(0o755)
    (tmp_path / 'rbnx-build/native-env.sh').write_text(
        f'export SCENE_NATIVE_PYTHON="{python}"\nexport ROS_DISTRO=jazzy\n')
    env = os.environ.copy()
    env.update(RBNX_PACKAGE_ROOT=str(tmp_path), NATIVE_LOG=str(tmp_path / 'calls'))
    env.pop('SCENE_NATIVE_PYTHON', None)
    command = ['bash', '-c', 'source() { if [[ "$1" == /opt/ros/* ]]; then return; fi; builtin source "$1"; }; '
               'export -f source; exec bash "$1"', 'native-test', str(SCENE_ROOT / 'scripts/start_native.sh')]
    subprocess.run(command, env=env, check=True, capture_output=True, text=True)
    calls = (tmp_path / 'calls').read_text().splitlines()
    assert calls[0].endswith('/prepare_native.py --check')
    assert calls[1].endswith('/prepare_models.py --check')
    assert calls[2] == '-m scene_service.service'
    (tmp_path / 'calls').unlink()
    env['STALE_ENV'] = '1'
    result = subprocess.run(command, env=env, capture_output=True, text=True)
    assert result.returncode == 18
    assert len((tmp_path / 'calls').read_text().splitlines()) == 1
