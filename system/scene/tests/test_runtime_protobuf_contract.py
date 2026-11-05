# SPDX-License-Identifier: MulanPSL-2.0
"""Offline contract for Scene's image-matched protobuf generation."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest


SCENE_ROOT = Path(__file__).resolve().parents[1]


class RuntimeProtobufContractTests(unittest.TestCase):
    def test_launcher_generates_with_runtime_image_without_network(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scene = root / "scene"
            (scene / "scripts").mkdir(parents=True)
            (scene / "docker").mkdir()
            (scene / "rbnx-build" / "proto-staging").mkdir(parents=True)
            (scene / "rbnx-build" / "codegen" / "proto_gen").mkdir(parents=True)
            runtime_proto = root / "runtime-proto"
            runtime_proto.mkdir()

            shutil.copy2(SCENE_ROOT / "scripts" / "start.sh", scene / "scripts")
            shutil.copy2(
                SCENE_ROOT / "docker" / "entrypoint.sh",
                scene / "docker" / "entrypoint.sh",
            )
            (scene / "rbnx-build" / "proto-staging" / "asr.proto").write_text(
                'syntax = "proto3";\n', encoding="utf-8"
            )
            (runtime_proto / "atlas.proto").write_text(
                'syntax = "proto3";\n', encoding="utf-8"
            )
            # The ambient output is intentionally retained and never imported by Scene.
            ambient = scene / "rbnx-build" / "codegen" / "proto_gen" / "asr_pb2.py"
            ambient.write_text("# incompatible host gencode\n", encoding="utf-8")

            fake_bin = root / "fake-bin"
            fake_bin.mkdir()
            docker_log = root / "docker.log"
            (fake_bin / "rbnx").write_text(
                "#!/usr/bin/env bash\n"
                "set -eu\n"
                "[[ \"${1:-}\" == path ]]\n"
                "if [[ \"${2:-}\" == runtime-proto ]]; then printf '%s\\n' \"$FAKE_RUNTIME_PROTO\"; else printf '/robonix-api\\n'; fi\n",
                encoding="utf-8",
            )
            (fake_bin / "docker").write_text(
                """#!/usr/bin/env bash
set -eu
printf '%q ' "$@" >> "$FAKE_DOCKER_LOG"
printf '\n' >> "$FAKE_DOCKER_LOG"
if [[ " $* " == *" --network none "* ]]; then
    for argument in "$@"; do
        case "$argument" in
            *:/proto-gen) output="${argument%:/proto-gen}" ;;
        esac
    done
    [[ -n "${output:-}" ]]
    printf '# image-matched\n' > "$output/atlas_pb2.py"
    printf '# image-matched\n' > "$output/robonix_contracts_pb2_grpc.py"
fi
exit 0
""",
                encoding="utf-8",
            )
            for executable in (fake_bin / "rbnx", fake_bin / "docker"):
                executable.chmod(0o755)

            env = os.environ.copy()
            env.update(
                {
                    "PATH": f"{fake_bin}{os.pathsep}{env['PATH']}",
                    "RBNX_PACKAGE_ROOT": str(scene),
                    "ROBONIX_SCENE_IMAGE": "scene-runtime:test",
                    "SCENE_DATA_DIR": str(root / "scene-data"),
                    "FAKE_RUNTIME_PROTO": str(runtime_proto),
                    "FAKE_DOCKER_LOG": str(docker_log),
                }
            )
            subprocess.run(
                ["bash", "scripts/start.sh"],
                cwd=scene,
                env=env,
                check=True,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            lines = docker_log.read_text(encoding="utf-8").splitlines()
            codegen = next(line for line in lines if "--network none" in line)
            service = next(line for line in lines if "--network host" in line)
            self.assertIn("--entrypoint sh", codegen)
            self.assertIn("grpc_tools.protoc", codegen)
            self.assertIn("--entrypoint /scene/docker/entrypoint.sh", service)
            self.assertIn(
                f"{scene / 'rbnx-build' / 'codegen' / 'scene_proto_gen'}:"
                "/scene/rbnx-build/codegen/proto_gen:ro",
                service,
            )
            self.assertEqual(ambient.read_text(encoding="utf-8"), "# incompatible host gencode\n")
            self.assertTrue(
                (scene / "rbnx-build" / "codegen" / "scene_proto_gen" / "atlas_pb2.py").is_file()
            )

    def test_packaged_launcher_stops_its_container_on_termination(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scene = root / "scene"
            (scene / "scripts").mkdir(parents=True)
            build = scene / "rbnx-build"
            build.mkdir()
            (build / "docker-env.json").write_text("{}")
            (build / "docker-runtime.sh").write_text(
                f"IMG=sha256:verified\nSCENE_DOCKER_CACHE={root / 'models'}\n"
            )
            shutil.copy2(SCENE_ROOT / "scripts/start.sh", scene / "scripts")
            fake_bin = root / "bin"
            fake_bin.mkdir()
            docker_log = root / "docker.log"
            ready = root / "ready"
            (fake_bin / "python3").write_text("#!/bin/sh\nexit 0\n")
            (fake_bin / "docker").write_text("""#!/usr/bin/env bash
set -eu
printf '%q ' "$@" >> "$FAKE_DOCKER_LOG"
printf '\n' >> "$FAKE_DOCKER_LOG"
if [[ "$1" == run ]]; then
    while [[ "$1" != --cidfile ]]; do shift; done
    printf 'owned-container' > "$2"
    touch "$FAKE_READY"
    while [[ ! -f "$FAKE_READY.stopped" ]]; do sleep 0.05; done
elif [[ "$1" == stop ]]; then
    [[ "$2" == owned-container ]]
    touch "$FAKE_READY.stopped"
fi
""")
            for executable in fake_bin.iterdir():
                executable.chmod(0o755)
            env = os.environ.copy()
            env.update(PATH=f"{fake_bin}:{env['PATH']}", RBNX_PACKAGE_ROOT=str(scene),
                       ROBONIX_SCENE_FORCE="docker", SCENE_CG_INFERENCE="torch", SCENE_CG_FORCE_CPU="true",
                       SCENE_DATA_DIR=str(root / "data"), FAKE_DOCKER_LOG=str(docker_log),
                       FAKE_READY=str(ready))
            process = subprocess.Popen(["bash", "scripts/start.sh"], cwd=scene, env=env,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                deadline = time.monotonic() + 5
                while not ready.exists() and process.poll() is None and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(ready.exists())
                process.terminate()
                stdout, stderr = process.communicate(timeout=5)
                self.assertEqual(process.returncode, 143, stdout + stderr)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()
                # Release a fake Docker child if a failing launcher never called cleanup.
                Path(str(ready) + ".stopped").touch()
            lines = docker_log.read_text().splitlines()
            service = next(line for line in lines if "--network host" in line)
            self.assertIn("sha256:verified", service)
            self.assertIn("NVIDIA_VISIBLE_DEVICES=void", service)
            self.assertIn(f"{root / 'models'}:/scene-models:ro", service)
            self.assertNotIn(f"{scene}:/scene", service)
            self.assertNotIn("/robonix-api", service)
            self.assertNotIn("grpc_tools.protoc", service)
            self.assertEqual([line for line in lines if line.startswith("stop ")],
                             ["stop owned-container "])
            self.assertEqual(list(build.glob(".scene-run.*")), [])

    def test_entrypoint_has_no_ambient_codegen_fallback(self):
        entrypoint = (SCENE_ROOT / "docker" / "entrypoint.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("SCENE_PROTO_GEN=/scene/rbnx-build/codegen/scene_proto_gen", entrypoint)
        export_line = next(
            line for line in entrypoint.splitlines() if line.startswith("export PYTHONPATH=")
        )
        self.assertIn("$SCENE_PROTO_GEN", export_line)
        self.assertNotIn("/codegen/proto_gen:", export_line)


if __name__ == "__main__":
    unittest.main()
