# SPDX-License-Identifier: MulanPSL-2.0
"""Launcher return modes retain sensor and browser readiness gates."""
import os
from pathlib import Path
import shutil
import subprocess

import tempfile
import unittest


class WebotsStartTest(unittest.TestCase):
    """Exercise the actual launcher without starting host containers."""

    def test_launcher_return_modes(self):
        for detach, ready, stream in [(True, True, False), (False, True, False),
                                      (True, False, False), (True, True, True)]:
            with self.subTest(detach=detach, ready=ready, stream=stream):
                with tempfile.TemporaryDirectory() as directory:
                    self.check_launcher(Path(directory), detach, ready, stream)

    def check_launcher(self, tmp_path, detach, ready, stream):
        source = Path(__file__).resolve().parents[1] / "examples/webots/sim"
        sim = tmp_path / 'repo/examples/webots/sim'
        sim.mkdir(parents=True)
        shutil.copy(source / 'start.sh', sim / 'start.sh')
        shutil.copy(source / 'runtime_state.sh', sim / 'runtime_state.sh')
        (sim / 'nvidia_host.sh').write_text('robonix_effective_webots_headless_mode() { echo cpu; }\n')
        scripts = tmp_path / 'repo/scripts'
        scripts.mkdir()
        (scripts / 'docker_base_image.sh').write_text('robonix_ensure_local_base_image() { :; }\n')
        (sim / 'start_rviz.sh').write_text('exit 0\n')
        bins = tmp_path / 'bin'
        bins.mkdir()
        log = tmp_path / 'calls'
        docker = bins / 'docker'
        docker.write_text('''#!/bin/bash
    printf '%s\\n' "$*" >> "$CALLS"
    if [[ "$1" == inspect ]]; then echo true; exit 0; fi
    if [[ "$1" == exec && "$READY" != 1 ]]; then exit 1; fi
    exit 0
    ''')
        docker.chmod(0o755)
        for name in ['sleep', 'xhost', 'ip', 'tailscale']:
            path = bins / name
            path.write_text('#!/bin/bash\nexit 0\n')
            path.chmod(0o755)
        env = os.environ | {'PATH': str(bins) + ':' + os.environ['PATH'], 'CALLS': str(log),
                            'READY': str(int(ready)), 'ROBONIX_FORCE_CPU': '1',
                            'ROBONIX_SIM_STREAM': str(int(stream)), 'WEBOTS_HEADLESS_MODE': 'cpu',
                            'DISPLAY': ':99', 'TMPDIR': str(tmp_path), 'XDG_RUNTIME_DIR': str(tmp_path)}
        result = subprocess.run(['bash', sim / 'start.sh', *(['--detach'] if detach else [])],
                                env=env, capture_output=True, text=True, timeout=10)
        calls = log.read_text() if log.exists() else ''
        self.assertEqual(result.returncode == 0, ready, result.stderr)
        self.assertIn('up --build -d', calls)
        self.assertIn('topic echo --once /odom', calls)
        self.assertEqual('logs -f' in calls, ready and not detach)
        if stream:
            self.assertIn('streaming_healthcheck.py', calls)


if __name__ == "__main__":
    unittest.main()
