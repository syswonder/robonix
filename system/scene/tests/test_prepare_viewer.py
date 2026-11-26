# SPDX-License-Identifier: MulanPSL-2.0
"""Native viewer assets are cached only after a complete extraction."""
import io
import os
from pathlib import Path
import subprocess
import tarfile

import pytest


@pytest.mark.parametrize('complete', [True, False])
def test_viewer_assets_publish_and_cache(tmp_path, complete):
    package = Path(__file__).resolve().parents[1]
    version = next(line.split('==')[1] for line in
                   (package / 'docker/requirements/scene-viewer.txt').read_text().splitlines()
                   if line.startswith('rerun-sdk=='))
    archive = tmp_path / 'registry/@rerun-io/web-viewer/-' / f'web-viewer-{version}.tgz'
    archive.parent.mkdir(parents=True)
    names = ['index.js', 're_viewer.js', 're_viewer_bg.wasm']
    with tarfile.open(archive, 'w:gz') as stream:
        for name in names if complete else names[:2]:
            content = b'asset'
            member = tarfile.TarInfo('package/' + name)
            member.size = len(content)
            stream.addfile(member, io.BytesIO(content))
    env = dict(os.environ, NPM_REGISTRY=(tmp_path / 'registry').as_uri())
    destination = tmp_path / 'assets'
    command = ['bash', str(package / 'scripts/prepare_viewer.sh'), str(destination)]
    result = subprocess.run(command, env=env, capture_output=True)
    assert (result.returncode == 0) == complete
    assert not list(destination.glob('.viewer.*'))
    if complete:
        assert all((destination / version / name).read_bytes() == b'asset' for name in names)
        archive.unlink()
        subprocess.run(command, env=env, capture_output=True, check=True)
    else:
        assert not (destination / version).exists()
