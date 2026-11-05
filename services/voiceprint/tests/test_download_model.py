# SPDX-License-Identifier: MulanPSL-2.0
import hashlib
import importlib.util
from pathlib import Path
import subprocess
from unittest.mock import Mock

import pytest


@pytest.fixture
def downloader(monkeypatch):
    source = Path(__file__).resolve().parents[1] / 'scripts/download_model.py'
    spec = importlib.util.spec_from_file_location('voiceprint_download_model', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'SHA256', hashlib.sha256(b'valid model').hexdigest())
    return module


def test_valid_cached_model_avoids_network(downloader, tmp_path, monkeypatch):
    destination = tmp_path / 'model.onnx'
    destination.write_bytes(b'valid model')
    run = Mock(side_effect=AssertionError('cache hit must not download'))
    monkeypatch.setattr(downloader.subprocess, 'run', run)
    downloader.download_model(destination)
    run.assert_not_called()


@pytest.mark.parametrize('first_failure', ['transfer', 'checksum'])
def test_failed_source_does_not_publish_partial_model(downloader, tmp_path, monkeypatch, first_failure):
    destination = tmp_path / 'model.onnx'
    destination.write_bytes(b'old incomplete download')
    calls = []

    def run(command, **kwargs):
        assert destination.read_bytes() == b'old incomplete download'
        temporary = Path(command[command.index('-o') + 1])
        calls.append(command)
        temporary.write_bytes(b'partial' if len(calls) == 1 else b'valid model')
        if len(calls) == 1 and first_failure == 'transfer':
            raise subprocess.CalledProcessError(28, command)

    monkeypatch.setattr(downloader.subprocess, 'run', run)
    downloader.download_model(destination)
    assert len(calls) == 2
    assert destination.read_bytes() == b'valid model'
    assert list(tmp_path.iterdir()) == [destination]


def test_all_sources_fail_without_overwriting_existing_file(downloader, tmp_path, monkeypatch):
    destination = tmp_path / 'model.onnx'
    destination.write_bytes(b'incomplete')
    monkeypatch.setattr(downloader.subprocess, 'run', Mock(
        side_effect=subprocess.TimeoutExpired(['curl'], 240)))
    with pytest.raises(RuntimeError, match='any source'):
        downloader.download_model(destination)
    assert destination.read_bytes() == b'incomplete'
    assert list(tmp_path.iterdir()) == [destination]
