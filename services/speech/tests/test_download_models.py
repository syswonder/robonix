# SPDX-License-Identifier: MulanPSL-2.0
import hashlib
import importlib.util
import io
from pathlib import Path
import subprocess
import tarfile
from unittest.mock import Mock

import pytest


@pytest.fixture
def downloader(monkeypatch):
    monkeypatch.delenv('ROBONIX_MODEL_CACHE_DIR', raising=False)
    source = Path(__file__).resolve().parents[1] / 'scripts/download_models.py'
    spec = importlib.util.spec_from_file_location('speech_download_models', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def archive_bytes(files, *, link=False):
    stream = io.BytesIO()
    with tarfile.open(fileobj=stream, mode='w:bz2') as archive:
        for name, content in files.items():
            member = tarfile.TarInfo('test-model/' + name)
            member.size = len(content)
            if link:
                member.type = tarfile.SYMTYPE
                member.linkname = '/tmp/forbidden-model-target'
            archive.addfile(member, io.BytesIO(content))
    return stream.getvalue()


def configure(module, monkeypatch, content):
    monkeypatch.setitem(module.MODELS, 'asr', ('test-model', 'asr-models',
        hashlib.sha256(content).hexdigest(), ('tokens.txt', 'encoder.int8.onnx')))


def test_download_verifies_cache_and_replaces_corrupted_files(downloader, monkeypatch, tmp_path):
    data = archive_bytes({'tokens.txt': b'tokens', 'encoder.int8.onnx': b'model', 'encoder.onnx': b'large'})
    configure(downloader, monkeypatch, data)
    download = Mock(side_effect=lambda cmd, **kw: Path(cmd[cmd.index('-o') + 1]).write_bytes(data))
    monkeypatch.setattr(downloader.subprocess, 'run', download)
    downloader.prepare(tmp_path, 'asr')
    dest = tmp_path / 'test-model'
    assert (dest / 'encoder.int8.onnx').read_bytes() == b'model'
    assert not (dest / 'encoder.onnx').exists()
    downloader.prepare(tmp_path, 'asr')
    assert download.call_count == 1
    (dest / 'encoder.int8.onnx').write_bytes(b'corrupt')
    downloader.prepare(tmp_path, 'asr')
    assert download.call_count == 2
    assert (dest / 'encoder.int8.onnx').read_bytes() == b'model'


@pytest.mark.parametrize('failure', ['transfer', 'checksum', 'missing', 'symlink'])
def test_failed_download_or_extraction_preserves_previous_model(downloader, monkeypatch, tmp_path, failure):
    data = archive_bytes({'tokens.txt': b'tokens'}, link=failure == 'symlink')
    configure(downloader, monkeypatch, data)
    dest = tmp_path / 'test-model'
    dest.mkdir()
    (dest / 'tokens.txt').write_bytes(b'previous')
    def run(cmd, **kwargs):
        Path(cmd[cmd.index('-o') + 1]).write_bytes(b'wrong' if failure == 'checksum' else data)
        if failure == 'transfer':
            raise subprocess.TimeoutExpired(cmd, 1250)
    monkeypatch.setattr(downloader.subprocess, 'run', run)
    with pytest.raises((RuntimeError, FileNotFoundError, ValueError)):
        downloader.prepare(tmp_path, 'asr')
    assert (dest / 'tokens.txt').read_bytes() == b'previous'
    assert not any(p.is_dir() and p.name.startswith('.test-model-') for p in tmp_path.iterdir())


def test_configured_mirror_and_official_fallback(downloader, monkeypatch, tmp_path):
    data = archive_bytes({'tokens.txt': b'tokens', 'encoder.int8.onnx': b'model'})
    configure(downloader, monkeypatch, data)
    monkeypatch.setenv('RBNX_GH_MIRROR', 'https://mirror.example/')
    calls = []
    def run(cmd, **kwargs):
        calls.append(cmd[-1])
        if len(calls) < 3:
            raise subprocess.CalledProcessError(22, cmd)
        Path(cmd[cmd.index('-o') + 1]).write_bytes(data)
    monkeypatch.setattr(downloader.subprocess, 'run', run)
    downloader.prepare(tmp_path, 'asr')
    assert calls[0].startswith('https://mirror.example/https://github.com/')
    assert calls[1].startswith('https://mirror.example/https://github.com/')
    assert '?robonix_sha256=' in calls[1]
    assert calls[2].startswith('https://github.com/')


def test_bad_primary_uses_verified_cache_busted_mirror(downloader, monkeypatch, tmp_path, capsys):
    data = archive_bytes({'tokens.txt': b'tokens', 'encoder.int8.onnx': b'model'})
    configure(downloader, monkeypatch, data)
    monkeypatch.setenv('RBNX_GH_MIRROR', 'https://mirror.example/')
    calls = []
    def run(cmd, **kwargs):
        calls.append(cmd[-1])
        Path(cmd[cmd.index('-o') + 1]).write_bytes(b'bad mirror' if len(calls) == 1 else data)
    monkeypatch.setattr(downloader.subprocess, 'run', run)
    downloader.prepare(tmp_path / 'models', 'asr')
    digest = hashlib.sha256(data).hexdigest()
    assert len(calls) == 2
    assert calls[1].startswith(calls[0] + '?') and digest in calls[1]
    error = capsys.readouterr().err
    assert digest in error and hashlib.sha256(b'bad mirror').hexdigest() in error


def test_verified_archive_cache_survives_fresh_build_and_rejects_tampering(downloader, monkeypatch, tmp_path):
    data = archive_bytes({'tokens.txt': b'tokens', 'encoder.int8.onnx': b'model'})
    configure(downloader, monkeypatch, data)
    cache = tmp_path / 'cache'
    monkeypatch.setenv('ROBONIX_MODEL_CACHE_DIR', str(cache))
    download = Mock(side_effect=lambda cmd, **kw: Path(cmd[cmd.index('-o') + 1]).write_bytes(data))
    monkeypatch.setattr(downloader.subprocess, 'run', download)
    downloader.prepare(tmp_path / 'first', 'asr')
    digest = hashlib.sha256(data).hexdigest()
    archive = cache / 'speech' / (digest + '.tar.bz2')
    assert archive.read_bytes() == data
    downloader.prepare(tmp_path / 'second', 'asr')
    assert download.call_count == 1
    archive.write_bytes(b'tampered')
    def fail(cmd, **kwargs):
        Path(cmd[cmd.index('-o') + 1]).write_bytes(b'bad download')
    monkeypatch.setattr(downloader.subprocess, 'run', fail)
    with pytest.raises(RuntimeError):
        downloader.prepare(tmp_path / 'third', 'asr')
    assert not (tmp_path / 'third/test-model').exists()
    monkeypatch.setattr(downloader.subprocess, 'run', download)
    downloader.prepare(tmp_path / 'fourth', 'asr')
    assert archive.read_bytes() == data
    assert not list((cache / 'speech').glob('.download-*'))


def test_kws_uses_official_api_when_mirror_and_direct_download_fail(downloader, monkeypatch, tmp_path):
    data = archive_bytes({'tokens.txt': b'tokens', 'encoder.int8.onnx': b'model'})
    configure(downloader, monkeypatch, data)
    monkeypatch.setitem(downloader.MODELS, 'kws', downloader.MODELS['asr'])
    calls = []
    def run(cmd, **kwargs):
        calls.append(cmd)
        if 'api.github.com' not in cmd[-1]:
            raise subprocess.CalledProcessError(22, cmd)
        assert 'Accept: application/octet-stream' in cmd
        Path(cmd[cmd.index('-o') + 1]).write_bytes(data)
    monkeypatch.setattr(downloader.subprocess, 'run', run)
    downloader.prepare(tmp_path, 'kws')
    assert len(calls) == 3
    assert (tmp_path / 'test-model/encoder.int8.onnx').read_bytes() == b'model'


def test_parallel_builds_share_one_verified_archive(downloader, monkeypatch, tmp_path):
    from concurrent.futures import ThreadPoolExecutor
    import threading
    data = archive_bytes({'tokens.txt': b'tokens', 'encoder.int8.onnx': b'model'})
    configure(downloader, monkeypatch, data)
    monkeypatch.setenv('ROBONIX_MODEL_CACHE_DIR', str(tmp_path / 'cache'))
    calls = []
    def run(cmd, **kwargs):
        calls.append(cmd)
        Path(cmd[cmd.index('-o') + 1]).write_bytes(data)
    monkeypatch.setattr(downloader.subprocess, 'run', run)
    start = threading.Barrier(2)
    def build(name):
        start.wait(timeout=5)
        downloader.prepare(tmp_path / name, 'asr')
    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(build, ['one', 'two']))
    assert len(calls) == 1
    for name in ['one', 'two']:
        assert (tmp_path / name / 'test-model/encoder.int8.onnx').read_bytes() == b'model'


def test_kws_verified_alternate_after_official_sources_fail(downloader, monkeypatch, tmp_path):
    data = archive_bytes({'tokens.txt': b'tokens', 'encoder.int8.onnx': b'model'})
    configure(downloader, monkeypatch, data)
    monkeypatch.setitem(downloader.MODELS, 'kws', downloader.MODELS['asr'])
    calls = []
    def run(cmd, **kwargs):
        calls.append(cmd[-1])
        if not cmd[-1].startswith('https://gh-proxy.org/'):
            raise subprocess.CalledProcessError(28, cmd)
        Path(cmd[cmd.index('-o') + 1]).write_bytes(data)
    monkeypatch.setattr(downloader.subprocess, 'run', run)
    downloader.prepare(tmp_path, 'kws')
    assert len(calls) == 5
    assert (tmp_path / 'test-model/encoder.int8.onnx').read_bytes() == b'model'
