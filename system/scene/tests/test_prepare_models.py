# SPDX-License-Identifier: MulanPSL-2.0
"""Preparation publishes only complete, verified model sets."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from unittest.mock import Mock

import pytest

from scene_service.inference_config import inference_backend, prepared_models_dir


def load_prepare():
    path = Path(__file__).resolve().parents[1] / 'scripts/prepare_models.py'
    spec = importlib.util.spec_from_file_location('prepare_models', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_default_and_invalid_selection(monkeypatch):
    monkeypatch.delenv('SCENE_CG_INFERENCE', raising=False)
    assert inference_backend() == 'torch'
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'invalid')
    with pytest.raises(ValueError, match='torch or tensorrt'):
        inference_backend()


def test_prepare_reuses_exports_and_preserves_selection_on_failure(tmp_path, monkeypatch):
    prepare = load_prepare()
    monkeypatch.setenv('SCENE_ENGINE_CACHE_DIR', str(tmp_path / 'engines'))
    key = 'a' * 64
    monkeypatch.setattr(prepare, 'source_fingerprint', lambda _: key)
    calls = []

    def export(cmd, **kwargs):
        calls.append(cmd[cmd.index('--only') + 1])
        (Path(cmd[-1]) / f'{calls[-1]}.onnx').write_bytes(b'onnx')

    monkeypatch.setattr(prepare.subprocess, 'run', export)
    build = Mock()
    monkeypatch.setattr(prepare, 'build_engines', build)
    prepare.prepare(tmp_path)
    assert calls == ['yolo', 'sam', 'clip']
    selected = prepared_models_dir(str(tmp_path))
    assert Path(selected).name == key
    prepare.prepare(tmp_path)
    assert len(calls) == 3
    assert build.call_count == 2

    monkeypatch.setattr(prepare, 'source_fingerprint', lambda _: 'b' * 64)
    build.side_effect = RuntimeError('parity gate failed')
    with pytest.raises(RuntimeError, match='parity gate'):
        prepare.prepare(tmp_path)
    assert prepared_models_dir(str(tmp_path)) == selected
    assert not list(tmp_path.glob('.export-*'))
    assert not (tmp_path / ('b' * 64)).exists()


def test_check_never_imports_torch_or_exports(tmp_path, monkeypatch):
    prepare = load_prepare()
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'tensorrt')
    monkeypatch.setenv('SCENE_ONNX_DIR', str(tmp_path))
    monkeypatch.setenv('SCENE_PROFILE', 'lite')
    monkeypatch.setenv('SCENE_PERCEPTION_BACKEND', 'concept_graphs')
    monkeypatch.delenv('SCENE_CG_FORCE_CPU', raising=False)
    monkeypatch.delenv('SCENE_CLIP_MODEL', raising=False)
    (tmp_path / 'prepared.json').write_text(json.dumps({'export': 'a' * 64}))
    build = Mock()
    monkeypatch.setattr(prepare, 'build_engines', build)
    monkeypatch.setattr(prepare, 'prepare', Mock(side_effect=AssertionError('must not prepare')))
    monkeypatch.setattr(sys, 'argv', ['prepare', '--check'])
    prepare.main()
    build.assert_called_once_with(tmp_path / ('a' * 64), check=True)


def test_check_runs_without_torch_installation(tmp_path):
    import os
    env = os.environ.copy()
    env['SCENE_CG_INFERENCE'] = 'torch'
    # -S removes site-packages, including Torch; the default path needs none.
    subprocess.run([sys.executable, '-S', str(Path(__file__).resolve().parents[1] /
                    'scripts/prepare_models.py'), '--check'], env=env, check=True)


def test_prepare_keeps_previous_generation_across_repeated_builds(tmp_path, monkeypatch):
    prepare = load_prepare()
    models = tmp_path / 'models'
    models.mkdir()
    monkeypatch.setenv('SCENE_ENGINE_CACHE_DIR', str(tmp_path / 'engines'))
    active = ['a' * 64]
    monkeypatch.setattr(prepare, 'source_fingerprint', lambda _: active[0])
    monkeypatch.setattr(prepare.subprocess, 'run', lambda *a, **k: None)

    def build(path, **kwargs):
        cache = kwargs.get('cache_dir') or prepare.engine_cache_dir(str(path))
        cache.mkdir(parents=True, exist_ok=True)
        (cache / 'model.trt').write_bytes(b'engine')

    monkeypatch.setattr(prepare, 'build_engines', build)
    prepare.prepare(models)
    active[0] = 'b' * 64
    prepare.prepare(models)
    prepare.prepare(models)
    assert (models / ('a' * 64)).is_dir()
    active[0] = 'c' * 64
    prepare.prepare(models)
    assert not (models / ('a' * 64)).exists()
    assert not (tmp_path / 'engines' / ('a' * 64)).exists()
    assert (models / ('b' * 64)).is_dir()
    assert (models / ('c' * 64)).is_dir()


@pytest.mark.parametrize('setting,value', [
    ('SCENE_PROFILE', 'full'), ('SCENE_PERCEPTION_BACKEND', 'dualmap'),
    ('SCENE_CG_FORCE_CPU', '1'), ('SCENE_CLIP_MODEL', 'ViT-H-14'),
])
def test_unsupported_tensorrt_config_fails_before_preparation(monkeypatch, setting, value):
    prepare = load_prepare()
    for name in ('SCENE_PROFILE', 'SCENE_PERCEPTION_BACKEND', 'SCENE_CG_FORCE_CPU', 'SCENE_CLIP_MODEL'):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'tensorrt')
    monkeypatch.setenv(setting, value)
    monkeypatch.setattr(prepare, 'prepare', Mock(side_effect=AssertionError('must not prepare')))
    monkeypatch.setattr(sys, 'argv', ['prepare'])
    with pytest.raises(ValueError, match='TensorRT'):
        prepare.main()


def test_packaged_onnx_compiles_on_target_without_reexporting(tmp_path, monkeypatch):
    prepare = load_prepare()
    baked = tmp_path / 'baked'
    baked.mkdir()
    (baked / 'sam.onnx').write_bytes(b'portable model')
    (baked / 'sam.npz').write_bytes(b'numerical references')
    monkeypatch.setattr(prepare, 'Path', lambda p: baked if str(p) == '/opt/models/onnx' else Path(p))
    monkeypatch.setattr(prepare, 'source_fingerprint', lambda _: 'a' * 64)
    monkeypatch.setattr(prepare.subprocess, 'run', Mock(side_effect=AssertionError('must not export')))
    builds = []
    def build(directory, **kwargs):
        assert (directory / 'sam.onnx').read_bytes() == b'portable model'
        assert (directory / 'sam.npz').read_bytes() == b'numerical references'
        builds.append(directory)
    monkeypatch.setattr(prepare, 'build_engines', build)
    models = tmp_path / 'models'
    models.mkdir()
    prepare.prepare(models)
    prepare.prepare(models)
    assert len(builds) == 2
    assert json.loads((models / 'prepared.json').read_text())['export'] == 'a' * 64
