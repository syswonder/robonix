# SPDX-License-Identifier: MulanPSL-2.0
"""Only verified engines become reusable cache entries."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


@pytest.mark.parametrize("outcome", ["pass", "fail", "raise"])
def test_engine_is_published_only_after_gate(tmp_path, monkeypatch, outcome):
    monkeypatch.setitem(sys.modules, "tensorrt", SimpleNamespace(__version__="test"))
    path = Path(__file__).resolve().parents[1] / "scripts" / "build_engines.py"
    spec = importlib.util.spec_from_file_location("scene_build_engines", path)
    build = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(build)
    monkeypatch.setattr(build, "SPECS", [{"name": "model", "runtime": "trt"}])
    monkeypatch.setattr(build, "compute_cap", lambda: "87")
    final = tmp_path / "model.trt"
    monkeypatch.setattr(build, "engine_file", lambda *args: str(final))
    (tmp_path / "model.onnx").write_bytes(b"onnx")
    np.savez(tmp_path / "model.npz", ref=np.ones(1))
    monkeypatch.setattr(sys, "argv", ["build", "--models-dir", str(tmp_path), "--cache-dir", str(tmp_path)])

    def compile_engine(onnx_path, candidate, cache_dir, data):
        assert Path(candidate) != final
        Path(candidate).write_bytes(b"engine")
        return 0

    monkeypatch.setattr(build, "build_trt_engine", compile_engine)
    monkeypatch.setattr(build, "TrtRunner", lambda candidate: SimpleNamespace(close=lambda: None))

    def gate(*args):
        assert not final.exists()
        if outcome == "raise":
            raise RuntimeError("interrupted validation")
        return outcome == "pass"

    monkeypatch.setattr(build, "run_gate", gate)
    assert build.main() == (0 if outcome == "pass" else 1)
    assert final.exists() == (outcome == "pass")
    assert not list(tmp_path.glob(".model.*.trt"))


@pytest.mark.parametrize("cached", [False, True])
def test_check_never_builds_engines(tmp_path, monkeypatch, cached):
    from unittest.mock import Mock

    monkeypatch.setitem(sys.modules, "tensorrt", SimpleNamespace(__version__="test"))
    path = Path(__file__).resolve().parents[1] / "scripts" / "build_engines.py"
    spec = importlib.util.spec_from_file_location("scene_build_engines", path)
    build = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(build)
    monkeypatch.setattr(build, "SPECS", [{"name": "model", "runtime": "trt"}])
    monkeypatch.setattr(build, "compute_cap", lambda: "87")
    final = tmp_path / "model.trt"
    if cached:
        final.write_bytes(b"verified engine")
    (tmp_path / "model.onnx").write_bytes(b"onnx")
    np.savez(tmp_path / "model.npz", ref=np.ones(1))
    monkeypatch.setattr(build, "engine_file", lambda *args: str(final))
    compile_engine = Mock(side_effect=AssertionError("startup must not compile"))
    monkeypatch.setattr(build, "build_trt_engine", compile_engine)
    monkeypatch.setattr(sys, "argv", ["build", "--models-dir", str(tmp_path),
                                      "--cache-dir", str(tmp_path), "--check"])
    assert build.main() == (0 if cached else 1)
    compile_engine.assert_not_called()
    assert final.exists() == cached


def test_embedding_gate_checks_each_row_instead_of_hiding_a_bad_small_vector(monkeypatch):
    monkeypatch.setitem(sys.modules, 'tensorrt', SimpleNamespace(__version__='test'))
    path = Path(__file__).resolve().parents[1] / 'scripts/build_engines.py'
    spec = importlib.util.spec_from_file_location('scene_gate', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    reference = np.array([[1000., 0.], [1., 0.]])
    actual = np.array([[1000., 0.], [0., 1.]])
    assert module.check('features', actual, reference, 'emb').startswith('FAIL')
    assert module.check('features', reference, reference, 'emb').startswith('OK')


def test_instance_gate_does_not_hide_score_errors_behind_large_box_coordinates(monkeypatch):
    monkeypatch.setitem(sys.modules, 'tensorrt', SimpleNamespace(__version__='test'))
    path = Path(__file__).resolve().parents[1] / 'scripts/build_engines.py'
    spec = importlib.util.spec_from_file_location('instance_gate', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    candidate = next(s for s in module.SPECS if s['name'] == 'fastsam')
    gate = dict(candidate, gates=candidate['gates'][:1])
    reference = dict(images=np.ones((1, 3, 32, 32), np.float32), boxes=np.full((1, 4, 2), 10000., np.float32),
                     scores=np.ones((1, 1, 2), np.float32), coefficients=np.ones((1, 1, 2), np.float32),
                     prototypes=np.ones((1, 1, 8, 8), np.float32))
    outputs = dict(predictions=np.concatenate([reference['boxes'], reference['scores'] * .8, reference['coefficients']], axis=1),
                   prototypes=reference['prototypes'])
    assert not module.run_gate(gate, reference, SimpleNamespace(run=lambda _: outputs))


def test_decoder_quality_gate_checks_scalar_magnitude(monkeypatch):
    monkeypatch.setitem(sys.modules, 'tensorrt', SimpleNamespace(__version__='test'))
    path = Path(__file__).resolve().parents[1] / 'scripts/build_engines.py'
    spec = importlib.util.spec_from_file_location('decoder_gate', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    candidate = next(s for s in module.SPECS if s['name'] == 'sam-decoder')
    gate = dict(candidate, gates=candidate['gates'][:1])
    reference = dict(enc_ref=np.ones((1, 2, 2, 2), np.float32), boxes2=np.ones((1, 4), np.float32),
                     masks_ref2=np.ones((1, 1, 2, 2), np.float32), iou_ref2=np.array([[.75]], np.float32))
    outputs = dict(masks=reference['masks_ref2'], iou=np.array([[.5]], np.float32))
    assert not module.run_gate(gate, reference, SimpleNamespace(run=lambda _: outputs))
    outputs['iou'] = reference['iou_ref2']
    assert module.run_gate(gate, reference, SimpleNamespace(run=lambda _: outputs))
