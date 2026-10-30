# SPDX-License-Identifier: MulanPSL-2.0
"""Resource lifetimes, cached scoring and engine identity without CUDA."""
import sys
from types import SimpleNamespace
from unittest.mock import Mock
import weakref

import numpy as np
import pytest

from scene_service.ingest import perception_engines as pe


def test_device_buffers_reused_grown_and_released(monkeypatch):
    allocations = []

    class Buffer:
        def __init__(self, nbytes):
            self.nbytes = nbytes
            allocations.append(nbytes)

    monkeypatch.setattr(pe, "DevBuf", Buffer)
    runner = object.__new__(pe.TrtRunner)
    runner.bufs = {}
    first = runner._buffer("images", 16)
    old = weakref.ref(first)
    assert runner._buffer("images", 8) is first
    del first
    grown = runner._buffer("images", 32)
    assert old() is None
    assert grown.nbytes == 32
    assert allocations == [16, 32]
    runner.close()
    assert runner.bufs == {}


def test_invalid_shapes_and_failed_execution_raise(monkeypatch):
    monkeypatch.setitem(sys.modules, "tensorrt", SimpleNamespace())
    runner = object.__new__(pe.TrtRunner)
    runner.ctx = SimpleNamespace(set_input_shape=lambda *args: False)
    with pytest.raises(ValueError, match="Invalid input shape"):
        runner.run({"images": np.zeros((1, 3), np.float32)})
    runner.eng = SimpleNamespace(num_io_tensors=0)
    runner.ctx = SimpleNamespace(execute_async_v3=lambda stream: False)
    monkeypatch.setattr(pe, "_cudart", lambda: SimpleNamespace(cudaDeviceSynchronize=lambda: (0,)))
    with pytest.raises(RuntimeError, match="execution failed"):
        runner.run({})


def test_engine_identity_tracks_model_and_build_settings(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, "tensorrt", SimpleNamespace(__version__="test"))
    monkeypatch.setattr(pe, "compute_cap", lambda: "87")
    onnx = tmp_path / "sam-decoder.onnx"
    onnx.write_bytes(b"model")
    first = pe.engine_file("sam-decoder", str(tmp_path), str(tmp_path))
    monkeypatch.setattr(pe, "WORKSPACE", pe.WORKSPACE // 2)
    second = pe.engine_file("sam-decoder", str(tmp_path), str(tmp_path))
    assert second != first
    onnx.write_bytes(b"new model")
    assert pe.engine_file("sam-decoder", str(tmp_path), str(tmp_path)) != second


def test_load_releases_startup_text_session_before_query_tower(tmp_path, monkeypatch):
    monkeypatch.setattr(pe, "_TOKENIZER", lambda classes: np.zeros((len(classes), 77), np.int64))
    for name in pe.TRT_MODELS + pe.ORT_MODELS:
        (tmp_path / f"{name}.onnx").write_bytes(b"model")
    np.savez(tmp_path / "yolo-world-cut.npz", scales=[1], biases=[0], ns=[1])
    monkeypatch.setattr(pe, "engine_file", lambda *args: str(tmp_path / "sam-decoder.onnx"))
    monkeypatch.setattr(pe, "TrtRunner", lambda *args: SimpleNamespace(
        device_memory_bytes=0, close=lambda: None))
    references = []

    class Text:
        def __init__(self, path):
            if references:
                assert references[0]() is None
            references.append(weakref.ref(self))

        def run(self, feeds):
            return {"txt_feats": np.ones((len(feeds["tokens"]), 512), np.float32)}

    monkeypatch.setattr(pe, "OrtSess", Text)
    bundle = pe.load_engines(["chair"], models_dir=str(tmp_path))
    assert references[0]() is None
    assert bundle.e.shape == (1, 512)
    assert references[1]() is bundle.clip_text
    bundle.close()
    assert references[1]() is None


def test_yolo_scores_preserve_features_and_reuse_class_constants(monkeypatch):
    features = np.array([[[3., 0., 0.], [0., 4., 0.]]], np.float32)
    original = features.copy()
    embeddings = np.array([[2., 0.], [0., 3.]], np.float32)
    expected = 1 / (1 + np.exp(-np.array([[3., 1., -1.], [1., 3., -1.]], np.float32)))
    runner = SimpleNamespace(run=lambda feeds: {"boxes": np.zeros((1, 4, 3)), "feats": features})
    bundle = pe.PerceptionEngines(["chair", "table"], runner, None, None, embeddings,
                                  None, None, [2., 3.], [1., -1.], [2, 1])
    monkeypatch.setattr(pe, "yolo_preprocess", lambda image: image)
    nms = Mock(return_value=np.empty((0, 6), np.float32))
    monkeypatch.setattr(pe, "yolo_nms", nms)
    monkeypatch.setattr(pe, "normalize_rows", lambda values: pytest.fail("repeated class normalization"))
    for _ in range(2):
        assert bundle.detect(np.zeros((1, 1, 3)), 0.3).shape == (0, 6)
        np.testing.assert_allclose(nms.call_args.args[1], expected)
    np.testing.assert_array_equal(features, original)
    np.testing.assert_array_equal(bundle.e, embeddings)
