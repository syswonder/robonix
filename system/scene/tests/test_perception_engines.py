# SPDX-License-Identifier: MulanPSL-2.0
"""Resource lifetimes, cached scoring and engine identity without CUDA."""
import sys
from types import SimpleNamespace
from unittest.mock import Mock
import weakref

import numpy as np
import pytest

from scene_service.ingest import perception_engines as pe


def test_yolo_nms_keeps_only_the_best_class_per_anchor():
    boxes = np.array([[10, 10, 4, 4], [30, 30, 4, 4]], np.float32)
    scores = np.array([[0.9, 0.1], [0.8, 0.7]], np.float32)
    detections = pe.yolo_nms(boxes, scores, conf=0.25, max_det=2)
    np.testing.assert_allclose(detections, [
        [8, 8, 12, 12, 0.9, 0],
        [28, 28, 32, 32, 0.7, 1],
    ])


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
    assert bundle.tokenizer is pe._TOKENIZER
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
                                  None, None, [2., 3.], [1., -1.], [2, 1],
                                  metadata={"clip_context_length": 77})
    monkeypatch.setattr(pe, "yolo_preprocess", lambda image: image)
    nms = Mock(return_value=np.empty((0, 6), np.float32))
    monkeypatch.setattr(pe, "yolo_nms", nms)
    monkeypatch.setattr(pe, "normalize_rows", lambda values: pytest.fail("repeated class normalization"))
    for _ in range(2):
        assert bundle.detect(np.zeros((1, 1, 3)), 0.3).shape == (0, 6)
        np.testing.assert_allclose(nms.call_args.args[1], expected)
    np.testing.assert_array_equal(features, original)
    np.testing.assert_array_equal(bundle.e, embeddings)


@pytest.mark.parametrize('count', [0, 33, 64])
def test_segment_preserves_all_boxes_with_single_box_decoder(count):
    engines = object.__new__(pe.PerceptionEngines)
    embedding = np.zeros((1, 256, 64, 64), np.float32)
    engines.metadata = {"sam_image_size": 1024}
    engines.sam_enc = SimpleNamespace(run=Mock(return_value={'image_embed': embedding}))
    calls = []

    def decode(inputs):
        assert inputs['image_embeddings'] is embedding
        assert inputs['boxes'].shape == (1, 4)
        index = len(calls)
        calls.append(inputs['boxes'].copy())
        return {'masks': np.full((1, 1, 256, 256), 1 if index % 2 else -1, np.float32)}

    engines.sam_dec = SimpleNamespace(run=Mock(side_effect=decode))
    image = np.zeros((12, 16, 3), np.uint8)
    boxes = np.tile(np.array([[1, 2, 8, 10]], np.float32), (count, 1))
    masks = engines.segment(image, boxes)
    assert masks.shape == (count, 12, 16)
    assert masks.dtype == np.bool_
    assert np.array_equal(masks[:, 0, 0], np.arange(count) % 2 == 1)
    assert len(calls) == count
    assert engines.sam_enc.run.call_count == bool(count)


@pytest.mark.parametrize("width", [512, 1024])
def test_clip_uses_exported_dimensions_and_preprocessing(width):
    engines = object.__new__(pe.PerceptionEngines)
    engines.metadata = {"clip_image_size": 32, "clip_batch_size": 2,
                        "clip_feature_dim": width, "clip_mean": [0.5] * 3,
                        "clip_std": [0.25] * 3}
    batches = []

    def encode(feeds):
        images = feeds["images"]
        assert images.shape == (2, 3, 32, 32)
        batches.append(images.copy())
        return {"image_feats": np.ones((2, width), np.float32)}

    engines.clip_vis = SimpleNamespace(run=encode)
    image = np.full((24, 24, 3), 255, np.uint8)
    features = engines.clip_feats(image, np.tile([0, 0, 24, 24], (3, 1)))
    assert features.shape == (3, width)
    np.testing.assert_allclose(np.linalg.norm(features, axis=1), 1, atol=1e-6)
    assert len(batches) == 2
    np.testing.assert_array_equal(batches[0], 2)
    np.testing.assert_array_equal(batches[1][1], 0)
    assert engines.clip_feats(image, np.empty((0, 4))).shape == (0, width)


def test_engine_identity_includes_external_tensor_data(tmp_path):
    graph = tmp_path / "clip-visual.onnx"
    graph.write_bytes(b"graph")
    sidecar = tmp_path / "clip-visual.onnx.data"
    sidecar.write_bytes(b"weights")
    first = pe.onnx_hash12(str(graph))
    sidecar.write_bytes(b"changed weights")
    assert pe.onnx_hash12(str(graph)) != first


def test_instance_masks_keep_coefficients_aligned_and_discard_empty_masks():
    # Confidence sorting changes anchor order; the negative prototype must be removed.
    predictions = np.array([[[40, 80, 120], [50, 50, 50], [30, 30, 30], [30, 30, 30],
                             [.7, .9, .8], [1, -1, 1]]], dtype=np.float32)
    boxes, masks = pe.instance_masks(predictions, np.ones((1, 1, 25, 40), np.float32),
                                     (100, 160), (100, 160), .25)
    np.testing.assert_allclose(boxes[:, 4], [.8, .7])
    assert masks.shape == (2, 100, 160)
    assert masks[0, 50, 120] and not masks[0, 50, 40]
    assert masks[1, 50, 40] and not masks[1, 50, 120]
    empty_boxes, empty_masks = pe.instance_masks(predictions, np.ones((1, 1, 25, 40), np.float32),
                                                 (100, 160), (100, 160), .99)
    assert empty_boxes.shape == (0, 6) and empty_masks.shape == (0, 100, 160)


@pytest.mark.parametrize('shape', [(61, 47), (47, 61)])
def test_instance_segmentation_matches_ultralytics_native_masks(shape):
    torch = pytest.importorskip('torch')
    pytest.importorskip('ultralytics')
    from ultralytics.data.augment import LetterBox
    from ultralytics.utils import ops
    from ultralytics.utils.nms import non_max_suppression
    from ultralytics.models.fastsam.utils import adjust_bboxes_to_image_border

    rng = np.random.default_rng(42)
    image = rng.integers(0, 256, (*shape, 3), dtype=np.uint8)
    inputs = pe.yolo_preprocess(image, 1024, auto=True)
    reference_image = LetterBox((1024, 1024), auto=True, stride=32)(image=image)
    np.testing.assert_array_equal(inputs, reference_image[..., ::-1].transpose(2, 0, 1).astype(np.float32) / 255)
    ih, iw = inputs.shape[1:]
    boxes = np.array([[20 + 100 * (i % 8), 20 + 100 * (i // 8), 70, 70] for i in range(56)], np.float32)
    prediction = np.concatenate([boxes.T, np.linspace(.9, .6, 56)[None], rng.normal(size=(4, 56))]).astype(np.float32)[None]
    proto = rng.normal(size=(1, 4, ih // 4, iw // 4)).astype(np.float32)
    pred = non_max_suppression(torch.from_numpy(prediction.copy()), .25, .9, nc=1)[0]
    pred[:, :4] = ops.scale_boxes((ih, iw), pred[:, :4], shape)
    expected_masks = ops.process_mask_native(torch.from_numpy(proto[0]), pred[:, 6:], pred[:, :4], shape).numpy().astype(bool)
    keep = expected_masks.any(axis=(1, 2))
    pred, expected_masks = pred[keep], expected_masks[keep]
    expected_boxes = adjust_bboxes_to_image_border(pred[:, :4], shape)
    area = (expected_boxes[:, 2] - expected_boxes[:, 0]) * (expected_boxes[:, 3] - expected_boxes[:, 1])
    expected_boxes[area / np.prod(shape) > .9] = torch.tensor([0, 0, shape[1], shape[0]], dtype=torch.float32)
    got, masks = pe.instance_masks(prediction, proto, shape, (ih, iw), .25)
    np.testing.assert_allclose(got, pred[:, :6].numpy(), atol=1e-4)
    np.testing.assert_array_equal(masks, expected_masks)


def test_instance_segmentation_derives_class_count_from_the_prototypes():
    # Two classes followed by one mask coefficient: the class scores are not coefficients.
    predictions = np.array([[[40], [40], [30], [30], [.3], [.8], [1]]], np.float32)
    detections, masks = pe.instance_masks(predictions, np.ones((1, 1, 25, 25), np.float32),
                                         (100, 100), (100, 100), .25)
    assert detections[0, 5] == 1 and masks[0, 40, 40]


def test_box_scaling_reuses_the_rounded_letterbox_padding():
    boxes = np.array([[1.6, 2.6, 6.4, 4.2]], np.float32)
    np.testing.assert_allclose(pe.unletterbox_boxes(boxes, (3, 5), (8, 8)),
                               [[1., 1., 4., 2.]], atol=1e-6)
