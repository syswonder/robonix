# SPDX-License-Identifier: MulanPSL-2.0
"""Torch adapter returns the same NumPy shapes consumed by fusion."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

torch = pytest.importorskip('torch')
from scene_service.ingest.perception_torch import TorchPerception


def test_detection_and_segmentation_outputs():
    bundle = TorchPerception.__new__(TorchPerception)
    bundle.device = 'cpu'
    boxes = SimpleNamespace(xyxy=torch.tensor([[1., 2., 8., 9.]]),
                            conf=torch.tensor([.75]), cls=torch.tensor([2.]))
    bundle.yolo = Mock()
    bundle.yolo.predict.return_value = [SimpleNamespace(boxes=boxes)]
    image = np.zeros((12, 12, 3), dtype=np.uint8)
    dets = bundle.detect(image, .3)
    np.testing.assert_allclose(dets, [[1, 2, 8, 9, .75, 2]])
    bundle.sam = Mock()
    bundle.sam.predict.return_value = [SimpleNamespace(masks=SimpleNamespace(
        data=torch.ones((1, 12, 12))))]
    masks = bundle.segment(image, dets[:, :4])
    assert masks.shape == (1, 12, 12) and masks.dtype == bool and masks.all()
    bundle.sam.predict.return_value = []
    assert bundle.segment(image, dets[:, :4]).shape == (0, 12, 12)


def test_clip_batches_and_normalizes_images_and_text():
    bundle = TorchPerception.__new__(TorchPerception)
    bundle.device = 'cpu'
    bundle.preprocess = lambda image: torch.ones((3, 4, 4))
    bundle.clip = Mock()
    bundle.clip.encode_image.side_effect = lambda batch: torch.ones((len(batch), 512))
    bundle.clip.encode_text.side_effect = lambda tokens: torch.ones((len(tokens), 512))
    bundle.tokenizer = lambda texts: torch.zeros((len(texts), 77), dtype=torch.int64)
    feats = bundle.clip_feats(np.zeros((12, 12, 3), np.uint8),
                              np.tile([1, 1, 9, 9], (9, 1)))
    assert feats.shape == (9, 512)
    assert [len(call.args[0]) for call in bundle.clip.encode_image.call_args_list] == [8, 1]
    np.testing.assert_allclose(np.linalg.norm(feats, axis=1), 1, atol=1e-6)
    text = bundle.embed_text(['chair', 'table'])
    assert text.shape == (2, 512)
    np.testing.assert_allclose(np.linalg.norm(text, axis=1), 1, atol=1e-6)
