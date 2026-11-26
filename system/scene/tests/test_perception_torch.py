# SPDX-License-Identifier: MulanPSL-2.0
"""Torch adapter returns the same NumPy shapes consumed by fusion."""
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

torch = pytest.importorskip('torch')
from scene_service.ingest.perception_torch import TorchPerception


def test_detection_and_segmentation_outputs():
    bundle = TorchPerception.__new__(TorchPerception)
    bundle.device = 'cpu'
    from ultralytics.engine.results import Boxes
    boxes = Boxes(torch.tensor([[1., 2., 8., 9., .75, 2.]]), (12, 12))
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
    def encode_image(batch):
        assert not torch.backends.cudnn.allow_tf32
        assert torch.backends.cudnn.enabled
        assert torch.backends.cudnn.benchmark
        assert torch.backends.cudnn.deterministic
        return torch.ones((len(batch), 512))
    bundle.clip.encode_image.side_effect = encode_image
    bundle.clip.encode_text.side_effect = lambda tokens: torch.ones((len(tokens), 512))
    bundle.tokenizer = lambda texts: torch.zeros((len(texts), 77), dtype=torch.int64)
    with torch.backends.cudnn.flags(enabled=True, benchmark=True, deterministic=True, allow_tf32=True):
        feats = bundle.clip_feats(np.zeros((12, 12, 3), np.uint8),
                                  np.tile([1, 1, 9, 9], (9, 1)))
        assert torch.backends.cudnn.allow_tf32
    assert feats.shape == (9, 512)
    assert [len(call.args[0]) for call in bundle.clip.encode_image.call_args_list] == [8, 1]
    np.testing.assert_allclose(np.linalg.norm(feats, axis=1), 1, atol=1e-6)
    text = bundle.embed_text(['chair', 'table'])
    assert text.shape == (2, 512)
    np.testing.assert_allclose(np.linalg.norm(text, axis=1), 1, atol=1e-6)


def test_fixed_yolo_vocabulary_does_not_retain_its_text_encoder():
    import open_clip
    import ultralytics
    yolo = Mock()
    features = torch.ones((1, 2, 512))
    yolo.model.txt_feats = features
    yolo.model.clip_model = torch.nn.Linear(2, 2)
    clip = torch.nn.Linear(2, 2)
    with patch.object(ultralytics, "YOLO", return_value=yolo), \
         patch.object(ultralytics, "SAM"), \
         patch.object(open_clip, "create_model_and_transforms", return_value=(clip, None, Mock())), \
         patch.object(open_clip, "get_tokenizer"):
        bundle = TorchPerception(["chair", "plant"], "yolo.pt", "sam.pt", "ViT-B-32", "clip.bin")
    assert bundle.yolo.model.clip_model is None
    assert bundle.yolo.model.txt_feats is features
    yolo.set_classes.assert_called_once_with(["chair", "plant"])


def test_clip_precision_setting_is_restored_after_forward_failure():
    bundle = TorchPerception.__new__(TorchPerception)
    bundle.device = 'cpu'
    bundle.preprocess = lambda image: torch.ones((3, 4, 4))
    bundle.clip = Mock()
    bundle.clip.encode_image.side_effect = RuntimeError('forward failed')
    with torch.backends.cudnn.flags(allow_tf32=True):
        with pytest.raises(RuntimeError, match='forward failed'):
            bundle.clip_feats(np.zeros((12, 12, 3), np.uint8), [[1, 1, 9, 9]])
        assert torch.backends.cudnn.allow_tf32
