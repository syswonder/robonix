# SPDX-License-Identifier: MulanPSL-2.0
"""Portable CLIP exports preserve model-specific shapes and preprocessing."""
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

torch = pytest.importorskip("torch")
open_clip = pytest.importorskip("open_clip")
pytest.importorskip("onnx")
pytest.importorskip("onnxruntime")

from scene_service.ingest.perception_engines import (
    OrtSess, SimpleTokenizer, clip_crop_crops, onnx_hash12,
)
from scene_service.inference_config import model_metadata


@pytest.mark.parametrize("profile,size,width,context", [
    ("lite", 32, 32, 77), ("full", 48, 64, 64),
])
def test_real_export_roundtrip_with_different_clip_shapes(tmp_path, monkeypatch, profile, size, width, context):
    name = f"SceneTestTiny{size}"
    config = {"embed_dim": width,
              "vision_cfg": {"image_size": size, "layers": 1, "width": 64, "patch_size": 16},
              "text_cfg": {"context_length": context, "vocab_size": 49408,
                           "width": 64, "heads": 1, "layers": 1}}
    config_file = tmp_path / f"{name}.json"
    config_file.write_text(json.dumps(config))
    open_clip.add_model_config(config_file)
    model, _, preprocess = open_clip.create_model_and_transforms(name)
    weights = tmp_path / "clip.bin"
    torch.save(model.state_dict(), weights)
    output = tmp_path / "onnx"
    source = Path(__file__).resolve().parents[1] / "scripts/export_models.py"
    spec = importlib.util.spec_from_file_location("scene_export_models", source)
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    monkeypatch.setattr(sys, "argv", [str(source), "--only", "clip", "--profile", profile,
                                     "--clip-model", name, "--clip-bin", str(weights),
                                     "--out", str(output)])
    previous_fastpath = torch.backends.mha.get_fastpath_enabled()
    try:
        exporter.main()
    finally:
        torch.backends.mha.set_fastpath_enabled(previous_fastpath)
    exporter.update_metadata(sam_image_size=1024)
    metadata = model_metadata(output, profile=profile, clip_model=name)
    assert metadata["clip_image_size"] == size
    assert metadata["clip_feature_dim"] == width
    assert metadata["clip_context_length"] == context
    for tower, input_key, output_key in (("visual", "images", "image_feats"),
                                         ("text", "tokens", "text_feats")):
        graph = output / f"clip-{tower}.onnx"
        assert graph.with_suffix(".onnx.data").stat().st_size > 0
        assert graph.stat().st_mode & 0o444 == 0o444
        assert graph.with_suffix(".onnx.data").stat().st_mode & 0o444 == 0o444
        with np.load(output / f"clip-{tower}.npz") as reference:
            got = OrtSess(str(graph)).run({input_key: reference[input_key]})[output_key]
            np.testing.assert_allclose(got, reference["ref"], atol=2e-5, rtol=2e-4)
        assert len(onnx_hash12(str(graph))) == 12
    texts = ["a chair", "a white table", "\u684c\u5b50\u65c1\u7684\u4e00\u628a\u6905\u5b50", "chair " * 100]
    tokens = SimpleTokenizer()(texts, context_length=context)
    np.testing.assert_array_equal(tokens, open_clip.get_tokenizer(name)(texts).numpy())
    from scene_service.ingest.perception_images import clip_crops

    image = np.random.default_rng(0).integers(0, 256, (65, 91, 3), dtype=np.uint8)
    boxes = np.array([[1, 2, 55, 64]], dtype=np.float32)
    expected = np.stack([preprocess(crop).numpy() for crop in clip_crops(image, boxes)])
    actual = clip_crop_crops(image, boxes, image_size=size,
                             mean=metadata["clip_mean"], std=metadata["clip_std"])
    np.testing.assert_array_equal(actual, expected)
    from scene_service.ingest.perception_torch import TorchPerception

    adapter = TorchPerception.__new__(TorchPerception)
    adapter.clip = model.eval()
    adapter.preprocess = preprocess
    adapter.device = "cpu"
    assert adapter.clip_feats(image, np.empty((0, 4))).shape == (0, width)


@pytest.mark.parametrize("size,width", [(32, 32), (48, 64)])
def test_shared_sam_export_uses_model_prompt_shapes(tmp_path, monkeypatch, size, width):
    ultralytics = pytest.importorskip("ultralytics")
    pytest.importorskip("onnxscript")
    from types import SimpleNamespace
    from ultralytics.models.sam.modules.encoders import ImageEncoderViT, PromptEncoder
    from ultralytics.models.sam.modules.decoders import MaskDecoder
    from ultralytics.models.sam.modules.transformer import TwoWayTransformer

    model = torch.nn.Module()
    model.image_encoder = ImageEncoderViT(img_size=1024, patch_size=16, embed_dim=64,
                                          depth=1, num_heads=4, out_chans=width,
                                          use_rel_pos=True, global_attn_indexes=(0,))
    # Ultralytics rescales the pretrained 1024-pixel positional grid at runtime.
    model.image_encoder.img_size = size
    model.prompt_encoder = PromptEncoder(width, (size // 16, size // 16), (size, size), 16)
    model.mask_decoder = MaskDecoder(width, TwoWayTransformer(1, width, 4, width * 2),
                                     iou_head_hidden_dim=width)
    monkeypatch.setattr(ultralytics, "SAM", lambda _: SimpleNamespace(model=model))
    source = Path(__file__).resolve().parents[1] / "scripts/export_models.py"
    spec = importlib.util.spec_from_file_location("scene_export_sam", source)
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    monkeypatch.setattr(exporter, "OUT", str(tmp_path))
    exporter.export_sam("sam_l.pt")
    exporter.export_sam_decoder("sam_l.pt")
    assert json.loads((tmp_path / "model-config.json").read_text())["sam_image_size"] == size
    with np.load(tmp_path / "sam-encoder.npz") as reference:
        output = OrtSess(str(tmp_path / "sam-encoder.onnx")).run({"image": reference["image"]})
        np.testing.assert_allclose(output["image_embed"], reference["ref"], atol=2e-5, rtol=2e-4)
    with np.load(tmp_path / "sam-decoder.npz") as reference:
        session = OrtSess(str(tmp_path / "sam-decoder.onnx"))
        for count in (2, 5):
            output = session.run({"image_embeddings": reference["enc_ref"],
                                  "boxes": reference[f"boxes{count}"]})
            np.testing.assert_allclose(output["masks"], reference[f"masks_ref{count}"],
                                       atol=2e-5, rtol=2e-4)
            assert output["iou"].shape == (count, model.mask_decoder.num_mask_tokens)
            np.testing.assert_allclose(output["iou"], reference[f"iou_ref{count}"], atol=2e-5, rtol=2e-4)
            for row in range(count):
                single = session.run({"image_embeddings": reference["enc_ref"],
                                      "boxes": reference[f"boxes{count}"][row:row + 1]})
                np.testing.assert_allclose(single["masks"], reference[f"masks_ref{count}"][row:row + 1],
                                           atol=2e-5, rtol=2e-4)
                np.testing.assert_allclose(single["iou"], reference[f"iou_ref{count}"][row:row + 1],
                                           atol=2e-5, rtol=2e-4)


def test_fused_attention_exports_portably_against_the_original_model(tmp_path, monkeypatch):
    from timm.models.fastvit import Attention
    from scene_service.ingest import perception_models

    class Clip(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.visual = torch.nn.Sequential(
                torch.nn.Conv2d(3, 32, 1), Attention(32, head_dim=16),
                torch.nn.AdaptiveAvgPool2d(1), torch.nn.Flatten(), torch.nn.Linear(32, 16),
            )
            self.text = torch.nn.Embedding(49408, 16)

        def encode_text(self, tokens):
            return self.text(tokens).mean(dim=1)

    model = Clip().eval()
    monkeypatch.setattr(perception_models, 'create_clip_model', lambda *args: (model, None, None))
    tokenizer = open_clip.get_tokenizer('ViT-B-32')
    monkeypatch.setattr(open_clip, 'get_tokenizer', lambda _: tokenizer)
    monkeypatch.setattr(open_clip, 'get_model_preprocess_cfg', lambda _: {
        'size': 8, 'mean': (0., 0., 0.), 'std': (1., 1., 1.), 'interpolation': 'bilinear',
    })
    source = Path(__file__).resolve().parents[1] / 'scripts/export_models.py'
    spec = importlib.util.spec_from_file_location('export_attention', source)
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    monkeypatch.setattr(exporter, 'OUT', str(tmp_path))
    exporter.export_clip('unused.bin', 'TestAttention')
    reference = np.load(tmp_path / 'clip-visual.npz')
    actual = OrtSess(str(tmp_path / 'clip-visual.onnx')).run({'images': reference['images']})['image_feats']
    np.testing.assert_allclose(actual, reference['ref'], atol=2e-6, rtol=2e-5)
    assert reference['images'][0].max() == 0
    assert reference['images'][1].min() == 1


def test_instance_segmenter_exports_dynamic_aspect_ratios(tmp_path, monkeypatch):
    from types import SimpleNamespace
    import ultralytics
    from ultralytics.nn.tasks import SegmentationModel
    from scripts import export_models

    model = SegmentationModel('yolov8n-seg.yaml', nc=1, verbose=False)
    monkeypatch.setattr(ultralytics, 'FastSAM', lambda _: SimpleNamespace(model=model))
    monkeypatch.setattr(export_models, 'OUT', str(tmp_path))
    export_models.export_fastsam('local-model.pt')
    session = OrtSess(str(tmp_path / 'fastsam.onnx'))
    with np.load(tmp_path / 'fastsam.npz') as data:
        for suffix in ('', '1'):
            outputs = session.run({'images': data['images' + suffix]})
            reference = np.concatenate([data['boxes' + suffix], data['scores' + suffix], data['coefficients' + suffix]], axis=1)
            np.testing.assert_allclose(outputs['predictions'], reference, atol=2e-3, rtol=1e-4)
            np.testing.assert_allclose(outputs['prototypes'], data['prototypes' + suffix], atol=2e-5, rtol=1e-4)


def test_replica_text_export_preserves_its_separate_feature_space(tmp_path, monkeypatch):
    from scripts import export_models
    from scene_service.ingest import perception_models

    model = open_clip.model.CLIP(embed_dim=32, vision_cfg=dict(image_size=32, layers=1, width=64, patch_size=16),
                                text_cfg=dict(context_length=77, vocab_size=49408, width=64, heads=1, layers=1)).eval()
    monkeypatch.setattr(perception_models, 'create_clip_model', lambda *args: (model, None, None))
    monkeypatch.setattr(export_models, 'OUT', str(tmp_path))
    export_models.export_replica('reference-checkpoint.bin')
    metadata = json.loads((tmp_path / 'model-config.json').read_text())
    assert metadata['replica_feature_dim'] == 32
    assert metadata['replica_clip_model'] == 'ViT-B-32'
    session = OrtSess(str(tmp_path / 'replica-text.onnx'))
    with np.load(tmp_path / 'replica-text.npz') as data:
        np.testing.assert_allclose(session.run({'tokens': data['tokens']})['text_feats'], data['ref'], atol=2e-5, rtol=1e-4)
