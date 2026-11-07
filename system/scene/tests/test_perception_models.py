# SPDX-License-Identifier: MulanPSL-2.0
"""Both mappers share model selection without importing Torch on TensorRT."""
import sys
from types import SimpleNamespace
from unittest.mock import Mock

from scene_service.ingest.perception_models import create_clip_model, load_perception


def test_factory_selects_the_mapper_model_and_prepared_cache(tmp_path, monkeypatch):
    from scene_service import inference_config

    monkeypatch.setenv("SCENE_CG_INFERENCE", "tensorrt")
    monkeypatch.setenv("SCENE_ONNX_DIR", str(tmp_path))
    monkeypatch.setattr(inference_config, "prepared_models_dir", lambda path: path + "/prepared")
    loader = Mock(return_value=object())
    monkeypatch.setitem(sys.modules, "scene_service.ingest.perception_engines", SimpleNamespace(load_engines=loader))
    result = load_perception(["chair"], backend="dualmap")
    assert result is loader.return_value
    assert loader.call_args.args == (["chair"],)
    assert loader.call_args.kwargs["clip_model"] == "MobileCLIP-S2"
    assert loader.call_args.kwargs["models_dir"] == str(tmp_path / "prepared")
    load_perception(["chair"], backend="concept_graphs")
    assert loader.call_args.kwargs["clip_model"] == "ViT-B-32"


def test_clip_loader_reparameterizes_by_model_capability(tmp_path, monkeypatch):
    model, fused = Mock(), Mock()
    model.eval.return_value = model
    fused.eval.return_value = fused
    model.modules.return_value = iter([SimpleNamespace(reparameterize=lambda: None)])
    create = Mock(return_value=(model, "train", "inference"))
    reparameterize = Mock(return_value=fused)
    monkeypatch.setitem(sys.modules, "open_clip", SimpleNamespace(create_model_and_transforms=create))
    monkeypatch.setitem(sys.modules, "timm.utils", SimpleNamespace(reparameterize_model=reparameterize))
    result, train, inference = create_clip_model("ArchitectureWithFusibleLayers", str(tmp_path / "weights.bin"))
    assert result is fused and (train, inference) == ("train", "inference")
    assert create.call_args.kwargs["precision"] == "fp32"
    reparameterize.assert_called_once_with(model)


def test_torch_factory_preserves_mapper_device_override(monkeypatch):
    monkeypatch.setenv('SCENE_CG_INFERENCE', 'torch')
    create = Mock()
    monkeypatch.setitem(sys.modules, 'scene_service.ingest.perception_torch', SimpleNamespace(TorchPerception=create))
    assert load_perception(['chair'], backend='dualmap', device='cpu') is create.return_value
    assert create.call_args.kwargs['device'] == 'cpu'
    assert create.call_args.args[3] == 'MobileCLIP-S2'


def test_local_weights_reuse_unambiguous_pretrained_transforms(tmp_path, monkeypatch):
    path = tmp_path / 'model.bin'
    path.touch()
    model = Mock()
    model.eval.return_value = model
    model.modules.return_value = iter([])
    create = Mock(return_value=(model, None, 'transform'))
    cfg = {'mean': (0, 0, 0), 'std': (1, 1, 1), 'interpolation': 'bilinear', 'resize_mode': 'shortest'}
    metadata = {'first': cfg, 'second': dict(cfg)}
    monkeypatch.setitem(sys.modules, 'open_clip', SimpleNamespace(
        list_pretrained_tags_by_model=lambda _: list(metadata),
        get_pretrained_cfg=lambda name, tag: metadata[tag], create_model_and_transforms=create,
    ))
    create_clip_model('AnyModel', str(path))
    assert create.call_args.kwargs['image_mean'] == (0, 0, 0)
    assert create.call_args.kwargs['image_std'] == (1, 1, 1)
    assert create.call_args.kwargs['image_interpolation'] == 'bilinear'
    metadata['second'] = dict(cfg, mean=(.5, .5, .5))
    model.modules.return_value = iter([])
    create_clip_model('AnyModel', str(path))
    assert 'image_mean' not in create.call_args.kwargs
