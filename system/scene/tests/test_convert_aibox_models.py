# SPDX-License-Identifier: MulanPSL-2.0
"""AIBOX model conversion assembles deployable, fixed-shape artifacts."""
import importlib.util
from argparse import Namespace
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np


class _FakeTensor:
    """Stands in for a torch.Tensor: carries the constructor value, reports
    it back through ``numpy()`` like the real thing."""

    def __init__(self, value):
        self.value = value

    @property
    def shape(self):
        return np.asarray(self.value).shape

    def numpy(self):
        return np.asarray(self.value, dtype=np.float32)


class _NullCtx:
    """Stand-in for ``torch.no_grad()``."""

    def __enter__(self):
        return None

    def __exit__(self, *args):
        return None


def load_converter():
    path = Path(__file__).resolve().parents[1] / "scripts/convert_aibox_models.py"
    spec = importlib.util.spec_from_file_location("convert_aibox_models", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_mobilesam_export_uses_deployed_input_size(tmp_path, monkeypatch):
    converter = load_converter()
    exports = []
    refs = []

    class Encoder:
        def eval(self):
            return self

    encoder = Encoder()
    torch = ModuleType("torch")
    torch.zeros = lambda *shape: shape
    torch.onnx = SimpleNamespace(
        export=lambda model, dummy, path, **kwargs: exports.append((model, dummy, path, kwargs)))
    ultralytics = ModuleType("ultralytics")
    ultralytics.SAM = lambda _: SimpleNamespace(model=SimpleNamespace(image_encoder=encoder))
    monkeypatch.setitem(sys.modules, "torch", torch)
    monkeypatch.setitem(sys.modules, "ultralytics", ultralytics)
    monkeypatch.setattr(converter, "_check_image", lambda _: object())
    monkeypatch.setattr(converter, "_sam_reference",
                        lambda _sam, _image, size: refs.append(size) or {})
    monkeypatch.setattr(converter, "_save_ref", lambda *_args, **_kwargs: None)

    converter._export_mobilesam_encoder(Namespace(
        weights_dir=str(tmp_path), out_dir=str(tmp_path), imgsz=640, opset=12, image="image.jpg"))

    assert exports[0][1] == (1, 3, 1024, 1024)
    assert refs == [1024]


def test_deploy_copies_mobilesam_decoder_pair(tmp_path):
    converter = load_converter()
    artifacts = tmp_path / "artifacts"
    output = tmp_path / "npu"
    artifacts.mkdir()
    classes = tmp_path / "classes.txt"
    classes.write_text("chair\n")
    for name in (
        "yoloworld.rknn", "yoloworld.weight",
        "mobilesam.rknn", "mobilesam.weight",
        "mobilesam_decoder.rknn", "mobilesam_decoder.weight",
        "clip_image.rknn", "clip_image.weight",
    ):
        (artifacts / name).write_bytes(name.encode())

    converter.cmd_deploy(Namespace(
        rknn_dir=str(artifacts), classes_file=str(classes), out_dir=str(output),
        dualmap_repo=None))

    assert (output / "mobilesam_decoder.rknn").read_bytes() == b"mobilesam_decoder.rknn"
    assert (output / "mobilesam_decoder.weight").read_bytes() == b"mobilesam_decoder.weight"
    # The pre-NMS graph bakes its text embeddings; no text npy is deployed.
    assert not (output / "yoloworld_text.npy").exists()


def test_mobilesam_decoder_export_is_static(tmp_path, monkeypatch):
    converter = load_converter()
    exports = []
    refs = {}

    class OnnxModel:
        def __init__(self, model, return_single_mask=False):
            assert model.prompt_encoder.embed_dim == 256
            assert return_single_mask is False  # multimask: runtime reads token 0

        def __call__(self, *args):
            return args[0], args[0], args[0]

    class Sam:
        def __init__(self):
            self.prompt_encoder = SimpleNamespace(embed_dim=256,
                                                  image_embedding_size=(64, 64))

        def eval(self):
            return self

    sam = Sam()
    torch = ModuleType("torch")
    torch.randn = lambda *shape: _FakeTensor(shape)
    torch.zeros = lambda *shape, **_: _FakeTensor(shape)
    torch.tensor = lambda values, **_: _FakeTensor(values)
    torch.from_numpy = _FakeTensor
    torch.no_grad = _NullCtx
    torch.onnx = SimpleNamespace(
        export=lambda model, dummy, path, **kwargs: exports.append((model, dummy, path, kwargs)))
    mobile_sam = ModuleType("mobile_sam")
    mobile_sam.sam_model_registry = {"vit_t": lambda checkpoint: sam}
    utils = ModuleType("mobile_sam.utils")
    onnx_mod = ModuleType("mobile_sam.utils.onnx")
    onnx_mod.SamOnnxModel = OnnxModel
    mobile_sam.utils = utils
    utils.onnx = onnx_mod
    monkeypatch.setitem(sys.modules, "torch", torch)
    monkeypatch.setitem(sys.modules, "mobile_sam", mobile_sam)
    monkeypatch.setitem(sys.modules, "mobile_sam.utils", utils)
    monkeypatch.setitem(sys.modules, "mobile_sam.utils.onnx", onnx_mod)
    # The reference embedding is re-exported through ultralytics' loader; the
    # decoder test stubs it out and only checks the export call shape.
    monkeypatch.setitem(sys.modules, "ultralytics", ModuleType("ultralytics"))
    ultralytics = sys.modules["ultralytics"]
    ultralytics.SAM = lambda _: SimpleNamespace(model=SimpleNamespace(image_encoder=None))
    monkeypatch.setattr(converter, "_check_image", lambda _: np.zeros((480, 640, 3), dtype=np.uint8))
    monkeypatch.setattr(converter, "_sam_reference",
                        lambda _sam, _img, _size: {"embedding": np.zeros((1, 256, 64, 64), np.float32)})
    monkeypatch.setattr(converter, "_save_ref",
                        lambda out_dir, name, **arrays: refs.update({name: arrays}))

    (tmp_path / "mobile_sam.pt").write_bytes(b"weights")
    converter._export_mobilesam_decoder(Namespace(
        weights_dir=str(tmp_path), out_dir=str(tmp_path),
        mobilesam_repo=str(tmp_path), opset=16, image="image.jpg"))

    _, dummy, path, kwargs = exports[0]
    assert path.endswith("mobilesam_decoder.onnx")
    assert kwargs["dynamic_axes"] is None and kwargs["opset_version"] == 16
    assert dummy[1].value == [[[0.0, 0.0], [1024.0, 1024.0]]]  # box corners
    assert dummy[2].value == [[2.0, 3.0]]  # corner labels, num_points=2
    assert dummy[3].value == (1, 1, 256, 256) and dummy[4].value == [0.0]
    assert refs["mobilesam_decoder_torch"]["in1"].shape == (1, 2, 2)
    assert refs["mobilesam_decoder_torch"]["out0"].shape == (1, 256, 64, 64)
