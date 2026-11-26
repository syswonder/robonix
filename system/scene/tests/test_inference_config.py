# SPDX-License-Identifier: MulanPSL-2.0
"""Model selection and portable exports agree before any GPU allocation."""
import json

import pytest

from scene_service.inference_config import model_config, model_metadata, validate_tensorrt_config


@pytest.mark.parametrize("profile,model,sam", [
    ("lite", "ViT-B-32", "mobile_sam.pt"),
    ("full", "ViT-H-14", "sam_l.pt"),
])
def test_profiles_resolve_model_files_without_gpu_libraries(tmp_path, monkeypatch, profile, model, sam):
    for name in ("SCENE_CLIP_MODEL", "SCENE_CLIP_PRETRAINED", "SCENE_MOBILE_SAM_WEIGHTS",
                 "SCENE_YOLO_WORLD_WEIGHTS", "SCENE_MODELS_DIR"):
        monkeypatch.delenv(name, raising=False)
    selected = model_config(profile, tmp_path)
    assert selected["clip_model"] == model
    assert selected["sam"].endswith(sam)
    assert selected["yolo"] == str(tmp_path / "yolov8l-world.pt")
    assert ("full/" in selected["clip_pretrained"]) == (profile == "full")
    validate_tensorrt_config(profile=profile)


def test_explicit_model_options_override_profile_defaults(tmp_path, monkeypatch):
    monkeypatch.setenv("SCENE_MODELS_DIR", str(tmp_path))
    monkeypatch.setenv("SCENE_CLIP_MODEL", "ViT-B-32")
    monkeypatch.setenv("SCENE_MOBILE_SAM_WEIGHTS", "/custom/sam_l.pt")
    selected = model_config("full")
    assert selected["clip_model"] == "ViT-B-32"
    assert selected["sam"] == "/custom/sam_l.pt"
    assert selected["clip_pretrained"] == str(tmp_path / "open_clip_vit_h14_laion2b_s32b_b79k.bin")


def test_legacy_exports_are_never_treated_as_full(tmp_path):
    assert model_metadata(tmp_path, profile="lite", clip_model="ViT-B-32")["clip_feature_dim"] == 512
    with pytest.raises(ValueError, match="Legacy exports"):
        model_metadata(tmp_path, profile="full", clip_model="ViT-H-14")


def test_full_metadata_is_used_and_mismatched_selection_is_rejected(tmp_path):
    config = {"profile": "full", "clip_model": "ViT-H-14", "sam_image_size": 1024,
              "clip_feature_dim": 1024, "clip_image_size": 224, "clip_context_length": 77,
              "clip_batch_size": 8, "clip_mean": [0.5] * 3, "clip_std": [0.25] * 3}
    (tmp_path / "model-config.json").write_text(json.dumps(config))
    assert model_metadata(tmp_path, profile="full", clip_model="ViT-H-14") == config
    with pytest.raises(ValueError, match="selection differs"):
        model_metadata(tmp_path, profile="lite", clip_model="ViT-B-32")
    del config["clip_feature_dim"]
    (tmp_path / "model-config.json").write_text(json.dumps(config))
    with pytest.raises(ValueError, match="Incomplete model metadata"):
        model_metadata(tmp_path, profile="full", clip_model="ViT-H-14")


def test_annotate_reuses_lite_models_without_loading_inference(tmp_path):
    assert model_config("annotate", tmp_path) == model_config("lite", tmp_path)
    validate_tensorrt_config(profile="annotate")


def test_dualmap_uses_its_model_space_and_preserves_legacy_weight_overrides(tmp_path, monkeypatch):
    monkeypatch.setenv("SCENE_PERCEPTION_BACKEND", "dualmap")
    monkeypatch.setenv("SCENE_DUALMAP_MOBILECLIP_WEIGHTS", "/custom/mobileclip.bin")
    selected = model_config("lite", tmp_path)
    assert selected["clip_model"] == "MobileCLIP-S2"
    assert selected["clip_pretrained"] == "/custom/mobileclip.bin"
    assert selected["sam"] == str(tmp_path / "mobile_sam.pt")
    assert model_config("lite", tmp_path, backend="concept_graphs")["clip_model"] == "ViT-B-32"
    validate_tensorrt_config(profile="lite", backend="dualmap")
    validate_tensorrt_config(profile="full", backend="dualmap")


@pytest.mark.parametrize("profile", ["lite", "full"])
@pytest.mark.parametrize("backend,expected", [
    ("concept_graphs", ("yolo", "sam", "clip")),
    ("dualmap", ("yolo", "sam", "clip", "fastsam", "replica")),
])
def test_portable_export_selection_covers_the_mapper_models(tmp_path, profile, backend, expected):
    from scene_service.inference_config import export_model_names

    assert export_model_names(model_config(profile, tmp_path, backend=backend)) == expected
