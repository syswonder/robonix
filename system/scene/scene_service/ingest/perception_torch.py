# SPDX-License-Identifier: MulanPSL-2.0
"""Torch implementation of the detector's NumPy inference interface."""
from __future__ import annotations

import os
from threading import RLock

import numpy as np

_CLIP_PRECISION_LOCK = RLock()


class TorchPerception:
    """YOLO-World, SAM and CLIP sessions loaded only on the Torch path."""

    def __init__(self, classes, yolo_path, sam_path, clip_model, clip_pretrained, *, device=None, fastsam_path=None):
        import torch
        import open_clip
        from ultralytics import YOLO, SAM
        from .perception_models import create_clip_model

        self.device = device or ("cpu" if os.environ.get("SCENE_CG_FORCE_CPU", "").strip().lower() in (
            "1", "true", "yes",
        ) else ("cuda" if torch.cuda.is_available() else "cpu"))
        self.yolo = YOLO(yolo_path)
        self.yolo.set_classes(classes)
        # Detection uses the fixed class features, not the cached text encoder.
        self.yolo.model.clip_model = None
        self.sam = SAM(sam_path)
        self.fastsam = None
        if fastsam_path:
            from ultralytics import FastSAM

            self.fastsam = FastSAM(fastsam_path)
        self.clip, _, self.preprocess = create_clip_model(clip_model, clip_pretrained)
        self.clip = self.clip.to(self.device).eval()
        self.tokenizer = open_clip.get_tokenizer(clip_model)

    def detect(self, bgr, conf):
        results = self.yolo.predict(bgr, conf=conf, device=self.device, verbose=False)
        if not results or results[0].boxes is None:
            return np.empty((0, 6), dtype=np.float32)
        return results[0].boxes.data.detach().cpu().numpy()

    def segment(self, bgr, xyxy):
        results = self.sam.predict(bgr, bboxes=xyxy.tolist(), device=self.device, verbose=False)
        if not results or results[0].masks is None:
            return np.empty((0, *bgr.shape[:2]), dtype=bool)
        return results[0].masks.data.detach().cpu().numpy().astype(bool)

    def segment_instances(self, bgr, conf, iou=0.9):
        result = self.fastsam.predict(bgr, device=self.device, retina_masks=True,
                                      imgsz=1024, conf=conf, iou=iou, verbose=False)[0]
        if result.masks is None:
            return np.empty((0, 6), np.float32), np.empty((0, *bgr.shape[:2]), bool)
        return result.boxes.data.detach().cpu().numpy(), result.masks.data.detach().cpu().numpy().astype(bool)

    def clip_feats(self, rgb, xyxy):
        import torch
        from .perception_images import clip_crops

        crops = [self.preprocess(crop) for crop in clip_crops(rgb, xyxy)]
        if not crops:
            return np.empty((0, self.clip.visual.output_dim), dtype=np.float32)
        feats = []
        # cuDNN flags are process-wide; serialize scopes across model instances.
        with _CLIP_PRECISION_LOCK, torch.inference_mode():
            allow_tf32 = torch.backends.cudnn.allow_tf32
            torch.backends.cudnn.allow_tf32 = False
            try:
                for start in range(0, len(crops), 8):
                    batch = torch.stack(crops[start:start + 8]).to(self.device)
                    encoded = self.clip.encode_image(batch)
                    encoded = encoded / encoded.norm(dim=-1, keepdim=True).clamp_min(1e-12)
                    feats.append(encoded.cpu().numpy())
            finally:
                torch.backends.cudnn.allow_tf32 = allow_tf32
        return np.concatenate(feats).astype(np.float32)

    def embed_text(self, texts):
        import torch

        with torch.inference_mode():
            encoded = self.clip.encode_text(self.tokenizer(texts).to(self.device))
            encoded = encoded / encoded.norm(dim=-1, keepdim=True).clamp_min(1e-12)
            return encoded.cpu().numpy().astype(np.float32)

    def close(self):
        import torch

        self.yolo = self.sam = self.fastsam = self.clip = self.preprocess = self.tokenizer = None
        if str(self.device).startswith("cuda"):
            torch.cuda.empty_cache()
