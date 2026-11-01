# SPDX-License-Identifier: MulanPSL-2.0
"""Torch implementation of the detector's NumPy inference interface."""
from __future__ import annotations

import os

import numpy as np


class TorchPerception:
    """YOLO-World, SAM and CLIP sessions loaded only on the Torch path."""

    def __init__(self, classes, yolo_path, sam_path, clip_model, clip_pretrained):
        import torch
        import open_clip
        from ultralytics import YOLO, SAM

        self.device = "cpu" if os.environ.get("SCENE_CG_FORCE_CPU", "").strip().lower() in (
            "1", "true", "yes",
        ) else ("cuda" if torch.cuda.is_available() else "cpu")
        self.yolo = YOLO(yolo_path)
        self.yolo.set_classes(classes)
        self.sam = SAM(sam_path)
        self.clip, _, self.preprocess = open_clip.create_model_and_transforms(
            clip_model, pretrained=clip_pretrained,
        )
        self.clip = self.clip.to(self.device).eval()
        self.tokenizer = open_clip.get_tokenizer(clip_model)

    def detect(self, bgr, conf):
        results = self.yolo.predict(bgr, conf=conf, device=self.device, verbose=False)
        if not results or results[0].boxes is None:
            return np.empty((0, 6), dtype=np.float32)
        boxes = results[0].boxes
        return np.concatenate([
            boxes.xyxy.detach().cpu().numpy(),
            boxes.conf.detach().cpu().numpy()[:, None],
            boxes.cls.detach().cpu().numpy()[:, None],
        ], axis=1)

    def segment(self, bgr, xyxy):
        results = self.sam.predict(bgr, bboxes=xyxy.tolist(), device=self.device, verbose=False)
        if not results or results[0].masks is None:
            return np.empty((0, *bgr.shape[:2]), dtype=bool)
        return results[0].masks.data.detach().cpu().numpy().astype(bool)

    def clip_feats(self, rgb, xyxy):
        import torch
        from .perception_images import clip_crops

        crops = [self.preprocess(crop) for crop in clip_crops(rgb, xyxy)]
        if not crops:
            return np.empty((0, 512), dtype=np.float32)
        feats = []
        with torch.inference_mode():
            for start in range(0, len(crops), 8):
                batch = torch.stack(crops[start:start + 8]).to(self.device)
                encoded = self.clip.encode_image(batch)
                encoded = encoded / encoded.norm(dim=-1, keepdim=True).clamp_min(1e-12)
                feats.append(encoded.cpu().numpy())
        return np.concatenate(feats).astype(np.float32)

    def embed_text(self, texts):
        import torch

        with torch.inference_mode():
            encoded = self.clip.encode_text(self.tokenizer(texts).to(self.device))
            encoded = encoded / encoded.norm(dim=-1, keepdim=True).clamp_min(1e-12)
            return encoded.cpu().numpy().astype(np.float32)

    def close(self):
        import torch

        self.yolo = self.sam = self.clip = self.preprocess = self.tokenizer = None
        if self.device == "cuda":
            torch.cuda.empty_cache()
