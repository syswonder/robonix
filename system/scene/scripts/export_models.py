#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Export the scene perception stack to ONNX + parity references (build machine).

Runs wherever torch/ultralytics/open_clip/onnx are installed (a Jetson with the
JetPack torch stack qualifies — the board itself can be the build machine; the
board never needs torch at *runtime*, only here, once per model-set change).
Outputs land in docker/_weights/onnx/ next to the weights so they ship through
the same distribution channel, and start_native.sh picks them up.

Each model has an ONNX graph, external tensor data (.onnx.data), and a .npz
with seeded inputs and Torch FP32 references. Shared model-config.json records
model identity and preprocessing. Keep these files together.
build_engines.py replays the references on-target to check numerical parity with Torch before accepting the engine; no tokenizer
or preprocessing library is needed at gate time because
token ids / preprocessed tensors are stored directly.

Models:
  yolo-world-cut    dual-tower YOLO-World: the class-score dot product is cut
                    out of the graph (cv3 emits 512-d anchor features, the
                    ContrastiveHead math moves to numpy), and the text
                    embeddings E are a RUNTIME input with dynamic nc — one
                    engine serves any class table, no rebuild on vocabulary
                    change.  imgsz must be 672: at 640 adaptive_max_pool2d is
                    not exportable (20x20 does not divide by 3).
  yolo-world-text   YOLO-World's own CLIP ViT-B/32 text encoder (the model
                    set_classes uses); computes E for arbitrary class names.
                    First use downloads ~354MB of CLIP weights via ultralytics.
  sam-encoder       Selected SAM image encoder (MobileSAM or SAM-L).
  sam-decoder       Shared SAM box-prompt decoder, built as a K=1 TRT engine.
  clip-visual       Selected open_clip image tower, fixed batch of 8.
  clip-text         Matching open_clip text tower, dynamic text batch.

Lite and full share the exporter; model identity and preprocessing are
recorded beside the graphs for Torch-free runtime validation.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile
import os
import sys
import time
import types
import warnings

warnings.filterwarnings("ignore")

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # system/scene
sys.path.insert(0, PKG)

import numpy as np  # noqa: E402
import torch  # noqa: E402

OUT = os.path.join(PKG, "docker", "_weights", "onnx")


def export_graph(model, inputs, path, **kwargs):
    """Keep large models outside protobuf's 2 GiB limit with named sidecars."""
    import onnx

    destination = Path(path)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        graph = Path(temporary) / destination.name
        torch.onnx.export(model, inputs, str(graph), **kwargs)
        loaded = onnx.load(str(graph), load_external_data=True)
        sidecar = destination.with_suffix(destination.suffix + ".data")
        sidecar.unlink(missing_ok=True)
        onnx.save_model(loaded, str(destination), save_as_external_data=True,
                        all_tensors_to_one_file=True, location=sidecar.name,
                        size_threshold=1024)
        # ONNX creates sidecars as 0600; packaged runtimes use a different UID.
        destination.chmod(0o644)
        if sidecar.exists():
            sidecar.chmod(0o644)


def update_metadata(**values):
    path = Path(OUT) / "model-config.json"
    config = json.loads(path.read_text()) if path.exists() else {}
    config.update(values)
    path.write_text(json.dumps(config, sort_keys=True) + "\n")


def log(*a):
    print(*a, flush=True)


def save_npz(path: str, **kw) -> None:
    np.savez_compressed(path, **kw)


def cmp_log(name: str, a: np.ndarray, b: np.ndarray) -> None:
    a, b = a.astype(np.float64), b.astype(np.float64)
    cos = float((a * b).sum() / (np.linalg.norm(a) * np.linalg.norm(b)))
    if a.ndim == 2:
        cos = float(np.min(np.sum(a * b, axis=-1) /
                           (np.linalg.norm(a, axis=-1) * np.linalg.norm(b, axis=-1))))
    log(f"  {name}: cos={cos:.6f} maxabs={np.abs(a - b).max():.3e}")
    if not np.isfinite(cos) or cos < 0.999:
        raise ValueError(f"{name}: export wrapper differs from the source model")


# ── YOLO-World dual-tower ────────────────────────────────────────────────────
def export_yolo(weights: str) -> None:
    from scene_service.ingest.perception_concept_graphs import _DEFAULT_OPEN_VOCAB

    from ultralytics import YOLO

    m = YOLO(weights)
    m.model.set_classes(list(_DEFAULT_OPEN_VOCAB))
    model = m.model.eval()
    head = model.model[-1]
    e55 = model.txt_feats.detach().clone()
    log(f"yolo: nc={head.nc} E={tuple(e55.shape)}")

    torch.manual_seed(0)
    im = torch.randn(1, 3, 672, 672)

    # Reference through the unmodified head: (1, 4+nc, N) decoded boxes+scores.
    head.export = True
    with torch.no_grad():
        y_ref = model(im)
    head.export = False
    y_box, y_cls = y_ref[:, :4], y_ref[:, 4:]

    # Cut: keep cv2 box branch + DFL/anchor decode in-graph; cv3 features out.
    def head_forward_cut(self, x, text):
        boxes, feats = [], []
        for i in range(self.nl):
            boxes.append(self.cv2[i](x[i]))
            feats.append(self.cv3[i](x[i]))
        bs, c = feats[0].shape[0], feats[0].shape[1]
        box_cat = torch.cat([b.view(bs, self.reg_max * 4, -1) for b in boxes], 2)
        dbox = self._get_decode_boxes({"boxes": box_cat, "feats": feats})
        feat_cat = torch.cat([f.view(bs, c, -1) for f in feats], 2)
        return dbox, feat_cat

    from ultralytics.nn.modules import C2fAttn, ImagePoolingAttn, WorldDetect

    class CutWrapper(torch.nn.Module):
        """(im, txt_feats) -> (dbox, feat_cat); mirrors WorldModel.predict."""

        def __init__(self, wm):
            super().__init__()
            self.layers = wm.model
            self.save = wm.save

        def forward(self, im, txt_feats):
            ori = txt_feats.type_as(im).clone()
            x, y = im, []
            for mod in self.layers:
                if mod.f != -1:
                    x = y[mod.f] if isinstance(mod.f, int) else [x if j == -1 else y[j] for j in mod.f]
                if isinstance(mod, C2fAttn):
                    x = mod(x, txt_feats.type_as(x))
                elif isinstance(mod, WorldDetect):
                    x = mod(x, ori)
                elif isinstance(mod, ImagePoolingAttn):
                    txt_feats = mod(x, txt_feats)
                else:
                    x = mod(x)
                y.append(x if mod.i in self.save else None)
            return x

    head.forward = types.MethodType(head_forward_cut, head)
    wrapper = CutWrapper(model).eval()
    with torch.no_grad():
        dbox, feat = wrapper(im, e55)

    # numpy score stage: identical math to the ContrastiveHead, per-scale scalars.
    ns = [int((672 / float(s)) ** 2) for s in head.stride]
    scales = [c.logit_scale.exp().item() for c in head.cv4]
    biases = [c.bias.item() for c in head.cv4]
    f = feat[0].double().numpy()
    fn = f / np.linalg.norm(f, axis=0, keepdims=True)
    en = e55[0].double().numpy()
    en = en / np.linalg.norm(en, axis=1, keepdims=True)
    d = fn.T @ en.T
    sv = np.concatenate([np.full(n, s) for s, n in zip(scales, ns)])
    bv = np.concatenate([np.full(n, b) for b, n in zip(biases, ns)])
    cls = (1.0 / (1.0 + np.exp(-(d * sv[:, None] + bv[:, None])))).T[None].astype(np.float32)
    log("yolo cut parity (torch):")
    cmp_log("boxes", y_box.numpy(), dbox.numpy())
    cmp_log("cls", y_cls.numpy(), cls)

    onnx_path = f"{OUT}/yolo-world-cut.onnx"
    with torch.no_grad():
        export_graph(
            wrapper, (im, e55), onnx_path, opset_version=17, dynamo=False,
            input_names=["images", "txt_feats"], output_names=["boxes", "feats"],
            dynamic_axes={"txt_feats": {1: "nc"}},
        )
    save_npz(f"{OUT}/yolo-world-cut.npz",
             images=im.numpy(), txt_feats=e55.numpy(),
             y_box=y_box.numpy(), y_cls=y_cls.numpy(),
             scales=np.array(scales), biases=np.array(biases), ns=np.array(ns))
    log(f"yolo-world-cut {os.path.getsize(onnx_path) / 2**20:.0f}MB")

    # Text tower: the CLIP text encoder set_classes uses, normalize included
    # (the score stage normalizes features too, so E arrives unit-norm).
    tm = model.clip_model

    class TextWrap(torch.nn.Module):
        def __init__(self, cm):
            super().__init__()
            self.cm = cm

        def forward(self, tokens):
            f = self.cm.encode_text(tokens)
            return f / f.norm(p=2, dim=-1, keepdim=True)

    tok = tm.tokenize(list(_DEFAULT_OPEN_VOCAB))
    tok10 = tm.tokenize(list(_DEFAULT_OPEN_VOCAB)[:10])
    with torch.no_grad():
        ref = tm.model.encode_text(tok)
        ref = (ref / ref.norm(p=2, dim=-1, keepdim=True)).numpy()
        ref10 = tm.model.encode_text(tok10)
        ref10 = (ref10 / ref10.norm(p=2, dim=-1, keepdim=True)).numpy()
    tw = TextWrap(tm.model).eval()
    path = f"{OUT}/yolo-world-text.onnx"
    with torch.no_grad():
        export_graph(
            tw, (tok,), path, opset_version=17, dynamo=False,
            input_names=["tokens"], output_names=["txt_feats"],
            dynamic_axes={"tokens": {0: "nc"}, "txt_feats": {0: "nc"}},
        )
    save_npz(f"{OUT}/yolo-world-text.npz", tokens=tok.numpy(), ref=ref,
             tokens10=tok10.numpy(), ref10=ref10)
    log(f"yolo-world-text {os.path.getsize(path) / 2**20:.0f}MB")


# ── SAM image encoder ────────────────────
def export_sam(weights: str) -> None:
    from ultralytics import SAM

    m = SAM(weights)
    enc = m.model.image_encoder.eval()
    torch.manual_seed(0)
    size = m.model.prompt_encoder.input_image_size
    im = torch.randn(1, 3, *size)
    with torch.no_grad():
        ref = enc(im).numpy()
    with torch.no_grad():
        export_graph(enc, (im,), f"{OUT}/sam-encoder.onnx", opset_version=18, dynamo=True,
                     input_names=["image"], output_names=["image_embed"])
    update_metadata(sam_image_size=int(m.model.prompt_encoder.input_image_size[0]))
    save_npz(f"{OUT}/sam-encoder.npz", image=im.numpy(), ref=ref)
    log(f"sam-encoder {os.path.getsize(f'{OUT}/sam-encoder.onnx') / 2**20:.0f}MB")


# ── SAM prompt encoder (box path) + mask decoder ──────────────────────
def export_sam_decoder(weights: str) -> None:
    """Export the box-prompt decoder; the target builds a static K=1 TRT engine.

    Takes the image embedding from the sam-encoder engine plus K boxes in
    encoder-input pixels, returns single-mask logits and the complete quality head.  The upstream
    predict_masks broadcasts the image embedding with torch.repeat_interleave
    on a dynamic repeat, which exports as a data-dependent op — the batching
    here is plain expand (ONNX Expand, K-dynamic) and the per-token
    hypernetwork loop is unrolled at export.  multimask_output=False is baked
    (single-mask token slice 0:1), matching the ultralytics predict path."""
    from ultralytics import SAM

    m = SAM(weights)
    m.model.eval()
    pe = m.model.prompt_encoder
    dec = m.model.mask_decoder

    class DecWrapper(torch.nn.Module):
        def __init__(self, pe, dec):
            super().__init__()
            self.pe = pe
            self.dec = dec
            self.register_buffer("image_pe", pe.get_dense_pe())  # (1,256,64,64)

        @staticmethod
        def project(mlp, features):
            # TensorRT 10.3 miscompiles consecutive GEMV; keep two rows through the MLP.
            return mlp(features.unsqueeze(1).expand(-1, 2, -1)).mean(dim=1)

        def forward(self, image_embeddings, boxes):
            k = boxes.shape[0]
            # prompt encoder, boxes-only path (_embed_boxes + no_mask_embed)
            corner = self.pe.pe_layer.forward_with_coords(
                (boxes + 0.5).reshape(-1, 2, 2), self.pe.input_image_size)
            corner[:, 0, :] += self.pe.point_embeddings[2].weight
            corner[:, 1, :] += self.pe.point_embeddings[3].weight
            sparse = corner  # (K,2,256)
            dense = self.pe.no_mask_embed.weight.reshape(1, -1, 1, 1).expand(
                k, -1, self.pe.image_embedding_size[0], self.pe.image_embedding_size[1])
            # MaskDecoder.predict_masks, repeat_interleave -> expand
            output_tokens = torch.cat(
                [self.dec.iou_token.weight, self.dec.mask_tokens.weight], dim=0)
            output_tokens = output_tokens.unsqueeze(0).expand(k, -1, -1)
            tokens = torch.cat((output_tokens, sparse), dim=1)  # (K, 5+2, 256)
            src = image_embeddings.expand(k, -1, -1, -1) + dense
            pos_src = self.image_pe.expand(k, -1, -1, -1)
            b, c, h, w = src.shape
            hs, src = self.dec.transformer(src, pos_src, tokens)
            mask_tokens_out = hs[:, 1:1 + self.dec.num_mask_tokens, :]
            src = src.transpose(1, 2).view(b, c, h, w)
            upscaled = self.dec.output_upscaling(src)
            hyper = torch.stack(
                [self.project(self.dec.output_hypernetworks_mlps[i], mask_tokens_out[:, i, :])
                 for i in range(self.dec.num_mask_tokens)], dim=1)
            bb, cc, hh, ww = upscaled.shape
            masks = (hyper @ upscaled.view(bb, cc, hh * ww)).view(bb, -1, hh, ww)
            iou_pred = self.project(self.dec.iou_prediction_head, hs[:, 0, :])
            # Keep quality columns intact so pruning cannot recreate a one-column GEMV.
            return masks[:, 0:1], iou_pred

    torch.manual_seed(0)
    im = torch.randn(1, 3, *pe.input_image_size)
    with torch.no_grad():
        enc_ref = m.model.image_encoder(im).numpy()

    box_sets = [torch.tensor([[100., 100., 400., 400.], [300., 200., 800., 900.]]),
                torch.tensor([[10., 10., 1000., 700.], [50., 50., 300., 300.],
                              [500., 100., 900., 800.], [200., 400., 700., 1000.],
                              [0., 0., 512., 512.]])]
    with torch.no_grad():
        refs = [DecWrapper(pe, dec)(torch.from_numpy(enc_ref), bx) for bx in box_sets]
    # sanity: wrapper vs the ultralytics prompt path on the first box set
    with torch.no_grad():
        sparse, dense = pe(points=None, boxes=box_sets[0], masks=None)
        masks_up, iou_up = dec.predict_masks(
            image_embeddings=torch.from_numpy(enc_ref),
            image_pe=pe.get_dense_pe(),
            sparse_prompt_embeddings=sparse,
            dense_prompt_embeddings=dense,
        )
    log("sam-decoder parity (torch wrapper vs ultralytics forward):")
    cmp_log("masks", masks_up[:, 0:1].numpy(), refs[0][0].numpy())
    cmp_log("iou", iou_up.numpy(), refs[0][1].numpy())

    wrap = DecWrapper(pe, dec).eval()
    path = f"{OUT}/sam-decoder.onnx"
    with torch.no_grad():
        export_graph(
            wrap, (torch.from_numpy(enc_ref), box_sets[0]), path,
            opset_version=17, dynamo=False,
            input_names=["image_embeddings", "boxes"],
            output_names=["masks", "iou"],
            dynamic_axes={"boxes": {0: "K"}, "masks": {0: "K"}, "iou": {0: "K"}},
        )
    save_npz(f"{OUT}/sam-decoder.npz",
             image=im.numpy(), enc_ref=enc_ref,
             boxes2=box_sets[0].numpy(), masks_ref2=refs[0][0].numpy(),
             iou_ref2=refs[0][1].numpy(),
             boxes5=box_sets[1].numpy(), masks_ref5=refs[1][0].numpy(),
             iou_ref5=refs[1][1].numpy())
    log(f"sam-decoder {os.path.getsize(path) / 2**20:.0f}MB")


# ── open_clip visual and text towers ────────────────────
def export_clip(openclip_bin: str, model_name: str) -> None:
    import open_clip
    from scene_service.ingest.perception_models import create_clip_model

    model, _, _ = create_clip_model(model_name, openclip_bin)

    preprocess = open_clip.get_model_preprocess_cfg(model)
    size = preprocess["size"]
    size = (size, size) if isinstance(size, int) else tuple(size)
    if len(size) != 2 or size[0] != size[1]:
        raise ValueError("TensorRT CLIP requires a square image input")
    if preprocess.get("resize_mode", "shortest") != "shortest" or preprocess.get("interpolation", "bicubic") not in ("bicubic", "bilinear"):
        raise ValueError("TensorRT CLIP requires shortest-side bicubic or bilinear preprocessing")
    tokenizer = open_clip.get_tokenizer(model_name)
    if type(tokenizer).__name__ != "SimpleTokenizer" or tokenizer.vocab_size != 49408:
        raise ValueError("TensorRT CLIP requires the OpenAI BPE tokenizer")
    torch.manual_seed(0)
    pixels = torch.rand(8, 3, *size)
    pixels[0].zero_()
    pixels[1].fill_(1)
    mean = torch.tensor(preprocess["mean"]).view(1, 3, 1, 1)
    std = torch.tensor(preprocess["std"]).view(1, 3, 1, 1)
    im = (pixels - mean) / std
    with torch.no_grad():
        ref = model.visual(im).numpy()
    # Explicit attention preserves the source scaling across inference backends.
    for module in model.visual.modules():
        if getattr(module, "fused_attn", False):
            module.fused_attn = False
    with torch.no_grad():
        cmp_log("clip-visual attention", model.visual(im).numpy(), ref)
    path = f"{OUT}/clip-visual.onnx"
    with torch.no_grad():
        export_graph(
            model.visual, (im,), path, opset_version=17, dynamo=False,
            input_names=["images"], output_names=["image_feats"],
        )
    save_npz(f"{OUT}/clip-visual.npz", images=im.numpy(), ref=ref)
    log(f"clip-visual {os.path.getsize(path) / 2**20:.0f}MB")

    context, feature_dim = export_text(model, tokenizer)
    update_metadata(clip_model=model_name, clip_image_size=size[0],
                    clip_interpolation=preprocess.get("interpolation", "bicubic"),
                    clip_mean=list(preprocess["mean"]), clip_std=list(preprocess["std"]),
                    clip_feature_dim=feature_dim,
                    clip_context_length=context, clip_batch_size=int(im.shape[0]))


def export_text(model, tokenizer, name="clip-text"):
    if type(tokenizer).__name__ != "SimpleTokenizer" or tokenizer.vocab_size != 49408:
        raise ValueError("Export requires the OpenAI BPE tokenizer")
    texts = ["a photo of a chair", "a white table", "a laptop on a desk",
             "a green plant", "a red mug", "a door", "a trash bin", "a monitor"]
    tok = tokenizer(texts).int()
    with torch.no_grad():
        ref = model.encode_text(tok).numpy()
    tok1 = tokenizer(["a photo of a chair"]).int()
    with torch.no_grad():
        ref1 = model.encode_text(tok1).numpy()

    class TextWrap(torch.nn.Module):
        def __init__(self, m):
            super().__init__()
            self.m = m

        def forward(self, tokens):
            return self.m.encode_text(tokens)

    tw = TextWrap(model).eval()
    path = f"{OUT}/{name}.onnx"
    with torch.no_grad():
        export_graph(
            tw, (tok,), path, opset_version=17, dynamo=False,
            input_names=["tokens"], output_names=["text_feats"],
            dynamic_axes={"tokens": {0: "n"}, "text_feats": {0: "n"}},
        )
    save_npz(f"{OUT}/{name}.npz", tokens=tok.numpy(), ref=ref,
             tokens1=tok1.numpy(), ref1=ref1)
    log(f"{name} {os.path.getsize(path) / 2**20:.0f}MB")
    return int(tok.shape[1]), int(ref.shape[-1])


def export_replica(weights):
    import open_clip
    from scene_service.ingest.perception_models import create_clip_model

    model, _, _ = create_clip_model("ViT-B-32", weights)
    context, width = export_text(model, open_clip.get_tokenizer("ViT-B-32"), "replica-text")
    update_metadata(replica_text=True, replica_clip_model="ViT-B-32",
                    replica_clip_pretrained=str(Path(weights).resolve()),
                    replica_context_length=context, replica_feature_dim=width)


def export_fastsam(weights: str) -> None:
    from ultralytics import FastSAM

    model = FastSAM(weights).model.float().eval()
    model.fuse(verbose=False)
    head = model.model[-1]
    head.export, head.dynamic, head.format = True, True, "onnx"
    torch.manual_seed(0)
    images = torch.rand(1, 3, 768, 1024)
    portrait = torch.rand(1, 3, 1024, 768)
    with torch.no_grad():
        predictions, prototypes = model(images)
        predictions1, prototypes1 = model(portrait)
        export_graph(
            model, (images,), f"{OUT}/fastsam.onnx", opset_version=17, dynamo=False,
            input_names=["images"], output_names=["predictions", "prototypes"],
            dynamic_axes={"images": {2: "height", 3: "width"},
                          "predictions": {2: "anchors"},
                          "prototypes": {2: "mask_height", 3: "mask_width"}},
        )
    class_end = 4 + int(head.nc)
    save_npz(f"{OUT}/fastsam.npz", images=images.numpy(), boxes=predictions[:, :4].numpy(),
             scores=predictions[:, 4:class_end].numpy(), coefficients=predictions[:, class_end:].numpy(),
             prototypes=prototypes.numpy(), images1=portrait.numpy(), boxes1=predictions1[:, :4].numpy(),
             scores1=predictions1[:, 4:class_end].numpy(), coefficients1=predictions1[:, class_end:].numpy(),
             prototypes1=prototypes1.numpy())


def main() -> None:
    global OUT
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--profile", default=os.environ.get("SCENE_PROFILE") or "lite", choices=("lite", "full", "annotate"))
    ap.add_argument("--yolo-weights")
    ap.add_argument("--sam-weights")
    ap.add_argument("--clip-bin")
    ap.add_argument("--clip-model")
    ap.add_argument("--fastsam-weights")
    ap.add_argument("--replica-clip-bin")
    ap.add_argument("--only", choices=("yolo", "sam", "clip", "fastsam", "replica"))
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args()
    OUT = args.out
    os.makedirs(OUT, exist_ok=True)
    from scene_service.inference_config import model_config, model_profile

    args.profile = model_profile(args.profile)
    models = model_config(args.profile)
    args.yolo_weights = args.yolo_weights or models["yolo"]
    args.sam_weights = args.sam_weights or models["sam"]
    args.clip_bin = args.clip_bin or models["clip_pretrained"]
    args.clip_model = args.clip_model or models["clip_model"]
    args.fastsam_weights = args.fastsam_weights or models.get("fastsam")
    args.replica_clip_bin = args.replica_clip_bin or models.get("replica_clip_pretrained")
    weights = [args.yolo_weights, args.sam_weights, args.clip_bin]
    if args.fastsam_weights:
        weights.append(args.fastsam_weights)
    if args.replica_clip_bin:
        weights.append(args.replica_clip_bin)
    identity = {"profile": args.profile, "clip_model": args.clip_model,
                "weights": [str(Path(p).resolve()) for p in weights]}
    metadata = Path(OUT) / "model-config.json"
    if metadata.exists():
        previous = json.loads(metadata.read_text())
        if any(previous.get(k) != v for k, v in identity.items()):
            raise ValueError("Export selection changed; use a separate output directory")
    elif any(Path(OUT).glob("*.onnx")):
        raise ValueError("Unidentified exports exist; use a separate output directory")
    update_metadata(**identity)
    if args.fastsam_weights:
        update_metadata(fastsam=True, fastsam_image_size=1024)

    def done(name: str) -> bool:
        return (os.path.isfile(f"{OUT}/{name}.onnx") and os.path.isfile(f"{OUT}/{name}.npz"))

    t0 = time.time()
    # eval+no_grad triggers torch's fused MHA fastpath, which has no opset-17
    # mapping; the plain attention decomposition exports fine.
    torch.backends.mha.set_fastpath_enabled(False)
    if args.only in (None, "yolo") and not (done("yolo-world-cut") and done("yolo-world-text")):
        export_yolo(args.yolo_weights)
    else:
        log("yolo exports present, skip")
    if args.only in (None, "sam") and not done("sam-encoder"):
        export_sam(args.sam_weights)
    else:
        log("sam export present, skip")
    if args.only in (None, "sam") and not done("sam-decoder"):
        export_sam_decoder(args.sam_weights)
    else:
        log("sam-decoder export present, skip")
    if args.only in (None, "clip") and not (done("clip-visual") and done("clip-text")):
        export_clip(args.clip_bin, args.clip_model)
    else:
        log("clip exports present, skip")
    if args.fastsam_weights and args.only in (None, "fastsam") and not done("fastsam"):
        export_fastsam(args.fastsam_weights)
    if args.replica_clip_bin and args.only in (None, "replica") and not done("replica-text"):
        export_replica(args.replica_clip_bin)
    log(f"all exports done in {time.time() - t0:.0f}s -> {OUT}")


if __name__ == "__main__":
    main()
