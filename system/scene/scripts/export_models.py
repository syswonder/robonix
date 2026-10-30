#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Export the scene perception stack to ONNX + parity references (build machine).

Runs wherever torch/ultralytics/open_clip/onnx are installed (a Jetson with the
JetPack torch stack qualifies — the board itself can be the build machine; the
board never needs torch at *runtime*, only here, once per model-set change).
Outputs land in docker/_weights/onnx/ next to the weights so they ship through
the same distribution channel, and start_native.sh picks them up.

Artifacts per model: one .onnx plus a .npz holding seeded reference inputs and
the torch fp32 reference outputs.  build_engines.py replays those on-target to
check numerical parity with Torch before accepting the engine; no tokenizer
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
  sam-encoder       MobileSAM image encoder (dynamo export keeps LayerNorm as
                    a single node).
  sam-decoder       MobileSAM box-prompt decoder (prompt encoder + mask
                    decoder, dynamic box count); built as a K=1 TRT engine.
  clip-visual       open_clip ViT-B-32 image tower, fixed batch 8 (the tick
                    pads to the detection cap; 23ms per batch).
  clip-text         open_clip ViT-B-32 text tower, dynamic batch (arbitrary
                    runtime text queries).

Only the "lite" perception profile is exported; the ViT-H-14 profile would
need its own export run.
"""
from __future__ import annotations

import argparse
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


def log(*a):
    print(*a, flush=True)


def save_npz(path: str, **kw) -> None:
    np.savez_compressed(path, **kw)


def cmp_log(name: str, a: np.ndarray, b: np.ndarray) -> None:
    a, b = a.astype(np.float64), b.astype(np.float64)
    cos = float((a * b).sum() / (np.linalg.norm(a) * np.linalg.norm(b)))
    log(f"  {name}: cos={cos:.6f} maxabs={np.abs(a - b).max():.3e}")


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
        torch.onnx.export(
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
        torch.onnx.export(
            tw, (tok,), path, opset_version=17, dynamo=False,
            input_names=["tokens"], output_names=["txt_feats"],
            dynamic_axes={"tokens": {0: "nc"}, "txt_feats": {0: "nc"}},
        )
    save_npz(f"{OUT}/yolo-world-text.npz", tokens=tok.numpy(), ref=ref,
             tokens10=tok10.numpy(), ref10=ref10)
    log(f"yolo-world-text {os.path.getsize(path) / 2**20:.0f}MB")


# ── MobileSAM image encoder (dynamo, merged single file) ────────────────────
def export_sam(weights: str) -> None:
    import onnx
    from ultralytics import SAM

    m = SAM(weights)
    enc = m.model.image_encoder.eval()
    torch.manual_seed(0)
    im = torch.randn(1, 3, 1024, 1024)
    with torch.no_grad():
        ref = enc(im).numpy()
    # dynamo exporter keeps LayerNorm as one node (fp32 internal reductions);
    # external weights data merged back into a single file for distribution.
    tmp = f"{OUT}/.sam-encoder-tmp.onnx"
    with torch.no_grad():
        torch.onnx.export(
            enc, (im,), tmp, opset_version=18, dynamo=True,
            input_names=["image"], output_names=["image_embed"],
        )
    merged = onnx.load(tmp, load_external_data=True)
    onnx.save(merged, f"{OUT}/sam-encoder.onnx")
    os.remove(tmp)
    if os.path.exists(tmp + ".data"):
        os.remove(tmp + ".data")
    save_npz(f"{OUT}/sam-encoder.npz", image=im.numpy(), ref=ref)
    log(f"sam-encoder {os.path.getsize(f'{OUT}/sam-encoder.onnx') / 2**20:.0f}MB")


# ── MobileSAM prompt encoder (box path) + mask decoder ──────────────────────
def export_sam_decoder(weights: str) -> None:
    """Export the box-prompt decoder; the target builds a static K=1 TRT engine.

    Takes the image embedding from the sam-encoder engine plus K boxes in
    1024-space pixels, returns (K,1,256,256) mask logits.  The upstream
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
                [self.dec.output_hypernetworks_mlps[i](mask_tokens_out[:, i, :])
                 for i in range(self.dec.num_mask_tokens)], dim=1)
            bb, cc, hh, ww = upscaled.shape
            masks = (hyper @ upscaled.view(bb, cc, hh * ww)).view(bb, -1, hh, ww)
            iou_pred = self.dec.iou_prediction_head(hs[:, 0, :])
            return masks[:, 0:1], iou_pred[:, 0:1]

    torch.manual_seed(0)
    im = torch.randn(1, 3, 1024, 1024)
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
        masks_up, iou_up = dec(
            image_embeddings=torch.from_numpy(enc_ref),
            image_pe=pe.get_dense_pe(),
            sparse_prompt_embeddings=sparse,
            dense_prompt_embeddings=dense,
            multimask_output=False,
        )
    log("sam-decoder parity (torch wrapper vs ultralytics forward):")
    cmp_log("masks", masks_up.numpy(), refs[0][0].numpy())
    cmp_log("iou", iou_up.numpy(), refs[0][1].numpy())

    wrap = DecWrapper(pe, dec).eval()
    path = f"{OUT}/sam-decoder.onnx"
    with torch.no_grad():
        torch.onnx.export(
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


# ── open_clip ViT-B-32 (visual b8 + text, dynamic batch) ────────────────────
def export_clip(openclip_bin: str) -> None:
    import open_clip

    model, _, _ = open_clip.create_model_and_transforms(
        "ViT-B-32", pretrained=openclip_bin)
    model.eval()

    torch.manual_seed(0)
    im = torch.randn(8, 3, 224, 224)
    with torch.no_grad():
        ref = model.visual(im).numpy()
    path = f"{OUT}/clip-visual.onnx"
    with torch.no_grad():
        torch.onnx.export(
            model.visual, (im,), path, opset_version=17, dynamo=False,
            input_names=["images"], output_names=["image_feats"],
        )
    save_npz(f"{OUT}/clip-visual.npz", images=im.numpy(), ref=ref)
    log(f"clip-visual {os.path.getsize(path) / 2**20:.0f}MB")

    tokenizer = open_clip.get_tokenizer("ViT-B-32")
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
    path = f"{OUT}/clip-text.onnx"
    with torch.no_grad():
        torch.onnx.export(
            tw, (tok,), path, opset_version=17, dynamo=False,
            input_names=["tokens"], output_names=["text_feats"],
            dynamic_axes={"tokens": {0: "n"}, "text_feats": {0: "n"}},
        )
    save_npz(f"{OUT}/clip-text.npz", tokens=tok.numpy(), ref=ref,
             tokens1=tok1.numpy(), ref1=ref1)
    log(f"clip-text {os.path.getsize(path) / 2**20:.0f}MB")


def main() -> None:
    global OUT
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--yolo-weights", default=os.path.join(PKG, "docker", "_weights", "yolov8l-world.pt"))
    ap.add_argument("--sam-weights", default=os.path.join(PKG, "docker", "_weights", "mobile_sam.pt"))
    ap.add_argument("--clip-bin", default=os.path.join(PKG, "docker", "_weights", "open_clip_pytorch_model.bin"))
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args()
    OUT = args.out
    os.makedirs(OUT, exist_ok=True)

    def done(name: str) -> bool:
        return (os.path.isfile(f"{OUT}/{name}.onnx") and os.path.isfile(f"{OUT}/{name}.npz"))

    t0 = time.time()
    # eval+no_grad triggers torch's fused MHA fastpath, which has no opset-17
    # mapping; the plain attention decomposition exports fine.
    torch.backends.mha.set_fastpath_enabled(False)
    if not (done("yolo-world-cut") and done("yolo-world-text")):
        export_yolo(args.yolo_weights)
    else:
        log("yolo exports present, skip")
    if not done("sam-encoder"):
        export_sam(args.sam_weights)
    else:
        log("sam export present, skip")
    if not done("sam-decoder"):
        export_sam_decoder(args.sam_weights)
    else:
        log("sam-decoder export present, skip")
    if not (done("clip-visual") and done("clip-text")):
        export_clip(args.clip_bin)
    else:
        log("clip exports present, skip")
    log(f"all exports done in {time.time() - t0:.0f}s -> {OUT}")


if __name__ == "__main__":
    main()
