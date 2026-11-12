#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Convert Scene's DualMap perception models for the AIBOX RKNPU3 (RK1828).

The deployed bundle (``model/npu``, see the `Scene NPU models` section in
examples/webots-aibox/README.md) runs the YOLO-World detector, the MobileSAM
encoder + its decoder, and the CLIP ViT-B/32 image tower. This tool exports
and builds the YOLO-World detector and both MobileSAM stages; the CLIP
graph comes from the zoo pipeline (``fetch_zoo_aibox_models.py``).
``deploy`` assembles the bundle directory from all of them.

- ``yoloworld``         YOLO-World detector with a *fixed* vocabulary baked in
                        at export time (its CLIP text branch runs once on the
                        PC). Input uint8 NHWC [1,640,640,3], output
                        [1,4+nc,8400] pre-NMS; letterbox/NMS post-processing
                        stays on CPU.
- ``mobilesam``         MobileSAM's TinyViT image encoder.
- ``mobilesam_decoder`` prompt encoder + mask decoder (SamOnnxModel) from the
                        pinned ``li041/MobileSAM`` checkout (``--mobilesam-repo``).
                        One box prompt per call (two corner points, labels
                        2/3); ``build`` prunes the in-graph orig-size resize
                        and the iou output, keeping the low-res logits whose
                        token 0 the runtime reads.

Two stages because the toolchains live in different virtualenvs:

    # stage 1 - export ONNX (+ .npz reference outputs); ML env (torch CUDA,
    # ultralytics, timm)
    convert_aibox_models.py export  <model> --weights-dir D --out-dir D \
        --classes-file F [--yoloworld-weights yolov8s-worldv2.pt] [--image IMG]
    # mobilesam_decoder additionally takes --mobilesam-repo <checkout>

    # stage 2 - compile to .rknn(+.weight) and validate in the PC simulator;
    # RKNN3 Toolkit env (version must match the board's rknn3_toolkit_lite)
    convert_aibox_models.py build   <model> --onnx F --out-dir D \
        [--dataset F] [--no-quant] [--optimization-level N] [--image IMG]

Run ``export`` with the ML venv python and ``build`` with the toolkit venv
python. The board-side smoke test is ``test_aibox_rknn_lite.py``.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil

MOBILE_SAM_IMAGE_SIZE = 1024

# The NPU runtime backend (npu_backends.py, decoder on the NPU) lives in the
# fork at this commit; ``deploy --dualmap-repo`` clones and pins it.
DUALMAP_REPO = "https://github.com/li041/DualMap.git"
DUALMAP_PIN = "c1a294d2c4bfc4938363e637766a115b49a2c690"


def _ensure_dualmap_checkout(path: str) -> None:
    """Clone (or align) the pinned li041/DualMap checkout that provides the
    NPU runtime backend; leaves it detached at ``DUALMAP_PIN``."""
    import subprocess

    if not os.path.isdir(os.path.join(path, ".git")):
        subprocess.run(["git", "clone", "--filter=blob:none", DUALMAP_REPO, path],
                       check=True)
    have = subprocess.run(["git", "-C", path, "cat-file", "-e", f"{DUALMAP_PIN}^{{commit}}"],
                          capture_output=True)
    if have.returncode != 0:
        subprocess.run(["git", "-C", path, "fetch", "origin", "main"], check=True)
    subprocess.run(["git", "-C", path, "checkout", "--detach", DUALMAP_PIN], check=True)
    print(f"[deploy] DualMap checkout {path} @ {DUALMAP_PIN[:12]}")


def _load_classes(path: str) -> list[str]:
    with open(path) as fh:
        names = [ln.strip() for ln in fh if ln.strip()]
    if not names:
        raise SystemExit(f"empty classes file: {path}")
    return names


# ── stage 1: ONNX export ────────────────────────────────────────────────────


def cmd_export(args: argparse.Namespace) -> None:
    os.makedirs(args.out_dir, exist_ok=True)
    if args.model == "yoloworld":
        _export_yoloworld(args)
    elif args.model == "mobilesam":
        _export_mobilesam_encoder(args)
    elif args.model == "mobilesam_decoder":
        _export_mobilesam_decoder(args)
    else:
        raise SystemExit(f"unknown model: {args.model}")


def _check_image(args: argparse.Namespace) -> str:
    import cv2

    if not args.image or not os.path.isfile(args.image):
        raise SystemExit(f"--image {args.image!r} is required for the reference dump")
    return cv2.imread(args.image)


def _save_ref(out_dir: str, name: str, **arrays) -> None:
    import numpy as np

    path = os.path.join(out_dir, name)
    np.savez(path, **arrays)
    print(f"[export] reference tensors -> {path}.npz")


def _patch_image_pooling_attn() -> None:
    """ONNX cannot trace AdaptiveMaxPool2d to (k,k) when the feature size is
    not divisible by k (YOLO-World's RepVL-PAN pools 80/40/20 -> 3x3). At the
    static export size the cells are known, so slice-and-reduce-max gives the
    same result with exportable ops."""
    import torch

    from ultralytics.nn.modules.block import ImagePoolingAttn

    def pool_fixed(x: "torch.Tensor", k: int) -> "torch.Tensor":
        h, w = int(x.shape[-2]), int(x.shape[-1])
        cells = []
        for i in range(k):
            h0, h1 = (i * h) // k, -((-(i + 1) * h) // k)
            for j in range(k):
                w0, w1 = (j * w) // k, -((-(j + 1) * w) // k)
                cells.append(x[..., h0:h1, w0:w1].amax(dim=(-2, -1)))
        out = torch.stack(cells, dim=-1)
        return out.view(*x.shape[:2], k, k)

    def forward(self, x, text):
        bs = x[0].shape[0]
        assert len(x) == self.nf
        num_patches = self.k**2
        x = [pool_fixed(proj(feat), self.k).view(bs, -1, num_patches)
             for feat, proj in zip(x, self.projections)]
        x = torch.cat(x, dim=-1).transpose(1, 2)
        q = self.query(text)
        k = self.key(x)
        v = self.value(x)

        q = q.reshape(bs, -1, self.nh, self.hc)
        k = k.reshape(bs, -1, self.nh, self.hc)
        v = v.reshape(bs, -1, self.nh, self.hc)

        aw = torch.einsum("bnmc,bkmc->bmnk", q, k)
        aw = aw / (self.hc**0.5)
        aw = torch.nn.functional.softmax(aw, dim=-1)

        x = torch.einsum("bmnk,bkmc->bnmc", aw, v)
        x = self.proj(x.reshape(bs, -1, self.ec))
        return x * self.scale + text

    # Keep DualMap's attention behavior while replacing adaptive pooling with static ops.
    ImagePoolingAttn.forward = forward


def _export_yoloworld(args: argparse.Namespace) -> None:
    """set_classes() bakes the vocabulary into the graph; export then emits a
    single-input, fixed-shape ONNX whose head output is [1,4+nc,8400]."""
    from ultralytics import YOLO

    _patch_image_pooling_attn()
    classes = _load_classes(args.classes_file)
    weight = args.yoloworld_weights or os.path.join(
        args.weights_dir, "yolov8s-worldv2.pt"
    )
    if not os.path.isfile(weight):
        raise SystemExit(f"YOLO-World checkpoint not found: {weight}")
    model = YOLO(weight)
    model.set_classes(classes)
    onnx_path = model.export(
        format="onnx", imgsz=args.imgsz, opset=args.opset, dynamic=False,
        simplify=False, half=False, nms=False,
    )
    output_path = os.path.join(args.out_dir, "yoloworld.onnx")
    if os.path.abspath(onnx_path) != os.path.abspath(output_path):
        shutil.copy2(onnx_path, output_path)
    onnx_path = output_path
    sidecar = {
        "model": "yoloworld", "classes": classes, "imgsz": args.imgsz,
        "weights": os.path.abspath(weight), "opset": args.opset,
        "output": "pre_nms [1,4+nc,8400]",
    }
    with open(os.path.join(args.out_dir, "yoloworld.json"), "w") as fh:
        json.dump(sidecar, fh, indent=2)

    img = _check_image(args)
    ref = model.predict(img[:, :, ::-1], conf=0.2, verbose=False)  # BGR->RGB
    r = ref[0]
    _save_ref(args.out_dir, "yoloworld_ref",
              xyxy=r.boxes.xyxy.cpu().numpy(), conf=r.boxes.conf.cpu().numpy(),
              cls=r.boxes.cls.cpu().numpy().astype(int))
    print(f"[export] {onnx_path}: {len(classes)} classes baked, "
          f"{len(r.boxes)} reference boxes from {args.image}")


def _export_mobilesam_encoder(args: argparse.Namespace) -> None:
    """Export ``image_encoder``; the decoder graph is the separate mobilesam_decoder model."""
    import torch
    from ultralytics import SAM

    weight = os.path.join(args.weights_dir, "mobile_sam.pt")
    sam = SAM(weight)
    encoder = sam.model.image_encoder.eval()
    image_size = MOBILE_SAM_IMAGE_SIZE
    dummy = torch.zeros(1, 3, image_size, image_size)
    out_path = os.path.join(args.out_dir, "mobilesam_encoder.onnx")
    torch.onnx.export(
        encoder, dummy, out_path, opset_version=args.opset,
        input_names=["images"], output_names=["image_embeddings"],
        dynamic_axes=None,
    )

    img = _check_image(args)
    ref = _sam_reference(sam, img, image_size)
    _save_ref(args.out_dir, "mobilesam_ref", **ref)
    print(f"[export] {out_path} (encoder only, {image_size}x{image_size})")


def _sam_reference(sam, img_bgr, image_size):
    """The exact preprocessing ultralytics' SAM predictor applies, plus the
    encoder output, so the board test can compare end to end."""
    import numpy as np
    import torch

    from ultralytics.data.augment import LetterBox

    letterbox = LetterBox(image_size, auto=False, stride=32)
    im = letterbox(image=img_bgr)
    im = im.transpose(2, 0, 1)[::-1]  # HWC BGR -> CHW RGB
    im = np.ascontiguousarray(im)
    im = torch.from_numpy(im).float()
    mean = torch.tensor([123.675, 116.28, 103.53]).view(-1, 1, 1)
    std = torch.tensor([58.395, 57.12, 57.375]).view(-1, 1, 1)
    im = (im - mean) / std
    with torch.no_grad():
        emb = sam.model.image_encoder(im[None])
    return {"image": img_bgr, "preprocessed": im.numpy(), "embedding": emb.numpy()}


def _export_mobilesam_decoder(args: argparse.Namespace) -> None:
    """SamOnnxModel (prompt encoder + mask decoder) from the pinned
    ``--mobilesam-repo`` checkout (li041/MobileSAM, the dualmap-1024 RKNN
    lineage). Static box prompt (``num_points=2``, corner labels 2/3); the
    reference replays a full-frame box over a real image's embedding. The
    graph still carries the orig-size resize and ``orig_im_size`` input —
    ``build`` prunes them, keeping ``iou_predictions`` + ``low_res_masks``."""
    import sys

    import torch

    from ultralytics import SAM

    sys.path.insert(0, args.mobilesam_repo)
    from mobile_sam import sam_model_registry
    from mobile_sam.utils.onnx import SamOnnxModel

    weight = os.path.join(args.weights_dir, "mobile_sam.pt")
    if not os.path.isfile(weight):
        raise SystemExit(f"MobileSAM checkpoint not found: {weight}")
    sam = sam_model_registry["vit_t"](checkpoint=weight).eval()
    # Multimask export like the deployed recipe: low_res_masks carries all
    # three mask tokens and the runtime reads token 0.
    onnx_model = SamOnnxModel(model=sam, return_single_mask=False)
    embed_dim = sam.prompt_encoder.embed_dim
    embed_size = sam.prompt_encoder.image_embedding_size
    dummy = (
        torch.randn(1, embed_dim, *embed_size),
        torch.tensor([[[0.0, 0.0], [1024.0, 1024.0]]]),
        torch.tensor([[2.0, 3.0]]),
        torch.zeros(1, 1, 4 * embed_size[0], 4 * embed_size[1]),
        torch.tensor([0.0]),  # no previous-mask iteration
        torch.tensor([1024.0, 1024.0]),  # orig_im_size; pruned at build
    )
    out_path = os.path.join(args.out_dir, "mobilesam_decoder.onnx")
    torch.onnx.export(onnx_model, dummy, out_path, opset_version=args.opset,
                      input_names=["image_embeddings", "point_coords", "point_labels",
                                   "mask_input", "has_mask_input", "orig_im_size"],
                      output_names=["masks", "iou_predictions", "low_res_masks"],
                      dynamic_axes=None)

    img = _check_image(args)
    # The reference embedding comes from the same ultralytics-exported
    # encoder the bundle ships, so the board replay exercises real values.
    ref = _sam_reference(SAM(weight), img, MOBILE_SAM_IMAGE_SIZE)
    emb = torch.from_numpy(ref["embedding"])
    orig = torch.tensor([float(img.shape[0]), float(img.shape[1])])
    with torch.no_grad():
        _, _, low_res = onnx_model(emb, dummy[1], dummy[2], dummy[3], dummy[4], orig)
    _save_ref(args.out_dir, "mobilesam_decoder_torch",
              **{f"in{i}": t.numpy() for i, t in enumerate(
                  (emb, dummy[1], dummy[2], dummy[3], dummy[4]))},
              out0=low_res.numpy())
    print(f"[export] {out_path} (embed {tuple(emb.shape)}, box prompt, low-res masks)")


# ── stage 2: RKNN build + simulator validation ──────────────────────────────


def cmd_build(args: argparse.Namespace) -> None:
    from rknn.api import RKNN

    os.makedirs(args.out_dir, exist_ok=True)
    quant = not args.no_quant
    if quant and not (args.dataset and os.path.isfile(args.dataset)):
        raise SystemExit("--dataset (a txt of calibration image paths) is required "
                         "for quantized builds, or pass --no-quant")
    rknn = RKNN(verbose=args.verbose)

    cfg = dict(target_platform="rk1820", optimization_level=args.optimization_level)
    if args.model == "yoloworld":
        # uint8 in, mean/std absorbed by the graph (ultralytics normalization)
        cfg.update(
            mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]],
            input_attrs={"images": {"dtype": "uint8", "layout": "NHWC"}},
            disable_rules=["fuse_conv_rs_exmatmul_to_conv"],
        )
        if quant:
            cfg.update(
                quantized_dtype="w8a8", quantized_algorithm="normal",
                quantized_method="channel",
            )
    elif args.model == "mobilesam":
        cfg.update(
            mean_values=[[123.675, 116.28, 103.53]], std_values=[[58.395, 57.12, 57.375]],
            input_attrs={"images": {"dtype": "uint8", "layout": "NHWC"}},
        )
    elif args.model == "mobilesam_decoder":
        # Feature/prompt inputs carry no mean/std; the zoo recipe does not
        # quantize the decoder.
        if quant:
            raise SystemExit("mobilesam_decoder builds fp only - pass --no-quant")
    else:
        raise SystemExit(f"unknown model: {args.model}")

    print("--> config")
    if rknn.config(**cfg) != 0:
        raise SystemExit("config failed")
    print("--> load_onnx")
    if args.model == "mobilesam_decoder":
        # Static input shapes pin the dynamic point axes; restricting the
        # outputs severs the orig-size resize, so orig_im_size never reaches
        # the graph. Only the low-res logits survive — the runtime reads
        # their token 0 — and iou is dropped like the deployed graph.
        e = MOBILE_SAM_IMAGE_SIZE // 16
        ret = rknn.load_onnx(
            model=args.onnx,
            inputs=["image_embeddings", "point_coords", "point_labels",
                    "mask_input", "has_mask_input"],
            input_size_list=[[1, 256, e, e], [1, 2, 2], [1, 2],
                             [1, 1, 4 * e, 4 * e], [1]],
            outputs=["low_res_masks"],
        )
    else:
        ret = rknn.load_onnx(model=args.onnx)
    if ret != 0:
        raise SystemExit("load_onnx failed")
    print("--> build")
    if rknn.build(do_quantization=quant, dataset=args.dataset) != 0:
        raise SystemExit("build failed")
    rknn_path = os.path.join(args.out_dir, f"{args.model}.rknn")
    print("--> export_rknn")
    if rknn.export_rknn(rknn_path) != 0:
        raise SystemExit("export_rknn failed")
    print(f"[build] {rknn_path} (quant={quant}, opt_level={args.optimization_level})")

    if args.model == "mobilesam_decoder":
        _sim_check_decoder(rknn, args)
    elif args.image:
        _sim_check(rknn, args)


def _sim_check(rknn, args: argparse.Namespace) -> None:
    """Run one real image through the PC simulator and stash outputs for the
    board-side comparison (the simulator approximates, not equals, the NPU)."""
    import cv2

    if rknn.init_runtime() != 0:
        raise SystemExit("simulator init_runtime failed")
    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit(f"cannot read {args.image}")
    x = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    import numpy as np

    h = w = 640 if args.model == "yoloworld" else 1024
    x = cv2.resize(x, (w, h))
    outputs = rknn.inference(inputs=[np.expand_dims(x, 0)], data_format=["nhwc"])
    out = os.path.join(args.out_dir, f"{args.model}_sim.npz")
    np.savez(out, **{f"out{i}": o for i, o in enumerate(outputs)})
    for i, o in enumerate(outputs):
        print(f"[sim] out{i}: shape={o.shape} dtype={o.dtype} "
              f"min={np.nanmin(o):.4g} max={np.nanmax(o):.4g} nonzero={np.count_nonzero(o)}")
    print(f"[sim] outputs -> {out}")


def _sim_check_decoder(rknn, args: argparse.Namespace) -> None:
    """Replay the export's reference inputs through the PC simulator; the npz
    keeps inputs and outputs so the board test replays identical inputs."""
    import numpy as np

    ref_path = os.path.join(os.path.dirname(os.path.abspath(args.onnx)),
                            "mobilesam_decoder_torch.npz")
    if not os.path.isfile(ref_path):
        raise SystemExit(f"missing export reference: {ref_path}")
    if rknn.init_runtime() != 0:
        raise SystemExit("simulator init_runtime failed")
    ref = np.load(ref_path)
    inputs = [ref[f"in{i}"] for i in range(5)]
    # The rk1820 simulator takes NHWC-ordered arrays and transposes them to
    # the graph's NCHW itself; the npz keeps the NCHW refs the board replays.
    nhwc = [a.transpose(0, 2, 3, 1) if a.ndim == 4 else a for a in inputs]
    outputs = rknn.inference(inputs=nhwc, data_format=["nhwc"] * len(nhwc))
    out = os.path.join(args.out_dir, f"{args.model}_sim.npz")
    np.savez(out, **{**{f"in{i}": ref[f"in{i}"] for i in range(5)},
                     **{f"out{i}": o for i, o in enumerate(outputs)}})
    for i, o in enumerate(outputs):
        print(f"[sim] out{i}: shape={o.shape} dtype={o.dtype} "
              f"min={np.nanmin(o):.4g} max={np.nanmax(o):.4g} nonzero={np.count_nonzero(o)}")
    print(f"[sim] outputs -> {out}")


def cmd_deploy(args: argparse.Namespace) -> None:
    """Assemble the DualMap `model/npu` directory from build artifacts."""
    os.makedirs(args.out_dir, exist_ok=True)
    classes = _load_classes(args.classes_file)
    # Build outputs keep their bundle names.
    artifacts = [
        ("yoloworld.rknn", "yoloworld.rknn"),
        ("mobilesam.rknn", "mobilesam.rknn"),
        ("mobilesam_decoder.rknn", "mobilesam_decoder.rknn"),
        ("clip_image.rknn", "clip_image.rknn"),
    ]
    for src_name, dst_name in artifacts:
        srcs = [(src_name, dst_name)]
        if src_name.endswith(".rknn"):
            srcs.append((src_name[: -len(".rknn")] + ".weight",
                         dst_name[: -len(".rknn")] + ".weight"))
        for src_name, dst_name in srcs:
            src = os.path.join(args.rknn_dir, src_name)
            if not os.path.isfile(src):
                raise SystemExit(f"missing artifact: {src}")
            shutil.copy2(src, os.path.join(args.out_dir, dst_name))
    with open(os.path.join(args.out_dir, "npu_meta.json"), "w") as fh:
        # "format": "pre_nms" is the runtime's decode-path selector — without
        # it npu_backends assumes a zoo bundle and demands a text sidecar the
        # baked-vocabulary graph does not take. Precision is only claimed
        # where deploy knows it: the zoo convert runs with an explicit "fp"
        # argument, while yoloworld/mobilesam come from `build`, whose
        # quantization deploy cannot see. The bundle contract (four fp16
        # graphs) lives in the webots-aibox README.
        json.dump({"classes": classes, "format": "pre_nms",
                   "graphs": {"yoloworld": "yolov8s-worldv2 fp16 pre_nms, set_classes baked",
                              "mobilesam": "encoder fp16",
                              "mobilesam_decoder": "prompt+mask decoder, low_res token0",
                              "clip_image": "clip ViT-B/32 visual fp16"},
                   "text": "set_classes text embeddings baked into the yoloworld graph"},
                  fh, indent=2)
    if args.dualmap_repo:
        _ensure_dualmap_checkout(args.dualmap_repo)
    else:
        print(f"[deploy] runtime checkout not requested — pass --dualmap-repo <path> "
              f"to clone the pinned li041/DualMap ({DUALMAP_PIN[:12]})")
    print(f"[deploy] {args.out_dir}: {len(classes)} classes")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p, *, with_onnx, models):
        p.add_argument("model", choices=models)
        p.add_argument("--out-dir", required=True)
        if with_onnx:
            p.add_argument("--onnx", required=True)
        p.add_argument("--image", help="real image for reference/simulator dumps")
        if with_onnx:
            p.add_argument("--verbose", action="store_true")

    p = sub.add_parser("export", help="ONNX export + reference tensors")
    common(p, with_onnx=False,
           models=["yoloworld", "mobilesam", "mobilesam_decoder"])
    p.add_argument("--weights-dir", required=True)
    p.add_argument(
        "--mobilesam-repo",
        help="pinned li041/MobileSAM checkout (mobilesam_decoder export)",
    )
    p.add_argument(
        "--yoloworld-weights",
        help="YOLO-World checkpoint; defaults to <weights-dir>/yolov8s-worldv2.pt",
    )
    p.add_argument("--classes-file", help="vocabulary, one name per line (yoloworld)")
    p.add_argument("--imgsz", type=int, default=640, help="YOLO-World input size")
    p.add_argument("--opset", type=int, default=12)

    # build implements the exported graphs; the CLIP graphs
    # come from the zoo pipeline (fetch_zoo_aibox_models.py).
    p = sub.add_parser("build", help="compile ONNX -> .rknn, optional sim check")
    common(p, with_onnx=True, models=["yoloworld", "mobilesam", "mobilesam_decoder"])
    p.add_argument("--dataset", help="calibration image list txt (quantized builds)")
    p.add_argument("--no-quant", action="store_true", help="fp build (no int8)")
    p.add_argument("--optimization-level", type=int, default=0)

    p = sub.add_parser("deploy", help="assemble DualMap model/npu from artifacts")
    p.add_argument("--rknn-dir", required=True)
    p.add_argument("--classes-file", required=True)
    p.add_argument("--out-dir", required=True)
    p.add_argument("--dualmap-repo",
                   help="clone/align the pinned li041/DualMap checkout here "
                        "(the NPU runtime backend)")

    args = ap.parse_args()
    if args.cmd == "export":
        cmd_export(args)
    elif args.cmd == "deploy":
        cmd_deploy(args)
    else:
        cmd_build(args)


if __name__ == "__main__":
    main()
