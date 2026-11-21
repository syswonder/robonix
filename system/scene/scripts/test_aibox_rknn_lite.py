#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Board-side RKNPU3 smoke test for Scene's converted DualMap models.

Loads a `.rknn` + `.weight` pair with `rknn3_toolkit_lite`, runs one real
image on the physical NPU and prints output statistics; with `--ref` (an
npz of `out0..` tensors saved by convert_aibox_models.py's simulator check)
it also prints the cosine similarity per output.

    python3 test_aibox_rknn_lite.py --rknn M.rknn --weight M.weight \
        --image IMG [--size 640] [--letterbox] [--extra-input TEXT.npy] \
        [--ref sim.npz] [--runs 3]
    python3 test_aibox_rknn_lite.py --rknn M.rknn --weight M.weight \
        --inputs ref.npz [--ref sim.npz]

`--extra-input` feeds a second (non-image) input, e.g. the YOLO-World text
embeddings. `--inputs` instead replays the `in0..inN` arrays stored by
convert_aibox_models.py's decoder reference (graphs without an image input,
e.g. mobilesam_decoder). The board's `rknn3_toolkit_lite` version must match
the PC toolkit that compiled the model (AIBOX: 1.0.4).
"""
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np


def letter_box(img, new_shape, pad_color=(114, 114, 114)):
    shape = img.shape[:2]
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    dw /= 2
    dh /= 2
    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    return cv2.copyMakeBorder(img, top, bottom, left, right,
                              cv2.BORDER_CONSTANT, value=pad_color)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--rknn", required=True)
    ap.add_argument("--weight", required=True)
    ap.add_argument("--image", help="real image for single-image graphs")
    ap.add_argument("--inputs", help="npz with in0..inN arrays to replay")
    ap.add_argument("--size", type=int, default=640)
    ap.add_argument("--letterbox", action="store_true",
                    help="aspect-preserving pad instead of squash resize")
    ap.add_argument("--extra-input", help="npy file for a second model input")
    ap.add_argument("--ref", help="npz with out0.. arrays from the PC simulator")
    ap.add_argument("--save", help="npz path to store board outputs")
    ap.add_argument("--runs", type=int, default=1, help="extra timed runs")
    args = ap.parse_args()
    if not args.image and not args.inputs:
        ap.error("--image or --inputs is required")

    from rknn3lite.api import RKNN3Lite

    rk = RKNN3Lite()
    t0 = time.time()
    if rk.load_rknn(model_path=args.rknn, weight_path=args.weight) != 0:
        raise SystemExit("load_rknn failed")
    print(f"load_rknn      {time.time() - t0:.3f}s")
    devices = rk.get_devices_id()
    print(f"devices        {devices}")
    t0 = time.time()
    if rk.init_runtime(target="rk1820", core_mask=0x01,
                       device_id=devices[0] if devices else None) != 0:
        raise SystemExit("init_runtime failed")
    print(f"init_runtime   {time.time() - t0:.3f}s")

    if args.inputs:
        replay = np.load(args.inputs)
        keys = sorted(k for k in replay.files if k.startswith("in"))
        if not keys:
            raise SystemExit(f"no in0.. arrays in {args.inputs}")
        inputs = [replay[k] for k in keys]
    else:
        img = cv2.cvtColor(cv2.imread(args.image), cv2.COLOR_BGR2RGB)
        if args.letterbox:
            img = letter_box(img, (args.size, args.size))
        else:
            img = cv2.resize(img, (args.size, args.size))
        x = np.expand_dims(img, 0)
        inputs = [x]
        if args.extra_input:
            extra = np.load(args.extra_input)
            if extra.ndim == 2:  # saved without the batch dim
                extra = extra[None]
            inputs.append(extra)

    outs = None
    for run in range(max(1, args.runs)):
        t0 = time.time()
        outs = rk.inference(inputs=inputs)
        print(f"inference[{run}] {time.time() - t0:.3f}s")
    for i, o in enumerate(outs):
        o = np.asarray(o, dtype=np.float32)
        print(f"out{i}: shape={o.shape} dtype={o.dtype} "
              f"min={np.min(o):.4g} max={np.max(o):.4g} nonzero={np.count_nonzero(o)}")
    if args.save:
        np.savez(args.save, **{f"out{i}": o for i, o in enumerate(outs)})
    if args.ref:
        ref = np.load(args.ref)
        for i, o in enumerate(outs):
            key = f"out{i}"
            if key not in ref:
                continue
            a = np.asarray(o, dtype=np.float32).reshape(-1)
            b = np.asarray(ref[key], dtype=np.float32).reshape(-1)
            if a.shape != b.shape:
                print(f"{key}: shape differs npu{a.shape} vs ref{b.shape}")
                continue
            cos = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9))
            print(f"{key}: cosine(npu, sim) = {cos:.5f}")
    rk.release()


if __name__ == "__main__":
    main()
