#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Fetch Scene's DualMap CLIP graphs from airockchip sources and verify them
for the AIBOX (RK1828). The yoloworld and mobilesam graphs are self-exported
by ``convert_aibox_models.py`` from the ultralytics weights instead.

The zoo ships ready-made CLIP ONNX; the stock examples only target the
rk35xx boards, so the forked rknn-model-zoo (branch ``aibox-dualmap``)
teaches their ``convert.py`` the rk1820 target:

- ``clip_image``  zoo ``clip_images.onnx`` (ViT-B/32 visual, dynamic input
                  fixed to 224) is the deployed graph, with the *correct*
                  OpenAI std (0.2613/0.2758).

    # any python - fetch zoo ONNX into <work>/models/zoo
    fetch_zoo_aibox_models.py download [--work-dir ~/aibox-npu]

    # ML venv (torch/clip) - torch reference for the board-side check
    fetch_zoo_aibox_models.py export [--image IMG]

    # toolkit venv (rknn) - zoo convert.py -> fp16 graph
    fetch_zoo_aibox_models.py verify

    # from the PC - board smoke test per deploy-aibox-scene
    fetch_zoo_aibox_models.py board [--host aibox] [--board-dir DIR] [--image IMG]

Artifacts land in ``<work>/models/zoo`` (ONNX, torch refs) and
``<work>/rknn-zoo`` (.rknn/.weight); the live board bundle is never touched.
"""
from __future__ import annotations

import argparse
import hashlib
import os
import shlex
import subprocess
import sys
import urllib.request

ZOO_BASE = ("https://ftrg.zbox.filez.com/v2/delivery/data"
            "/95f00b0fc900458ba134f8b180b3f7a1/examples")
CLIP_MEAN = (0.48145466, 0.4578275, 0.40821073)
CLIP_STD = (0.26862954, 0.26130258, 0.27577711)

DOWNLOADS = {
    "clip_images.onnx": "clip/clip_images.onnx",
}
# graph -> zoo convert.py, board test size
VERIFY = {
    "clip_image": (os.path.join("examples", "clip", "python", "images",
                                "convert.py"), 224),
}


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def zoo_dir(work: str) -> str:
    return os.path.join(work, "models", "zoo")


def sim_image(path: str, size: int):
    """The exact image the simulator/board test feeds: RGB uint8, squash
    resize; mean/std are absorbed by the graph config."""
    import cv2

    img = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2RGB)
    return cv2.resize(img, (size, size))


# ── download (any python) ───────────────────────────────────────────────────


def cmd_download(args: argparse.Namespace) -> None:
    os.makedirs(zoo_dir(args.work_dir), exist_ok=True)
    for fname, zoo_path in DOWNLOADS.items():
        dst = os.path.join(zoo_dir(args.work_dir), fname)
        if os.path.isfile(dst) and os.path.getsize(dst) > 1 << 20:
            print(f"[download] {fname}: present")
        else:
            url = f"{ZOO_BASE}/{zoo_path}"
            print(f"[download] {fname} <- {url}")
            urllib.request.urlretrieve(url, dst)
            if os.path.getsize(dst) < 1 << 20:
                raise SystemExit(f"{dst} suspiciously small, download failed")
        print(f"           sha256 {sha256(dst)}")


# ── export (ML venv: torch/clip) ────────────────────────────────────────────


def cmd_export(args: argparse.Namespace) -> None:
    for model in args.models.split(","):
        if model not in VERIFY:
            raise SystemExit(f"unknown model {model!r} (choices: {', '.join(VERIFY)})")
    _clip_torch_ref(args)
    print("[export] done (torch reference in models/zoo/clip_image_torch.npz)")


def _clip_torch_ref(args: argparse.Namespace) -> None:
    import numpy as np
    import torch
    import clip

    model, _ = clip.load("ViT-B/32", device="cpu")
    visual = model.visual.eval()
    img = sim_image(args.image, 224)
    x = torch.from_numpy(img.copy()).permute(2, 0, 1).float() / 255.0
    x = (x - torch.tensor(CLIP_MEAN).view(3, 1, 1)) / torch.tensor(CLIP_STD).view(3, 1, 1)
    with torch.no_grad():
        feat = visual(x[None])
    out = os.path.join(zoo_dir(args.work_dir), "clip_image_torch.npz")
    np.savez(out, out0=feat[0].numpy(), feature=feat[0].numpy())
    print(f"[export] torch ViT-B/32 feature {tuple(feat.shape)} -> {out}")


# ── verify (toolkit venv: zoo convert.py + simulator) ──────────────────────


def _zoo_convert(args: argparse.Namespace, model: str, out_dir: str) -> str:
    """Run the zoo example's own convert.py for the graph."""
    script = os.path.join(args.zoo_dir, VERIFY[model][0])
    if not os.path.isfile(script):
        raise SystemExit(f"missing zoo checkout {script} "
                         f"(airockchip/rknn_model_zoo, branch aibox-dualmap)")
    out = os.path.join(out_dir, f"{model}.rknn")
    subprocess.run([args.toolkit_python, script,
                    zoo_onnx(args.work_dir, model), "rk1820", "fp", out],
                   check=True, cwd=os.path.dirname(script))
    return out


def zoo_onnx(work: str, model: str) -> str:
    fname = {"clip_image": "clip_images.onnx"}[model]
    return os.path.join(zoo_dir(work), fname)


def cmd_verify(args: argparse.Namespace) -> None:
    out_dir = os.path.join(args.work_dir, "rknn-zoo")
    os.makedirs(out_dir, exist_ok=True)

    for model in args.models.split(","):
        if model not in VERIFY:
            raise SystemExit(f"unknown model {model!r} (choices: {', '.join(VERIFY)})")
        onnx_path = zoo_onnx(args.work_dir, model)
        if not os.path.isfile(onnx_path):
            raise SystemExit(f"missing {onnx_path} - run download/export first")
        rknn_path = _zoo_convert(args, model, out_dir)
        print(f"[verify] {model}: {rknn_path}")
    # The PC simulator can only run models built in the same process
    # (load_rknn is board-only, see the `Scene NPU models` section in
    # examples/webots-aibox/README.md) - the numeric check happens on the
    # board against the torch references.
    print(f"[verify] artifacts in {out_dir}; numeric check: the 'board' stage "
          f"(cosine vs <work>/models/zoo/*_torch.npz)")


# ── board (deploy-aibox-scene verification) ────────────────────────────────


def cmd_board(args: argparse.Namespace) -> None:
    out_dir = os.path.join(args.work_dir, "rknn-zoo")
    files = [os.path.join(os.path.dirname(__file__), "test_aibox_rknn_lite.py"),
             args.image]
    runs = []
    for model, (_, size) in VERIFY.items():
        torch_ref = os.path.join(zoo_dir(args.work_dir), f"{model}_torch.npz")
        art = [os.path.join(out_dir, f"{model}{suf}") for suf in
               (".rknn", ".weight")]
        if not all(os.path.isfile(a) for a in art) or not os.path.isfile(torch_ref):
            print(f"[board] {model}: artifacts missing, skipped")
            continue
        files += art + [torch_ref]
        runs.append(f"python3 test_aibox_rknn_lite.py --rknn {model}.rknn "
                    f"--weight {model}.weight --image {os.path.basename(args.image)} "
                    f"--size {size} --ref {model}_torch.npz --runs 3")
    if not runs:
        raise SystemExit("nothing to test - run verify first")
    subprocess.run(["ssh", "-o", "BatchMode=yes", args.host,
                    f"mkdir -p {shlex.quote(args.board_dir)}"], check=True)
    subprocess.run(["scp", "-o", "BatchMode=yes", *files,
                    f"{args.host}:{args.board_dir}/"], check=True)
    subprocess.run(["ssh", "-o", "BatchMode=yes", args.host,
                    f"cd {shlex.quote(args.board_dir)} && " + " && ".join(runs)], check=True)
    print(f"[board] done. The live bundle at .../model/npu/ is untouched; "
          f"swap clip_image.rknn/.weight there manually after review.")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--work-dir", default=os.path.expanduser("~/aibox-npu"))
    ap.add_argument("--zoo-dir", default=os.path.expanduser("~/aibox-npu/rknn-model-zoo"),
                    help="rknn_model_zoo checkout (branch aibox-dualmap)")
    ap.add_argument("--toolkit-python", default=os.path.expanduser(
        "~/aibox-npu/venv-toolkit/bin/python"))
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("download", help="fetch zoo ONNX into <work>/models/zoo")
    p = sub.add_parser("export", help="clip torch reference (ML venv)")
    p.add_argument("--models", default="clip_image")
    p.add_argument("--image", default=os.path.expanduser(
        "~/aibox-npu/rknn-model-zoo/examples/yolo_world/model/bus.jpg"))
    p = sub.add_parser("verify", help="zoo convert (toolkit venv)")
    p.add_argument("--models", default="clip_image")
    p.add_argument("--image", default=os.path.expanduser(
        "~/aibox-npu/rknn-model-zoo/examples/yolo_world/model/bus.jpg"))
    p = sub.add_parser("board", help="board smoke test via ssh")
    p.add_argument("--host", default="aibox")
    p.add_argument("--board-dir", default="/tmp/aibox-npu-zoo",
                   help="scratch directory on the board for the test files")
    p.add_argument("--image", default=os.path.expanduser(
        "~/aibox-npu/rknn-model-zoo/examples/yolo_world/model/bus.jpg"))

    args = ap.parse_args()
    {"download": cmd_download, "export": cmd_export,
     "verify": cmd_verify, "board": cmd_board}[args.cmd](args)


if __name__ == "__main__":
    main()
