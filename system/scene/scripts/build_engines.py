#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Build and self-verify the scene TensorRT engines on the target board.

Runs on any TRT >= 10 machine (JetPack 6/7 Orin family) with only
numpy + tensorrt + cuda-bindings (+ onnxruntime for the CPU text towers) —
no torch. Native builds compile engines before managed startup;
start_native.sh uses --check to reject missing caches without compiling.

Engine files are named `<model>.trt<trtver>.sm<cc>.<onnx-hash12>.<build-hash12>.trt` so that
a TRT upgrade, a re-exported ONNX or a different board compute capability
automatically invalidates the cache — no stale engine is ever loaded.  Every
freshly built engine is replayed against the torch fp32 reference outputs
shipped beside the ONNX (parity .npz from export_models.py) and only accepted
if numerical thresholds pass; a failed gate discards the candidate and exits
non-zero. Each target checks its own build against the exported references.

The two text towers are parity-gated with CPU ORT. The SAM decoder uses a
static K=1 TRT profile and runs once per detection box.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
import tempfile

import numpy as np
import tensorrt as trt

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # system/scene
sys.path.insert(0, PKG)

from scene_service.ingest.perception_engines import (  # noqa: E402
    OrtSess, TrtRunner, compute_cap, engine_file, WORKSPACE, YOLO_NC_MAX,
)

DEFAULT_MODELS = os.path.join(PKG, "docker", "_weights", "onnx")
DEFAULT_CACHE = os.path.expanduser("~/.cache/robonix/scene-engines")


def log(*a):
    print(*a, flush=True)


# ── gate math ────────────────────────────────────────────────────────────────
def yolo_score_stage(feats: np.ndarray, e: np.ndarray, scales, biases, ns) -> np.ndarray:
    """The ContrastiveHead math cut out of the graph: normalize feats · normalize(E)
    × per-scale scale/bias + sigmoid.  fp64 to match the export-time reference."""
    f = feats[0].astype(np.float64)
    fn = f / np.linalg.norm(f, axis=0, keepdims=True)
    en = e[0].astype(np.float64)
    en = en / np.linalg.norm(en, axis=1, keepdims=True)
    d = fn.T @ en.T
    sv = np.concatenate([np.full(int(n), s) for s, n in zip(scales, ns)])
    bv = np.concatenate([np.full(int(n), b) for b, n in zip(biases, ns)])
    return (1.0 / (1.0 + np.exp(-(d * sv[:, None] + bv[:, None])))).T[None]


def check(out_name: str, got: np.ndarray, ref: np.ndarray, kind: str) -> str:
    a, b = got.astype(np.float64), ref.astype(np.float64)
    cos = float((a * b).sum() / (np.linalg.norm(a) * np.linalg.norm(b)))
    mx = float(np.abs(a - b).max())
    if kind == "boxes":
        ok, thr = cos >= 0.999 and mx <= 8.0, "cos>=0.999 maxabs<=8px"
    elif kind == "cls_score":
        ok, thr = cos >= 0.999 and mx <= 0.02, "cos>=0.999 maxabs<=2e-2"
    elif kind == "emb":
        ok, thr = cos >= 0.999, "cos>=0.999"
    else:
        ok, thr = cos >= 0.999, "cos>=0.999"
    tag = "OK " if ok else "FAIL"
    return f"{tag} {out_name}({kind}): cos={cos:.6f} maxabs={mx:.3e} [{thr}]"


# ── model specs ──────────────────────────────────────────────────────────────
# Each: builder profile (TRT models), then gates.  A gate = feeds (engine input
# -> npz key) + checks (engine output / derived value -> npz reference key).
SPECS = [
    dict(
        name="yolo-world-cut", runtime="trt",
        gates=[dict(
            feeds={"images": "images", "txt_feats": "txt_feats"},
            checks=[("boxes", "y_box", "boxes"), ("cls_score", "y_cls", "cls_score")],
        )],
    ),
    dict(name="yolo-world-text", runtime="ort",
         gates=[dict(feeds={"tokens": "tokens"}, checks=[("txt_feats", "ref", "emb")]),
                dict(feeds={"tokens": "tokens10"}, checks=[("txt_feats", "ref10", "emb")])]),
    dict(name="sam-encoder", runtime="trt",
         gates=[dict(feeds={"image": "image"}, checks=[("image_embed", "ref", "cos")])]),
    dict(name="sam-decoder", runtime="trt", per_box=True,
         # boxes are called one at a time (static (1,4) profile): the batched-K
         # tactic costs ~38MB device scratch per profiled box (~1.2GB at K_max
         # 32) on a 20MB model — unacceptable on the 8GB board; K=1 runs ~10ms
         gates=[dict(feeds={"image_embeddings": "enc_ref", "boxes": "boxes2"},
                     checks=[("masks", "masks_ref2", "cos"), ("iou", "iou_ref2", "cos")]),
                dict(feeds={"image_embeddings": "enc_ref", "boxes": "boxes5"},
                     checks=[("masks", "masks_ref5", "cos"), ("iou", "iou_ref5", "cos")])]),
    dict(name="clip-visual", runtime="trt",
         gates=[dict(feeds={"images": "images"}, checks=[("image_feats", "ref", "cos")])]),
    dict(name="clip-text", runtime="ort",
         gates=[dict(feeds={"tokens": "tokens"}, checks=[("text_feats", "ref", "emb")]),
                dict(feeds={"tokens": "tokens1"}, checks=[("text_feats", "ref1", "emb")])]),
]


def run_gate(spec, d, sess):
    """sess.run(feats_dict) -> outputs dict; wired for both TRT and ORT.
    per_box specs (the K=1 decoder) replay the gate feeds row by row."""
    all_ok = True
    for gi, gate in enumerate(spec["gates"]):
        feeds = {k: d[v] for k, v in gate["feeds"].items()}
        if spec.get("per_box"):
            # the K=1 decoder is called with one box row; shared feeds (the
            # image embedding) stay whole — only boxes take row i
            for i in range(feeds["boxes"].shape[0]):
                outs = sess.run({**feeds, "boxes": feeds["boxes"][i:i + 1]})
                for out_name, ref_key, kind in gate["checks"]:
                    line = check(out_name, outs[out_name], d[ref_key][i:i + 1], kind)
                    if line.startswith("FAIL"):
                        all_ok = False
                    log(f"    gate{gi} box{i} {line}")
            continue
        outs = sess.run(feeds)
        vals = dict(outs)
        if spec["name"] == "yolo-world-cut":
            vals["cls_score"] = yolo_score_stage(
                outs["feats"], d["txt_feats"], d["scales"], d["biases"], d["ns"])
        for out_name, ref_key, kind in gate["checks"]:
            line = check(out_name, vals[out_name], d[ref_key], kind)
            if line.startswith("FAIL"):
                all_ok = False
            log(f"    gate{gi} {line}")
    return all_ok


def build_trt_engine(onnx_path: str, eng_path: str, cache_dir: str, d) -> float:
    """Returns build seconds.  Profile: YOLO txt_feats dynamic (1..YOLO_NC_MAX,
    opt at the shipped table); sam-decoder boxes static K=1 (per-box calls, see
    the spec); everything else pinned to the reference shapes."""
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network(0)
    parser = trt.OnnxParser(network, logger)
    assert parser.parse_from_file(onnx_path), (
        "onnx parse failed: " + "\n".join(str(parser.get_error(i)) for i in range(parser.num_errors)))
    profile = builder.create_optimization_profile()
    for i in range(network.num_inputs):
        inp = network.get_input(i)
        name = inp.name
        if name == "txt_feats":
            nc = int(d["txt_feats"].shape[1])
            profile.set_shape(name, trt.Dims((1, 1, 512)), trt.Dims((1, nc, 512)),
                              trt.Dims((1, YOLO_NC_MAX, 512)))
        elif name == "boxes":
            dims = trt.Dims((1, 4))  # static K=1: see the sam-decoder spec
            profile.set_shape(name, dims, dims, dims)
        else:
            # every other input is fully static in our model set; pin it to the
            # reference shape from the parity npz (key usually equals the input
            # name; the sam-decoder embedding reference is the one exception)
            ref = d[name] if name in d else d["enc_ref"]
            profile.set_shape(name, trt.Dims(ref.shape), trt.Dims(ref.shape),
                              trt.Dims(ref.shape))
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, WORKSPACE)
    # timing cache: same-board rebuilds land 30-50% faster
    tcache = os.path.join(cache_dir, f"timing-cache-trt{trt.__version__}.bin")
    if os.path.exists(tcache):
        config.set_timing_cache(config.create_timing_cache(open(tcache, "rb").read()), False)
    config.add_optimization_profile(profile)
    t0 = time.time()
    ser = builder.build_serialized_network(network, config)
    assert ser is not None, "TRT build failed"
    with open(eng_path, "wb") as f:
        f.write(ser)
    tc = config.get_timing_cache()
    if tc is not None:
        with open(tcache, "wb") as f:
            f.write(memoryview(tc.serialize()))
    return time.time() - t0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--models-dir", default=DEFAULT_MODELS)
    ap.add_argument("--cache-dir", default=os.environ.get("SCENE_ENGINE_CACHE_DIR", DEFAULT_CACHE))
    ap.add_argument("--only", action="append", help="build just this model (repeatable)")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--force", action="store_true", help="rebuild even if fingerprint matches")
    mode.add_argument("--check", action="store_true",
                      help="check model files and caches without building missing engines")
    ap.add_argument("--verify", action="store_true",
                    help="re-run the parity gate on cached engines too")
    args = ap.parse_args()
    os.makedirs(args.cache_dir, exist_ok=True)
    cc = compute_cap()
    log(f"build_engines: TRT {trt.__version__} sm_{cc} models={args.models_dir} cache={args.cache_dir}")

    failed = False
    for spec in SPECS:
        name = spec["name"]
        if args.only and name not in args.only:
            continue
        onnx_path = os.path.join(args.models_dir, f"{name}.onnx")
        npz_path = os.path.join(args.models_dir, f"{name}.npz")
        if not (os.path.isfile(onnx_path) and os.path.isfile(npz_path)):
            log(f"[{name}] FAIL missing {onnx_path if not os.path.isfile(onnx_path) else npz_path}")
            failed = True
            continue
        d = dict(np.load(npz_path))
        t_start = time.time()

        if spec["runtime"] == "ort":
            # text towers: no engine file; gate the ONNX itself with CPU ORT
            sess = OrtSess(onnx_path)
            log(f"[{name}] ORT CPU gate ({os.path.getsize(onnx_path) / 2**20:.0f}MB onnx)")
            if not run_gate(spec, d, sess):
                failed = True
            sess = None
            continue

        eng_path = engine_file(name, args.models_dir, args.cache_dir)
        cached = os.path.isfile(eng_path) and not args.force
        if args.check and not cached:
            log(f"[{name}] FAIL missing verified engine; run scripts/build_engines.py before startup")
            failed = True
            continue
        if cached:
            log(f"[{name}] cached {os.path.basename(eng_path)} "
                f"({os.path.getsize(eng_path) / 2**20:.0f}MB)")
            if not args.verify:
                continue

        pending = None
        sess = None
        try:
            if cached:
                candidate = eng_path
            else:
                fd, pending = tempfile.mkstemp(prefix=f".{name}.", suffix=".trt", dir=args.cache_dir)
                os.close(fd)
                candidate = pending
                log(f"[{name}] building fp32 engine ...")
                secs = build_trt_engine(onnx_path, candidate, args.cache_dir, d)
                log(f"[{name}] built in {secs:.0f}s, gating ...")
            sess = TrtRunner(candidate)
            if not run_gate(spec, d, sess):
                raise RuntimeError("parity gate failed")
            sess.close()
            sess = None
            if pending is not None:
                os.replace(pending, eng_path)
            log(f"[{name}] OK ({time.time() - t_start:.0f}s total)")
        except Exception as e:
            log(f"[{name}] FAIL: {e}")
            if cached:
                os.remove(eng_path)
            failed = True
        finally:
            if sess is not None:
                sess.close()
            if pending is not None and os.path.exists(pending):
                os.remove(pending)

    if failed:
        log("build_engines: FAILED")
        return 1
    log("build_engines: all engines verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
