"""Torch-free TRT/ORT inference and NumPy preprocessing for Scene.

YOLO-World, SAM encoder/decoder and CLIP visual use TRT; text towers use CPU
ORT. Optional runtimes are imported lazily. scripts/build_engines.py builds
and verifies fingerprinted engines before runtime.
"""
from __future__ import annotations

import gzip
import hashlib
import html
import json
import logging
import os
import subprocess
from functools import lru_cache

# The tokenizer pattern uses \p{L}/\p{N} unicode properties, which stdlib re
# cannot compile — openai/CLIP and open_clip both run it on the regex module.
import regex as re

import numpy as np

from ..inference_config import LEGACY_MODEL_METADATA

try:  # scene dependency; only the letterbox paths need it
    import cv2
except Exception:  # pragma: no cover - cv2 is part of every scene deploy
    cv2 = None

log = logging.getLogger(__name__)

PKG = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # system/scene package root
DEFAULT_MODELS_DIR = os.path.join(PKG, "docker", "_weights", "onnx")
DEFAULT_CACHE_DIR = os.environ.get("SCENE_ENGINE_CACHE_DIR",
                                   os.path.expanduser("~/.cache/robonix/scene-engines"))

TRT_MODELS = ("yolo-world-cut", "sam-encoder", "sam-decoder", "clip-visual")
ORT_MODELS = ("yolo-world-text", "clip-text")

YOLO_IMGSZ = 672
SAM_IMGSZ = LEGACY_MODEL_METADATA["sam_image_size"]
CLIP_CROP = LEGACY_MODEL_METADATA["clip_image_size"]
CLIP_MEAN = np.array(LEGACY_MODEL_METADATA["clip_mean"], dtype=np.float32)
CLIP_STD = np.array(LEGACY_MODEL_METADATA["clip_std"], dtype=np.float32)
SAM_MEAN = np.array([123.675, 116.28, 103.53], dtype=np.float32).reshape(3, 1, 1)
SAM_STD = np.array([58.395, 57.12, 57.375], dtype=np.float32).reshape(3, 1, 1)

# ultralytics 8.4 NMS constants
NMS_IOU = 0.7        # predict default iou
NMS_MAX_DET = 300    # predict default max_det, before Scene's own top-k cap
NMS_MAX_NMS = 30000  # candidate cap before the greedy pass
NMS_MAX_WH = 7680    # per-class box offset for batched NMS

YOLO_NC_MAX = 256
WORKSPACE = 1 << 30
# Increment when builder semantics change beyond the settings fingerprinted below.
ENGINE_BUILD_VERSION = 1


def compute_cap() -> str:
    out = subprocess.run(
        ["nvidia-smi", "--query-gpu=compute_cap", "--format=csv,noheader"],
        capture_output=True, text=True, timeout=30)
    cc = out.stdout.strip().splitlines()[0].strip()  # e.g. "8.7"
    return cc.replace(".", "")


def onnx_hash12(onnx_path: str) -> str:
    digest = hashlib.sha256()
    for path in (onnx_path, onnx_path + ".data"):
        if path.endswith(".data") and not os.path.exists(path):
            continue
        with open(path, "rb") as source:
            for chunk in iter(lambda: source.read(1 << 20), b""):
                digest.update(chunk)
    return digest.hexdigest()[:12]


def engine_file(name: str, models_dir: str, cache_dir: str) -> str:
    """Engine path fingerprinted by model, hardware and build settings."""
    import tensorrt as trt  # lazy: not needed for ORT-only callers

    onnx_path = os.path.join(models_dir, f"{name}.onnx")
    settings = json.dumps({"version": ENGINE_BUILD_VERSION, "workspace": WORKSPACE,
                           "yolo_nc_max": YOLO_NC_MAX, "sam_decoder_k": 1}, sort_keys=True)
    build_hash = hashlib.sha256(settings.encode()).hexdigest()[:12]
    return os.path.join(
        cache_dir, f"{name}.trt{trt.__version__}.sm{compute_cap()}.{onnx_hash12(onnx_path)}.{build_hash}.trt")


# ── CUDA device buffers via cuda-bindings (torch stays out of the process) ──
def _cudart():
    from cuda.bindings import runtime as rt
    return rt


class DevBuf:
    def __init__(self, nbytes: int):
        rt = _cudart()
        err, self.ptr = rt.cudaMalloc(nbytes)
        assert int(err) == 0, f"cudaMalloc({nbytes}) failed: {err}"
        self.nbytes = nbytes

    def h2d(self, arr: np.ndarray) -> None:
        rt = _cudart()
        arr = np.ascontiguousarray(arr)
        err, = rt.cudaMemcpy(self.ptr, arr.ctypes.data, arr.nbytes,
                             rt.cudaMemcpyKind.cudaMemcpyHostToDevice)
        assert int(err) == 0, f"H2D memcpy failed: {err}"

    def d2h(self, shape, dtype) -> np.ndarray:
        rt = _cudart()
        out = np.empty(shape, dtype=dtype)
        err, = rt.cudaMemcpy(out.ctypes.data, self.ptr, out.nbytes,
                             rt.cudaMemcpyKind.cudaMemcpyDeviceToHost)
        assert int(err) == 0, f"D2H memcpy failed: {err}"
        return out

    def __del__(self):
        try:
            _cudart().cudaFree(self.ptr)
        except Exception:
            pass


_TRT_TO_NP = {"FLOAT": "float32", "HALF": "float16",
              "INT32": "int32", "INT64": "int64", "BOOL": "bool"}


class TrtRunner:
    """One engine, one context; run(feeds: {name: np}) -> {name: np}."""

    def __init__(self, eng_path: str):
        import tensorrt as trt
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.rt = trt.Runtime(self.logger)
        self.eng = self.rt.deserialize_cuda_engine(open(eng_path, "rb").read())
        assert self.eng is not None, f"deserialize failed: {eng_path}"
        self.ctx = self.eng.create_execution_context()
        self.bufs: dict[str, DevBuf] = {}

    @property
    def device_memory_bytes(self) -> int:
        try:
            return int(self.eng.device_memory_size)
        except Exception:
            return 0

    def run(self, feeds: dict) -> dict:
        import tensorrt as trt
        outs = {}
        for name, arr in feeds.items():
            if not self.ctx.set_input_shape(name, arr.shape):
                raise ValueError(f"Invalid input shape for {name}: {arr.shape}")
        for name, arr in feeds.items():
            b = self._buffer(name, arr.nbytes)
            b.h2d(arr)
            if not self.ctx.set_tensor_address(name, b.ptr):
                raise RuntimeError(f"Cannot bind tensor {name}")
        for i in range(self.eng.num_io_tensors):
            name = self.eng.get_tensor_name(i)
            if self.eng.get_tensor_mode(name) == trt.TensorIOMode.OUTPUT:
                shape = tuple(self.ctx.get_tensor_shape(name))
                dt = str(self.eng.get_tensor_dtype(name)).split(".")[-1]  # DataType.FLOAT
                npdt = np.dtype(_TRT_TO_NP[dt])
                if any(dim < 0 for dim in shape):
                    raise ValueError(f"Unresolved output shape for {name}: {shape}")
                b = self._buffer(name, int(np.prod(shape)) * npdt.itemsize)
                if not self.ctx.set_tensor_address(name, b.ptr):
                    raise RuntimeError(f"Cannot bind tensor {name}")
                outs[name] = (b, shape, npdt)
        err, = _cudart().cudaDeviceSynchronize()
        assert int(err) == 0
        if not self.ctx.execute_async_v3(0):
            raise RuntimeError("TensorRT execution failed")
        err, = _cudart().cudaDeviceSynchronize()
        assert int(err) == 0
        return {name: b.d2h(shape, npdt) for name, (b, shape, npdt) in outs.items()}

    def _buffer(self, name: str, nbytes: int) -> DevBuf:
        buf = self.bufs.get(name)
        if buf is None or buf.nbytes < nbytes:
            self.bufs.pop(name, None)
            del buf
            self.bufs[name] = DevBuf(nbytes)
        return self.bufs[name]

    def close(self):
        self.ctx = None
        self.eng = None
        self.bufs.clear()


class OrtSess:
    def __init__(self, onnx_path: str):
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.sess = ort.InferenceSession(onnx_path, so, providers=["CPUExecutionProvider"])
        # The tokenizer emits int64 but the exported clip-text declares int32;
        # onnxruntime does not auto-cast, so coerce feeds to declared dtypes.
        self._feed_dtype = {i.name: {"tensor(int32)": np.int32,
                                     "tensor(int64)": np.int64}.get(i.type)
                            for i in self.sess.get_inputs()}

    def run(self, feeds: dict) -> dict:
        feeds = {k: v.astype(self._feed_dtype[k], copy=False)
                 if self._feed_dtype.get(k) and self._feed_dtype[k] != v.dtype else v
                 for k, v in feeds.items()}
        names = [o.name for o in self.sess.get_outputs()]
        return dict(zip(names, self.sess.run(None, feeds)))


# ── Tokenization: vendored OpenAI SimpleTokenizer ────────────────────────────
# Copied from open_clip 2.24 tokenizer.py (itself copied from openai/CLIP).
# clean='lower' is what open_clip uses for ViT-B-32, and the identical vocab +
# pattern + BPE is what ultralytics' clip-anytorch tokenizer uses, so one
# tokenizer serves both text towers.  Returns numpy int64 (n, 77) rows of
# [sot] + tokens + [eot], zero-padded, truncation keeping the eot — the same
# array open_clip's __call__ produced as a torch long tensor.
@lru_cache()
def _bytes_to_unicode():
    bs = (list(range(ord("!"), ord("~") + 1))
          + list(range(ord("¡"), ord("¬") + 1))
          + list(range(ord("®"), ord("ÿ") + 1)))
    cs = bs[:]
    n = 0
    for b in range(2 ** 8):
        if b not in bs:
            bs.append(b)
            cs.append(2 ** 8 + n)
            n += 1
    cs = [chr(n) for n in cs]
    return dict(zip(bs, cs))


def _get_pairs(word):
    return set(zip(word, word[1:]))


class SimpleTokenizer:
    def __init__(self, bpe_path: str = None, context_length: int = 77):
        if bpe_path is None:
            bpe_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "assets", "bpe_simple_vocab_16e6.txt.gz")
        self.byte_encoder = _bytes_to_unicode()
        merges = gzip.open(bpe_path).read().decode("utf-8").split("\n")
        merges = merges[1:49152 - 256 - 2 + 1]
        merges = [tuple(merge.split()) for merge in merges]
        vocab = list(_bytes_to_unicode().values())
        vocab = vocab + [v + "</w>" for v in vocab]
        for merge in merges:
            vocab.append("".join(merge))
        special_tokens = ["<start_of_text>", "<end_of_text>"]
        vocab.extend(special_tokens)
        self.encoder = dict(zip(vocab, range(len(vocab))))
        self.bpe_ranks = dict(zip(merges, range(len(merges))))
        self.cache = {t: t for t in special_tokens}
        special = "|".join(special_tokens)
        self.pat = re.compile(
            special + r"""|'s|'t|'re|'ve|'m|'ll|'d|[\p{L}]+|[\p{N}]|[^\s\p{L}\p{N}]+""",
            re.IGNORECASE,
        )
        self.sot_token_id = self.encoder[special_tokens[0]]
        self.eot_token_id = self.encoder[special_tokens[1]]
        self.context_length = context_length

    def bpe(self, token: str) -> str:
        if token in self.cache:
            return self.cache[token]
        word = tuple(token[:-1]) + (token[-1] + "</w>",)
        pairs = _get_pairs(word)
        if not pairs:
            return token + "</w>"
        while True:
            bigram = min(pairs, key=lambda pair: self.bpe_ranks.get(pair, float("inf")))
            if bigram not in self.bpe_ranks:
                break
            first, second = bigram
            new_word = []
            i = 0
            while i < len(word):
                try:
                    j = word.index(first, i)
                    new_word.extend(word[i:j])
                    i = j
                except ValueError:
                    new_word.extend(word[i:])
                    break
                if word[i] == first and i < len(word) - 1 and word[i + 1] == second:
                    new_word.append(first + second)
                    i += 2
                else:
                    new_word.append(word[i])
                    i += 1
            word = tuple(new_word)
            if len(word) == 1:
                break
            pairs = _get_pairs(word)
        out = " ".join(word)
        self.cache[token] = out
        return out

    def _basic_clean(self, text: str) -> str:
        try:
            import ftfy
            text = ftfy.fix_text(text)
        except ImportError:
            pass  # ftfy fixes mojibake; class labels and user queries are clean already
        text = html.unescape(html.unescape(text))
        return text.strip()

    def encode(self, text: str) -> list[int]:
        bpe_tokens: list[int] = []
        text = re.sub(r"\s+", " ", self._basic_clean(text)).strip().lower()
        for token in re.findall(self.pat, text):
            token = "".join(self.byte_encoder[b] for b in token.encode("utf-8"))
            bpe_tokens.extend(self.encoder[t] for t in self.bpe(token).split(" "))
        return bpe_tokens

    def __call__(self, texts, context_length=None) -> np.ndarray:
        if isinstance(texts, str):
            texts = [texts]
        n_ctx = self.context_length if context_length is None else context_length
        rows = np.zeros((len(texts), n_ctx), dtype=np.int64)
        for i, text in enumerate(texts):
            tokens = [self.sot_token_id] + self.encode(text) + [self.eot_token_id]
            if len(tokens) > n_ctx:
                tokens = tokens[:n_ctx]
                tokens[-1] = self.eot_token_id
            rows[i, :len(tokens)] = tokens
        return rows


# ── Preprocessing (bit-matched to the ultralytics / open_clip pipelines) ─────
def _letterbox(img: np.ndarray, new_shape, center: bool = True, pad: int = 114) -> np.ndarray:
    """ultralytics LetterBox (auto=False, scaleup=True) on an HWC uint8 image."""
    h, w = img.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    r = min(new_shape[0] / h, new_shape[1] / w)
    new_unpad = (round(w * r), round(h * r))  # (w, h)
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    if center:
        dw /= 2
        dh /= 2
    top = round(dh - 0.1) if center else 0
    left = round(dw - 0.1) if center else 0
    if (w, h) != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    canvas = np.full((new_shape[0], new_shape[1], img.shape[2]), pad, dtype=img.dtype)
    canvas[top:top + img.shape[0], left:left + img.shape[1]] = img
    return canvas


def yolo_preprocess(bgr: np.ndarray) -> np.ndarray:
    """(3, 672, 672) float32 (call sites add the batch dim): letterbox square,
    BGR->RGB (ultralytics flips numpy input, assuming BGR), HWC->CHW, /255."""
    im = _letterbox(bgr, YOLO_IMGSZ, center=True)
    im = im[:, :, ::-1].transpose(2, 0, 1)  # flip channels, HWC->CHW
    return np.ascontiguousarray(im, dtype=np.float32) / 255.0


def sam_preprocess(bgr: np.ndarray, image_size: int = SAM_IMGSZ) -> np.ndarray:
    """Top-left letterbox and normalize at the exported SAM input size."""
    im = _letterbox(bgr, image_size, center=False)
    im = im[:, :, ::-1].transpose(2, 0, 1).astype(np.float32)
    im = (im - SAM_MEAN) / SAM_STD
    return np.ascontiguousarray(im)


def sam_scale_boxes(xyxy: np.ndarray, src_hw: tuple[int, int], image_size: int = SAM_IMGSZ) -> np.ndarray:
    """Scale boxes into the encoder input without rounding prompt coordinates."""
    r = min(image_size / src_hw[0], image_size / src_hw[1])
    return np.asarray(xyxy, dtype=np.float32) * r


def yolo_unletterbox(xyxy: np.ndarray, src_hw: tuple[int, int]) -> np.ndarray:
    """Engine boxes (672-letterbox space) -> source pixels: the inverse of
    ops.scale_boxes as ultralytics predict applies it after NMS (float pad,
    clamped to the frame) — NMS itself runs in letterbox space on both
    paths, so the only difference is this shared transform."""
    h, w = src_hw
    gain = min(YOLO_IMGSZ / h, YOLO_IMGSZ / w)
    pad_x = (YOLO_IMGSZ - w * gain) / 2
    pad_y = (YOLO_IMGSZ - h * gain) / 2
    out = np.asarray(xyxy, dtype=np.float32).copy()
    out[:, [0, 2]] -= pad_x
    out[:, [1, 3]] -= pad_y
    out /= gain
    out[:, [0, 2]] = out[:, [0, 2]].clip(0, w)
    out[:, [1, 3]] = out[:, [1, 3]].clip(0, h)
    return out


def sam_postprocess_masks(logits: np.ndarray, src_hw: tuple[int, int]) -> np.ndarray:
    """Decoder logits (K,1,256,256) -> bool masks (K,H,W) at the source
    resolution: undo the letterbox in mask space (content sits top-left),
    bilinear-resize the content crop to the source size, threshold at 0.0 —
    the ultralytics postprocess (scale_masks padding=False + mask_threshold)."""
    k = logits.shape[0]
    m = logits[:, 0]  # (K,256,256)
    im1_h = im1_w = m.shape[1]
    im0_h, im0_w = src_hw
    if im1_h != im0_h or im1_w != im0_w:
        gain = min(im1_h / im0_h, im1_w / im0_w)
        pad_w = im1_w - round(im0_w * gain)
        pad_h = im1_h - round(im0_h * gain)
        bottom = im1_h - round(pad_h + 0.1)  # top/left stay 0 (padding=False)
        right = im1_w - round(pad_w + 0.1)
        m = m[:, :bottom, :right]
        out = np.zeros((k, im0_h, im0_w), dtype=np.float32)
        for i in range(k):
            out[i] = cv2.resize(m[i], (im0_w, im0_h), interpolation=cv2.INTER_LINEAR)
        m = out
    return m > 0.0  # model.mask_threshold


def clip_crop_crops(rgb: np.ndarray, xyxy: np.ndarray, padding: int = 20,
                    image_size: int = CLIP_CROP, mean=CLIP_MEAN, std=CLIP_STD) -> np.ndarray:
    """Apply the exported CLIP resize, center crop and normalization."""
    from PIL import Image
    from .perception_images import clip_crops

    crops = []
    for crop in clip_crops(rgb, xyxy, padding):
        cw, ch = crop.size
        if (cw <= ch and cw != image_size) or (ch <= cw and ch != image_size):
            if cw < ch:
                ow, oh = image_size, int(image_size * ch / cw)
            else:
                ow, oh = int(image_size * cw / ch), image_size
            crop = crop.resize((ow, oh), Image.BICUBIC)
        cw, ch = crop.size
        top = int(round((ch - image_size) / 2.0))
        left = int(round((cw - image_size) / 2.0))
        crop = crop.crop((left, top, left + image_size, top + image_size))

        arr = np.asarray(crop, dtype=np.float32) / 255.0  # ToTensor
        arr = (arr - np.asarray(mean, np.float32)) / np.asarray(std, np.float32)                # Normalize
        crops.append(arr.transpose(2, 0, 1))              # HWC -> CHW
    return np.ascontiguousarray(np.stack(crops), dtype=np.float32) if crops else np.zeros((0, 3, image_size, image_size), np.float32)


def clip_batches(crops: np.ndarray, batch_size: int = 8) -> list[np.ndarray]:
    """Pad the final batch to the exported engine's fixed batch size."""
    n = crops.shape[0]
    out = []
    for i in range(0, n, batch_size):
        chunk = crops[i:i + batch_size]
        if chunk.shape[0] < batch_size:
            chunk = np.concatenate(
                [chunk, np.zeros((batch_size - chunk.shape[0], *crops.shape[1:]), np.float32)])
        out.append(np.ascontiguousarray(chunk))
    return out


def normalize_rows(x: np.ndarray) -> np.ndarray:
    x = np.asarray(x, dtype=np.float32)
    n = np.linalg.norm(x, axis=-1, keepdims=True)
    return x / np.maximum(n, 1e-12)


# ── YOLO head math + NMS ─────────────────────────────────────────────────────
def yolo_score_stage(feats: np.ndarray, en: np.ndarray,
                     scales: np.ndarray, biases: np.ndarray) -> np.ndarray:
    """Score anchor features with normalized class columns and per-anchor constants."""
    f = np.asarray(feats[0], dtype=np.float32)
    norm = np.maximum(np.linalg.norm(f, axis=0, keepdims=True), 1e-12)
    d = (f / norm).T @ en
    return (1.0 / (1.0 + np.exp(-(d * scales[:, None] + biases[:, None])))).T


def _nms(boxes: np.ndarray, scores: np.ndarray, iou_thres: float) -> np.ndarray:
    """Greedy NMS, torchvision-ordered (descending score, stable for ties)."""
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = np.argsort(-scores, kind="stable")
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        iou = inter / (areas[i] + areas[rest] - inter)
        order = rest[iou <= iou_thres]
    return np.array(keep, dtype=np.int64)


def yolo_nms(boxes_xywh: np.ndarray, scores: np.ndarray, conf: float,
             iou: float = NMS_IOU, max_det: int = NMS_MAX_DET) -> np.ndarray:
    """ultralytics 8.4 non_max_suppression, multi-label path (nc > 1).

    boxes_xywh (N,4) decoded center-format pixels; scores (nc,N) post-sigmoid.
    Returns (n,6) rows [x1, y1, x2, y2, conf, cls], descending confidence."""
    si, sj = np.where(scores > conf)  # (anchor, class) pairs, row-major like torch.where
    if si.size == 0:
        return np.zeros((0, 6), dtype=np.float32)
    confs = scores[si, sj]
    if confs.size > NMS_MAX_NMS:
        top = np.argsort(-confs, kind="stable")[:NMS_MAX_NMS]
        si, sj, confs = si[top], sj[top], confs[top]

    # si walks the class rows, sj the anchor columns: boxes follow sj, the
    # class id (and the batched-NMS offset) follows si.
    x = boxes_xywh[sj]
    xyxy = np.empty_like(x)
    xyxy[:, 0] = x[:, 0] - x[:, 2] / 2  # xywh -> xyxy
    xyxy[:, 1] = x[:, 1] - x[:, 3] / 2
    xyxy[:, 2] = x[:, 0] + x[:, 2] / 2
    xyxy[:, 3] = x[:, 1] + x[:, 3] / 2
    boxes = xyxy + si.astype(np.float32)[:, None] * NMS_MAX_WH
    keep = _nms(boxes.astype(np.float64), confs.astype(np.float64), iou)[:max_det]
    return np.concatenate(
        [xyxy[keep], confs[keep, None], si[keep, None].astype(np.float32)], axis=1)


# ── Session bundle ───────────────────────────────────────────────────────────
class PerceptionEngines:
    """All model sessions for one detector instance.

    E — the (nc, 512) class table — is computed once at load from the YOLO
    text tower; SCENE_OPEN_VOCAB_CLASSES changes therefore need a scene
    restart but never an engine rebuild."""

    def __init__(self, classes: list[str], yolo: TrtRunner, sam_enc: TrtRunner,
                 clip_vis: TrtRunner, e: np.ndarray, clip_text: OrtSess,
                 sam_dec: TrtRunner, scales, biases, ns, metadata):
        self.metadata = metadata
        self.tokenizer = _TOKENIZER or SimpleTokenizer()
        self.classes = list(classes)
        self.yolo = yolo
        self.sam_enc = sam_enc
        self.clip_vis = clip_vis
        self.clip_text = clip_text
        self.sam_dec = sam_dec
        self.e = e
        self._score_e = normalize_rows(e).T
        self._score_scales = np.concatenate([
            np.full(int(n), s, dtype=np.float32) for s, n in zip(scales, ns)])
        self._score_biases = np.concatenate([
            np.full(int(n), b, dtype=np.float32) for b, n in zip(biases, ns)])

    def close(self) -> None:
        for runner in (self.yolo, self.sam_enc, self.sam_dec, self.clip_vis):
            runner.close()
        self.clip_text = None

    @property
    def device_memory_bytes(self) -> int:
        return sum(r.device_memory_bytes for r in
                   (self.yolo, self.sam_enc, self.sam_dec, self.clip_vis))

    def detect(self, bgr: np.ndarray, conf: float):
        """Full YOLO-World pass: preprocess -> engine -> score stage -> NMS.
        Returns (n,6) [x1,y1,x2,y2,conf,cls] rows, empty when nothing fires."""
        im = yolo_preprocess(bgr)
        outs = self.yolo.run({"images": im[None], "txt_feats": self.e[None]})
        boxes = outs["boxes"]  # (1,4,N) decoded xywh in 672-letterbox space
        feats = outs["feats"]  # (1,512,N)
        scores = yolo_score_stage(
            feats, self._score_e, self._score_scales, self._score_biases)
        dets = yolo_nms(boxes[0].T, scores, conf)
        if dets.shape[0] == 0:
            return dets
        dets[:, :4] = yolo_unletterbox(dets[:, :4], bgr.shape[:2])
        return dets

    def segment(self, bgr: np.ndarray, xyxy: np.ndarray) -> np.ndarray:
        """Box-prompted masks (K,H,W) bool at the input resolution."""
        if xyxy.shape[0] == 0:
            return np.zeros((0, bgr.shape[0], bgr.shape[1]), dtype=bool)
        enc = self.sam_enc.run({"image": sam_preprocess(bgr, self.metadata["sam_image_size"])[None]})["image_embed"]
        boxes = sam_scale_boxes(xyxy, bgr.shape[:2], self.metadata["sam_image_size"])  # (K,4) in 1024-space
        # The decoder runs one box at a time (static K=1 profile): batched-K
        # tactics cost ~38MB device scratch per profiled box, which is a bad
        # trade on the 8GB board; a K=1 call is ~10ms.
        masks = np.empty((len(boxes), *bgr.shape[:2]), dtype=bool)
        for i in range(len(boxes)):
            logits = self.sam_dec.run({"image_embeddings": enc, "boxes": boxes[i:i + 1]})["masks"]
            masks[i] = sam_postprocess_masks(logits, bgr.shape[:2])[0]
        return masks

    def clip_feats(self, rgb: np.ndarray, xyxy: np.ndarray) -> np.ndarray:
        """L2-normalized image features using the exported preprocessing."""
        crops = clip_crop_crops(rgb, xyxy, image_size=self.metadata["clip_image_size"],
                                mean=self.metadata["clip_mean"], std=self.metadata["clip_std"])
        n = crops.shape[0]
        feats = []
        for chunk in clip_batches(crops, self.metadata["clip_batch_size"]):
            outs = self.clip_vis.run({"images": chunk})
            feats.append(outs["image_feats"])
        if not feats:
            return np.zeros((0, self.metadata["clip_feature_dim"]), np.float32)
        # drop the zero-padded tail rows: the engine sees the padded batch,
        # the caller only wants n real crops back
        return normalize_rows(np.concatenate(feats, axis=0)[:n])

    def embed_text(self, texts: list[str]) -> np.ndarray:
        """Normalized CLIP text features in the image feature space."""
        tokens = self.tokenizer(texts, context_length=self.metadata["clip_context_length"])
        feats = self.clip_text.run({"tokens": tokens})["text_feats"]
        return normalize_rows(feats)


_TOKENIZER: SimpleTokenizer = None


def load_engines(classes: list[str], models_dir: str = None,
                 cache_dir: str = None, *, profile=None, clip_model=None) -> PerceptionEngines:
    """Load every session or raise with the reason.  Engine fingerprints are
    resolved the same way build_engines.py names them; a missing or stale
    engine raises FileNotFoundError (the preflight hook builds them)."""
    global _TOKENIZER
    models_dir = models_dir or DEFAULT_MODELS_DIR
    cache_dir = cache_dir or DEFAULT_CACHE_DIR

    from ..inference_config import model_metadata

    metadata = model_metadata(models_dir, profile=profile, clip_model=clip_model)
    missing = [m for m in TRT_MODELS + ORT_MODELS
               if not os.path.isfile(os.path.join(models_dir, f"{m}.onnx"))]
    if missing:
        raise FileNotFoundError(f"ONNX models missing in {models_dir}: {missing}")

    npz = dict(np.load(os.path.join(models_dir, "yolo-world-cut.npz")))
    scales, biases, ns = npz["scales"], npz["biases"], npz["ns"]

    eng_paths = {}
    for name in TRT_MODELS:
        path = engine_file(name, models_dir, cache_dir)
        if not os.path.isfile(path):
            raise FileNotFoundError(
                f"TRT engine for {name} not found in {cache_dir} — run the "
                "start_native.sh preflight (scripts/build_engines.py) first")
        eng_paths[name] = path

    if _TOKENIZER is None:
        _TOKENIZER = SimpleTokenizer()

    yolo = TrtRunner(eng_paths["yolo-world-cut"])
    sam_enc = TrtRunner(eng_paths["sam-encoder"])
    # CPU ORT runs the decoder at ~0.5s/call on the board — TRT keeps the whole
    # segment stage inside the tick budget.
    sam_dec = TrtRunner(eng_paths["sam-decoder"])
    clip_vis = TrtRunner(eng_paths["clip-visual"])
    yolo_text = OrtSess(os.path.join(models_dir, "yolo-world-text.onnx"))
    e = yolo_text.run({"tokens": _TOKENIZER(classes)})["txt_feats"].astype(np.float32)
    del yolo_text
    clip_text = OrtSess(os.path.join(models_dir, "clip-text.onnx"))
    bundle = PerceptionEngines(classes, yolo, sam_enc, clip_vis,
                               e, clip_text, sam_dec, scales, biases, ns, metadata=metadata)
    log.info("[scene-engines] loaded: E(%d,512), yolo/sam/clip TRT %.0fMB device mem",
             bundle.e.shape[0], bundle.device_memory_bytes / 2**20)
    return bundle
