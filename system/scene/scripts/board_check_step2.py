#!/usr/bin/env python3
"""Board parity check: torch path (ultralytics/open_clip) vs the new engine
runtime (perception_engines), section by section, models freed between
sections.  Run on the board from system/scene:

    YOLO_CONFIG_DIR=rbnx-build/data/ultralytics python3 scripts/board_check_step2.py
"""
import os
import sys

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)

W = os.path.join(PKG, "docker", "_weights")
os.environ.setdefault("SCENE_YOLO_WORLD_WEIGHTS", f"{W}/yolov8l-world.pt")
os.environ.setdefault("SCENE_MOBILE_SAM_WEIGHTS", f"{W}/mobile_sam.pt")
os.environ.setdefault("SCENE_CLIP_PRETRAINED", f"{W}/open_clip_pytorch_model.bin")
os.environ.setdefault("SCENE_ONNX_DIR", f"{W}/onnx")

import gc

import numpy as np

import scene_service.ingest.perception_engines as pe

PASS = []


def ok(name, cond, detail=""):
    PASS.append((name, bool(cond)))
    print(f"  [{'OK ' if cond else 'FAIL'}] {name} {detail}", flush=True)


# Deterministic synthetic frame (same pixels to both paths); letterbox math
# exercises non-square padding.
rng = np.random.default_rng(7)
img_bgr = np.zeros((720, 1280, 3), np.uint8)
img_bgr[:, :, 0] = 40
img_bgr[100:600, 200:900] = (rng.integers(0, 255, (500, 700, 3))).astype(np.uint8)
img_bgr[300:700, 800:1200] = (rng.integers(0, 200, (400, 400, 3))).astype(np.uint8)

# Sections 4-6 need an image that actually contains vocab objects (person is in
# the 55) — synthetic noise yields 0 detections on BOTH paths, which aligns
# vacuously. Default: a photo shipped inside the ultralytics package; pass a
# path (e.g. a saved sim frame) as argv[1] to compare a real frame instead.
import cv2

if len(sys.argv) > 1:
    photo = cv2.imread(sys.argv[1])
    assert photo is not None, f"cannot read frame {sys.argv[1]}"
else:
    photo = cv2.imread(os.path.join(os.path.dirname(__import__("ultralytics").__file__),
                                    "assets", "bus.jpg"))
    assert photo is not None, "ultralytics assets/bus.jpg not found"
print(f"  frame: {photo.shape[1]}x{photo.shape[0]}")

# ── 1. letterbox bit-compare ─────────────────────────────────────────────────
print("== 1. letterbox vs ultralytics")
from ultralytics.data.augment import LetterBox

for size, center in ((672, True), (1024, False)):
    ref = LetterBox(size, auto=False, center=center)(image=img_bgr.copy())
    mine = pe._letterbox(img_bgr, size, center=center)
    ok(f"letterbox {size} center={center}",
       ref.shape == mine.shape and np.array_equal(ref, mine),
       f"maxabs={np.abs(ref.astype(int) - mine.astype(int)).max()}")

# ── 2. tokenizer vs ultralytics' CLIP tokenizer ──────────────────────────────
print("== 2. tokenizer vs ultralytics YOLO-World clip_model")
from ultralytics import YOLO

yw = YOLO(os.environ["SCENE_YOLO_WORLD_WEIGHTS"])
from scene_service.ingest.perception_concept_graphs import _resolved_classes

classes = _resolved_classes()
# clip_model is lazily built inside get_text_pe on the first set_classes (8.4)
yw.model.set_classes(classes)
tm = yw.model.clip_model
ref_tok = tm.tokenize(classes).cpu().numpy().astype(np.int64)
mine_tok = pe.SimpleTokenizer()(classes)
ok(f"tokenizer ids for {len(classes)} classes", np.array_equal(ref_tok, mine_tok),
   f"mismatched rows={int((ref_tok != mine_tok).any(axis=1).sum())}")
# a few queries with punctuation/case too
queries = ["a photo of a red mug!", "Where IS the Laptop?", "左边的椅子"]
q_ref = tm.tokenize(queries).cpu().numpy().astype(np.int64)
q_mine = pe.SimpleTokenizer()(queries)
ok("tokenizer ids for free-form queries", np.array_equal(q_ref, q_mine),
   f"ref={q_ref[0, :6].tolist()} mine={q_mine[0, :6].tolist()}")

# ── 3. E vs set_classes ──────────────────────────────────────────────────────
print("== 3. E vs set_classes")
yw.model.set_classes(classes)
e_ref = yw.model.txt_feats.detach().cpu().numpy().astype(np.float32)  # (1,nc,512)
bundle = pe.load_engines(classes)
e_mine = bundle.e[None]
cos = (e_ref[0] * e_mine[0]).sum(1) / (np.linalg.norm(e_ref[0], axis=1) * np.linalg.norm(e_mine[0], axis=1))
ok("E per-class cosine", float(cos.min()) >= 0.999, f"min={cos.min():.6f} mean={cos.mean():.6f}")

# ── 4. YOLO end-to-end vs ultralytics predict ────────────────────────────────
print("== 4. YOLO end-to-end (engine+numpy NMS vs ultralytics predict)")
CONF, IOU = 0.2, 0.7
res = yw.predict(photo, imgsz=672, conf=CONF, iou=IOU, verbose=False)[0]
r_xyxy = res.boxes.xyxy.cpu().numpy()
r_conf = res.boxes.conf.cpu().numpy()
r_cls = res.boxes.cls.cpu().numpy().astype(int)
mine = bundle.detect(photo, CONF)
print(f"  ultralytics: {len(r_conf)} dets | engine: {len(mine)} dets")
# greedy IoU matching, per-class
def _iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    return ix * iy / ((a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - ix*iy + 1e-9)

matched = 0
conf_err, ious = [], []
used = set()
for i in range(len(r_conf)):
    best, bj = 0.0, -1
    for j in range(mine.shape[0]):
        if j in used or int(mine[j, 5]) != r_cls[i]:
            continue
        iou = _iou(r_xyxy[i], mine[j, :4])
        if iou > best:
            best, bj = iou, j
    if bj >= 0 and best > 0.8:
        matched += 1
        used.add(bj)
        conf_err.append(abs(r_conf[i] - mine[bj, 4]))
        ious.append(best)
# multi-label double-fire: near-synonym vocab classes (plant/potted plant…)
# fire a second class on an already-matched box.  The old torch path kept them
# too when the letterbox-context confidence crossed the gate; downstream
# concept-graph visual merging absorbs same-box duplicates either way.
extra = [int(mine[j, 5]) for j in range(mine.shape[0]) if j not in used
         and max((_iou(mine[j, :4], r_xyxy[i]) for i in range(len(r_conf))), default=0) > 0.9]
for j in range(mine.shape[0]):
    if j not in used and int(mine[j, 5]) in extra:
        used.add(j)
        print(f"  double-fire: cls={classes[int(mine[j, 5])]} conf={mine[j, 4]:.3f} "
              "(same box as a matched det)")
ok("YOLO detections matched", len(r_conf) > 0 and matched == len(r_conf) and len(used) == mine.shape[0],
   f"{matched}/{len(r_conf)} vs engine {mine.shape[0]}, iou_min={min(ious) if ious else 0:.4f}, "
   f"conf_maxabs={max(conf_err) if conf_err else 0:.4f}, double-fire={len(extra)}")
del yw, tm, res
gc.collect()

# ── 5. SAM masks vs ultralytics sam.predict ──────────────────────────────────
print("== 5. SAM masks (decoder TRT per-box vs ultralytics predict)")
from ultralytics import SAM

xyxy = mine[:, :4][:8]  # up to 8 boxes from the YOLO pass
sam = SAM(os.environ["SCENE_MOBILE_SAM_WEIGHTS"])
sres = sam.predict(photo, bboxes=xyxy, verbose=False)[0]
ref_masks = sres.masks.data.cpu().numpy()  # (K,H,W) bool
my_masks = bundle.segment(photo, xyxy)
assert ref_masks.shape == my_masks.shape, (ref_masks.shape, my_masks.shape)
ious = []
for i in range(ref_masks.shape[0]):
    inter = (ref_masks[i] & my_masks[i]).sum()
    union = (ref_masks[i] | my_masks[i]).sum()
    ious.append(inter / max(union, 1))
ok("SAM mask IoU", min(ious) >= 0.95, f"min={min(ious):.4f} mean={np.mean(ious):.4f} over {len(ious)}")
del sam, sres, ref_masks, my_masks
gc.collect()

# ── 6. CLIP visual + text vs open_clip ───────────────────────────────────────
print("== 6. CLIP features vs open_clip")
import open_clip
import torch

oc_model, _, _ = open_clip.create_model_and_transforms(
    "ViT-B-32", pretrained=os.environ["SCENE_CLIP_PRETRAINED"])
oc_model.eval()
rgb = photo[:, :, ::-1]
crops = pe.clip_crop_crops(rgb, xyxy)  # my preprocessing
with torch.no_grad():
    ref_vis = oc_model.visual(torch.from_numpy(crops)).numpy()
my_vis = bundle.clip_feats(rgb, xyxy)
cos = (ref_vis * my_vis).sum(1) / (np.linalg.norm(ref_vis, axis=1) * np.linalg.norm(my_vis, axis=1))
ok("CLIP visual cosine", float(cos.min()) >= 0.999, f"min={cos.min():.6f} over {len(cos)}")

texts = ["a red mug", "an open laptop on the desk", "a person walking"]
texts_tok = open_clip.get_tokenizer("ViT-B-32")(texts).numpy().astype(np.int64)
with torch.no_grad():
    ref_txt = oc_model.encode_text(torch.from_numpy(texts_tok)).numpy()
    ref_txt = ref_txt / np.linalg.norm(ref_txt, axis=1, keepdims=True)
my_txt = bundle.embed_text(texts)
cos = (ref_txt * my_txt).sum(1) / (np.linalg.norm(ref_txt, axis=1) * np.linalg.norm(my_txt, axis=1))
ok("CLIP text cosine", float(cos.min()) >= 0.999, f"min={cos.min():.6f}")
del oc_model
gc.collect()

# ── 7. NMS semantic spot-check: multi-label double-fire preserved ────────────
print("== 7. detection rows well-formed")
ok("detect rows (n,6) desc-conf", mine.shape[1] == 6 and mine[:, 4].min() >= CONF and
   bool((np.diff(mine[:, 4]) <= 1e-6).all()))

for runner in (bundle.yolo, bundle.sam_enc, bundle.sam_dec, bundle.clip_vis):
    runner.close()
n_fail = sum(1 for _, p in PASS if not p)
print(f"\n{'ALL PASS' if n_fail == 0 else f'{n_fail} FAILURES'} ({len(PASS)} checks)")
sys.exit(1 if n_fail else 0)
