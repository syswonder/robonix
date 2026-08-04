#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Remove Ultralytics' unused semantic task without breaking detect loaders.

Ultralytics 8.4.115 eagerly imports the semantic trainer, which brings in
Matplotlib even though Scene only uses detect and SAM. Removing only that
package import is insufficient: ``YOLO.task_map`` still dereferences
``yolo.semantic`` and then breaks cold-loaded TensorRT engines. Keep the import
surface and task registry consistent.
"""

from pathlib import Path
import sysconfig


root = Path(sysconfig.get_paths()["purelib"]) / "ultralytics" / "models" / "yolo"
init_path = root / "__init__.py"
model_path = root / "model.py"

init_text = init_path.read_text(encoding="utf-8")
old_import = (
    "from ultralytics.models.yolo import classify, depth, detect, obb, "
    "pose, segment, semantic, world, yoloe"
)
new_import = (
    "from ultralytics.models.yolo import classify, depth, detect, obb, "
    "pose, segment, world, yoloe"
)
if old_import in init_text:
    init_text = init_text.replace(old_import, new_import, 1)
elif new_import not in init_text:
    raise RuntimeError("unexpected Ultralytics yolo import layout")
if '    "semantic",\n' in init_text:
    init_text = init_text.replace('    "semantic",\n', "", 1)
elif '    "world",\n' not in init_text:
    raise RuntimeError("unexpected Ultralytics yolo __all__ layout")
init_path.write_text(init_text, encoding="utf-8")

model_text = model_path.read_text(encoding="utf-8")
semantic_task = '''            "semantic": {
                "model": SemanticSegmentationModel,
                "trainer": yolo.semantic.SemanticSegmentationTrainer,
                "validator": yolo.semantic.SemanticSegmentationValidator,
                "predictor": yolo.semantic.SemanticSegmentationPredictor,
            },
'''
if semantic_task in model_text:
    model_text = model_text.replace(semantic_task, "", 1)
elif "yolo.semantic.SemanticSegmentationPredictor" in model_text:
    raise RuntimeError("unexpected Ultralytics YOLO task-map layout")
model_path.write_text(model_text, encoding="utf-8")

compile(init_text, str(init_path), "exec")
compile(model_text, str(model_path), "exec")
