# SPDX-License-Identifier: MulanPSL-2.0
"""Detection crops shared by Torch and TensorRT CLIP preprocessing."""
from __future__ import annotations

import numpy as np


def clip_crops(rgb, xyxy, padding=20):
    from PIL import Image

    image = Image.fromarray(np.ascontiguousarray(rgb))
    width, height = image.size
    for x_min, y_min, x_max, y_max in np.asarray(xyxy, dtype=np.float64):
        x0, y0, x1, y1 = round(x_min), round(y_min), round(x_max), round(y_max)
        yield image.crop((x0 - min(padding, max(x0, 0)),
                          y0 - min(padding, max(y0, 0)),
                          x1 + min(padding, width - x1),
                          y1 + min(padding, height - y1)))
