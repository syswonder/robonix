# SPDX-License-Identifier: MulanPSL-2.0
"""
Export Scene's live ConceptGraphs map in the layout the upstream Replica scorer
reads.
"""

from __future__ import annotations

import gzip
import os
import pickle
import time
from pathlib import Path


def export_map_objects(
    map_objects,
    exp_name: str,
    out_dir: str | os.PathLike[str],
    clip_model: str,
    clip_pretrained: str,
    bg_objects=None,
) -> Path:
    """Write ``map_objects`` as ``<out_dir>/full_pcd_<exp_name>.pkl.gz`` and
    return the path."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "objects": map_objects.to_serializable(),
        "bg_objects": bg_objects.to_serializable() if bg_objects is not None else [],
        "clip_model": clip_model,
        "clip_pretrained": clip_pretrained,
        "exported_at": time.time(),
        "object_count": len(map_objects),
    }
    final = out_dir / f"full_pcd_{exp_name}.pkl.gz"
    tmp = out_dir / f".full_pcd_{exp_name}.{os.getpid()}.tmp"
    with gzip.open(tmp, "wb") as fh:
        pickle.dump(payload, fh, protocol=pickle.HIGHEST_PROTOCOL)
    os.replace(tmp, final)
    return final


def export_from_env(map_objects, clip_model: str, clip_pretrained: str) -> Path | None:
    """Export when ``SCENE_EXPORT_CG_PICKLE`` names a directory; otherwise do
    nothing."""
    out_dir = os.environ.get("SCENE_EXPORT_CG_PICKLE", "").strip()
    if not out_dir:
        return None
    exp = os.environ.get("SCENE_EXPORT_CG_EXP", "scene").strip() or "scene"
    return export_map_objects(map_objects, exp, out_dir, clip_model, clip_pretrained)
