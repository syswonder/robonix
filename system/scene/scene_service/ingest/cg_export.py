# SPDX-License-Identifier: MulanPSL-2.0
"""Export Scene's live ConceptGraphs map in the layout the upstream Replica scorer reads.

``conceptgraph/scripts/eval_replica_semseg.py`` loads
``$REPLICA_ROOT/<scene>/pcd_saves/full_pcd_<exp_name>*.pkl.gz`` and expects a gzip pickle
``{"objects": MapObjectList.to_serializable(), "bg_objects": ...}``. Scene already holds a
``MapObjectList`` (``ConceptGraphsDetector._map_objects``); this writes that object in that
shape and nothing else, so the scorer sees our map exactly as it would see the paper's.

Two fields are added beyond what the scorer requires, and it ignores both: ``clip_model``
and ``clip_pretrained``. The scorer encodes class names with a CLIP of its own choosing and
dots the result against each object's ``clip_ft``; if the two models differ the feature
dimensions do not match and the comparison is meaningless rather than merely wrong. Recording
which model produced the features lets the scoring side pick the matching text encoder —
see ``eval_replica_scene.py``.

Usage from inside the detector, guarded by an env var so it is inert in deployments:

    export_map_objects(self._map_objects, exp_name, out_dir, clip_model, clip_pretrained)
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
    """Write ``map_objects`` as ``<out_dir>/full_pcd_<exp_name>.pkl.gz`` and return the path.

    ``map_objects`` and ``bg_objects`` are ConceptGraphs ``MapObjectList`` instances (or
    anything with ``to_serializable()``). The write is atomic: the pickle goes to a
    temporary file in the same directory and is renamed into place, so a scorer that polls
    the directory never reads a half-written archive. An empty map is still written — a
    scorer given nothing to score should say so, not fail to find a file.
    """
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
    """Export when ``SCENE_EXPORT_CG_PICKLE`` names a directory; otherwise do nothing.

    ``SCENE_EXPORT_CG_EXP`` names the experiment (default ``scene``), matching the
    ``--pred_exp_name`` the scorer is given. Returning ``None`` when the variable is unset
    keeps the call site a single line with no branching of its own.
    """
    out_dir = os.environ.get("SCENE_EXPORT_CG_PICKLE", "").strip()
    if not out_dir:
        return None
    exp = os.environ.get("SCENE_EXPORT_CG_EXP", "scene").strip() or "scene"
    return export_map_objects(map_objects, exp, out_dir, clip_model, clip_pretrained)
