# SPDX-License-Identifier: MulanPSL-2.0
"""`config.spec` documents every key the service accepts.

A spec drifts the moment it is only prose. A key added to `DUALMAP_KEYS`
without a line in the spec is a key a deployment can set and nobody can find.

The spec is tiered: the handful a deployment has to set are live YAML, the rest
are documented in comments. So coverage is checked against the text, and the
live part is additionally run through the real validator -- a spec whose own
example fails at boot is worse than no spec, because the reader followed it.
"""
from __future__ import annotations

import pathlib
import re

import yaml

from scene_service.ingest.capabilities import (
    DUALMAP_KEYS,
    PERCEPTION_KEYS,
    perception_config,
)

_SPEC = pathlib.Path(__file__).resolve().parents[1] / "config.spec"


def _text() -> str:
    return _SPEC.read_text(encoding="utf-8")


def _live() -> dict:
    return yaml.safe_load(_text())["config"]


def test_the_spec_is_readable_yaml():
    assert isinstance(_live(), dict)


def test_every_dualmap_key_is_documented():
    text = _text()
    missing = sorted(k for k in DUALMAP_KEYS if not re.search(rf"\b{k}\b", text))
    assert not missing, missing


def test_every_perception_key_is_documented():
    text = _text()
    missing = sorted(k for k in PERCEPTION_KEYS if not re.search(rf"\b{k}\b", text))
    assert not missing, missing


def test_the_spec_documents_no_key_the_code_would_reject():
    """A documented DualMap key the validator rejects is the worst failure
    mode here: the reader writes it down and the boot fails naming it."""
    text = _text()
    start = text.index("perception.dualmap, continued")
    # Bounded to that sub-block: section 3 below it documents top-level keys,
    # which are not DualMap's and would look like rejections.
    end = text.index("3. Leave alone", start)
    named = set(re.findall(r"^\s*#\s{3}([a-z_0-9]+):", text[start:end], re.M))
    assert named <= set(DUALMAP_KEYS), sorted(named - set(DUALMAP_KEYS))


def test_the_live_example_is_one_the_code_accepts():
    cfg = perception_config(_live()["perception"])
    assert cfg.backend in ("concept_graphs", "dualmap")
    assert not cfg.ignored_keys, cfg.ignored_keys


def test_the_four_required_keys_are_the_ones_marked_required():
    """The spec's own claim about which keys matter, checked against itself."""
    required = set(re.findall(r"#\s*REQUIRED.*?\n(?:.*?\n)*?\s+([a-z_0-9]+):",
                              _text()))
    assert required == {"classes", "stable_num",
                        "keyframe_translation_m", "keyframe_rotation_deg"}, required


# ── the ConceptGraphs backend's own knobs ────────────────────────────────────
# They are the default backend's, there are more of them than DualMap's, and
# until now nothing documented or checked them.

def test_concept_graphs_keys_mirror_the_detector_defaults():
    """`CONCEPT_GRAPHS_KEYS` is a second copy of `_CFG_DEFAULTS`, declared to
    avoid importing torch at config time. A second copy drifts unless it is
    pinned, and the cost of drift is a key the manifest rejects and the
    detector would have read."""
    import ast
    import pathlib
    import re

    src = (pathlib.Path(__file__).resolve().parents[1]
           / "scene_service" / "ingest" / "perception_concept_graphs.py"
           ).read_text(encoding="utf-8")
    block = re.search(r"_CFG_DEFAULTS\s*=\s*\{(.*?)^\}", src, re.S | re.M).group(1)
    literal = "\n".join(l.split("#")[0] for l in block.split("\n"))
    defaults = set(ast.literal_eval("{" + literal.strip().rstrip(",") + "}"))

    from scene_service.ingest.capabilities import CONCEPT_GRAPHS_KEYS
    assert set(CONCEPT_GRAPHS_KEYS) == defaults, {
        "declared but not a real default": sorted(set(CONCEPT_GRAPHS_KEYS) - defaults),
        "a real default nobody declared": sorted(defaults - set(CONCEPT_GRAPHS_KEYS)),
    }


def test_a_misspelled_concept_graphs_key_fails_rather_than_doing_nothing():
    import pytest

    from scene_service.ingest.capabilities import perception_config
    with pytest.raises(ValueError, match="same_class_merge_dist_metres"):
        perception_config({"perception": {
            "concept_graphs": {"same_class_merge_dist_metres": 0.4}}})


def test_every_concept_graphs_key_is_documented():
    from scene_service.ingest.capabilities import CONCEPT_GRAPHS_KEYS
    text = _text()
    missing = sorted(k for k in CONCEPT_GRAPHS_KEYS
                     if not re.search(rf"\b{k}\b", text))
    assert not missing, missing
