# SPDX-License-Identifier: MulanPSL-2.0
"""Scene's configuration schema declares accepted keys and usable settings."""
from __future__ import annotations

import pathlib
import re

import pytest
import yaml

from scene_service.ingest.capabilities import (
    CONCEPT_GRAPHS_KEYS,
    DUALMAP_KEYS,
    PERCEPTION_KEYS,
    perception_config,
)

_SPEC = pathlib.Path(__file__).resolve().parents[1] / "config.spec"


def _schema() -> dict:
    return yaml.safe_load(_SPEC.read_text(encoding="utf-8"))


def _perception_properties() -> dict:
    return _schema()["properties"]["perception"]["properties"]


def test_the_spec_is_a_structured_configuration_schema():
    schema = _schema()
    assert schema["specVersion"] == 1
    assert isinstance(schema["properties"], dict)
    assert schema["properties"]["perception"]["type"] == "object"


@pytest.mark.parametrize("block,accepted", [
    (None, PERCEPTION_KEYS),
    ("dualmap", DUALMAP_KEYS),
    ("concept_graphs", CONCEPT_GRAPHS_KEYS),
])
def test_schema_properties_match_accepted_keys(block, accepted):
    properties = _perception_properties()
    if block is not None:
        properties = properties[block]["properties"]
    assert set(properties) == set(accepted)


@pytest.mark.parametrize("backend", ["concept_graphs", "dualmap"])
def test_documented_defaults_are_accepted_by_runtime(backend):
    properties = _perception_properties()
    values = {key: field["default"] for key, field in properties.items() if "default" in field}
    for name in ("dualmap", "concept_graphs"):
        values[name] = {key: field["default"] for key, field in properties[name]["properties"].items()
                        if "default" in field}
    values["backend"] = backend
    parsed = perception_config({"perception": values}, env={})
    assert parsed.backend == backend
    assert parsed.profile == values["profile"]
    assert parsed.period_s == values["period_s"]
    assert parsed.confidence_threshold == values["confidence_threshold"]
    assert parsed.max_detections == values["max_detections"]
    assert parsed.dualmap == values["dualmap"]
    assert parsed.concept_graphs == values["concept_graphs"]
    assert not parsed.ignored_keys


def test_dualmap_tuning_is_optional_and_overrides_reach_the_parser():
    schema = _perception_properties()["dualmap"]
    values = {"classes": ["chair", "desk"], "stable_num": 3,
              "keyframe_translation_m": 0.2, "keyframe_rotation_deg": 5.0}
    assert set(values) <= set(schema["properties"])
    assert not set(values).intersection(schema.get("required", []))
    empty = perception_config({"perception": {"backend": "dualmap"}}, env={})
    assert empty.dualmap == {}
    explicit = perception_config({"perception": {"backend": "dualmap", "dualmap": values}}, env={})
    assert explicit.dualmap == values
    assert not explicit.ignored_keys


def test_concept_graphs_keys_mirror_the_detector_defaults():
    """Compare declared keys without importing the perception models."""
    import ast

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


@pytest.mark.parametrize("block,key", [
    ("concept_graphs", "same_class_merge_dist_metres"),
    ("dualmap", "stable_numm"),
])
def test_unknown_backend_keys_are_rejected(block, key):
    with pytest.raises(ValueError, match=key):
        perception_config({"perception": {block: {key: 1}}}, env={})
