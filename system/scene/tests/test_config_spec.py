# SPDX-License-Identifier: MulanPSL-2.0
"""`config.spec` documents every key the service accepts, and only those.

A spec drifts the moment it is only prose. A key added to `DUALMAP_KEYS`
without a line in the spec is a key a deployment can set and nobody can find;
a key in the spec that the code rejects is worse, because a manifest written
from the documentation fails at boot naming a key the reader just read.
"""
from __future__ import annotations

import pathlib

import yaml

from scene_service.ingest.capabilities import DUALMAP_KEYS, PERCEPTION_KEYS

_SPEC = pathlib.Path(__file__).resolve().parents[1] / "config.spec"


def _spec() -> dict:
    return yaml.safe_load(_SPEC.read_text(encoding="utf-8"))["config"]


def test_the_spec_is_readable_yaml():
    assert isinstance(_spec(), dict)


def test_every_dualmap_key_is_documented_and_no_others():
    documented = set(_spec()["perception"]["dualmap"])
    assert documented == set(DUALMAP_KEYS), {
        "missing from config.spec": sorted(DUALMAP_KEYS - documented),
        "in config.spec but rejected by the code": sorted(documented - DUALMAP_KEYS),
    }


def test_every_perception_key_is_documented():
    documented = set(_spec()["perception"])
    assert set(PERCEPTION_KEYS) <= documented, sorted(
        set(PERCEPTION_KEYS) - documented)


def test_the_example_the_spec_gives_is_one_the_code_accepts():
    """The spec's own values are a manifest fragment; validate them as one."""
    from scene_service.ingest.capabilities import perception_config

    cfg = perception_config(_spec()["perception"])
    assert cfg.backend in ("concept_graphs", "dualmap")
    assert not cfg.ignored_keys, cfg.ignored_keys
