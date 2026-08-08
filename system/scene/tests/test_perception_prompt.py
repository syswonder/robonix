# SPDX-License-Identifier: MulanPSL-2.0
"""The detection prompt decides what perception is able to see at all.

Its domain, preferred class set and detection cap are deployment facts, so
they are configurable; these tests pin both the defaults (unchanged for
existing deployments) and the override path.
"""
import pytest

from scene_service.ingest.perception_vlm import (
    _DEFAULT_CLASSES,
    _DEFAULT_DOMAIN,
    _DEFAULT_LIMIT,
    _detection_prompt,
)


def test_defaults_are_used_when_nothing_is_configured(monkeypatch):
    for name in (
        "SCENE_DETECTION_DOMAIN",
        "SCENE_DETECTION_CLASSES",
        "SCENE_DETECTION_LIMIT",
    ):
        monkeypatch.delenv(name, raising=False)

    prompt = _detection_prompt()

    assert _DEFAULT_DOMAIN in prompt
    assert _DEFAULT_CLASSES in prompt
    assert f"at most {_DEFAULT_LIMIT} detections" in prompt


def test_deployment_can_describe_its_own_environment(monkeypatch):
    monkeypatch.setenv("SCENE_DETECTION_DOMAIN", "a household kitchen")
    monkeypatch.setenv("SCENE_DETECTION_CLASSES", "pear, apple, sofa, counter")
    monkeypatch.setenv("SCENE_DETECTION_LIMIT", "30")

    prompt = _detection_prompt()

    assert "a household kitchen" in prompt
    assert "pear, apple, sofa, counter" in prompt
    assert "at most 30 detections" in prompt
    # The office vocabulary is replaced, not appended: a deployment that names
    # its classes should not still be steered toward someone else's.
    assert _DEFAULT_CLASSES not in prompt
    assert _DEFAULT_DOMAIN not in prompt


@pytest.mark.parametrize("value", ["", "   ", "not-a-number", "0", "-5"])
def test_an_unusable_limit_falls_back_to_the_default(monkeypatch, value):
    monkeypatch.setenv("SCENE_DETECTION_LIMIT", value)

    assert f"at most {_DEFAULT_LIMIT} detections" in _detection_prompt()


def test_the_prompt_stays_valid_json_instructions(monkeypatch):
    monkeypatch.setenv("SCENE_DETECTION_DOMAIN", "a warehouse")

    prompt = _detection_prompt()

    # The template is `.format`-ed, so the literal braces of the example
    # response must survive as single braces rather than doubled ones.
    assert '{"detections": [{"cls": "..."' in prompt
    assert "{{" not in prompt
