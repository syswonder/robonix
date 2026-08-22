# SPDX-License-Identifier: MulanPSL-2.0
"""Static checks for the dynamically imported Webots dropout fixture."""

from verify_scene_object_dropout import (
    FIXTURE_NAME,
    FIXTURE_PATH,
    _visible_object_not_in,
)


def test_dropout_fixture_uses_runtime_importable_builtin_nodes() -> None:
    node = FIXTURE_PATH.read_text(encoding="utf-8")
    assert f'name "{FIXTURE_NAME}"' in node
    assert "Solid {" in node
    assert "boundingObject Group {" in node
    # Webots importMFNodeFromString cannot resolve external PROTO instances in
    # the CI supervisor world; keep this fixture self-contained.
    assert "EXTERNPROTO" not in node
    assert "OfficeChair {" not in node
    assert "PottedTree {" not in node


def test_dropout_fixture_is_visually_chair_shaped() -> None:
    node = FIXTURE_PATH.read_text(encoding="utf-8")
    # One seat, one back, and four legs are represented in both visual and
    # collision trees.
    assert node.count("size 0.62 0.58 0.12") == 2
    assert node.count("size 0.62 0.10 0.72") == 2
    assert node.count("size 0.07 0.07 0.48") == 8


def test_dropout_tracks_any_new_visible_semantic_object() -> None:
    predicate = _visible_object_not_in({"scene.object.existing"})
    assert predicate(
        [
            {
                "id": "scene.object.existing",
                "cls": "chair",
                "missing": False,
            },
            {
                "id": "scene.robot",
                "cls": "robot",
                "missing": False,
            },
            {
                "id": "scene.object.fixture",
                "cls": "cabinet",
                "missing": False,
            },
        ]
    ) == ("scene.object.fixture", "cabinet")
    assert predicate(
        [
            {
                "id": "scene.object.hidden",
                "cls": "chair",
                "missing": True,
            }
        ]
    ) is None
