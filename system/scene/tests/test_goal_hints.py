from types import SimpleNamespace

from scene_service import mcp_tools


class _Store:
    def __init__(self, rooms=None):
        self._rooms = rooms or [
            SimpleNamespace(annotation_id="anno.315", kind="room", name="room 315"),
            SimpleNamespace(annotation_id="anno.100", kind="room", name="room 100"),
        ]

    def list(self):
        return self._rooms


def _object(object_id: str, label: str):
    return SimpleNamespace(object_id=object_id, cls=label)


def test_room_hint_lists_names_and_exact_ids():
    mcp_tools.attach_annotation_store(_Store())
    hint = mcp_tools._room_id_hint()
    assert "room 315" in hint
    assert "scene.room.anno.315" in hint
    assert "room 100" in hint
    assert "scene.room.anno.100" in hint


def test_object_hint_ranks_similar_label_first():
    hint = mcp_tools._object_id_hint(
        "cardbord box",
        [
            _object("scene.object.chair_001", "chair"),
            _object("scene.object.cardboard_box_001", "cardboard box"),
            _object("scene.object.table_001", "table"),
        ],
    )
    assert hint.index("cardboard box") < hint.index("chair")
    assert "scene.object.cardboard_box_001" in hint


def test_room_reference_resolves_stable_id_name_and_short_alias():
    mcp_tools.attach_annotation_store(_Store())
    for reference in ("scene.room.anno.315", "room 315", "ROOM   315", "315"):
        room, ambiguous = mcp_tools._resolve_room_target(reference)
        assert room is not None
        assert room.annotation_id == "anno.315"
        assert ambiguous == []


def test_room_reference_reports_ambiguous_aliases_without_guessing():
    rooms = [
        SimpleNamespace(annotation_id="anno.a", kind="room", name="room 315"),
        SimpleNamespace(annotation_id="anno.b", kind="room", name="315"),
    ]
    mcp_tools.attach_annotation_store(_Store(rooms))
    room, ambiguous = mcp_tools._resolve_room_target("315")
    assert room is None
    assert [item.annotation_id for item in ambiguous] == ["anno.a", "anno.b"]
