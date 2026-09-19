from types import SimpleNamespace

from scene_service import mcp_tools


class _Store:
    def __init__(self, regions=None):
        self._rooms = regions or [
            SimpleNamespace(annotation_id="anno.315", kind="region", name="region 315"),
            SimpleNamespace(annotation_id="anno.100", kind="region", name="region 100"),
        ]

    def list(self):
        return self._rooms


def _object(object_id: str, label: str):
    return SimpleNamespace(object_id=object_id, cls=label)


def test_room_hint_lists_names_and_exact_ids():
    mcp_tools.attach_annotation_store(_Store())
    hint = mcp_tools._region_id_hint()
    assert "region 315" in hint
    assert "scene.region.anno.315" in hint
    assert "region 100" in hint
    assert "scene.region.anno.100" in hint


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
    for reference in ("scene.region.anno.315", "region 315", "ROOM   315", "315"):
        region, ambiguous = mcp_tools._resolve_region_target(reference)
        assert region is not None
        assert region.annotation_id == "anno.315"
        assert ambiguous == []


def test_room_reference_reports_ambiguous_aliases_without_guessing():
    regions = [
        SimpleNamespace(annotation_id="anno.a", kind="region", name="region 315"),
        SimpleNamespace(annotation_id="anno.b", kind="region", name="315"),
    ]
    mcp_tools.attach_annotation_store(_Store(regions))
    region, ambiguous = mcp_tools._resolve_region_target("315")
    assert region is None
    assert [item.annotation_id for item in ambiguous] == ["anno.a", "anno.b"]
