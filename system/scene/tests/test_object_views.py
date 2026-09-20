# SPDX-License-Identifier: MulanPSL-2.0
"""Pictures of an object have to show different sides of it.

The point of storing several is that a person asked "which chair did you
mean" can tell them apart. Five frames of the same approach are one picture
five times, so what these pin is the selection: a new angle gets in, a
repeat of an angle we hold only gets in as an upgrade, and a genuinely new
side displaces the most redundant view rather than the oldest.
"""
import math
import tempfile

from scene_service.object_views import (
    ObjectViewStore,
    bearing_of,
    crop_quality,
)

# A crop is stored as whatever the encoder returns; the selection logic never
# looks at pixels, so the tests hand it a stand-in and read the index.
FAKE = object()


class _Image:
    """Slice like a numpy array without needing numpy."""

    def __getitem__(self, _key):
        return FAKE


def _store(tmp, max_views=3):
    return ObjectViewStore(
        tmp, max_views=max_views, encode=lambda crop: b"jpeg-bytes",
    )


def _offer(store, bearing, rect=(0, 0, 200, 200), oid="chair_1"):
    return store.offer(
        map_id="lab", object_id=oid, image_bgr=_Image(), rect=rect,
        bearing=bearing, img_w=640, img_h=480, now=1000.0,
    )


def test_bearing_is_the_side_the_camera_looks_from():
    """Not the camera's heading: two cameras on opposite sides of a chair are
    pi apart however each is pointed, and that is what spread is built on."""
    east = bearing_of((1.0, 0.0), (0.0, 0.0))
    west = bearing_of((-1.0, 0.0), (0.0, 0.0))
    assert math.isclose(east, 0.0, abs_tol=1e-6)
    assert math.isclose(abs(west), math.pi, abs_tol=1e-6)


def test_a_crop_running_off_the_frame_scores_below_a_whole_one():
    """Which part is cut off is exactly what a person is trying to judge, so a
    truncated object is a worse portrait than a smaller whole one."""
    whole = crop_quality((100, 100, 300, 300), 640, 480)
    cut = crop_quality((0, 100, 200, 300), 640, 480)
    assert whole > cut > 0.0


def test_a_crop_too_small_to_read_is_refused_outright():
    with tempfile.TemporaryDirectory() as tmp:
        store = _store(tmp)
        assert _offer(store, 0.0, rect=(10, 10, 20, 20)) is False
        assert store.views("lab", "chair_1") == []


def test_a_stationary_robot_does_not_fill_every_slot_with_one_angle():
    """Observed on real data before this rule moved: a couch held two slots
    whose bearings differed by 0.0002 radians, because free slots were spent
    without asking whether the side was already covered. The slots are for
    sides, not for frames."""
    with tempfile.TemporaryDirectory() as tmp:
        store = _store(tmp, max_views=5)
        for i in range(6):
            _offer(store, 0.80 + i * 0.0002)
        assert len(store.views("lab", "chair_1")) == 1


def test_the_first_views_are_simply_kept():
    with tempfile.TemporaryDirectory() as tmp:
        store = _store(tmp, max_views=3)
        assert _offer(store, 0.0)
        assert _offer(store, 2.0)
        assert len(store.views("lab", "chair_1")) == 2


def test_another_frame_of_the_same_side_is_not_a_second_picture():
    """The robot drives at a chair and produces a dozen near-identical frames.
    Keeping them would spend the whole budget on one angle."""
    with tempfile.TemporaryDirectory() as tmp:
        store = _store(tmp, max_views=3)
        _offer(store, 0.0)
        _offer(store, 1.6)
        _offer(store, 3.0)
        # Full, and this is a hair away from the first one.
        assert _offer(store, 0.05, rect=(0, 0, 200, 200)) is False
        assert len(store.views("lab", "chair_1")) == 3


def test_the_same_side_gets_in_when_it_is_a_better_look():
    """A closer, uncut frame of a side we already have replaces the worse one
    rather than being discarded -- it is the same information, better."""
    with tempfile.TemporaryDirectory() as tmp:
        store = _store(tmp, max_views=3)
        _offer(store, 0.0, rect=(0, 0, 120, 120))   # small and edge-touching
        _offer(store, 1.6)
        _offer(store, 3.0)
        before = store.views("lab", "chair_1")
        assert _offer(store, 0.02, rect=(100, 100, 400, 400)) is True
        after = store.views("lab", "chair_1")
        assert len(after) == 3
        assert max(v["quality"] for v in after) > max(
            v["quality"] for v in before)


def test_a_new_side_displaces_the_most_redundant_view_not_the_oldest():
    """Dropping the oldest would rotate the set through whatever the robot
    saw last. Dropping the most redundant makes it spread out."""
    with tempfile.TemporaryDirectory() as tmp:
        store = _store(tmp, max_views=3)
        _offer(store, 0.00)    # these two are nearly the same side
        _offer(store, 0.50)
        _offer(store, 3.00)    # and this one is the opposite side
        assert _offer(store, 1.70) is True
        kept = sorted(v["bearing"] for v in store.views("lab", "chair_1"))
        # The survivor of the redundant pair, the far side, and the newcomer.
        assert len(kept) == 3
        gaps = [abs(b - a) for a, b in zip(kept, kept[1:])]
        assert min(gaps) > 0.5, kept


def test_views_come_back_best_looking_first():
    """The caller showing one thumbnail should get the most legible one."""
    with tempfile.TemporaryDirectory() as tmp:
        store = _store(tmp, max_views=3)
        _offer(store, 0.0, rect=(100, 100, 400, 400))
        _offer(store, 2.0, rect=(0, 0, 130, 130))
        got = store.views("lab", "chair_1")
        assert got[0]["quality"] > got[-1]["quality"]


def test_reading_back_a_stored_view_returns_its_bytes():
    with tempfile.TemporaryDirectory() as tmp:
        store = _store(tmp)
        _offer(store, 0.0)
        index = store.views("lab", "chair_1")[0]["index"]
        assert store.read("lab", "chair_1", index) == b"jpeg-bytes"
        assert store.read("lab", "chair_1", 99) is None


def test_deleting_an_object_takes_its_pictures_with_it():
    """A person said this is not a thing. Photographs of it should not
    outlive that."""
    with tempfile.TemporaryDirectory() as tmp:
        store = _store(tmp)
        _offer(store, 0.0)
        _offer(store, 2.0)
        assert store.forget("lab", "chair_1") == 2
        assert store.views("lab", "chair_1") == []


def test_two_maps_do_not_share_an_objects_pictures():
    """Ids are only unique within a map, and these are photographs of a
    room -- mixing two maps' would show the wrong house."""
    with tempfile.TemporaryDirectory() as tmp:
        store = _store(tmp)
        _offer(store, 0.0, oid="chair_1")
        store.offer(
            map_id="other", object_id="chair_1", image_bgr=_Image(),
            rect=(0, 0, 200, 200), bearing=0.0, img_w=640, img_h=480,
        )
        assert len(store.views("lab", "chair_1")) == 1
        assert len(store.views("other", "chair_1")) == 1
        store.forget_map("other")
        assert store.views("other", "chair_1") == []
        assert len(store.views("lab", "chair_1")) == 1


def test_an_id_cannot_escape_its_directory():
    """The id is scene's own, but it reaches this as a string and becomes a
    path segment, which is where traversal lives."""
    with tempfile.TemporaryDirectory() as tmp:
        store = _store(tmp)
        store.offer(
            map_id="../../etc", object_id="../../passwd", image_bgr=_Image(),
            rect=(0, 0, 200, 200), bearing=0.0, img_w=640, img_h=480,
        )
        written = list(__import__("pathlib").Path(tmp).rglob("view_*.jpg"))
        assert written, "nothing was written at all"
        for path in written:
            assert str(path).startswith(tmp), path


def test_no_encoder_costs_pictures_not_tracking():
    """A deployment without OpenCV still tracks objects; it just cannot show
    you one. The perception tick must not learn about that."""
    with tempfile.TemporaryDirectory() as tmp:
        store = ObjectViewStore(tmp, encode=lambda crop: None)
        assert _offer(store, 0.0) is False
        assert store.views("lab", "chair_1") == []
