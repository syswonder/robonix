# SPDX-License-Identifier: MulanPSL-2.0
"""Pictures of an unsaved session belong to that session.

The failure this covers: an unsaved session has no map id and object ids
restart at _001 every boot, so filing the pictures under one fixed name put
three runs' shelf_002 in one directory. The panel then showed five
photographs of five different shelves, taken up to seventeen hours apart.
"""
import numpy as np

from scene_service.object_views import ObjectViewStore


def _store(tmp_path):
    # An encoder of our own: the real one needs cv2, which is in the scene
    # image and not in the test environment. What is being tested is where
    # the bytes are filed, not how they are compressed.
    return ObjectViewStore(
        str(tmp_path), max_views=5, encode=lambda crop: b"jpeg")


def _frame():
    return np.zeros((64, 64, 3), dtype="uint8")


def test_two_sessions_do_not_share_a_directory(tmp_path):
    s = _store(tmp_path)
    s.offer(map_id="session-A", object_id="shelf_002", image_bgr=_frame(),
            rect=(4, 4, 40, 40), bearing=0.0, img_w=64, img_h=64)
    s.offer(map_id="session-B", object_id="shelf_002", image_bgr=_frame(),
            rect=(4, 4, 40, 40), bearing=0.0, img_w=64, img_h=64)
    assert len(s.views("session-A", "shelf_002")) == 1
    assert len(s.views("session-B", "shelf_002")) == 1


def test_a_previous_unsaved_session_is_dropped(tmp_path):
    s = _store(tmp_path)
    s.offer(map_id="session-old", object_id="shelf_002", image_bgr=_frame(),
            rect=(4, 4, 40, 40), bearing=0.0, img_w=64, img_h=64)
    assert s.forget_stale_sessions("session-new") == 1
    assert s.views("session-old", "shelf_002") == []


def test_a_saved_map_keeps_its_pictures(tmp_path):
    # A named map describes a place that still exists; only the anonymous
    # per-run sessions are swept.
    s = _store(tmp_path)
    s.offer(map_id="office", object_id="shelf_002", image_bgr=_frame(),
            rect=(4, 4, 40, 40), bearing=0.0, img_w=64, img_h=64)
    s.forget_stale_sessions("session-new")
    assert len(s.views("office", "shelf_002")) == 1


def test_the_current_session_survives_the_sweep(tmp_path):
    s = _store(tmp_path)
    s.offer(map_id="session-now", object_id="shelf_002", image_bgr=_frame(),
            rect=(4, 4, 40, 40), bearing=0.0, img_w=64, img_h=64)
    assert s.forget_stale_sessions("session-now") == 0
    assert len(s.views("session-now", "shelf_002")) == 1
