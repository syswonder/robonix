# SPDX-License-Identifier: MulanPSL-2.0
"""A projected box is not a sighting.

These cover the case that produced five photographs of five different objects
under one id: the geometry put the box in frame, and a wall was in the way.
"""
import numpy as np
import pytest

from scene_service.object_views import looks_like


def _patch(value: float, shape=(40, 40)):
    return np.full(shape, value, dtype="float32")


def test_an_unobstructed_object_is_kept():
    # Depth agrees with where the object is: this is the object.
    assert looks_like(_patch(3.0), (0, 0, 40, 40), 3.0)


def test_a_wall_in_front_of_the_object_is_refused():
    # The object is four metres away; the camera sees a surface at one.
    assert not looks_like(_patch(1.0), (0, 0, 40, 40), 4.0)


def test_a_near_face_is_still_the_object():
    # A big shelf's front face reads nearer than its centre. Half a metre of
    # slack keeps that from being mistaken for an occluder.
    assert looks_like(_patch(3.7), (0, 0, 40, 40), 4.0)


def test_missing_depth_does_not_convict():
    # No depth stream is not evidence against the crop. A deployment
    # without depth would otherwise keep no pictures at all.
    assert looks_like(None, (0, 0, 40, 40), 3.0)


def test_sparse_depth_does_not_convict():
    # Too few readings to disagree with; the crop is not refused on the
    # strength of a handful of pixels.
    d = np.zeros((40, 40), dtype="float32")
    d[0, :2] = 3.0
    assert looks_like(d, (0, 0, 40, 40), 3.0)


def test_a_few_stray_near_pixels_do_not_condemn_a_clear_view():
    # A depth edge along the object's border leaves some very near samples;
    # the tenth percentile is what decides, not the minimum.
    d = _patch(3.0)
    d[:2, :2] = 0.3
    assert looks_like(d, (0, 0, 40, 40), 3.0)


@pytest.mark.parametrize("rect", [(0, 0, 0, 10), (5, 5, 5, 5), (10, 10, 2, 2)])
def test_an_empty_rect_is_refused(rect):
    assert not looks_like(_patch(3.0), rect, 3.0)
