import pytest

from scene_service.geometry import disc_inside_polygon, point_in_polygon, polygon_centroid


ROOM_315 = [
    (-4.653655402257238, 0.771744687675239),
    (2.0362935039927557, 1.456772135591905),
    (2.7504710560760888, -2.4201917185747583),
    (-3.779152277257239, -3.3384199998247572),
    (-4.609930246007238, 0.7425945835085723),
]


def test_room_centroid_is_area_weighted_and_inside():
    x, y = polygon_centroid(ROOM_315)
    assert x == pytest.approx(-0.9369215188)
    assert y == pytest.approx(-0.8818296313)
    assert point_in_polygon(x, y, ROOM_315)


def test_rejects_the_goal_returned_outside_room_315():
    assert not point_in_polygon(-0.9148793979, -3.7743059028, ROOM_315)


def test_robot_footprint_must_remain_inside_room():
    assert disc_inside_polygon(-0.94, -0.88, 0.3, ROOM_315)
    assert not disc_inside_polygon(-3.75, -3.25, 0.3, ROOM_315)
