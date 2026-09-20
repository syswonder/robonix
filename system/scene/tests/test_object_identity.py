# SPDX-License-Identifier: MulanPSL-2.0
"""Whether the object list names one thing once.

Written from an operator looking at four monitors -- 001, 002, 006, 007,
the first three flagged missing -- piled at one desk, and asking how anyone
is supposed to point at the one they mean. They are not: a query for "the
monitor on the desk" had four right answers, which is the same as none.
Both halves of that were mechanical.

Association only ever asks "which existing object is this detection?" and
never "are these two the same?", so a detection landing outside the gate
minted a second record and nothing ever put them back together. The gate
itself was one 3D radius, letting a single camera's depth estimate -- the
least reliable number in the record -- decide identity on its own.

And the `missing` flag meant "not seen for five seconds", which after a
turn is everything. The guard that was supposed to prevent that read an
attribute nothing in the tree ever set.
"""
from scene_service.state import BBox3D, Pose3D
from scene_service.state.object_registry import ObjectRegistry


def _pose(x, y, z=0.0, frame="map"):
    return Pose3D(x=x, y=y, z=z, yaw=0.0, frame_id=frame)


def _bbox(frame="map"):
    return BBox3D(size_x=0.5, size_y=0.3, size_z=0.4, yaw=0.0, frame_id=frame)


def _insert(reg, cls, x, y, z=0.0, *, conf=0.8, now=1000.0, frame="map"):
    return reg.insert_object(cls=cls, pose=_pose(x, y, z, frame),
                             bbox=_bbox(frame), confidence=conf, now=now)


# ── the collapse ──────────────────────────────────────────────────────────

def test_two_records_of_one_monitor_become_one():
    """The case from the operator's screen: same class, same desk, a depth
    estimate apart."""
    reg = ObjectRegistry()
    first = _insert(reg, "monitor", 1.0, 2.0, 0.9)
    second = _insert(reg, "monitor", 1.1, 2.05, 1.6)
    first.observation_count = 40
    second.observation_count = 3

    merged = reg.merge_duplicates(2000.0)
    assert merged == [(second.object_id, first.object_id)]
    assert len(list(reg.all_objects())) == 1


def test_the_absorbed_id_still_resolves():
    """Whoever was holding it -- an operator mid-edit, Pilot mid-task --
    asked about a real object and should still get one."""
    reg = ObjectRegistry()
    keep = _insert(reg, "monitor", 1.0, 2.0)
    keep.observation_count = 40
    gone = _insert(reg, "monitor", 1.1, 2.05)

    reg.merge_duplicates(2000.0)
    live, departure = reg.resolve_id(gone.object_id)
    assert live == keep.object_id
    assert departure["reason"] == "merged_duplicate"


def test_the_survivor_inherits_the_sightings():
    """Those observations were of this object. A survivor that under-counts
    them looks less established than it is, and `find` ranks on that."""
    reg = ObjectRegistry()
    keep = _insert(reg, "monitor", 1.0, 2.0, now=900.0)
    keep.observation_count = 40
    gone = _insert(reg, "monitor", 1.1, 2.05, now=500.0)
    gone.observation_count = 7

    reg.merge_duplicates(2000.0)
    assert keep.observation_count == 47
    assert keep.first_seen == 500.0


def test_a_missing_record_merges_into_the_live_one():
    """Exactly the screen that prompted this: 001 missing beside 006 live.
    The live record is the survivor whatever the counts say, because it is
    the one perception can still confirm."""
    reg = ObjectRegistry()
    stale = _insert(reg, "monitor", 1.0, 2.0)
    stale.observation_count = 500
    stale.missing = True
    live = _insert(reg, "monitor", 1.1, 2.05)
    live.observation_count = 4

    merged = reg.merge_duplicates(2000.0)
    assert merged == [(stale.object_id, live.object_id)]
    assert not live.missing


def test_two_real_monitors_are_left_alone():
    """A wrong merge erases a distinction that exists, which is worse than
    showing one object twice. Across the floor the gate stays tight."""
    reg = ObjectRegistry()
    _insert(reg, "monitor", 1.0, 2.0)
    _insert(reg, "monitor", 2.0, 2.0)
    assert reg.merge_duplicates(2000.0) == []


def test_height_alone_never_splits_or_joins_wrongly():
    """Loose in z because depth is the unreliable axis -- but not unbounded:
    a tabletop object and one on the floor below it stay separate."""
    reg = ObjectRegistry()
    _insert(reg, "cup", 1.0, 2.0, 0.75)
    _insert(reg, "cup", 1.02, 2.0, 2.30)
    assert reg.merge_duplicates(2000.0) == []


def test_different_classes_never_merge():
    reg = ObjectRegistry()
    _insert(reg, "monitor", 1.0, 2.0)
    _insert(reg, "keyboard", 1.02, 2.01)
    assert reg.merge_duplicates(2000.0) == []


def test_coordinates_in_different_frames_are_not_comparable():
    """However near their numbers look."""
    reg = ObjectRegistry()
    _insert(reg, "monitor", 1.0, 2.0, frame="map")
    _insert(reg, "monitor", 1.02, 2.01, frame="odom")
    assert reg.merge_duplicates(2000.0) == []


def test_an_operator_corrected_record_is_never_the_one_absorbed():
    """A human looked at this object and said something about it.
    Perception's count does not outrank that."""
    reg = ObjectRegistry()
    corrected = _insert(reg, "monitor", 1.0, 2.0)
    corrected.observation_count = 2
    corrected.attributes["operator_geometry"] = True
    busy = _insert(reg, "monitor", 1.1, 2.05)
    busy.observation_count = 900

    merged = reg.merge_duplicates(2000.0)
    assert merged == [(busy.object_id, corrected.object_id)]


def test_the_robot_is_never_merged_into_anything():
    reg = ObjectRegistry()
    robot = _insert(reg, "robot", 1.0, 2.0)
    robot.attributes["is_robot"] = True
    _insert(reg, "robot", 1.02, 2.01)
    merged = reg.merge_duplicates(2000.0)
    assert all(robot.object_id not in pair for pair in merged)


def test_one_pass_is_bounded():
    """A registry that has drifted badly is repaired over several ticks
    rather than in one long hold of the lock."""
    reg = ObjectRegistry()
    for i in range(12):
        _insert(reg, "monitor", 1.0 + i * 0.01, 2.0)
    merged = reg.merge_duplicates(2000.0, max_merges=3)
    assert len(merged) == 3


# ── what `missing` is allowed to mean ─────────────────────────────────────

def test_wall_clock_silence_does_not_flag_a_visibility_tracked_object():
    """The guard that was in the code and never fired. Silence is occlusion,
    out of frame, a dead sensor and a failed model all at once; the
    concept-graphs path decides absence by looking instead."""
    reg = ObjectRegistry(grace_period_s=5.0)
    watched = _insert(reg, "monitor", 1.0, 2.0, now=1000.0)
    watched.attributes["observation_lifecycle"] = "visibility"
    unwatched = _insert(reg, "monitor", 9.0, 9.0, now=1000.0)

    reg.mark_stale(1000.0 + 60.0)
    assert not watched.missing
    assert unwatched.missing


def test_the_perception_path_arms_the_guard_it_relies_on():
    """The guard is only worth having if something sets the attribute.
    Nothing in the tree did, so every object was flagged five seconds after
    it left the frame -- a dead branch that read as a working safeguard."""
    import inspect

    from scene_service.ingest import perception_concept_graphs as cg

    source = inspect.getsource(cg)
    assert "observation_lifecycle" in source
    assert "visibility" in source
