# SPDX-License-Identifier: MulanPSL-2.0
"""An id that leaves has to leave a forwarding address.

Everything durable in scene hangs off `object_id` -- an operator's label
correction, the graph's edges, the object's stored views, a candidate a user
confirmed. Measured on the Webots stack, ids are stable while the robot is
still and churn while it drives, because concept-graphs merges and splits its
own objects underneath the registry. These tests pin the three answers a
holder of a stale id needs to be able to tell apart: it is still live, it
became this other one, or it is gone and here is why.
"""
import asyncio

from scene_service.state import BBox3D, Pose3D
from scene_service.state.object_registry import ObjectRegistry


def _reg():
    return ObjectRegistry()


def _add(reg, cls, x, y, *, now=1000.0):
    return reg.insert_object(
        cls=cls,
        pose=Pose3D(x=x, y=y, z=0.0, yaw=0.0, frame_id="map"),
        bbox=BBox3D(size_x=0.3, size_y=0.3, size_z=0.3, yaw=0.0, frame_id="map"),
        confidence=0.9,
        now=now,
    )


def test_a_live_id_resolves_to_itself_and_reports_nothing():
    """The common case must not pay for the machinery: a live id comes back
    unchanged, with no departure record to reason about."""
    reg = _reg()
    obj = _add(reg, "chair", 1.0, 1.0)
    live, departure = reg.resolve_id(obj.object_id)
    assert live == obj.object_id
    assert departure is None


def test_an_unknown_id_is_not_confused_with_a_departed_one():
    """'I have never heard of this' and 'this used to exist' are different
    answers, and a caller deciding whether to ask the user again needs them
    apart."""
    reg = _reg()
    live, departure = reg.resolve_id("scene.object.chair_999")
    assert live is None
    assert departure is None


def test_a_pruned_object_forwards_to_the_one_that_absorbed_it():
    """The merge case, which is where the churn comes from: the loser is
    soft-evicted, sits out the TTL, and is pruned while the survivor stands a
    few centimetres away. A holder of the loser's id should land on the
    survivor rather than on nothing."""
    reg = _reg()
    loser = _add(reg, "chair", 1.0, 1.0, now=1000.0)
    winner = _add(reg, "chair", 1.2, 1.0, now=1000.0)
    reg.soft_evict(loser)
    pruned = reg.prune_expired(now=1100.0, ttl_s=30.0, merge_dist_m=1.5)

    assert pruned == [loser.object_id]
    live, departure = reg.resolve_id(loser.object_id)
    assert live == winner.object_id
    assert departure["reason"] == "ttl_pruned"
    # It was decided by proximity, not reported by the merge, and says so.
    assert departure["inferred"] is True


def test_a_guessed_successor_can_be_refused():
    """Following a guess is right when redrawing a panel and wrong when acting
    on what a person confirmed, so the caller chooses rather than the
    registry."""
    reg = _reg()
    loser = _add(reg, "chair", 1.0, 1.0)
    _add(reg, "chair", 1.2, 1.0)
    reg.soft_evict(loser)
    reg.prune_expired(now=1100.0, ttl_s=30.0, merge_dist_m=1.5)

    live, departure = reg.resolve_id(loser.object_id, follow_inferred=False)
    assert live is None
    assert departure["superseded_by"] is not None


def test_an_object_alone_when_it_expires_leaves_a_tombstone():
    """Nothing absorbed it, so inventing a successor would be a lie. The
    record still exists, which is what lets a caller say 'that is gone'
    instead of 'that is unknown'."""
    reg = _reg()
    obj = _add(reg, "chair", 1.0, 1.0)
    reg.soft_evict(obj)
    reg.prune_expired(now=1100.0, ttl_s=30.0, merge_dist_m=1.5)

    live, departure = reg.resolve_id(obj.object_id)
    assert live is None
    assert departure["reason"] == "ttl_pruned"
    assert departure["superseded_by"] is None


def test_a_successor_is_never_guessed_across_classes():
    """The forwarding address uses the same class gate re-adoption does. A
    chair must not forward to the table it was standing next to."""
    reg = _reg()
    chair = _add(reg, "chair", 1.0, 1.0)
    _add(reg, "table", 1.05, 1.0)
    reg.soft_evict(chair)
    reg.prune_expired(now=1100.0, ttl_s=30.0, merge_dist_m=1.5)

    live, departure = reg.resolve_id(chair.object_id)
    assert live is None
    assert departure["superseded_by"] is None


def test_a_successor_is_never_guessed_beyond_the_merge_distance():
    """Two chairs at opposite ends of a room are two chairs. Forwarding one to
    the other would send a confirmed choice somewhere the user did not mean."""
    reg = _reg()
    near = _add(reg, "chair", 0.0, 0.0)
    _add(reg, "chair", 6.0, 0.0)
    reg.soft_evict(near)
    reg.prune_expired(now=1100.0, ttl_s=30.0, merge_dist_m=1.5)

    live, _ = reg.resolve_id(near.object_id)
    assert live is None


def test_an_operator_deletion_reads_as_deleted_not_unknown():
    """A person said this is not a thing. A later reference should carry that,
    so the answer is 'you deleted it' rather than a shrug."""
    reg = _reg()
    obj = _add(reg, "chair", 1.0, 1.0)
    reg.delete_derived_object(obj.object_id)

    live, departure = reg.resolve_id(obj.object_id)
    assert live is None
    assert departure["reason"] == "operator_deleted"
    assert departure["superseded_by"] is None


def test_a_chain_of_supersessions_lands_on_the_live_one():
    """Churn compounds: A folds into B, then B folds into C. A reference to A
    has to reach C, not stop at a B that is itself gone."""
    reg = _reg()
    a = _add(reg, "chair", 1.00, 1.0)
    b = _add(reg, "chair", 1.10, 1.0)
    c = _add(reg, "chair", 1.20, 1.0)

    reg.soft_evict(a)
    reg.prune_expired(now=1100.0, ttl_s=30.0, merge_dist_m=0.15)
    assert reg.resolve_id(a.object_id)[0] == b.object_id

    reg.soft_evict(b)
    reg.prune_expired(now=1200.0, ttl_s=30.0, merge_dist_m=0.15)

    live, _ = reg.resolve_id(a.object_id)
    assert live == c.object_id


def test_the_epoch_flush_forwards_nowhere():
    """A map-frame reset unanchors every stored coordinate, so nothing in the
    new frame stands for what was in the old one. Guessing here would be
    guessing across a discontinuity."""
    reg = _reg()
    obj = _add(reg, "chair", 1.0, 1.0)
    reg.clear_objects()

    live, departure = reg.resolve_id(obj.object_id)
    assert live is None
    assert departure["reason"] == "epoch_flush"
    assert departure["superseded_by"] is None
