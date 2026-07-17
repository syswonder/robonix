# SPDX-License-Identifier: MulanPSL-2.0
"""Scene camera-transform precedence and compatibility validation tests."""

from types import SimpleNamespace

import numpy as np
import pytest

from scene_service.ingest.perception_concept_graphs import ConceptGraphsDetector


def _xyz(x: float, y: float, z: float) -> SimpleNamespace:
    """Build a message-like xyz vector."""
    return SimpleNamespace(x=x, y=y, z=z)


def _identity_quaternion() -> SimpleNamespace:
    """Build a message-like identity quaternion."""
    return SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


def _pose_message(x: float, y: float, z: float) -> SimpleNamespace:
    """Build a PoseWithCovarianceStamped-like sample."""
    pose = SimpleNamespace(position=_xyz(x, y, z), orientation=_identity_quaternion())
    return SimpleNamespace(pose=SimpleNamespace(pose=pose))


def _odom_message(
    x: float, y: float, z: float, *, child_frame: str
) -> SimpleNamespace:
    """Build an Odometry-like sample with an explicit body frame."""
    msg = _pose_message(x, y, z)
    msg.child_frame_id = child_frame
    return msg


def _extrinsics_message(
    *, parent: str, child: str, x: float, y: float, z: float
) -> SimpleNamespace:
    """Build a TransformStamped-like compatibility sample."""
    transform = SimpleNamespace(
        translation=_xyz(x, y, z),
        rotation=_identity_quaternion(),
    )
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=parent),
        child_frame_id=child,
        transform=transform,
    )


class _FakeHub:
    """Expose the transform and latest-slot surface used by the detector."""

    def __init__(self, tf_transform, samples: dict[str, object]) -> None:
        self.tf_transform = tf_transform
        self.samples = samples
        self.lookup_calls: list[tuple[str, str]] = []
        self.latest_calls: list[str] = []

    def lookup_transform_4x4(self, target_frame: str, source_frame: str):
        """Record TF queries and return the configured result."""
        self.lookup_calls.append((target_frame, source_frame))
        return self.tf_transform

    def has(self, kind: str) -> bool:
        """Report whether a compatibility slot was configured."""
        return kind in self.samples

    def latest(self, kind: str):
        """Return a live compatibility sample."""
        self.latest_calls.append(kind)
        return self.samples[kind], 1.0, 1


def _detector(
    hub: _FakeHub,
    *,
    camera_frame: str = "selected_camera_optical",
    base_frame: str | None = None,
):
    """Construct only the transform-selection state without loading models."""
    detector = ConceptGraphsDetector.__new__(ConceptGraphsDetector)
    detector._hub = hub
    detector._camera_frame = camera_frame
    detector._base_frame = base_frame or ""
    detector._world_frame_fn = lambda: "map"
    detector._chassis = lambda: (0.0, 0.0, 0.0, 0.0)
    detector._cam_z = 1.1
    return detector


def _compatibility_samples(
    *, parent: str = "base_link", child: str = "selected_camera_optical"
) -> dict[str, object]:
    """Return numerically distinct pose and camera-extrinsics samples."""
    return {
        "pose": _pose_message(10.0, 0.0, 0.0),
        "camera_extrinsics": _extrinsics_message(
            parent=parent,
            child=child,
            x=0.0,
            y=2.0,
            z=0.5,
        ),
    }


def test_tf_transform_wins_when_explicit_extrinsics_are_also_available() -> None:
    tf_transform = np.eye(4, dtype=np.float64)
    tf_transform[:3, 3] = [1.0, 2.0, 3.0]
    hub = _FakeHub(tf_transform, _compatibility_samples())

    actual = _detector(hub)._build_camera_to_map_transform()

    np.testing.assert_allclose(actual, tf_transform.astype(np.float32))
    assert hub.lookup_calls == [("selected_camera_optical", "map")]
    assert hub.latest_calls == []


def test_explicit_pose_and_extrinsics_are_used_only_when_tf_is_unavailable() -> None:
    hub = _FakeHub(None, _compatibility_samples())

    actual = _detector(hub)._build_camera_to_map_transform()

    expected = np.eye(4, dtype=np.float32)
    expected[:3, 3] = [10.0, 2.0, 0.5]
    np.testing.assert_allclose(actual, expected)
    assert hub.lookup_calls == [("selected_camera_optical", "map")]
    assert hub.latest_calls == ["pose", "camera_extrinsics"]


def test_extrinsics_accept_matching_parent_and_selected_child_frames() -> None:
    hub = _FakeHub(None, _compatibility_samples())

    actual = _detector(hub)._slot_extrinsics_4x4("base_link")

    assert actual is not None
    np.testing.assert_allclose(actual[:3, 3], [0.0, 2.0, 0.5])


def test_extrinsics_accept_configured_non_base_link_robot_frame() -> None:
    hub = _FakeHub(
        None,
        _compatibility_samples(parent="mobile_base", child="selected_camera_optical"),
    )

    actual = _detector(hub, base_frame="mobile_base")._slot_extrinsics_4x4(
        "mobile_base"
    )

    assert actual is not None
    np.testing.assert_allclose(actual[:3, 3], [0.0, 2.0, 0.5])


def test_extrinsics_parent_can_be_derived_from_live_odometry() -> None:
    samples = _compatibility_samples(
        parent="base_footprint", child="selected_camera_optical"
    )
    odom = _odom_message(10.0, 0.0, 0.0, child_frame="base_footprint")
    samples.pop("pose")
    samples["odom"] = odom
    hub = _FakeHub(None, samples)

    detector = _detector(hub)
    pose_transform = detector._slot_pose_transform()
    assert pose_transform is not None
    _pose, body_frame = pose_transform
    actual = detector._slot_extrinsics_4x4(body_frame)

    assert actual is not None
    assert body_frame == "base_footprint"
    assert hub.latest_calls == ["odom", "camera_extrinsics"]


def test_preferred_pose_does_not_borrow_unselected_odometry_body_frame() -> None:
    samples = _compatibility_samples(parent="base_link")
    samples["odom"] = _odom_message(
        99.0,
        0.0,
        0.0,
        child_frame="base_footprint",
    )
    hub = _FakeHub(None, samples)

    actual = _detector(hub)._build_camera_to_map_transform()

    expected = np.eye(4, dtype=np.float32)
    expected[:3, 3] = [10.0, 2.0, 0.5]
    np.testing.assert_allclose(actual, expected)
    assert hub.latest_calls == ["pose", "camera_extrinsics"]


def test_explicit_base_frame_applies_to_pose_and_extrinsics_together() -> None:
    hub = _FakeHub(
        None,
        _compatibility_samples(parent="mobile_base"),
    )

    actual = _detector(
        hub,
        base_frame="mobile_base",
    )._build_camera_to_map_transform()

    expected = np.eye(4, dtype=np.float32)
    expected[:3, 3] = [10.0, 2.0, 0.5]
    np.testing.assert_allclose(actual, expected)
    assert hub.latest_calls == ["pose", "camera_extrinsics"]


def test_explicit_base_frame_rejects_conflicting_odometry_child(caplog) -> None:
    samples = _compatibility_samples(parent="mobile_base")
    samples.pop("pose")
    samples["odom"] = _odom_message(
        10.0,
        0.0,
        0.0,
        child_frame="base_footprint",
    )
    hub = _FakeHub(None, samples)

    actual = _detector(hub, base_frame="mobile_base")._slot_pose_transform()

    assert actual is None
    assert "configured base_frame asserts 'mobile_base'" in caplog.text


@pytest.mark.parametrize(
    ("parent", "child"),
    [
        ("odom", "selected_camera_optical"),
        ("base_link", "other_camera_optical"),
        ("", "selected_camera_optical"),
        ("base_link", ""),
    ],
)
def test_extrinsics_reject_missing_or_mismatched_parent_child_frames(
    parent: str, child: str
) -> None:
    hub = _FakeHub(None, _compatibility_samples(parent=parent, child=child))

    actual = _detector(hub)._slot_extrinsics_4x4("base_link")

    assert actual is None
