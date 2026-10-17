# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

from types import SimpleNamespace

from robonix_api.atlas_types import (
    CapabilityTimeDescriptor,
    from_pb_time_descriptor,
)


class FakeTimeDescriptor(SimpleNamespace):
    """Provide the protobuf presence API needed by the pure converter test."""

    def HasField(self, name: str) -> bool:
        return name == "pipeline_delay_ns" and self.pipeline_delay_ns is not None


def test_time_descriptor_preserves_optional_delay() -> None:
    """A known fixed delay remains distinguishable from an omitted delay."""
    wire = FakeTimeDescriptor(
        domain_id="lidar_clock",
        timestamp_semantics="scan_start",
        is_hardware_timestamped=True,
        pipeline_delay_ns=0,
        pipeline_uncertainty_ns=500,
        supports_per_point_timestamp=True,
    )
    assert from_pb_time_descriptor(wire) == CapabilityTimeDescriptor(
        domain_id="lidar_clock",
        timestamp_semantics="scan_start",
        is_hardware_timestamped=True,
        pipeline_delay_ns=0,
        pipeline_uncertainty_ns=500,
        supports_per_point_timestamp=True,
    )
