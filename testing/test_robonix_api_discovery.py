# SPDX-License-Identifier: MulanPSL-2.0
"""Regression tests for the public Python Atlas discovery helpers."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import Mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pylib" / "robonix-api"))

from robonix_api.atlas import _Atlas  # noqa: E402
from robonix_api.atlas_types import (  # noqa: E402
    Capability,
    CapabilityProvider,
    Kind,
    Transport,
)


def _provider() -> CapabilityProvider:
    return CapabilityProvider(
        id="front_camera",
        kind=Kind.PRIMITIVE,
        namespace="robonix/primitive/camera",
        capabilities=(
            Capability(
                provider_id="front_camera",
                provider_kind=Kind.PRIMITIVE,
                contract_id="robonix/primitive/camera/driver",
                transport=Transport.GRPC,
            ),
            Capability(
                provider_id="front_camera",
                provider_kind=Kind.PRIMITIVE,
                contract_id="robonix/primitive/camera/rgb",
                transport=Transport.ROS2,
            ),
            Capability(
                provider_id="front_camera",
                provider_kind=Kind.PRIMITIVE,
                contract_id="robonix/primitive/camera/rgb",
                transport=Transport.MCP,
            ),
        ),
    )


class AtlasDiscoveryTest(unittest.TestCase):
    def test_find_capability_filters_whole_provider_records(self) -> None:
        atlas = _Atlas()
        atlas.query = Mock(return_value=[_provider()])

        capabilities = atlas.find_capability(
            contract_id="robonix/primitive/camera/rgb",
            transport=Transport.ROS2,
        )

        self.assertEqual(
            [(cap.contract_id, cap.transport) for cap in capabilities],
            [("robonix/primitive/camera/rgb", Transport.ROS2)],
        )

    def test_find_unique_capability_ignores_unrelated_provider_capabilities(self) -> None:
        atlas = _Atlas()
        atlas.query = Mock(return_value=[_provider()])

        capability = atlas.find_unique_capability(
            contract_id="robonix/primitive/camera/rgb",
            transport="ros2",
        )

        self.assertEqual(capability.provider_id, "front_camera")
        self.assertEqual(capability.transport, Transport.ROS2)


if __name__ == "__main__":
    unittest.main()
