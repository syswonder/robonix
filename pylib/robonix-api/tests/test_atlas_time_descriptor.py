# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

import os
from pathlib import Path
import socket
import subprocess
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

import atlas_pb2
import atlas_pb2_grpc
import grpc
import pytest

from robonix_api import CapabilityTimeDescriptor, Primitive, Ros2Params, Transport
from robonix_api import capability
from robonix_api.atlas import _Atlas

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def atlas(tmp_path, monkeypatch):
    binary = Path(os.environ.get("CARGO_TARGET_DIR", ROOT / "target")) / "debug/robonix-atlas"
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        endpoint = f"127.0.0.1:{reservation.getsockname()[1]}"
    with (tmp_path / "atlas.log").open("w") as log:
        process = subprocess.Popen(
            [str(binary), "--listen", endpoint, "--capabilities", str(ROOT / "capabilities")],
            stdout=log, stderr=subprocess.STDOUT,
            env={**os.environ, "SCRIBE_LOG_DIR": str(tmp_path / "logs")},
        )
        try:
            with grpc.insecure_channel(endpoint, options=[("grpc.enable_http_proxy", 0)]) as channel:
                grpc.channel_ready_future(channel).result(timeout=5)
                client = _Atlas(endpoint)
                client._pb = atlas_pb2
                client._stub = atlas_pb2_grpc.AtlasStub(channel)
                monkeypatch.setattr(capability, "ATLAS", client)
                yield client
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


@pytest.mark.parametrize(
    "descriptor",
    [None, *[
        CapabilityTimeDescriptor(
            domain_id="lidar_clock", timestamp_semantics="scan_start",
            is_hardware_timestamped=True, pipeline_delay_ns=delay,
            pipeline_uncertainty_ns=500, supports_per_point_timestamp=True,
        )
        for delay in (None, 0, 1234)
    ]],
    ids=["no-descriptor", "unknown-delay", "zero-delay", "nonzero-delay"],
)
def test_time_descriptor_round_trips_through_provider_and_atlas(atlas, tmp_path, descriptor):
    provider = Primitive("metadata-lidar", "robonix/primitive/lidar", pkg_root=tmp_path)
    atlas.register_primitive(provider.id, provider.namespace)
    contract = "robonix/primitive/lidar/scan"
    endpoint = provider.declare_capability(
        contract, "/metadata_scan", Transport.ROS2, Ros2Params(),
        time_descriptor=descriptor,
    )
    providers = atlas.query_primitives(id=provider.id)
    assert providers[0].capabilities[0].time_descriptor == descriptor
    with atlas.connect_capability(
        consumer_id="metadata-consumer", provider_id=provider.id,
        contract_id=contract, transport=Transport.ROS2,
    ) as channel:
        assert channel.endpoint == endpoint
        assert channel.time_descriptor == descriptor
        assert channel.channel_id in atlas.inspect()["channels"]
    assert channel.channel_id not in atlas.inspect()["channels"]


def test_declaring_metadata_with_old_local_stubs_requires_rebuild(monkeypatch):
    atlas = _Atlas()
    atlas._pb = atlas_pb2
    atlas._stub = SimpleNamespace()
    monkeypatch.setattr(
        atlas_pb2, "DeclareCapabilityRequest", lambda **values: SimpleNamespace(**values),
    )
    with pytest.raises(RuntimeError, match="rebuild this package"):
        atlas.declare_capability(
            "metadata-lidar", "robonix/primitive/lidar/scan", Transport.ROS2,
            "/metadata_scan", time_descriptor=CapabilityTimeDescriptor("lidar_clock", "scan_start"),
        )


def test_redeclaring_timing_metadata_reports_the_duplicate(atlas):
    provider = "metadata-lidar"
    contract = "robonix/primitive/lidar/scan"
    descriptor = CapabilityTimeDescriptor("lidar_clock", "scan_start")
    atlas.register_primitive(provider, "robonix/primitive/lidar")
    atlas.declare_capability(
        provider, contract, Transport.ROS2, "/metadata_scan", time_descriptor=descriptor,
    )
    for repeated in (descriptor, replace(descriptor, domain_id="changed_clock")):
        with pytest.raises(grpc.RpcError) as error:
            atlas.declare_capability(
                provider, contract, Transport.ROS2, "/metadata_scan", time_descriptor=repeated,
            )
        assert error.value.code() == grpc.StatusCode.ALREADY_EXISTS
        assert atlas.query_primitives(id=provider)[0].capabilities[0].time_descriptor == descriptor
    assert atlas.declare_capability(provider, contract, Transport.ROS2, "/metadata_scan") == "/metadata_scan"
    assert atlas.query_primitives(id=provider)[0].capabilities[0].time_descriptor == descriptor


def test_query_with_old_response_types_exposes_unavailable_metadata():
    old_capability = SimpleNamespace(
        provider_id="metadata-lidar", provider_kind=1,
        contract_id="robonix/primitive/lidar/scan", transport=Transport.ROS2,
        description="Legacy lidar", params=None,
    )
    old_provider = SimpleNamespace(
        id="metadata-lidar", kind=1, namespace="robonix/primitive/lidar",
        capability_md_path="", last_heartbeat_ms=0, state=1, state_detail="",
        capabilities=[old_capability],
    )
    atlas = _Atlas()
    atlas._pb = atlas_pb2
    atlas._stub = SimpleNamespace(Query=Mock(return_value=SimpleNamespace(providers=[old_provider])))

    provider, = atlas.query_primitives(id="metadata-lidar")

    assert provider.id == "metadata-lidar"
    cap, = provider.capabilities
    assert cap.contract_id == old_capability.contract_id
    assert cap.transport == Transport.ROS2
    assert cap.time_descriptor is None
    atlas._stub.Query.assert_called_once_with(atlas_pb2.QueryRequest(kind=1, id=provider.id))


def test_connect_with_old_response_types_exposes_unavailable_metadata():
    old_response = SimpleNamespace(
        endpoint="/metadata_scan", channel_id="legacy-channel",
        params=atlas_pb2.TransportParams(ros2=atlas_pb2.Ros2Params(qos_profile="sensor_data")),
        HasField=lambda name: name == "params",
    )
    atlas = _Atlas()
    atlas._pb = atlas_pb2
    atlas._stub = SimpleNamespace(
        ConnectCapability=Mock(return_value=old_response),
        DisconnectCapability=Mock(return_value=SimpleNamespace(was_open=True)),
    )

    with atlas.connect_capability(
        consumer_id="metadata-consumer", provider_id="metadata-lidar",
        contract_id="robonix/primitive/lidar/scan", transport=Transport.ROS2,
    ) as channel:
        assert channel.endpoint == "/metadata_scan"
        assert channel.channel_id == "legacy-channel"
        assert channel.params == Ros2Params(qos_profile="sensor_data")
        assert channel.time_descriptor is None
    atlas._stub.DisconnectCapability.assert_called_once_with(
        atlas_pb2.DisconnectCapabilityRequest(channel_id="legacy-channel"),
    )
