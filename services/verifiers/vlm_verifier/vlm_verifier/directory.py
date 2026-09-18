# SPDX-License-Identifier: MulanPSL-2.0
"""Deadline-bounded Atlas camera queries (shared helper has no RPC deadlines)."""
import os
from types import SimpleNamespace

import grpc


class CameraDirectory:
    """Open short-lived Atlas transports so timed-out observations cannot leak workers."""

    def rpc(self, method, request, timeout=1.0):
        """Close the transport even if Atlas is unreachable or stops answering."""
        import atlas_pb2_grpc
        endpoint = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
        with grpc.insecure_channel(endpoint, options=[("grpc.enable_http_proxy", 0)]) as channel:
            stub = atlas_pb2_grpc.AtlasStub(channel)
            return getattr(stub, method)(request, timeout=timeout)

    def find_capability(self, *, contract_id, transport, provider_id):
        """Filter both provider and capability, since Query returns whole providers."""
        import atlas_pb2 as pb
        response = self.rpc("Query", pb.QueryRequest(
            id=provider_id, contract_id=contract_id, transport=pb.TRANSPORT_ROS2,
        ))
        return [
            SimpleNamespace(provider_id=provider.id, contract_id=cap.contract_id)
            for provider in response.providers if provider.id == provider_id
            for cap in provider.capabilities
            if cap.contract_id == contract_id and cap.transport == pb.TRANSPORT_ROS2
        ]

    def connect_capability(self, *, consumer_id, provider_id, contract_id, transport):
        """Return a bounded, explicitly closable camera channel after Atlas resolution."""
        import atlas_pb2 as pb
        response = self.rpc("ConnectCapability", pb.ConnectCapabilityRequest(
            consumer_id=consumer_id, provider_id=provider_id,
            contract_id=contract_id, transport=pb.TRANSPORT_ROS2,
        ))

        def close():
            result = self.rpc("DisconnectCapability", pb.DisconnectCapabilityRequest(
                channel_id=response.channel_id))
            if not result.was_open:
                return

        return SimpleNamespace(endpoint=response.endpoint, close=close)
