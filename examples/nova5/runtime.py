# SPDX-License-Identifier: MulanPSL-2.0
"""Release the local Atlas and gRPC resources used by one operation."""
from contextlib import ExitStack
import subprocess
import sys

import grpc
import atlas_pb2
import lifecycle_pb2
import robonix_contracts_pb2_grpc as contracts


def connect_capability(atlas, provider, contract, channels, bindings):
    binding = atlas.ConnectCapability(atlas_pb2.ConnectCapabilityRequest(
        consumer_id="nova5_example", provider_id=provider,
        contract_id=contract, transport=atlas_pb2.TRANSPORT_GRPC), timeout=3)
    bindings.append(binding)
    if binding.endpoint.rsplit(":", 1)[0] != "127.0.0.1":
        raise RuntimeError("Unexpected nonlocal provider endpoint")
    channel = grpc.insecure_channel(binding.endpoint, options=[
        ("grpc.enable_http_proxy", 0), ("grpc.enable_retries", 0)])
    channels.append(channel)
    service = binding.params.grpc.service_name.rsplit(".", 1)[-1]
    return getattr(contracts, service + "Stub")(channel)


def stop_process(process):
    if process.poll() is None:
        process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def close_runtime(process, channels, driver=None, atlas=None, bindings=()):
    # A failed shutdown RPC must not leave channels or Atlas running.
    with ExitStack() as closing:
        closing.callback(stop_process, process)
        for channel in channels:
            closing.callback(channel.close)
        if driver is not None:
            try:
                response = driver.Driver(lifecycle_pb2.Driver_Request(command=3), timeout=5)
                if not response.ok:
                    print(f"Provider cleanup: {response.error}", file=sys.stderr)
            except grpc.RpcError as error:
                print(f"Provider cleanup: {error}", file=sys.stderr)
        if atlas is not None:
            for binding in bindings:
                try:
                    atlas.DisconnectCapability(atlas_pb2.DisconnectCapabilityRequest(
                        channel_id=binding.channel_id), timeout=1)
                except grpc.RpcError as error:
                    print(f"Binding cleanup: {error}", file=sys.stderr)
