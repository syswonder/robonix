# SPDX-License-Identifier: MulanPSL-2.0
"""Controllable gRPC provider using the generated contract wire types."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys
import time

import grpc
from google.protobuf.json_format import ParseDict
from google.protobuf.message_factory import GetMessageClass

parser = argparse.ArgumentParser()
parser.add_argument("--types", required=True)
parser.add_argument("--atlas")
parser.add_argument("--port", type=int, required=True)
parser.add_argument("--control", type=Path, required=True)
args = parser.parse_args()
sys.path.insert(0, args.types)
import robonix_contracts_pb2 as contracts

runs = {}


def config():
    return json.loads(args.control.read_text())


def trace(name, **values):
    with args.control.with_suffix(".grpc-trace").open("a") as output:
        output.write(json.dumps({"name": name, **values}) + "\n")


def handler(method):
    output = GetMessageClass(method.output_type)
    name = method.name

    def unary(request, context):
        settings = config()
        trace(name)
        time.sleep(settings.get("delay", 0))
        if settings.get("grpc_error"):
            context.abort(grpc.StatusCode.FAILED_PRECONDITION, "downstream chassis busy")
        if name == "GetMode":
            return output(ok=True, mode="mapping")
        if name == "Navigate":
            run_id = settings.get("run_id", "grpc-run")
            runs[run_id] = False
            return output(accepted=not settings.get("reject", False), run_id=run_id)
        if name == "GetNavigationStatus":
            if settings.get("status_error"):
                context.abort(grpc.StatusCode.FAILED_PRECONDITION, "status unavailable: downstream chassis busy")
            return output(known=True, state=settings.get("status_state") or ("SUCCEEDED" if settings.get("finish") else
                          "CANCELED" if runs.get(request.run_id) else "RUNNING"))
        if name == "CancelNavigation":
            if not settings.get("ignore_cancel"):
                runs[request.run_id] = True
            return output(accepted=True)
        if name == "Driver":
            if args.atlas and settings.get("driver_provider"):
                from robonix_api.atlas import _Atlas
                atlas = _Atlas(args.atlas)
                atlas._ensure_stub()
                atlas._wire_stub.SetLifecycleState(atlas._wire_pb.SetLifecycleStateRequest(
                    id=settings["driver_provider"], state=atlas._wire_pb.STATE_ERROR if settings.get("driver_reject") else atlas._wire_pb.STATE_ACTIVE))
                atlas._channel.close()
            if settings.get("driver_reject"):
                return output(ok=False, state="ERROR", error="activation failed")
            return output(ok=True, state="ACTIVE")
        return output()

    def server_stream(request, context):
        settings = config()
        for index in range(settings.get("stream_count", 3)):
            time.sleep(settings.get("stream_delay", 0))
            if name == "SynthesizeStream":
                yield ParseDict({"chunk": {"data": "AQI=", "sequence": index}}, output())
            elif name == "Mic":
                yield output(data=b"\x01\x02", sequence=index)
            else:
                yield output()

    def client_stream(requests, context):
        if config().get("early_return"):
            next(requests)
            time.sleep(0.05)
            return output(detected=True, keyword="early")
        sequence = [item.sequence for item in requests]
        trace(name, sequence=sequence)
        if name == "DetectWakeWord":
            return output(detected=True, keyword=str(sequence), confidence=1)
        return output()

    def bidi(requests, context):
        for item in requests:
            yield output(text=str(item.chunk.sequence), is_final=True)

    shape = ("stream" if method.client_streaming else "unary") + "_" + (
        "stream" if method.server_streaming else "unary")
    callback = {"unary_unary": unary, "unary_stream": server_stream,
                "stream_unary": client_stream, "stream_stream": bidi}[shape]
    return getattr(grpc, shape + "_rpc_method_handler")(
        callback, request_deserializer=GetMessageClass(method.input_type).FromString,
        response_serializer=output.SerializeToString,
    )


server = grpc.server(ThreadPoolExecutor(max_workers=16))
for service in contracts.DESCRIPTOR.services_by_name.values():
    server.add_generic_rpc_handlers((grpc.method_handlers_generic_handler(
        service.full_name, {m.name: handler(m) for m in service.methods},
    ),))
driver = contracts.DESCRIPTOR.services_by_name["RobonixLifecycleDriver"].methods[0]
server.add_generic_rpc_handlers((grpc.method_handlers_generic_handler(
    "robonix.contracts.RobonixSkillChannelTestDriver", {"Driver": handler(driver)}),))
server.add_insecure_port(f"127.0.0.1:{args.port}")
server.start()
server.wait_for_termination()
