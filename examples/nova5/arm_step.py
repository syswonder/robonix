# SPDX-License-Identifier: MulanPSL-2.0
"""Execute one RGB observation move through Robonix arm/pos_command."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--calibration-root", type=Path, required=True)
    parser.add_argument("--sdk-python", type=Path, required=True)
    parser.add_argument("--atlas-binary", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    plan = json.loads(args.plan.read_text())
    output = args.plan.parent
    # A consumed plan cannot submit a second physical action after an uncertain result.
    with args.plan.with_name("submission-started.json").open("x") as marker:
        json.dump({"plan": str(args.plan)}, marker)
    with socket.socket() as reserved:
        reserved.bind(("127.0.0.1", 0))
        endpoint = f"127.0.0.1:{reserved.getsockname()[1]}"
    os.environ.update(ROBONIX_ATLAS=endpoint, ROBONIX_PROVIDER_BIND_HOST="127.0.0.1",
                      ROBONIX_ADVERTISE_HOST="127.0.0.1", SCRIBE_LOG_DIR=str(output),
                      ROBONIX_DRIVER_CONTRACT_ID="robonix/lifecycle/driver")
    os.environ.pop("ROBONIX_ATLAS_ENDPOINT", None)
    os.environ.pop("RBNX_INSTANCE_NAME", None)
    os.environ.pop("RBNX_DEPLOY_MANAGED", None)
    import grpc
    import atlas_pb2_grpc
    import lifecycle_pb2
    import geometry_msgs_pb2
    from google.protobuf.empty_pb2 import Empty
    from robonix_api import Primitive, Ok
    from runtime import close_runtime, connect_capability

    pose = plan["target_flange_mm_deg"]
    if len(pose) != 6 or not all(type(v) in (int, float) and math.isfinite(v) for v in pose):
        raise ValueError("Target must contain six finite numbers")
    rx, ry, rz = [math.radians(v) / 2 for v in pose[3:]]
    sx, sy, sz = math.sin(rx), math.sin(ry), math.sin(rz)
    cx, cy, cz = math.cos(rx), math.cos(ry), math.cos(rz)
    target = geometry_msgs_pb2.Pose(
        position=geometry_msgs_pb2.Point(x=pose[0]/1000, y=pose[1]/1000, z=pose[2]/1000),
        orientation=geometry_msgs_pb2.Quaternion(x=sx*cy*cz-cx*sy*sz, y=cx*sy*cz+sx*cy*sz,
                                               z=cx*cy*sz-sx*sy*cz, w=cx*cy*cz+sx*sy*sz))
    primitive = Primitive(id="nova5_sdk_observation", namespace="robonix/primitive/arm")
    used = threading.Lock()

    @primitive.on_init
    def initialize(config):
        return Ok()

    @primitive.grpc("robonix/primitive/arm/pos_command")
    def move(requests, context):
        values = list(requests)
        if len(values) != 1 or values[0] != target or not used.acquire(blocking=False):
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "Only the single planned observation pose is accepted")
        env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "PYTHONHOME")}
        with (output / "sdk.log").open("w") as log:
            child = subprocess.Popen([str(args.sdk_python.absolute()),
                str(Path(__file__).with_name("arm_worker.py")), str(args.calibration_root.resolve()),
                str(args.plan.resolve())], env=env, stdout=log, stderr=subprocess.STDOUT)
            def cancel():
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
            context.add_callback(cancel)
            result = child.wait()
        if result:
            context.abort(grpc.StatusCode.ABORTED, (output / "sdk.log").read_text()[-2500:])
        return Empty()

    channels = []
    atlas = driver = None
    bindings = []
    options = [("grpc.enable_http_proxy", 0), ("grpc.enable_retries", 0)]
    with (output / "atlas.log").open("w") as log:
        process = subprocess.Popen([str(args.atlas_binary.resolve()), "--listen", endpoint,
            "--capabilities", str(root / "capabilities")], stdout=log, stderr=subprocess.STDOUT)
        try:
            channel = grpc.insecure_channel(endpoint, options=options)
            channels.append(channel)
            grpc.channel_ready_future(channel).result(timeout=10)
            atlas = atlas_pb2_grpc.AtlasStub(channel)
            primitive.bootstrap()
            def connect(contract):
                return connect_capability(atlas, primitive.id, contract, channels, bindings)

            driver = connect("robonix/lifecycle/driver")
            for command in (0, 1):
                response = driver.Driver(lifecycle_pb2.Driver_Request(command=command, config_json="{}"), timeout=5)
                if not response.ok:
                    raise RuntimeError(response.error)
            arm = connect("robonix/primitive/arm/pos_command")
            call = arm.PosCommand.future(iter([target]), timeout=40)
            signal.signal(signal.SIGINT, lambda *_: call.cancel())
            signal.signal(signal.SIGTERM, lambda *_: call.cancel())
            call.result()
            evidence = {"provider": primitive.id, "contract": "robonix/primitive/arm/pos_command",
                        "transport": "grpc", "speed_mm_s": 50, "target_flange_mm_deg": pose,
                        "arrival": json.loads(args.plan.with_name("arrival.json").read_text())}
            (output / "robonix-call.json").write_text(json.dumps(evidence, indent=2) + "\n")
            print(json.dumps(evidence, indent=2), flush=True)
        finally:
            close_runtime(process, channels, driver, atlas, bindings)


if __name__ == "__main__":
    main()
