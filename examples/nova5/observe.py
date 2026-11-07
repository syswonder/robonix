# SPDX-License-Identifier: MulanPSL-2.0
"""Call a Gemini RGB snapshot through a private Robonix Atlas and gRPC primitive."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration-root", type=Path, required=True)
    parser.add_argument("--camera-python", type=Path, required=True)
    parser.add_argument("--atlas-binary", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    output_root = Path(__file__).resolve().parent / ".artifacts/nova5_rgb"
    output_root.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix="observation-", dir=output_root))
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        endpoint = f"127.0.0.1:{reservation.getsockname()[1]}"
    os.environ.update(ROBONIX_ATLAS=endpoint, ROBONIX_PROVIDER_BIND_HOST="127.0.0.1",
                      ROBONIX_ADVERTISE_HOST="127.0.0.1", SCRIBE_LOG_DIR=str(output),
                      ROBONIX_DRIVER_CONTRACT_ID="robonix/lifecycle/driver")
    os.environ.pop("ROBONIX_ATLAS_ENDPOINT", None)
    os.environ.pop("RBNX_INSTANCE_NAME", None)
    os.environ.pop("RBNX_DEPLOY_MANAGED", None)
    import grpc
    import atlas_pb2_grpc
    import camera_pb2
    import lifecycle_pb2
    import sensor_msgs_pb2
    from robonix_api import Primitive, Ok
    from runtime import close_runtime, connect_capability

    primitive = Primitive(id="nova5_wrist_rgb", namespace="robonix/primitive/camera")
    captured = False

    @primitive.on_init
    def initialize(config):
        return Ok()

    @primitive.grpc("robonix/primitive/camera/snapshot")
    def snapshot(request, context):
        nonlocal captured
        if captured:
            context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, "This observation is single-use")
        captured = True
        try:
            result = subprocess.run(
                [str(args.camera_python.absolute()), str(Path(__file__).with_name("capture_rgb.py")),
                 str(args.calibration_root.resolve()), str(output)],
                capture_output=True, text=True, timeout=25,
                env={k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "PYTHONHOME")})
            (output / "capture.log").write_text(result.stdout + result.stderr)
            if result.returncode:
                raise RuntimeError(f"RGB acquisition failed: {result.stdout[-2000:]} {result.stderr[-2000:]}")
            meta = json.loads((output / "capture.json").read_text())
            image = sensor_msgs_pb2.Image(height=meta["height"], width=meta["width"],
                                           encoding="bgr8", step=meta["width"] * 3,
                                           data=(output / "rgb.bgr").read_bytes())
            image.header.frame_id = "nova5_wrist_camera_optical"
            image.header.stamp.sec = meta["stamp_sec"]
            image.header.stamp.nanosec = meta["stamp_nanosec"]
            return camera_pb2.GetCameraImage_Response(image=image)
        except Exception as error:
            context.abort(grpc.StatusCode.UNAVAILABLE, str(error))

    channels, bindings = [], []
    options = [("grpc.enable_http_proxy", 0), ("grpc.enable_retries", 0)]
    atlas = driver = None
    with (output / "atlas.log").open("w") as log:
        process = subprocess.Popen([str(args.atlas_binary.resolve()), "--listen", endpoint,
                                    "--capabilities", str(root / "capabilities")],
                                   stdout=log, stderr=subprocess.STDOUT)
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
            camera = connect("robonix/primitive/camera/snapshot")
            image = camera.GetCameraImage(camera_pb2.GetCameraImage_Request(), timeout=30).image
            if image.encoding != "bgr8" or len(image.data) != image.height * image.step:
                raise RuntimeError("Invalid RGB message from camera capability")
            if bytes(image.data) != (output / "rgb.bgr").read_bytes():
                raise RuntimeError("RPC image differs from the paired capture")
            evidence = {"atlas": endpoint, "provider": primitive.id,
                        "contract": "robonix/primitive/camera/snapshot", "transport": "grpc",
                        "width": image.width, "height": image.height, "motion_sent": False,
                        "image": str(output / "rgb.png"), "sample": str(output / "capture.json")}
            (output / "robonix-call.json").write_text(json.dumps(evidence, indent=2) + "\n")
            print(json.dumps(evidence, indent=2), flush=True)
        finally:
            close_runtime(process, channels, driver, atlas, bindings)


if __name__ == "__main__":
    main()
