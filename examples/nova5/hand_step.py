# SPDX-License-Identifier: MulanPSL-2.0
"""One PGI opening/closing operation through Robonix hand/move_joint."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--target-raw", type=int, required=True)
    parser.add_argument("--expected-start-raw", type=int, required=True)
    parser.add_argument("--atlas-binary", type=Path, required=True)
    args = parser.parse_args()
    if not all(0 <= v <= 1000 for v in (args.target_raw,args.expected_start_raw)):
        raise ValueError("Invalid PGI position")
    root = Path(__file__).resolve().parents[2]
    output_root = Path(__file__).resolve().parent / ".artifacts/nova5_rgb"
    output_root.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix="hand-",dir=output_root))
    with socket.socket() as s:
        s.bind(("127.0.0.1",0))
        endpoint = f"127.0.0.1:{s.getsockname()[1]}"
    os.environ.update(ROBONIX_ATLAS=endpoint,ROBONIX_PROVIDER_BIND_HOST="127.0.0.1",
                      ROBONIX_ADVERTISE_HOST="127.0.0.1",SCRIBE_LOG_DIR=str(output),
                      ROBONIX_DRIVER_CONTRACT_ID="robonix/lifecycle/driver")
    os.environ.pop("ROBONIX_ATLAS_ENDPOINT",None)
    os.environ.pop("RBNX_INSTANCE_NAME",None)
    os.environ.pop("RBNX_DEPLOY_MANAGED",None)
    import grpc
    import serial
    import atlas_pb2_grpc, lifecycle_pb2, hand_pb2
    from robonix_api import Primitive, Ok
    from pgi_hand.protocol import read_status,read_register,write_position_once
    from pgi_hand.motion import check_closing,check_opening,GuardedPort
    from runtime import close_runtime, connect_capability
    primitive = Primitive(id="nova5_pgi_hand",namespace="robonix/primitive/hand")
    used = threading.Lock()
    evidence = {"contract":"robonix/primitive/hand/move_joint","transport":"grpc",
                "target_raw":args.target_raw,"expected_start_raw":args.expected_start_raw,"steps":[]}

    @primitive.on_init
    def initialize(config):
        return Ok()

    @primitive.grpc("robonix/primitive/hand/move_joint")
    def move(request,context):
        if (len(request.targets)!=1 or request.targets[0].name!="pgi_opening"
                or abs(request.targets[0].value-args.target_raw/1000)>1e-9
                or not used.acquire(blocking=False)):
            context.abort(grpc.StatusCode.INVALID_ARGUMENT,"Only the planned single PGI operation is accepted")
        deadline=time.monotonic()+90
        def active():
            if not context.is_active() or time.monotonic()>deadline:
                raise RuntimeError("Operation canceled or expired; no additional gripper step sent")
        try:
            with serial.Serial(args.port,baudrate=115200,timeout=.25,write_timeout=.25,exclusive=True) as port:
                def read():
                    active()
                    state=read_status(port,1,.25)
                    state["configuration"]={"force_setpoint_percent":read_register(port,1,0x0101,.25),
                         "target_position_raw":read_register(port,1,0x0103,.25),
                         "speed_setpoint_percent":read_register(port,1,0x0104,.25)}
                    return state
                state=read()
                if state["position_raw"]!=args.expected_start_raw:
                    raise ValueError("Gripper changed since preflight")
                opening=args.target_raw>state["position_raw"]
                for _ in range(51):
                    active()
                    if not opening and state["grip_status_raw"]==2:
                        time.sleep(.3)
                        confirmed=read()
                        if confirmed["grip_status_raw"]!=2 or abs(confirmed["position_raw"]-state["position_raw"])>2:
                            raise RuntimeError("Object contact did not remain stable")
                        state=confirmed
                        break
                    if state["position_raw"]==args.target_raw:
                        break
                    position=state["position_raw"]
                    target=min(position+20,args.target_raw) if opening else max(position-20,args.target_raw)
                    if opening:
                        check_opening(state,target)
                    else:
                        check_closing(state,position-target)
                    write_position_once(GuardedPort(port,active),1,target,.25)
                    evidence["steps"].append({"from_raw":position,"target_raw":target})
                    (output/"result.json").write_text(json.dumps(evidence,indent=2))
                    end=time.monotonic()+5
                    while True:
                        time.sleep(.05)
                        state=read()
                        if state["grip_status_raw"]==2 and not opening:
                            break
                        if state["grip_status_raw"]==1 and state["position_raw"]==target:
                            break
                        # Releasing a supported object can briefly report
                        # dropped-object status before the opening step settles.
                        # Wait for idle/target; never dispatch another step in 3.
                        allowed = (0, 1, 2, 3) if opening else (0, 1)
                        if state["grip_status_raw"] not in allowed or time.monotonic()>end:
                            raise RuntimeError("PGI did not reach the step or reported a fault; no retry")
                else:
                    raise RuntimeError("Gripper step budget exceeded")
                evidence["final_state"]=state
                evidence["contact_detected"]=state["grip_status_raw"]==2
                (output/"result.json").write_text(json.dumps(evidence,indent=2))
                if not opening and state["grip_status_raw"]!=2:
                    raise RuntimeError("Closed without detecting an object; do not lift")
                return hand_pb2.MoveJoint_Response(ok=True,message="Gripper operation finished; see state evidence")
        except Exception as error:
            evidence["error"]=str(error)
            (output/"result.json").write_text(json.dumps(evidence,indent=2))
            return hand_pb2.MoveJoint_Response(ok=False,message=str(error))

    options=[("grpc.enable_http_proxy",0),("grpc.enable_retries",0)]
    channels=[]
    bindings=[]
    atlas=driver=None
    with (output/"atlas.log").open("w") as log:
        process=subprocess.Popen([str(args.atlas_binary.resolve()),"--listen",endpoint,
                                  "--capabilities",str(root/"capabilities")],stdout=log,stderr=subprocess.STDOUT)
        try:
            channel=grpc.insecure_channel(endpoint,options=options)
            channels.append(channel)
            grpc.channel_ready_future(channel).result(timeout=10)
            atlas=atlas_pb2_grpc.AtlasStub(channel)
            primitive.bootstrap()
            def connect(contract):
                return connect_capability(atlas, primitive.id, contract, channels, bindings)

            driver=connect("robonix/lifecycle/driver")
            for command in (0,1):
                response=driver.Driver(lifecycle_pb2.Driver_Request(command=command,config_json="{}"),timeout=5)
                if not response.ok:
                    raise RuntimeError(response.error)
            hand=connect("robonix/primitive/hand/move_joint")
            call=hand.MoveJoint.future(hand_pb2.MoveJoint_Request(targets=[
                hand_pb2.JointValue(name="pgi_opening",value=args.target_raw/1000)]),timeout=95)
            import signal
            signal.signal(signal.SIGINT,lambda *_:call.cancel())
            signal.signal(signal.SIGTERM,lambda *_:call.cancel())
            result=call.result()
            print(json.dumps({"evidence":str(output/"result.json"),**evidence},indent=2),flush=True)
            if not result.ok:
                raise RuntimeError(result.message)
        finally:
            close_runtime(process, channels, driver, atlas, bindings)


if __name__ == "__main__":
    main()
