"""Shared forwarding logic for the Robonix body adapters.

Each adapter is one Robonix Service with one asynchronous capability (the
executor allows only one async capability per provider, because the status
and cancel tool names collide otherwise). The capability starts an operation
on the benchmark's embodiment service over gRPC; its /status and /cancel
sub-contracts read and stop that operation. Nothing here decides anything:
it translates transport and schemas only."""
import json
import os
import sys

import grpc

sys.path.insert(0, os.environ["EMBODIEDOSBENCH_ROOT"])
from embodiedosbench.proto import embodiment_pb2 as pb  # noqa: E402
from embodiedosbench.proto import embodiment_pb2_grpc as pb_grpc  # noqa: E402

# Embodiment-service operation state -> Robonix executor async state.
STATE = {"accepted": "PENDING", "running": "RUNNING", "succeeded": "SUCCEEDED",
         "failed": "FAILED", "stopped": "CANCELED"}

_stub = None
_last = {"op_id": ""}


def stub():
    """Lazily connected embodiment-service stub (address from BENCH_EMBODIMENT_ADDR)."""
    global _stub
    if _stub is None:
        _stub = pb_grpc.EmbodimentStub(grpc.insecure_channel(os.environ.get("BENCH_EMBODIMENT_ADDR", "127.0.0.1:50191")))
    return _stub


def start(skill, args, caller):
    """Start an operation; returns (accepted, run_id, detail)."""
    r = stub().Start(pb.StartRequest(skill=skill, args_json=json.dumps(args), caller=caller), timeout=10)
    if r.accepted:
        _last["op_id"] = r.op_id
        return True, r.op_id, f"{skill} {args} started"
    return False, "", f"{skill} rejected: {r.reason}"


def status(run_id):
    """(known, executor_state, detail) for run_id, or for the most recent operation when empty."""
    op_id = run_id or _last["op_id"]
    st = stub().Status(pb.StatusRequest(op_id=op_id), timeout=10)
    if st.state == "unknown":
        return False, "FAILED", f"unknown operation {op_id!r}"
    return True, STATE[st.state], st.error or st.state


def cancel(run_id):
    """Stop the operation; returns (accepted, detail)."""
    op_id = run_id or _last["op_id"]
    r = stub().Stop(pb.StopRequest(op_id=op_id), timeout=10)
    return r.ok, f"stop {op_id}: {r.state}"
