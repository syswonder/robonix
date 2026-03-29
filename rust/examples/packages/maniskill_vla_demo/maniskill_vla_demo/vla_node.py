#!/usr/bin/env python3
"""VLA (Vision-Language-Action) policy node.

Fetches observations from env_node via gRPC, runs a policy, exposes MCP tool
for robonix-agent.

Policies (--policy flag):
  octo      (default) Octo-small model (~93M params).  Language-conditioned.
  scripted  Heuristic reach-toward-target + close gripper.  No model.

Install Octo (requires JAX+CUDA):
  pip install git+https://github.com/octo-models/octo
  pip install --upgrade "jax[cuda12]" \\
    -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html
"""
import argparse
import asyncio
import json
import logging
import os
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

for _n in (
    "mcp", "mcp.server", "mcp.server.streamable_http",
    "mcp.server.streamable_http_manager",
    "httpx", "httpcore", "uvicorn", "uvicorn.access",
):
    logging.getLogger(_n).setLevel(logging.WARNING)

from mcp.server.fastmcp import FastMCP


def _ensure_proto_paths() -> None:
    pkg = Path(__file__).resolve().parent
    sys.path.insert(0, str(pkg))
    d = pkg
    while d.parent != d:
        pc = d / "proto_stubs"
        pg = d / "proto_gen"
        if pc.is_dir() and (pc / "robonix_runtime_pb2.py").exists():
            sys.path.insert(0, str(pc))
            if pg.is_dir():
                sys.path.append(str(pg))
            return
        if pg.is_dir() and (pg / "robonix_runtime_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent


_ensure_proto_paths()

import grpc  # noqa: E402
import robonix_runtime_pb2 as pb  # noqa: E402
import robonix_runtime_pb2_grpc as pb_grpc  # noqa: E402
import maniskill_env_pb2 as env_pb  # noqa: E402
import maniskill_env_pb2_grpc as env_pb_grpc  # noqa: E402

mcp = FastMCP("vla")

_env_stub: env_pb_grpc.EnvDataServiceStub | None = None
_policy_name: str = "octo"
_octo_model = None

# JAX tree_map compat shim (jax.tree.map added in jax>=0.4.25)
def _jax_tree_map(fn, *args):
    import jax
    try:
        return jax.tree.map(fn, *args)
    except AttributeError:
        return jax.tree_util.tree_map(fn, *args)

# ── Policy implementations ───────────────────────────────────────────────────


def _scripted_policy(rgb: np.ndarray, instruction: str,
                     target_position: list[float] | None,
                     proprio: list[float]) -> list[float]:
    """Heuristic for Fetch 12-DoF: arm(6)+gripper(1)+body(3)+base(2).
    Keeps base/body still, moves arm ee toward target, closes gripper when close."""
    inst = (instruction or "").lower()

    # Base-only intent from natural language (mobile manipulation)
    base_linear = 0.0
    base_angular = 0.0
    if any(k in inst for k in ("forward", "ahead", "go straight", "前进", "往前")):
        base_linear = 0.25
    elif any(k in inst for k in ("backward", "back", "后退", "往后")):
        base_linear = -0.25
    if any(k in inst for k in ("turn left", "rotate left", "向左转", "左转")):
        base_angular = 0.5
    elif any(k in inst for k in ("turn right", "rotate right", "向右转", "右转")):
        base_angular = -0.5

    if target_position is None or len(target_position) < 3:
        # If no 3D target, still allow base movement from language.
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, base_linear, base_angular]

    tx, ty, tz = target_position
    gain = 0.5
    dx = np.clip(tx * gain, -0.05, 0.05)
    dy = np.clip(ty * gain, -0.05, 0.05)
    dz = np.clip((tz - 0.15) * gain, -0.05, 0.05)

    dist = (tx ** 2 + ty ** 2 + (tz - 0.15) ** 2) ** 0.5
    gripper = -1.0 if dist < 0.05 else 1.0

    # arm ee delta(6: dx,dy,dz,drx,dry,drz) + gripper(1) + body(3: torso,head_pan,head_tilt) + base(2: linear,angular)
    action = [
        dx, dy, dz, 0.0, 0.0, 0.0,
        gripper,
        0.0, 0.0, 0.0,
        base_linear, base_angular,
    ]
    return [round(float(a), 5) for a in action]


def _load_octo() -> bool:
    """Load Octo-small. Returns True on success, False if not installed."""
    global _octo_model
    try:
        from octo.model.octo_model import OctoModel
    except ImportError:
        print(
            "[vla] WARNING: octo not installed — falling back to scripted policy.\n"
            "  Run:  ./run.sh setup   (creates .venv with full Octo dependency stack)",
            file=sys.stderr,
        )
        return False

    # Optional: warm up JAX compilation cache
    try:
        from octo.utils.jax_utils import initialize_compilation_cache
        initialize_compilation_cache()
    except Exception:
        pass

    hf_ep = os.environ.get("HF_ENDPOINT", "")
    model_id = os.environ.get("OCTO_MODEL", "hf://rail-berkeley/octo-small-1.5")
    print(f"[vla] loading {model_id} (via {hf_ep or 'HuggingFace'})…", file=sys.stderr)
    try:
        _octo_model = OctoModel.load_pretrained(model_id)
    except Exception as e:
        print(f"[vla] WARNING: Octo load failed ({e}) — falling back to scripted", file=sys.stderr)
        return False
    print("[vla] Octo ready", file=sys.stderr)
    return True


def _octo_policy(rgb: np.ndarray, instruction: str,
                 target_position: list[float] | None,
                 proprio: list[float]) -> list[float]:
    """Run Octo-small model."""
    if _octo_model is None:
        return [0.0] * 12

    from PIL import Image as PILImage
    import jax
    import jax.numpy as jnp

    # Octo was trained on 256×256 images
    img_256 = np.array(PILImage.fromarray(rgb).resize((256, 256)), dtype=np.uint8)
    # observation shape: (batch=1, history=1, H, W, C)
    step_mask = np.array([[True]], dtype=bool)
    obs = {
        "image_primary": img_256[np.newaxis, np.newaxis],
        # Octo expects timestep_pad_mask (and may read pad_mask_dict for modalities).
        "timestep_pad_mask": step_mask,
        "pad_mask_dict": {
            "image_primary": step_mask,
            "timestep": step_mask,
        },
    }
    # Optionally include proprio if the model config supports it
    if proprio:
        obs["proprio"] = np.array(proprio, dtype=np.float32)[np.newaxis, np.newaxis]

    task = _octo_model.create_tasks(texts=[instruction])

    # sample_actions → (batch, pred_horizon, action_dim)
    rng = jax.random.PRNGKey(int(time.time() * 1000) % (2**31))
    try:
        actions = _octo_model.sample_actions(obs, task, rng=rng)
    except Exception as e:
        print(f"[vla] Octo sample_actions failed: {e}", file=sys.stderr)
        return [0.0] * 12

    # action_7: [dx, dy, dz, droll, dpitch, dyaw, gripper_logic]
    raw_action = np.array(actions[0, 0]).flatten()
    gripper_logic = raw_action[6]
    gripper_target = float(np.clip(gripper_logic * 0.06 - 0.01, -0.01, 0.05))
    arm_delta = raw_action[:6].tolist()
    action_12 = arm_delta + [gripper_target] + [0.0, 0.0, 0.0, 0.0, 0.0]
    return [round(float(a), 5) for a in action_12]


# ── MCP tool ─────────────────────────────────────────────────────────────────


@mcp.tool()
def execute_instruction(instruction: str, max_cycles: int = 50,
                        steps_per_cycle: int = 4,
                        sim_rate_hz: float = 30.0) -> str:
    """Execute a skill: closed-loop VLA predict->step until done or timeout.

    - instruction: natural-language command (e.g. "pick the red cup")
    - max_cycles: max predict->step cycles before giving up
    - steps_per_cycle: sim steps to hold each predicted action
    - sim_rate_hz: wall-clock throttle so Rerun can visualize smooth motion
    """
    if _env_stub is None:
        return json.dumps({"error": "env gRPC connection not established"})

    max_cycles = max(1, min(int(max_cycles), 500))
    steps_per_cycle = max(1, min(int(steps_per_cycle), 50))
    sim_rate_hz = max(1.0, float(sim_rate_hz))
    step_interval = (1.0 / sim_rate_hz) * steps_per_cycle

    total_reward = 0.0
    total_steps = 0
    last_action = []
    info = {}

    for cycle in range(max_cycles):
        t0 = time.monotonic()

        try:
            obs = _env_stub.GetObs(env_pb.Empty())
        except grpc.RpcError as e:
            return json.dumps({
                "status": "error",
                "error": f"env GetObs failed: {e.details()}",
                "instruction": instruction,
                "cycles": cycle,
                "total_steps": total_steps,
                "reward": total_reward,
            })

        rgb = np.frombuffer(obs.rgb, dtype=np.uint8).reshape(obs.height, obs.width, 3)
        proprio = list(obs.proprio)
        target_pos = list(obs.goal_pos) if obs.goal_pos else None

        if _policy_name == "octo":
            action = _octo_policy(rgb, instruction, target_pos, proprio)
        else:
            action = _scripted_policy(rgb, instruction, target_pos, proprio)

        last_action = action

        for _ in range(steps_per_cycle):
            try:
                step_res = _env_stub.Step(env_pb.Action(values=action))
            except grpc.RpcError as e:
                return json.dumps({
                    "status": "error",
                    "error": f"env Step failed: {e.details()}",
                    "instruction": instruction,
                    "action": action,
                    "policy": _policy_name,
                    "cycles": cycle,
                    "total_steps": total_steps,
                    "reward": total_reward,
                })
            total_reward += float(step_res.reward)
            total_steps += 1
            if step_res.info_json:
                try:
                    info = json.loads(step_res.info_json)
                except Exception:
                    info = {"raw": step_res.info_json}
            if bool(step_res.done):
                return json.dumps({
                    "status": "done",
                    "instruction": instruction,
                    "policy": _policy_name,
                    "cycles": cycle + 1,
                    "total_steps": total_steps,
                    "reward": total_reward,
                    "info": info,
                })

        elapsed = time.monotonic() - t0
        if elapsed < step_interval:
            time.sleep(step_interval - elapsed)

    return json.dumps({
        "status": "timeout",
        "instruction": instruction,
        "policy": _policy_name,
        "max_cycles": max_cycles,
        "total_steps": total_steps,
        "reward": total_reward,
        "last_action": last_action,
        "info": info,
    })


@mcp.tool()
def move_base(linear: float = 0.2, angular: float = 0.0, hold_steps: int = 20) -> str:
    """Directly command mobile base motion (Fetch last 2 action dims).

    linear: forward/back velocity in [-1,1]
    angular: yaw rate in [-1,1]
    hold_steps: repeat count for visible movement
    """
    if _env_stub is None:
        return json.dumps({"error": "env gRPC connection not established"})

    linear = float(np.clip(linear, -1.0, 1.0))
    angular = float(np.clip(angular, -1.0, 1.0))
    action = [0.0] * 10 + [linear, angular]

    hold_steps = max(1, min(int(hold_steps), 300))
    total_reward = 0.0
    done = False
    info = {}
    for _ in range(hold_steps):
        try:
            step_res = _env_stub.Step(env_pb.Action(values=action))
        except grpc.RpcError as e:
            return json.dumps({"error": f"env Step failed: {e.details()}", "action": action})
        total_reward += float(step_res.reward)
        done = bool(step_res.done)
        if step_res.info_json:
            try:
                info = json.loads(step_res.info_json)
            except Exception:
                info = {"raw_info_json": step_res.info_json}
        if done:
            break

    return json.dumps({
        "action": action,
        "hold_steps": hold_steps,
        "reward": total_reward,
        "done": done,
        "info": info,
    })


# ── Boilerplate ──────────────────────────────────────────────────────────────


def _pick_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _mcp_tools_list() -> list[dict]:
    async def _list():
        return await mcp.list_tools()
    tools = asyncio.run(_list())
    return [{"name": t.name, "description": t.description or "",
             "input_schema": dict(t.inputSchema)} for t in tools]


def _iface_meta_mcp() -> str:
    return json.dumps({"tools": _mcp_tools_list()})


def _start_mcp_http(port: int) -> None:
    import uvicorn
    app = mcp.streamable_http_app()
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")


def _heartbeat_loop(stub, node_id: str) -> None:
    while True:
        time.sleep(15.0)
        try:
            stub.NodeHeartbeat(pb.NodeHeartbeatRequest(node_id=node_id))
        except Exception as e:
            print(f"[vla] heartbeat failed: {e}", file=sys.stderr)


def _discover_env_grpc(stub, node_id: str) -> str:
    for attempt in range(30):
        try:
            resp = stub.NegotiateChannel(pb.NegotiateChannelRequest(
                consumer_id=node_id,
                provider_node_id="com.robonix.demo.maniskill",
                interface_name="env_data",
                transport="grpc",
            ))
            return resp.endpoint
        except grpc.RpcError:
            if attempt < 29:
                time.sleep(2)
            else:
                raise
    raise RuntimeError("could not discover env gRPC endpoint")


# ── Main ─────────────────────────────────────────────────────────────────────


def main() -> None:
    global _env_stub, _policy_name

    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", choices=["scripted", "octo"], default="octo")
    args = ap.parse_args()
    _policy_name = args.policy

    if _policy_name == "octo":
        if not _load_octo():
            print("[vla] falling back to scripted policy", file=sys.stderr)
            _policy_name = "scripted"

    server_addr = os.environ.get("ROBONIX_SERVER", "localhost:50051")
    channel = grpc.insecure_channel(server_addr)
    stub = pb_grpc.RobonixRuntimeStub(channel)

    node_id = "com.robonix.demo.vla"
    stub.RegisterNode(pb.RegisterNodeRequest(
        node_id=node_id,
        namespace="robonix/prm/manipulation",
        kind="primitive",
    ))

    mcp_port = _pick_port()
    stub.DeclareInterface(pb.DeclareInterfaceRequest(
        node_id=node_id,
        name="mcp_tools",
        supported_transports=["mcp"],
        metadata_json=_iface_meta_mcp(),
        listen_port=mcp_port,
    ))

    # Start MCP HTTP + heartbeat BEFORE env discovery (so agent can connect)
    threading.Thread(target=_heartbeat_loop, args=(stub, node_id), daemon=True).start()
    threading.Thread(target=_start_mcp_http, args=(mcp_port,), daemon=True).start()
    print(f"[vla] MCP :{mcp_port}  policy={_policy_name} (serving)", file=sys.stderr)

    print("[vla] discovering env gRPC endpoint...", file=sys.stderr)
    env_endpoint = _discover_env_grpc(stub, node_id)
    env_channel = grpc.insecure_channel(env_endpoint)
    _env_stub = env_pb_grpc.EnvDataServiceStub(env_channel)
    print(f"[vla] connected to env gRPC at {env_endpoint}", file=sys.stderr)
    print("[vla] ready", file=sys.stderr)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
