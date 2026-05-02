#!/usr/bin/env python3
# [ARCHIVED — Octo VLA shelved; use graspnet_node.py instead.]
# The Octo loading path below is commented out. This node now runs in
# "scripted" mode only and serves as a reference / lightweight fallback.
# To enable the GraspNet pick skill, start graspnet_node instead:
#   python3 -m maniskill_vla_demo.graspnet_node
"""VLA (Vision-Language-Action) policy node — scripted fallback only.

Fetches observations from env_node via gRPC, runs the scripted heuristic
policy, and exposes MCP tools for robonix-pilot.

Octo model support has been removed from the active path. See pyproject.octo.toml
for the original full dependency set if you want to restore it.
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

def _ensure_proto_paths() -> None:
    pkg = Path(__file__).resolve().parent
    sys.path.insert(0, str(pkg))
    d = pkg
    while d.parent != d:
        pg = d / "proto_gen"
        if pg.is_dir() and (pg / "robonix_runtime_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent


def _ensure_robonix_py() -> None:
    """Add the shared Python helper lib (crates/robonix-py) to sys.path.

    Uses `rbnx path robonix-py` when available; falls back to PYTHONPATH
    injected by the package build.sh (rbnx-build/ws/install/setup.bash).
    """
    import subprocess
    try:
        out = subprocess.run(
            ["rbnx", "path", "robonix-py"],
            capture_output=True, text=True, timeout=5, check=False,
        )
        if out.returncode == 0:
            lib = Path(out.stdout.strip())
            if lib.is_dir() and str(lib) not in sys.path:
                sys.path.insert(0, str(lib))
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass  # rbnx not installed → rely on PYTHONPATH set by build.sh


_ensure_proto_paths()
_ensure_robonix_py()

from mcp.server.fastmcp import FastMCP  # noqa: E402

import grpc  # noqa: E402
import robonix_runtime_pb2 as pb  # noqa: E402
import robonix_runtime_pb2_grpc as pb_grpc  # noqa: E402
import maniskill_env_pb2 as env_pb  # noqa: E402
import maniskill_env_pb2_grpc as env_pb_grpc  # noqa: E402

mcp = FastMCP("vla")

_env_stub: env_pb_grpc.EnvDataServiceStub | None = None
_policy_name: str = "openvla"
_openvla_model = None
_openvla_processor = None
_openvla_device = "cuda:0"
_openvla_unnorm_key: str = "bridge_orig"
# Kept for reference; Octo path is shelved, not loaded in active code.
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


def _load_openvla() -> bool:
    """Load OpenVLA 7B from HuggingFace. Returns True on success."""
    global _openvla_model, _openvla_processor, _openvla_device, _openvla_unnorm_key
    try:
        import torch
        from transformers import AutoModelForVision2Seq, AutoProcessor
    except Exception as e:
        print(f"[vla] OpenVLA deps unavailable: {e}", file=sys.stderr)
        return False

    model_id = os.environ.get("VLA_MODEL", "openvla/openvla-7b")
    dtype_str = os.environ.get("VLA_DTYPE", "bfloat16").lower()
    dtype = {
        "bfloat16": torch.bfloat16,
        "float16": torch.float16,
        "float32": torch.float32,
    }.get(dtype_str, torch.bfloat16)
    _openvla_unnorm_key = os.environ.get("VLA_UNNORM_KEY", "bridge_orig")

    if not torch.cuda.is_available():
        print("[vla] no CUDA available; OpenVLA needs GPU — abort", file=sys.stderr)
        return False

    print(f"[vla] loading {model_id} (dtype={dtype_str}, unnorm_key={_openvla_unnorm_key})…",
          file=sys.stderr)
    try:
        _openvla_processor = AutoProcessor.from_pretrained(model_id, trust_remote_code=True)
        # OpenVLA's prismatic model does not support HF's attn_implementation
        # dispatch ('_supports_sdpa' missing) — let it use default attention.
        _openvla_model = AutoModelForVision2Seq.from_pretrained(
            model_id,
            torch_dtype=dtype,
            low_cpu_mem_usage=True,
            trust_remote_code=True,
        ).to(_openvla_device)
        _openvla_model.eval()
    except Exception as e:
        print(f"[vla] OpenVLA load failed: {e}", file=sys.stderr)
        return False

    print(f"[vla] OpenVLA ready on {_openvla_device}", file=sys.stderr)
    return True


def _openvla_policy(rgb: np.ndarray, instruction: str,
                    target_position: list[float] | None,
                    proprio: list[float]) -> list[float]:
    """Run OpenVLA 7B on a single observation.

    OpenVLA outputs 7-DoF delta end-effector action
        [dx, dy, dz, drx, dry, drz, gripper_open].
    We map this into Fetch's 12-DoF action space
        arm_delta(6) + gripper(1) + body(3) + base(2).
    Body and base are zeroed (tabletop tasks only; mobile nav not yet routed).
    """
    if _openvla_model is None or _openvla_processor is None:
        return [0.0] * 12

    import torch
    from PIL import Image as PILImage

    img = PILImage.fromarray(rgb)
    # OpenVLA-LIBERO training used center_crop=0.9 augmentation (see
    # openvla/experiments/robot/libero/run_libero_eval.py). Without it the
    # discrete action tokens collapse to one constant prediction regardless of
    # how the arm moves.
    if os.environ.get("VLA_CENTER_CROP", "0") == "1":
        import torchvision.transforms.functional as tvF
        w, h = img.size
        crop_pct = float(os.environ.get("VLA_CROP_PCT", "0.9"))
        sq = int(min(w, h) * crop_pct)
        img = tvF.center_crop(img, [sq, sq]).resize((w, h), PILImage.BILINEAR)
    instr = (instruction.strip() or "pick up the object").lower()
    prompt = f"In: What action should the robot take to {instr}?\nOut:"

    try:
        inputs = _openvla_processor(prompt, img).to(
            _openvla_device,
            dtype=next(_openvla_model.parameters()).dtype,
        )
        with torch.inference_mode():
            action = _openvla_model.predict_action(
                **inputs, unnorm_key=_openvla_unnorm_key, do_sample=False,
            )
        action = np.asarray(action, dtype=np.float32).reshape(-1)
    except Exception as e:
        print(f"[vla] OpenVLA inference failed: {e}", file=sys.stderr)
        return [0.0] * 12

    if action.size < 7:
        return [0.0] * 12
    dx, dy, dz, drx, dry, drz, grip_open = action[:7]
    # OpenVLA gripper: 1.0 = open, 0.0 = close. Fetch cmd: +1 open, -1 close.
    gripper = float(1.0 if grip_open >= 0.5 else -1.0)
    fetch_action = [
        float(dx), float(dy), float(dz),
        float(drx), float(dry), float(drz),
        gripper,
        0.0, 0.0, 0.0,   # body (torso, head_pan, head_tilt)
        0.0, 0.0,        # base (linear, angular)
    ]
    return [round(a, 5) for a in fetch_action]


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


def _run_vla_loop(instruction: str, max_cycles: int, steps_per_cycle: int,
                  sim_rate_hz: float, reset_env: bool,
                  stop_on_success: bool = True) -> str:
    """Core VLA closed-loop runner — shared by execute_instruction / pick / place.

    When `stop_on_success` is False, the loop ignores `step_res.done` and always
    runs the full `max_cycles`. This is what Pilot uses to chain multi-stage
    plans where the env's sparse success predicate latches True after the very
    first stage (e.g. libero_spatial "bowl on plate") but later stages still
    need to execute to actually place the second / third object. Whether a
    given stage succeeded in that case must be judged from per-stage object
    positions, not from `info.success`.
    """
    if _env_stub is None:
        return json.dumps({"error": "env gRPC connection not established"})

    max_cycles = max(1, min(int(max_cycles), 500))
    steps_per_cycle = max(1, min(int(steps_per_cycle), 50))
    sim_rate_hz = max(1.0, float(sim_rate_hz))
    step_interval = (1.0 / sim_rate_hz) * steps_per_cycle

    if reset_env:
        try:
            _env_stub.Reset(env_pb.Empty())
        except grpc.RpcError as e:
            return json.dumps({
                "status": "error",
                "error": f"env Reset failed: {e.details()}",
                "instruction": instruction,
            })

    total_reward = 0.0
    total_steps = 0
    last_action = []
    info = {}

    _debug_action = os.environ.get("VLA_DEBUG_ACTION", "0") == "1"
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

        if _policy_name == "openvla":
            action = _openvla_policy(rgb, instruction, target_pos, proprio)
            if _debug_action and cycle % 20 == 0:
                import hashlib
                rgb_h = hashlib.md5(rgb.tobytes()).hexdigest()[:8]
                print(f"[vla] cycle={cycle} eef={proprio[:3]} rgb_md5={rgb_h} rgb_mean={rgb.mean():.1f} action7={action[:7]}", file=sys.stderr, flush=True)
        elif _policy_name == "octo":
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
            if bool(step_res.done) and stop_on_success:
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
def execute_instruction(instruction: str, max_cycles: int = 50,
                        steps_per_cycle: int = 4,
                        sim_rate_hz: float = 30.0,
                        reset_env: bool = True) -> str:
    """Low-level escape hatch: send an arbitrary natural-language instruction to
    the VLA policy. Prefer the discrete skills `pick` / `place` above — those are
    what Pilot's plan graph dispatches to.
    """
    return _run_vla_loop(instruction, max_cycles, steps_per_cycle,
                         sim_rate_hz, reset_env)


@mcp.tool()
def pick_and_place(object: str, destination: str, max_cycles: int = 150,
                   steps_per_cycle: int = 1, sim_rate_hz: float = 30.0,
                   reset_env: bool = False,
                   stop_on_success: bool = False) -> str:
    """PREFERRED SKILL: grasp a named object and put it at a named destination,
    in one atomic call.

    This is the **primary** skill Pilot should use for LIBERO-style tabletop
    manipulation. It matches the exact prompt form the OpenVLA-LIBERO policy
    was trained on (`"pick up X and place it on Y"`), so the policy knows how
    to drive the full motion in one shot. Pilot composes longer tasks by
    chaining several `pick_and_place` nodes (each manipulating a different
    object), NOT by splitting one motion across `pick` + `place`.

    - object: phrase for the object to pick (e.g. "the alphabet soup").
    - destination: phrase for where to put it (e.g. "the basket").
    - max_cycles: up to ~300 cycles (~10 s) is usually enough for one pick+place.
    - reset_env: Pilot sets this true only on the FIRST node of a fresh
      episode. Middle-of-plan nodes MUST set it false so the scene persists
      across nodes.

    The instruction sent to the VLA is `"pick up <object> and place it on <destination>"`.
    """
    # Choose preposition based on destination phrase. Container-like destinations
    # ("basket", "bin", "tray", "bowl", "drawer") take "in"; flat surfaces
    # ("plate", "table", "stove", "rack", "shelf") take "on".
    dest_lower = destination.lower()
    if any(tok in dest_lower for tok in ("basket", "bin", "tray", "drawer",
                                          "microwave", "cabinet", "bowl", "cup",
                                          "caddy", "compartment")):
        prep = "in"
    else:
        prep = "on"
    instruction = f"pick up {object} and place it {prep} {destination}".strip()
    return _run_vla_loop(instruction, max_cycles, steps_per_cycle,
                         sim_rate_hz, reset_env, stop_on_success=stop_on_success)


@mcp.tool()
def pick(object: str, max_cycles: int = 250,
         steps_per_cycle: int = 1, sim_rate_hz: float = 30.0,
         reset_env: bool = True) -> str:
    """SKILL: grasp a named object on the tabletop and lift it clear.

    Pilot dispatches this node of the plan graph to the VLA policy. Internally
    the instruction sent to the VLA is `"pick up <object>"`.

    - object: natural-language phrase for the target (e.g. "the black bowl on
      the plate", "the red cube"). Pass exactly what Pilot put in the node.
    - max_cycles: upper bound on VLA predict->step cycles before the skill
      returns timeout. Pilot treats timeout as failure.
    - reset_env: Pilot sets this true on the first plan node, false on all
      subsequent nodes so downstream skills inherit the post-pick state.

    Returns the same JSON shape as `execute_instruction`. Look at
    `info.success` to decide whether the skill succeeded (it may also be false
    at this stage and become true after a later `place`).
    """
    instruction = f"pick up {object}".strip()
    return _run_vla_loop(instruction, max_cycles, steps_per_cycle,
                         sim_rate_hz, reset_env)


@mcp.tool()
def place(object: str, destination: str, max_cycles: int = 250,
          steps_per_cycle: int = 1, sim_rate_hz: float = 30.0,
          reset_env: bool = False) -> str:
    """SKILL: place a previously-grasped object at a named destination.

    Pilot dispatches this node after a successful `pick`. Internally the VLA
    instruction is `"place the <object> on <destination>"`. Do not reset the
    env unless you are starting a brand new episode.

    - object: phrase identifying the held object (usually matches the previous
      `pick` node's object arg).
    - destination: phrase naming where to place it (e.g. "the plate",
      "the wooden tray", "the ramekin").
    - reset_env: should be false in a normal plan. Set true only when this is
      the very first skill call of a fresh episode.

    Returns the same JSON shape as `execute_instruction`. `info.success` flips
    to true when the env judges the task complete.
    """
    instruction = f"place the {object} on {destination}".strip()
    return _run_vla_loop(instruction, max_cycles, steps_per_cycle,
                         sim_rate_hz, reset_env)


@mcp.tool()
def operate(action: str, target: str, max_cycles: int = 250,
            steps_per_cycle: int = 1, sim_rate_hz: float = 30.0,
            reset_env: bool = False) -> str:
    """SKILL: execute a non-pick/place primitive such as open, close,
    turn on, turn off, push, pull on a named target.

    Use this for things like: `operate(action="close", target="the bottom
    drawer")`, `operate(action="turn on", target="the stove")`,
    `operate(action="open", target="the microwave")`. Pilot selects this skill
    when neither `pick` nor `place` fits the next subgoal.

    - action: verb phrase ("close", "open", "turn on", "turn off", "push", ...)
    - target: object phrase ("the bottom drawer of the cabinet",
      "the microwave", "the stove")
    - reset_env: keep false in the middle of a plan.

    The instruction sent to the VLA is `"<action> <target>"`.
    """
    instruction = f"{action.strip()} {target.strip()}"
    return _run_vla_loop(instruction, max_cycles, steps_per_cycle,
                         sim_rate_hz, reset_env)


@mcp.tool()
def snapshot_scene() -> str:
    """Query the current env state and return all movable-object positions.

    Pilot / the eval driver call this between plan nodes to judge per-stage
    success ("did this object reach its destination?") independently of the
    env's sparse task-level success predicate.
    """
    if _env_stub is None:
        return json.dumps({"error": "env gRPC connection not established"})
    try:
        # Take one no-op step with a dummy-open gripper to get the latest obs +
        # info (including object_positions from libero_env_node).
        step = _env_stub.Step(env_pb.Action(values=[0, 0, 0, 0, 0, 0, -1]))
        info = json.loads(step.info_json) if step.info_json else {}
        return json.dumps({
            "status": "ok",
            "object_positions": info.get("object_positions", {}),
            "proprio": list(step.obs.proprio),
        })
    except grpc.RpcError as e:
        return json.dumps({"status": "error", "error": f"env Step failed: {e.details()}"})


@mcp.tool()
def reset_episode() -> str:
    """SKILL: reset the LIBERO env to the next fixed initial state. Pilot should
    call this once at the start of each episode, before any `pick` / `place`."""
    if _env_stub is None:
        return json.dumps({"error": "env gRPC connection not established"})
    try:
        _env_stub.Reset(env_pb.Empty())
    except grpc.RpcError as e:
        return json.dumps({"status": "error",
                           "error": f"env Reset failed: {e.details()}"})
    return json.dumps({"status": "ok"})


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
    default_policy = os.environ.get("VLA_BACKEND", "openvla")
    ap.add_argument(
        "--policy",
        choices=["scripted", "openvla", "octo"],
        default=default_policy,
    )
    args = ap.parse_args()
    _policy_name = args.policy

    if _policy_name == "openvla":
        if not _load_openvla():
            print("[vla] OpenVLA load failed — falling back to scripted policy",
                  file=sys.stderr)
            _policy_name = "scripted"
    elif _policy_name == "octo":
        # Octo path is shelved; keep fallback for reference.
        print("[vla] Octo is shelved — using scripted policy.", file=sys.stderr)
        _policy_name = "scripted"

    server_addr = os.environ.get("ROBONIX_ATLAS", "localhost:50051")
    channel = grpc.insecure_channel(server_addr)
    stub = pb_grpc.RobonixRuntimeStub(channel)

    node_id = "com.robonix.demo.vla"
    stub.RegisterNode(pb.RegisterNodeRequest(
        node_id=node_id,
        namespace="robonix/primitive/manipulation",
        kind="primitive",
    ))

    mcp_port = _pick_port()
    stub.DeclareInterface(pb.DeclareInterfaceRequest(
        node_id=node_id,
        name="mcp_tools",
        supported_transports=["mcp"],
        metadata_json=_iface_meta_mcp(),
        listen_port=mcp_port,
        contract_id="robonix/primitive/manipulation/tools",
    ))

    # Start MCP HTTP + heartbeat BEFORE env discovery (so agent can connect)
    threading.Thread(target=_heartbeat_loop, args=(stub, node_id), daemon=True).start()
    threading.Thread(target=_start_mcp_http, args=(mcp_port,), daemon=True).start()
    print(f"[vla] MCP :{mcp_port}  policy={_policy_name} (serving)", file=sys.stderr)

    override = os.environ.get("ENV_GRPC_OVERRIDE", "").strip()
    if override:
        env_endpoint = override
        print(f"[vla] using ENV_GRPC_OVERRIDE={env_endpoint}", file=sys.stderr)
    else:
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
