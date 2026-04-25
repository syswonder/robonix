#!/usr/bin/env python3
"""LIBERO env bridge — speaks the same EnvDataService gRPC as maniskill env_node.

Launched from inside LIBERO's own venv (separate from OpenVLA venv due to
numpy/gym conflicts). vla_node connects via gRPC and does not know this is
LIBERO rather than ManiSkill3.
"""
import json
import os
import sys
import threading
import time
from concurrent import futures

import grpc
import numpy as np
import signal

os.environ.setdefault("MUJOCO_GL", "egl")

# Ensure LIBERO repo (proto stubs live here too) is importable.
# LIBERO_ROOT env var overrides; otherwise default to ~/robonix_eval/LIBERO.
_libero_root = os.environ.get("LIBERO_ROOT",
                              os.path.expanduser("~/robonix_eval/LIBERO"))
sys.path.insert(0, _libero_root)

import maniskill_env_pb2 as env_pb
import maniskill_env_pb2_grpc as env_pb_grpc

from libero.libero import benchmark, get_libero_path
from libero.libero.envs import OffScreenRenderEnv


_lock = threading.Lock()
_env = None
_task = None
_task_description = ""
_suite = None
_suite_name = "libero_spatial"
_current_task_id = 0
_init_states = None
_next_init_idx = 0
_latest_obs = None
_latest_done = False
_latest_reward = 0.0

# Per-episode capture state (set when LIBERO_RECORD_DIR is set)
_record_dir = None          # Path or None
_episode_counter = 0        # increments on every Reset when recording
_frame_buffer = None        # list[np.ndarray] for current episode
_fps = 30


def _load_task(suite, task_id: int, resolution: int):
    """(Re)build env for a specific task_id under a given suite. Caches init_states."""
    global _env, _task, _task_description, _current_task_id, _init_states, _next_init_idx
    task = suite.get_task(task_id)
    bddl = os.path.join(get_libero_path("bddl_files"), task.problem_folder, task.bddl_file)
    print(f"[libero-env] load task_id={task_id} desc={task.language!r}",
          file=sys.stderr, flush=True)
    env = OffScreenRenderEnv(bddl_file_name=bddl, camera_heights=resolution,
                             camera_widths=resolution)
    env.seed(0)
    _env = env
    _task = task
    _task_description = task.language
    _current_task_id = task_id
    _init_states = suite.get_task_init_states(task_id)
    _next_init_idx = 0


def _flush_episode_video():
    """Write buffered frames to {record_dir}/episode_{N:04d}.mp4. Silently noop if no buffer."""
    global _frame_buffer, _episode_counter
    if _record_dir is None or _frame_buffer is None or len(_frame_buffer) == 0:
        _frame_buffer = [] if _record_dir is not None else None
        return
    path = os.path.join(_record_dir, f"episode_{_episode_counter:04d}.mp4")
    try:
        import imageio
        # imageio needs HWC uint8 frames. The render is raw from sim so already HWC uint8.
        with imageio.get_writer(path, fps=_fps, codec="libx264",
                                quality=6, macro_block_size=1) as w:
            for frame in _frame_buffer:
                w.append_data(frame)
        print(f"[libero-env] recorded {len(_frame_buffer)} frames -> {path}",
              file=sys.stderr, flush=True)
    except Exception as e:
        print(f"[libero-env] video save failed: {e}", file=sys.stderr, flush=True)
    _frame_buffer = []
    _episode_counter += 1


def _build_env():
    global _suite, _suite_name, _latest_obs, _record_dir, _frame_buffer, _fps
    suite_name = os.environ.get("LIBERO_SUITE", "libero_spatial")
    task_id = int(os.environ.get("LIBERO_TASK_ID", "0"))
    resolution = int(os.environ.get("LIBERO_CAM", "256"))
    rec_dir = os.environ.get("LIBERO_RECORD_DIR", "").strip()
    if rec_dir:
        _record_dir = rec_dir
        os.makedirs(_record_dir, exist_ok=True)
        _frame_buffer = []
        _fps = int(os.environ.get("LIBERO_RECORD_FPS", "30"))
        print(f"[libero-env] recording frames to {_record_dir} @ {_fps}fps",
              file=sys.stderr, flush=True)

    d = benchmark.get_benchmark_dict()
    _suite = d[suite_name]()
    _suite_name = suite_name
    print(f"[libero-env] suite={suite_name} n_tasks={_suite.n_tasks}",
          file=sys.stderr, flush=True)
    _load_task(_suite, task_id, resolution)
    _latest_obs = _env.reset()


def _obs_to_proto(obs, done=False, reward=0.0) -> env_pb.Observation:
    # agentview_image is HWC uint8, rotate 180° to match OpenVLA training preprocessing.
    img = obs["agentview_image"]
    img = np.ascontiguousarray(img[::-1, ::-1])  # rotate 180
    h, w = img.shape[:2]

    # proprio 8D: eef_pos(3) + eef_quat(4) + gripper_qpos[0](1)
    eef_pos = np.asarray(obs["robot0_eef_pos"], dtype=np.float32).flatten()
    eef_quat = np.asarray(obs["robot0_eef_quat"], dtype=np.float32).flatten()
    grip = np.asarray(obs["robot0_gripper_qpos"], dtype=np.float32).flatten()
    proprio = np.concatenate([eef_pos, eef_quat, grip]).tolist()

    return env_pb.Observation(
        rgb=img.tobytes(),
        depth=b"",
        width=w,
        height=h,
        proprio=proprio,
        done=bool(done),
        reward=float(reward),
        fx=0.0, fy=0.0, cx=0.0, cy=0.0,
        goal_pos=[],
        camera_pose=[],
        tcp_pose=list(eef_pos) + list(eef_quat),
    )


def _reset_env():
    """Reset env using the next fixed initial state (cycles through the 10 per task).

    Env-var `LIBERO_INIT_ID` (if set to int >=0) pins to that index; otherwise
    each Reset advances to the next one (wraps at the end).
    """
    global _latest_obs, _latest_done, _latest_reward, _next_init_idx, _frame_buffer
    with _lock:
        # Flush previous episode's frames to mp4 BEFORE reset
        _flush_episode_video()
        pinned = os.environ.get("LIBERO_INIT_ID", "").strip()
        if pinned.isdigit():
            idx = int(pinned) % len(_init_states)
        else:
            idx = _next_init_idx % len(_init_states)
            _next_init_idx = (idx + 1) % len(_init_states)
        _env.reset()
        obs = _env.set_init_state(_init_states[idx])
        # LIBERO scene needs a few no-op steps after reset for physics to
        # settle. OpenVLA's run_libero_eval uses 10 dummy steps.
        dummy_wait = int(os.environ.get("LIBERO_DUMMY_WAIT", "10"))
        for _ in range(dummy_wait):
            obs, _, _, _ = _env.step([0, 0, 0, 0, 0, 0, -1])
        if _record_dir is not None and _frame_buffer is not None:
            # Capture the post-reset initial frame so the mp4 starts from rest
            _frame_buffer.append(np.asarray(obs["agentview_image"])[::-1, ::-1].copy())
        print(f"[libero-env] reset task_id={_current_task_id} init_idx={idx} "
              f"episode_counter={_episode_counter}",
              file=sys.stderr, flush=True)
        _latest_obs = obs
        _latest_done = False
        _latest_reward = 0.0
    return obs


def _step_env(action_list):
    global _latest_obs, _latest_done, _latest_reward
    # LIBERO action space is 7D (6 pose delta + gripper)
    action = list(action_list[:7])
    while len(action) < 7:
        action.append(0.0)
    # vla_node emits gripper ±1 using VLA convention (+1=close, -1=open after its
    # threshold). LIBERO/robosuite wants the OpenVLA-LIBERO convention of
    # -1=close, +1=open (normalize_gripper_action + invert_gripper_action from
    # openvla/experiments/robot/libero_utils.py). Flip the sign here so the env
    # gets what the fine-tune expects.
    action[6] = -float(action[6])
    with _lock:
        obs, reward, done, info = _env.step(action)
        # LIBERO doesn't always populate info["success"] — call env.check_success()
        success = False
        try:
            success = bool(_env.check_success())
        except Exception:
            pass
        info = dict(info or {})
        info["success"] = success
        # Expose richer object state so orchestration layer can check
        # intermediate predicates (e.g. "bowl grasped?") between sub-task calls.
        try:
            for key in ("robot0_gripper_qpos", "robot0_eef_pos"):
                if key in obs:
                    info[key] = np.asarray(obs[key]).flatten().tolist()
        except Exception:
            pass
        # Attach per-object positions so multi-stage eval can judge each
        # pick_and_place independently (LIBERO's env.check_success latches on
        # the original task only; downstream stages need ground-truth poses).
        try:
            obj_poses = {}
            for k, v in obs.items():
                if k.endswith("_pos") and not k.startswith("robot"):
                    obj_poses[k] = np.asarray(v).flatten().tolist()
            if obj_poses:
                info["object_positions"] = obj_poses
        except Exception:
            pass
        _latest_obs = obs
        _latest_reward = float(reward)
        _latest_done = bool(done) or success
        if _record_dir is not None and _frame_buffer is not None:
            # rotate to match what the model sees (same as proto rgb convention)
            _frame_buffer.append(np.asarray(obs["agentview_image"])[::-1, ::-1].copy())
    return obs, float(reward), bool(_latest_done), info


class EnvService(env_pb_grpc.EnvDataServiceServicer):
    def GetObs(self, request, context):
        with _lock:
            obs = _latest_obs
        if obs is None:
            obs = _reset_env()
        return _obs_to_proto(obs, done=_latest_done, reward=_latest_reward)

    def Step(self, request, context):
        obs, r, done, info = _step_env(list(request.values))
        info_dict = {"success": bool(info.get("success", done and r >= 1.0))}
        return env_pb.StepResult(
            obs=_obs_to_proto(obs, done=done, reward=r),
            reward=r,
            done=done,
            info_json=json.dumps(info_dict),
        )

    def Reset(self, request, context):
        obs = _reset_env()
        return _obs_to_proto(obs, done=False, reward=0.0)


def main():
    _build_env()
    port = int(os.environ.get("LIBERO_GRPC_PORT", "50062"))
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    env_pb_grpc.add_EnvDataServiceServicer_to_server(EnvService(), server)
    addr = f"0.0.0.0:{port}"
    server.add_insecure_port(addr)
    server.start()
    print(f"[libero-env] gRPC serving at {addr}  task={_task_description!r}",
          file=sys.stderr, flush=True)
    print(f"[libero-env] ready", file=sys.stderr, flush=True)

    # Catch SIGTERM so we flush the last episode before the eval driver moves on.
    def _on_sig(*_):
        try:
            with _lock:
                _flush_episode_video()
        except Exception:
            pass
        sys.exit(0)
    signal.signal(signal.SIGTERM, _on_sig)
    signal.signal(signal.SIGINT, _on_sig)

    try:
        while True:
            time.sleep(60)
    except KeyboardInterrupt:
        _on_sig()


if __name__ == "__main__":
    main()
