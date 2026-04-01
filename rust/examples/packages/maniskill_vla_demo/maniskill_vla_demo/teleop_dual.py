#!/usr/bin/env python3
"""Dual-window keyboard teleoperation for ManiSkill3 / Fetch.

Opens the SAPIEN 3-D viewer AND a matplotlib sensor-camera panel (RGB + depth).
Robot control keys are read from the **terminal** (stdin in cbreak mode),
so neither GUI window steals keyboard input.

Usage
-----
  python -m maniskill_vla_demo.teleop_dual
  python -m maniskill_vla_demo.teleop_dual PickCube-v1
  SHADER=rt-fast python -m maniskill_vla_demo.teleop_dual

Controls (type in the TERMINAL where the script runs)
-----------------------------------------------------
  EE position : i/k  (+/- X)   j/l  (+/- Y)   u/o  (+/- Z)
  EE rotation : 1/2  (roll)    3/4  (pitch)    5/6  (yaw)
  Gripper     : f=open   g=close
  Base        : w/s (fwd/back)   a/d (rotate)   z/x (torso)
  Reset env   : r
  Quit        : q  or  Ctrl-C

SAPIEN window  = 3D view (WASD / mouse to orbit camera)
matplotlib     = sensor camera feed (view-only)
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt  # noqa: E402


# ── Helpers ──────────────────────────────────────────────────────────────────

def _to_np(t):
    return t.cpu().numpy() if hasattr(t, "cpu") else np.asarray(t)


def _to_bool(t) -> bool:
    if hasattr(t, "item"):
        return bool(t.item())
    return bool(t)


def _get_rgb(obs, cam: str) -> np.ndarray | None:
    if not isinstance(obs, dict):
        return None
    images = obs.get("sensor_data", obs.get("image", {}))
    c = images.get(cam) or (next(iter(images.values())) if images else None)
    if c is None or "rgb" not in c:
        return None
    rgb = _to_np(c["rgb"])
    if rgb.ndim == 4:
        rgb = rgb[0]
    return np.ascontiguousarray(rgb[..., :3]).astype(np.uint8)


def _get_depth(obs, cam: str) -> np.ndarray | None:
    if not isinstance(obs, dict):
        return None
    images = obs.get("sensor_data", obs.get("image", {}))
    c = images.get(cam) or (next(iter(images.values())) if images else None)
    if c is None or "depth" not in c:
        return None
    d = _to_np(c["depth"])
    if d.ndim == 4:
        d = d[0]
    if d.ndim == 3:
        d = d[..., 0]
    return d.astype(np.float32)


def _list_cameras(obs) -> list[str]:
    if not isinstance(obs, dict):
        return []
    return list(obs.get("sensor_data", obs.get("image", {})).keys())


# ── Terminal keyboard (stdin cbreak) ─────────────────────────────────────────

class JsonKeyboard:
    """Track pressed/released keys via pynput — no repeat-delay lag."""

    def __init__(self):
        from pynput import keyboard as _kb
        self._pressed: set[str] = set()
        self._lock = threading.Lock()
        self._listener = _kb.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.daemon = True
        self._listener.start()

    @staticmethod
    def _key_char(key) -> str | None:
        if hasattr(key, "char") and key.char:
            return key.char
        return None

    def _on_press(self, key):
        ch = self._key_char(key)
        if ch:
            with self._lock:
                self._pressed.add(ch)

    def _on_release(self, key):
        ch = self._key_char(key)
        if ch:
            with self._lock:
                self._pressed.discard(ch)

    def read_chars(self) -> set[str]:
        with self._lock:
            return set(self._pressed)

    def restore(self):
        self._listener.stop()


# ── Action from keys ────────────────────────────────────────────────────────

_EE_POS   = 0.15
_EE_ROT   = 0.20
_BASE_LIN = 0.30
_BASE_ROT = 0.40
_TORSO    = 0.10

# Fetch pd_ee_delta_pose actual layout (13 DOF):
#   arm(6) → gripper(2: L+R finger) → body(3: head_pan, head_tilt, torso) → base(2: fwd, yaw)
_IDX_GRIP_L   = 6    # l_gripper_finger_joint
_IDX_GRIP_R   = 7    # r_gripper_finger_joint (mimic, but still occupies a dim)
_IDX_TORSO    = 10   # torso_lift_joint (body[2])
_IDX_BASE_FWD = 11   # base forward velocity
_IDX_BASE_YAW = 12   # base yaw velocity


def _build_key_map(n_dof: int) -> dict[str, tuple[int, float]]:
    return {
        # EE position
        "i": (0,  _EE_POS),  "k": (0, -_EE_POS),
        "j": (1,  _EE_POS),  "l": (1, -_EE_POS),
        "u": (2,  _EE_POS),  "o": (2, -_EE_POS),
        # EE rotation
        "1": (3,  _EE_ROT),  "2": (3, -_EE_ROT),
        "3": (4,  _EE_ROT),  "4": (4, -_EE_ROT),
        "5": (5,  _EE_ROT),  "6": (5, -_EE_ROT),
        # Base
        "w": (_IDX_BASE_FWD,  _BASE_LIN),
        "s": (_IDX_BASE_FWD, -_BASE_LIN),
        "a": (_IDX_BASE_YAW,  _BASE_ROT),
        "d": (_IDX_BASE_YAW, -_BASE_ROT),
        # Torso
        "z": (_IDX_TORSO,  _TORSO),
        "x": (_IDX_TORSO, -_TORSO),
    }


def _build_action(keys: set[str], n_dof: int, key_map: dict) -> np.ndarray:
    a = np.zeros(n_dof, dtype=np.float32)
    for ch in keys:
        m = key_map.get(ch)
        if m and m[0] < n_dof:
            a[m[0]] += m[1]
        # Gripper: set BOTH fingers symmetrically
        if ch == "f":
            if _IDX_GRIP_L < n_dof: a[_IDX_GRIP_L] = 1.0
            if _IDX_GRIP_R < n_dof: a[_IDX_GRIP_R] = 1.0
        if ch == "g":
            if _IDX_GRIP_L < n_dof: a[_IDX_GRIP_L] = -1.0
            if _IDX_GRIP_R < n_dof: a[_IDX_GRIP_R] = -1.0
    return a


# ── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("env_id", nargs="?",
                        default=os.environ.get("MANISKILL_ENV_ID",
                                               "ReplicaCADTidyHouseTrain_SceneManipulation-v1"))
    parser.add_argument("--control", default=os.environ.get("MANISKILL_CONTROL_MODE",
                                                             "pd_ee_delta_pose"))
    parser.add_argument("--shader",  default=os.environ.get("SHADER", "default"))
    parser.add_argument("--cam-w",   type=int, default=int(os.environ.get("CAM_W", "640")))
    parser.add_argument("--cam-h",   type=int, default=int(os.environ.get("CAM_H", "480")))
    parser.add_argument("--cam",     default="fetch_head")
    parser.add_argument("--display-every", type=int, default=3)
    parser.add_argument("--record-dir", default="")
    args = parser.parse_args()

    import gymnasium as gym
    import mani_skill.envs  # noqa: F401

    print(f"\n{'═'*58}")
    print(f"  Dual-window teleop   env={args.env_id}")
    print(f"  shader={args.shader}  cam={args.cam_w}x{args.cam_h}  ctrl={args.control}")
    print(f"{'═'*58}")
    print("  ┌── Keys: type HERE in the terminal! ──────────────┐")
    print("  │ EE pos : i/k  j/l  u/o      Gripper : f/g        │")
    print("  │ EE rot : 1-6                Reset   : r          │")
    print("  │ Base   : w/s  a/d           Torso   : z/x        │")
    print("  │ Quit   : q                                       │")
    print("  └──────────────────────────────────────────────────┘")
    print("  SAPIEN = 3D view (WASD/mouse = camera orbit)")
    print("  matplotlib = sensor feed (view-only)")
    print(f"{'═'*58}\n")

    # ── Create environment ────────────────────────────────────────────────────
    env = gym.make(
        args.env_id,
        robot_uids="fetch",
        obs_mode="rgbd",
        render_mode="human",
        sensor_configs={
            "shader_pack": args.shader,
            "width":  args.cam_w,
            "height": args.cam_h,
        },
    )
    obs, _ = env.reset(seed=42)
    cams = _list_cameras(obs)
    cam_name = args.cam if args.cam in cams else (cams[0] if cams else "")
    print(f"[teleop] cameras: {cams}  using: {cam_name!r}")

    n_dof = env.action_space.shape[-1]
    print(f"[teleop] action_space: dof={n_dof}")

    # Print controller breakdown so we can verify action indices
    try:
        ctrl = env.unwrapped.agent.controller
        controllers = getattr(ctrl, "controllers", {})
        idx = 0
        for name, sub in controllers.items():
            dim = sub.action_space.shape[-1] if hasattr(sub, "action_space") else "?"
            jnames = getattr(sub, "joint_names", [])
            print(f"[teleop]   [{idx}:{idx+dim if isinstance(dim,int) else '?'}] "
                  f"{name} (dim={dim}) joints={jnames}")
            if isinstance(dim, int):
                idx += dim
    except Exception as exc:
        print(f"[teleop] (could not introspect controllers: {exc})")

    key_map = _build_key_map(n_dof)
    print(f"[teleop] key map: gripper=[{_IDX_GRIP_L},{_IDX_GRIP_R}] torso=[{_IDX_TORSO}] "
          f"base_fwd=[{_IDX_BASE_FWD}] base_yaw=[{_IDX_BASE_YAW}]")

    env.render()

    # ── matplotlib sensor panel ───────────────────────────────────────────────
    plt.ion()
    fig, (ax_rgb, ax_dep) = plt.subplots(1, 2, figsize=(11, 4.5),
                                          num="Sensor Camera — ManiSkill3")
    fig.patch.set_facecolor("#1e1e2e")
    for ax in (ax_rgb, ax_dep):
        ax.set_xticks([]); ax.set_yticks([])
        ax.title.set_color("white")

    rgb0 = np.zeros((args.cam_h, args.cam_w, 3), dtype=np.uint8)
    dep0 = np.zeros((args.cam_h, args.cam_w), dtype=np.float32)
    im_rgb = ax_rgb.imshow(rgb0)
    im_dep = ax_dep.imshow(dep0, cmap="plasma")
    ax_rgb.set_title("RGB", color="white")
    ax_dep.set_title("Depth (m)", color="white")
    cbar = fig.colorbar(im_dep, ax=ax_dep, fraction=0.046, pad=0.04)
    cbar.ax.yaxis.set_tick_params(color="white")
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color="white")
    fig.tight_layout()
    fig.canvas.draw()
    plt.pause(0.001)

    # ── Terminal keyboard ─────────────────────────────────────────────────────
    term_kb = JsonKeyboard()

    # ── Recording ─────────────────────────────────────────────────────────────
    record_dir = Path(args.record_dir) if args.record_dir else None
    if record_dir:
        record_dir.mkdir(parents=True, exist_ok=True)
    episode_steps: list[dict] = []
    prev_obs = obs
    step = 0
    episode = 0
    ep_boundary_logged = False

    print("[teleop] ready — type keys in this terminal\n")

    try:
        while True:
            keys = term_kb.read_chars()

            if "q" in keys:
                break

            if "r" in keys:
                if record_dir and episode_steps:
                    _save_episode(episode_steps, record_dir, episode)
                episode_steps.clear()
                obs, _ = env.reset()
                prev_obs = obs
                step = 0
                episode += 1
                ep_boundary_logged = False
                env.render()
                print(f"[teleop] reset → episode {episode}")
                continue

            action = _build_action(keys, n_dof, key_map)

            obs, reward, terminated, truncated, info = env.step(action[None])

            if record_dir:
                r_val = float(reward) if not hasattr(reward, "item") else reward.item()
                episode_steps.append({
                    "obs":    prev_obs,
                    "action": action,
                    "reward": r_val,
                })
            prev_obs = obs

            env.render()

            if step % args.display_every == 0:
                rgb = _get_rgb(obs, cam_name)
                dep = _get_depth(obs, cam_name)
                if rgb is not None:
                    im_rgb.set_data(rgb)
                    r_val = float(reward) if not hasattr(reward, "item") else reward.item()
                    ax_rgb.set_title(f"RGB  ep={episode} step={step}  r={r_val:.3f}",
                                     color="white")
                if dep is not None:
                    finite = dep[np.isfinite(dep)]
                    if finite.size > 0:
                        dmin = max(0.0, float(finite.min()))
                        dmax = float(np.percentile(finite, 98))
                        if dmax <= dmin:
                            dmax = dmin + 1.0
                        im_dep.set_data(np.clip(dep, dmin, dmax))
                        im_dep.set_clim(dmin, dmax)
                    else:
                        im_dep.set_data(dep)
                fig.canvas.draw_idle()
                fig.canvas.flush_events()

            if (_to_bool(terminated) or _to_bool(truncated)) and not ep_boundary_logged:
                print(f"[teleop] episode boundary at step {step} — press 'r' to reset")
                ep_boundary_logged = True

            step += 1

    except KeyboardInterrupt:
        pass
    finally:
        term_kb.restore()
        if record_dir and episode_steps:
            _save_episode(episode_steps, record_dir, episode)
        env.close()
        plt.close("all")
        print("\n[teleop] bye")


def _save_episode(steps: list[dict], out_dir: Path, idx: int) -> None:
    import pickle
    path = out_dir / f"teleop_{idx:04d}.pkl"
    with open(path, "wb") as f:
        pickle.dump(steps, f)
    print(f"[teleop] saved {len(steps)} steps → {path}")


if __name__ == "__main__":
    main()
