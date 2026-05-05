# SPDX-License-Identifier: MulanPSL-2.0
"""Regulated Pure Pursuit follower (Macenski et al., 2023).

Implements the same algorithmic ideas as `nav2_regulated_pure_pursuit_controller`
but as a standalone Python module so we don't drag in nav2's lifecycle
machinery. Three regulators in `compute(...)`:

  1. CURVATURE regulator: linear vel scales with 1/(|κ|·k_c). Tight
     turns → slow down, smooth turns → cruise.
  2. PROXIMITY regulator: linear vel scales with min(scan)/d_obs. Walls
     close → slow down. Floor-clamped, never reverses.
  3. TERMINAL regulator: linear vel scales with remaining-distance so
     the robot decelerates into the goal cleanly instead of overshooting.

Heading-only spin-in-place when the lookahead bearing is large
(angle_threshold). Goal-tolerance and angular gains are constructor
inputs.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


def _norm_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


@dataclass
class RPPParams:
    # Speeds. 0.20 m/s felt visibly slow on the office-explore demo —
    # 40 m² took ~25 minutes when the robot was actually moving. 0.40
    # m/s gets the same coverage in under 8 minutes; the regulators
    # below still throttle on curves, near walls, and the terminal
    # phase, so peak speed only happens on straight clear segments.
    desired_linear_vel: float = 0.40
    # 1.2 rad/s ≈ 70°/s pairs with the bumped linear vel above without
    # making the body feel twitchy on align/turn ticks. Was 0.9.
    max_angular_vel: float = 1.2
    min_linear_vel: float = 0.05
    # lookahead
    lookahead_dist: float = 0.6      # base lookahead in metres
    min_lookahead: float = 0.3
    max_lookahead: float = 1.2
    use_velocity_scaled_lookahead: bool = True
    # Regulators. cost_threshold_m=1.2 (was 1.0) gives the proximity
    # regulator more runway to slow down BEFORE the lethal-halo edge,
    # which avoids the "blocked → reverse → re-plan → blocked" oscillation
    # we used to see on chair clusters.
    curvature_threshold: float = 0.5    # rad/m above which we slow
    cost_threshold_m: float = 1.2       # obstacle-proximity slow-down trigger
    cost_min_m: float = 0.4             # below this: collision-imminent
    terminal_decel_m: float = 0.8       # start decelerating when this close
    # angle gain
    rotate_to_heading_angle: float = 0.6  # rad — bigger → spin in place first
    # tolerance for "arrived"
    goal_tolerance_m: float = 0.10


def _select_lookahead(path: List[Tuple[float, float]], pos: Tuple[float, float],
                      L: float) -> Optional[Tuple[float, float]]:
    """Walk the path from current position; return the first point at
    arc-distance ≥ L from `pos`. Falls back to the last point if the
    path ends earlier (we're close to the goal)."""
    if not path:
        return None
    px, py = pos
    L2 = L * L
    # Start search from the path point closest to pos to avoid the
    # "lookahead picks an old segment behind the robot" failure when
    # the path doubles back near the start.
    best_i, best_d = 0, float("inf")
    for i, (x, y) in enumerate(path):
        d = (x - px) ** 2 + (y - py) ** 2
        if d < best_d:
            best_d, best_i = d, i
    for i in range(best_i, len(path)):
        x, y = path[i]
        if (x - px) ** 2 + (y - py) ** 2 >= L2:
            return (x, y)
    return path[-1]


def compute(params: RPPParams,
            path: List[Tuple[float, float]],
            robot_pose: Tuple[float, float, float],   # x, y, yaw
            *, forward_clearance_m: Optional[float] = None,
            current_linear_vel: float = 0.0,
            ) -> Tuple[float, float, str]:
    """Returns (linear, angular, mode). `mode` is a short string for
    logging — useful when debugging which regulator clipped the
    velocity (`turn`, `prox`, `curve`, `term`, `cruise`, `arrived`)."""
    if not path:
        return 0.0, 0.0, "no_path"

    rx, ry, ryaw = robot_pose
    # Distance to final goal — terminal regulator + arrival check.
    gx, gy = path[-1]
    dist_goal = math.hypot(gx - rx, gy - ry)
    if dist_goal <= params.goal_tolerance_m:
        return 0.0, 0.0, "arrived"

    # Velocity-scaled lookahead: longer at speed, shorter when slow
    # near goal so we don't overshoot.
    L = params.lookahead_dist
    if params.use_velocity_scaled_lookahead:
        L = max(params.min_lookahead,
                min(params.max_lookahead,
                    params.min_lookahead + 1.0 * abs(current_linear_vel)))
    # Clamp to remaining distance so the lookahead never sits beyond
    # the goal — that avoids the chronic "circles around goal" artefact.
    L = min(L, max(params.min_lookahead * 0.6, dist_goal))

    target = _select_lookahead(path, (rx, ry), L)
    if target is None:
        return 0.0, 0.0, "no_target"
    tx, ty = target
    dx, dy = tx - rx, ty - ry
    target_yaw = math.atan2(dy, dx)
    yaw_err = _norm_angle(target_yaw - ryaw)

    # ── Heading-first: rotate in place when lookahead is far off-axis ──
    if abs(yaw_err) > params.rotate_to_heading_angle:
        v_ang = max(-params.max_angular_vel,
                    min(params.max_angular_vel, 1.5 * yaw_err))
        return 0.0, v_ang, "turn"

    # ── Pure pursuit curvature ─────────────────────────────────────
    # Standard PP: κ = 2·sin(θ) / L_actual, where θ is the angle to
    # the target measured in robot frame.
    L_actual = math.hypot(dx, dy)
    if L_actual < 1e-3:
        return 0.0, 0.0, "stuck"
    curvature = 2.0 * math.sin(yaw_err) / L_actual

    # ── Regulators on linear velocity ──────────────────────────────
    v = params.desired_linear_vel

    # 1. Curvature regulator
    mode = "cruise"
    if abs(curvature) > params.curvature_threshold:
        v *= params.curvature_threshold / max(abs(curvature), 1e-6)
        mode = "curve"

    # 2. Proximity (cost) regulator — only kicks in if we have a scan.
    if forward_clearance_m is not None:
        c = forward_clearance_m
        if c < params.cost_min_m:
            return 0.0, 0.0, "blocked"
        if c < params.cost_threshold_m:
            ratio = (c - params.cost_min_m) / (params.cost_threshold_m - params.cost_min_m)
            v *= max(0.0, ratio)
            mode = "prox"

    # 3. Terminal regulator: decelerate into goal.
    if dist_goal < params.terminal_decel_m:
        v *= max(0.0, dist_goal / params.terminal_decel_m)
        mode = "term"

    v = max(params.min_linear_vel, v) if dist_goal > params.goal_tolerance_m else 0.0
    v = min(params.desired_linear_vel, v)

    # Angular tracks curvature·v (bicycle model).
    v_ang = max(-params.max_angular_vel,
                min(params.max_angular_vel, curvature * max(v, 0.1)))

    return v, v_ang, mode
