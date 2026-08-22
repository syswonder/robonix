# SPDX-License-Identifier: MulanPSL-2.0
"""World-independent reactive motion policy for Scene visibility sweeps."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any


def _wrapped_angle(value: float) -> float:
    return math.atan2(math.sin(float(value)), math.cos(float(value)))


def _percentile(values: list[float], quantile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    position = max(0.0, min(1.0, float(quantile))) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def sector_clearance_m(
    *,
    ranges: list[float] | tuple[float, ...],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    low: float,
    high: float,
) -> float:
    """Return the closest valid hit in an angular sector.

    ROS ``LaserScan`` uses positive infinity for a ray with no return inside
    the sensor range.  If every selected ray has that value, the sector is
    observed and clear through ``range_max``; it is not missing sensor data.
    Empty sectors, NaNs, and below-minimum-only samples remain unavailable so
    the motion policy continues to fail closed on genuinely invalid scans.
    """

    finite_hits: list[float] = []
    selected_count = 0
    positive_infinity_count = 0
    for index, raw_value in enumerate(ranges):
        angle = float(angle_min) + index * float(angle_increment)
        if not float(low) <= angle <= float(high):
            continue
        selected_count += 1
        value = float(raw_value)
        if math.isfinite(value) and value > float(range_min):
            finite_hits.append(value)
        elif math.isinf(value) and value > 0.0:
            positive_infinity_count += 1
    if finite_hits:
        return min(finite_hits)
    if (
        selected_count > 0
        and positive_infinity_count == selected_count
        and math.isfinite(float(range_max))
        and float(range_max) > float(range_min)
    ):
        return float(range_max)
    return float("nan")


@dataclass
class SynchronizedPoseAgreement:
    """Compare localized and independent reference motion at paired timestamps."""

    max_time_delta_s: float = 0.10
    sample_count: int = 0
    rejected_time_pairs: int = 0
    duplicate_or_regressed_pairs: int = 0
    max_time_delta_observed_s: float = 0.0
    last_localized_stamp_s: float | None = None
    localized_origin: tuple[float, float, float] | None = None
    reference_origin: tuple[float, float, float] | None = None
    translation_errors_m: list[float] = field(default_factory=list)
    yaw_errors_rad: list[float] = field(default_factory=list)

    @staticmethod
    def _relative_xy(
        x_m: float,
        y_m: float,
        origin: tuple[float, float, float],
    ) -> tuple[float, float]:
        delta_x = float(x_m) - origin[0]
        delta_y = float(y_m) - origin[1]
        cosine = math.cos(origin[2])
        sine = math.sin(origin[2])
        return (
            cosine * delta_x + sine * delta_y,
            -sine * delta_x + cosine * delta_y,
        )

    def observe(
        self,
        *,
        localized_stamp_s: float,
        localized_x_m: float,
        localized_y_m: float,
        localized_yaw_rad: float,
        reference_stamp_s: float,
        reference_x_m: float,
        reference_y_m: float,
        reference_yaw_rad: float,
    ) -> bool:
        """Record one synchronized displacement comparison."""

        localized_stamp = float(localized_stamp_s)
        if (
            self.last_localized_stamp_s is not None
            and localized_stamp <= self.last_localized_stamp_s
        ):
            self.duplicate_or_regressed_pairs += 1
            return False
        self.last_localized_stamp_s = localized_stamp
        time_delta_s = abs(localized_stamp - float(reference_stamp_s))
        self.max_time_delta_observed_s = max(
            self.max_time_delta_observed_s,
            time_delta_s,
        )
        if time_delta_s > self.max_time_delta_s:
            self.rejected_time_pairs += 1
            return False
        if self.localized_origin is None or self.reference_origin is None:
            self.localized_origin = (
                float(localized_x_m),
                float(localized_y_m),
                float(localized_yaw_rad),
            )
            self.reference_origin = (
                float(reference_x_m),
                float(reference_y_m),
                float(reference_yaw_rad),
            )
        localized_xy = self._relative_xy(
            localized_x_m,
            localized_y_m,
            self.localized_origin,
        )
        reference_xy = self._relative_xy(
            reference_x_m,
            reference_y_m,
            self.reference_origin,
        )
        localized_yaw = _wrapped_angle(
            float(localized_yaw_rad) - self.localized_origin[2]
        )
        reference_yaw = _wrapped_angle(
            float(reference_yaw_rad) - self.reference_origin[2]
        )
        self.translation_errors_m.append(math.dist(localized_xy, reference_xy))
        self.yaw_errors_rad.append(
            abs(_wrapped_angle(localized_yaw - reference_yaw))
        )
        self.sample_count += 1
        return True

    def as_dict(self) -> dict[str, Any]:
        translation = self.translation_errors_m
        yaw = self.yaw_errors_rad
        return {
            "sample_count": self.sample_count,
            "rejected_time_pairs": self.rejected_time_pairs,
            "duplicate_or_regressed_pairs": self.duplicate_or_regressed_pairs,
            "max_allowed_time_delta_s": round(self.max_time_delta_s, 6),
            "max_time_delta_observed_s": round(
                self.max_time_delta_observed_s,
                6,
            ),
            "translation_error_m": {
                "median": round(_percentile(translation, 0.50), 6),
                "p95": round(_percentile(translation, 0.95), 6),
                "max": round(max(translation, default=0.0), 6),
                "final": round(translation[-1] if translation else 0.0, 6),
            },
            "yaw_error_rad": {
                "median": round(_percentile(yaw, 0.50), 6),
                "p95": round(_percentile(yaw, 0.95), 6),
                "max": round(max(yaw, default=0.0), 6),
                "final": round(yaw[-1] if yaw else 0.0, 6),
            },
        }


def odometry_path_agreement(
    odometry_sources: dict[str, Any],
    *,
    reference_source: str = "ground_truth",
    compared_sources: tuple[str, ...] = ("wheel", "fused"),
    required_sources: tuple[str, ...] | None = None,
    max_relative_error: float = 0.10,
    max_absolute_error_m: float = 0.50,
    min_samples: int = 2,
) -> dict[str, Any]:
    """Check measured path lengths against independent Webots ground truth.

    Continuity gates catch abrupt pose jumps, but a smooth scale error can
    still stretch every Scene object without producing a discontinuity.  This
    independent end-of-run check rejects that acquisition before any semantic
    or geometry score is trusted. ``required_sources`` distinguishes the pose
    source that owns Scene's map-frame geometry from diagnostic sources such as
    raw wheel odometry. Diagnostic drift remains recorded as a warning instead
    of silently disappearing.
    """

    relative_limit = float(max_relative_error)
    absolute_limit = float(max_absolute_error_m)
    required_samples = int(min_samples)
    if relative_limit < 0.0:
        raise ValueError("max_relative_error must not be negative")
    if absolute_limit < 0.0:
        raise ValueError("max_absolute_error_m must not be negative")
    if required_samples < 2:
        raise ValueError("min_samples must be at least two")
    compared = tuple(dict.fromkeys(str(source) for source in compared_sources))
    required = (
        compared
        if required_sources is None
        else tuple(dict.fromkeys(str(source) for source in required_sources))
    )
    missing_required = sorted(set(required) - set(compared))
    if missing_required:
        raise ValueError(
            "required_sources must be included in compared_sources: "
            + ", ".join(missing_required)
        )

    failures: list[str] = []
    warnings: list[str] = []
    reference = odometry_sources.get(reference_source) or {}
    reference_samples = int(reference.get("sample_count") or 0)
    reference_path_m = float(reference.get("path_length_m") or 0.0)
    if reference_samples < required_samples:
        failures.append(
            f"{reference_source} odometry has {reference_samples} samples; "
            f"need at least {required_samples}"
        )
    tolerance_m = max(
        absolute_limit,
        relative_limit * abs(reference_path_m),
    )
    comparisons: dict[str, dict[str, Any]] = {}
    for source in compared:
        track = odometry_sources.get(source) or {}
        samples = int(track.get("sample_count") or 0)
        path_m = float(track.get("path_length_m") or 0.0)
        error_m = abs(path_m - reference_path_m)
        relative_error = (
            error_m / abs(reference_path_m)
            if abs(reference_path_m) > 1e-9
            else 0.0 if error_m <= tolerance_m else float("inf")
        )
        source_valid = samples >= required_samples and error_m <= tolerance_m
        comparisons[source] = {
            "required": source in required,
            "sample_count": samples,
            "path_length_m": round(path_m, 6),
            "absolute_error_m": round(error_m, 6),
            "relative_error": (
                round(relative_error, 6)
                if math.isfinite(relative_error)
                else "inf"
            ),
            "valid": source_valid,
        }
        if samples < required_samples:
            message = (
                f"{source} odometry has {samples} samples; "
                f"need at least {required_samples}"
            )
            (failures if source in required else warnings).append(message)
        elif error_m > tolerance_m:
            message = (
                f"{source} path differs from {reference_source} by "
                f"{error_m:.3f} m ({relative_error:.1%}); "
                f"limit is {tolerance_m:.3f} m"
            )
            (failures if source in required else warnings).append(message)

    return {
        "valid": not failures,
        "reference_source": reference_source,
        "required_sources": list(required),
        "reference_sample_count": reference_samples,
        "reference_path_length_m": round(reference_path_m, 6),
        "max_relative_error": relative_limit,
        "max_absolute_error_m": absolute_limit,
        "tolerance_m": round(tolerance_m, 6),
        "comparisons": comparisons,
        "failures": failures,
        "warnings": warnings,
    }


def publish_stop_if_available(publisher, twist_factory) -> None:
    """Publish one stop command only when this process owns a publisher."""

    if publisher is not None:
        publisher.publish(twist_factory())


@dataclass(frozen=True)
class MotionDecision:
    linear_x_mps: float
    angular_z_rps: float
    mode: str


@dataclass
class OdometryContinuity:
    """Timestamp-aware odometry continuity evidence.

    A slow evaluator may not receive every high-rate odometry message.  A
    fixed distance between two *received* samples is therefore not sufficient
    evidence of a pose reset: the header-stamp interval must also imply an
    impossible translational speed.
    """

    max_step_gate_m: float
    max_speed_gate_mps: float
    sample_count: int = 0
    path_length_m: float = 0.0
    max_step_m: float = 0.0
    max_speed_mps: float = 0.0
    last_stamp_s: float | None = None
    last_receipt_s: float | None = None
    last_xy: tuple[float, float] | None = None
    large_steps: list[dict[str, Any]] = field(default_factory=list)
    discontinuities: list[dict[str, Any]] = field(default_factory=list)

    def observe(
        self,
        *,
        stamp_s: float,
        receipt_s: float,
        x_m: float,
        y_m: float,
    ) -> bool:
        """Record one distinct stamped sample; return true on discontinuity."""

        if self.last_stamp_s is not None and stamp_s == self.last_stamp_s:
            return False
        self.sample_count += 1
        current_xy = (float(x_m), float(y_m))
        if self.last_stamp_s is None or self.last_xy is None:
            self.last_stamp_s = float(stamp_s)
            self.last_receipt_s = float(receipt_s)
            self.last_xy = current_xy
            return False

        stamp_delta_s = float(stamp_s) - self.last_stamp_s
        receipt_delta_s = (
            float(receipt_s) - self.last_receipt_s
            if self.last_receipt_s is not None
            else 0.0
        )
        step_m = math.dist(current_xy, self.last_xy)
        speed_mps = (
            step_m / stamp_delta_s
            if stamp_delta_s > 1e-9
            else math.inf if step_m > 0.0 else 0.0
        )
        self.max_step_m = max(self.max_step_m, step_m)
        if math.isfinite(speed_mps):
            self.max_speed_mps = max(self.max_speed_mps, speed_mps)
        event = {
            "stamp_s": round(float(stamp_s), 6),
            "stamp_delta_s": round(stamp_delta_s, 6),
            "receipt_delta_s": round(receipt_delta_s, 6),
            "step_m": round(step_m, 6),
            "speed_mps": (
                round(speed_mps, 6) if math.isfinite(speed_mps) else "inf"
            ),
            "previous_xy_m": [round(value, 6) for value in self.last_xy],
            "current_xy_m": [round(value, 6) for value in current_xy],
        }
        timestamp_regressed = stamp_delta_s < -1e-9
        implausible_jump = (
            step_m > self.max_step_gate_m
            and speed_mps > self.max_speed_gate_mps
        )
        if step_m > self.max_step_gate_m or timestamp_regressed:
            event["timestamp_regressed"] = timestamp_regressed
            event["implausible_jump"] = implausible_jump
            self.large_steps.append(event)
        discontinuity = timestamp_regressed or implausible_jump
        if discontinuity:
            self.discontinuities.append(event)
        else:
            self.path_length_m += step_m
        self.last_stamp_s = float(stamp_s)
        self.last_receipt_s = float(receipt_s)
        self.last_xy = current_xy
        return discontinuity

    def as_dict(self) -> dict[str, Any]:
        return {
            "sample_count": self.sample_count,
            "path_length_m": round(self.path_length_m, 6),
            "max_step_m": round(self.max_step_m, 6),
            "max_speed_mps": round(self.max_speed_mps, 6),
            "large_steps": self.large_steps,
            "discontinuities": self.discontinuities,
        }


def choose_motion(
    *,
    elapsed_s: float,
    front_m: float,
    left_m: float,
    right_m: float,
    rear_m: float,
    linear_speed_mps: float,
    angular_speed_rps: float,
    stalled_s: float = 0.0,
    stalled_turn_sign: float = 0.0,
    turn_clearance_m: float = 0.50,
) -> MotionDecision:
    """Choose a footprint-aware command without any world-specific waypoint.

    Front/rear clearance constrains longitudinal motion; left/right clearance
    constrains turning.  Treating all four sectors with one radius can produce
    a zero-command deadlock in a safe corridor: both side walls are closer than
    the turning threshold even though the path straight ahead is clear.
    """

    scan_phase = (
        elapsed_s < 15.8
        or 30.0 < elapsed_s < 36.5
        or 56.0 < elapsed_s < 62.5
    )
    if not all(math.isfinite(value) for value in (front_m, left_m, right_m)):
        return MotionDecision(0.0, 0.0, "scan_unavailable_hold")

    rear_observed = math.isfinite(rear_m)
    front_stop_m = 0.55
    rear_stop_m = 0.55
    # TIAGo's Webots footprint has a 0.385 m circumscribed radius.  The
    # default retains 0.115 m of rotational clearance without treating the
    # normal 0.55 m kitchen start clearance as a blocked side.  Keep this
    # independent from the longitudinal stop gates.
    corridor_side_m = 0.30

    if front_m < front_stop_m:
        if rear_observed and rear_m >= 0.65:
            return MotionDecision(-0.06, 0.0, "front_blocked_reverse")
        if max(left_m, right_m) >= turn_clearance_m:
            direction = 1.0 if left_m >= right_m else -1.0
            return MotionDecision(
                0.0,
                direction * 0.24,
                "front_blocked_turn",
            )
        return MotionDecision(0.0, 0.0, "fully_blocked_hold")

    if rear_observed and rear_m < rear_stop_m:
        # Moving forward increases rear clearance and cannot sweep a chassis
        # corner into the obstacle.
        return MotionDecision(0.06, 0.0, "rear_blocked_forward")

    if left_m < turn_clearance_m and right_m < turn_clearance_m:
        if min(left_m, right_m) >= corridor_side_m:
            # The 0.544 m square Webots footprint fits: proceed slowly without
            # steering instead of publishing the previous zero command.
            return MotionDecision(0.05, 0.0, "narrow_corridor_forward")
        if rear_observed and rear_m >= 0.65:
            return MotionDecision(-0.04, 0.0, "corridor_too_narrow_reverse")
        return MotionDecision(0.0, 0.0, "fully_blocked_hold")

    if (
        not scan_phase
        and stalled_s >= 8.0
        and front_m >= 0.70
        and min(left_m, right_m) >= corridor_side_m
    ):
        # Repeated in-place turns can alternate forever around a threshold
        # while translation stays at zero. A bounded slow straight command is
        # safe whenever the footprint has longitudinal and lateral clearance.
        return MotionDecision(0.05, 0.0, "stalled_safe_forward")

    if (
        not scan_phase
        and stalled_s >= 8.0
        and front_m >= front_stop_m
        and max(left_m, right_m) >= turn_clearance_m
        and stalled_turn_sign
    ):
        # Do not choose a new side every frame. Near a threshold that creates
        # an endless left/right oscillation with no change in robot position.
        direction = 1.0 if stalled_turn_sign > 0.0 else -1.0
        return MotionDecision(
            0.0,
            direction * 0.20,
            "stalled_committed_turn",
        )

    if left_m < turn_clearance_m:
        return MotionDecision(0.0, -0.24, "left_blocked_turn_right")
    if right_m < turn_clearance_m:
        return MotionDecision(0.0, 0.24, "right_blocked_turn_left")
    if scan_phase:
        return MotionDecision(0.0, angular_speed_rps, "scan")
    if front_m < 0.70:
        direction = 1.0 if left_m >= right_m else -1.0
        return MotionDecision(
            0.0,
            direction * (angular_speed_rps + 0.08),
            "front_near_turn",
        )
    return MotionDecision(
        linear_speed_mps,
        0.10 if left_m < right_m else -0.06,
        "cruise",
    )
