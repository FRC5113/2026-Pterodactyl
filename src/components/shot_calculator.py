"""
Newton-method shoot-on-the-move (SOTM) fire control with drag compensation.

Translated from ShotCalculator.java (FRC Team 5962 perSEVERE, MIT License).

The core idea: if you're moving, you can't just aim at the target because the
ball inherits your velocity. We use Newton's method to find the self-consistent
time-of-flight where the projected aim point and the LUT-predicted TOF agree.
Usually converges in 2-3 iterations.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple

from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Twist2d
from wpimath.kinematics import ChassisSpeeds


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _is_finite(value: float) -> bool:
    return not math.isnan(value) and not math.isinf(value)


def _angle_modulus(angle_rad: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _interpolate(a: float, b: float, t: float) -> float:
    """Linear interpolation between a and b by fraction t."""
    return a + (b - a) * t


class _InterpolatingMap:
    """Sorted-key linear interpolation table, equivalent to WPILib's
    InterpolatingDoubleTreeMap."""

    __slots__ = ("_keys", "_values")

    def __init__(self) -> None:
        self._keys: list[float] = []
        self._values: list[float] = []

    def put(self, key: float, value: float) -> None:
        import bisect

        idx = bisect.bisect_left(self._keys, key)
        if idx < len(self._keys) and self._keys[idx] == key:
            self._values[idx] = value
        else:
            self._keys.insert(idx, key)
            self._values.insert(idx, value)

    def get(self, key: float) -> float:
        keys = self._keys
        vals = self._values
        n = len(keys)
        if n == 0:
            return 0.0
        if n == 1 or key <= keys[0]:
            return vals[0]
        if key >= keys[-1]:
            return vals[-1]
        import bisect

        hi = bisect.bisect_left(keys, key)
        if hi == 0:
            return vals[0]
        lo = hi - 1
        t = (key - keys[lo]) / (keys[hi] - keys[lo])
        return vals[lo] + t * (vals[hi] - vals[lo])

    def clear(self) -> None:
        self._keys.clear()
        self._values.clear()

    def __len__(self) -> int:
        return len(self._keys)


"""Data classes"""


@dataclass
class ShotConfig:
    """Tuning parameters.  Set these to match your robot."""

    # Launcher geometry (metres, measured from CAD)
    launcher_offset_x: float = 0.20  # forward of robot centre
    launcher_offset_y: float = 0.0  # left of robot centre

    # Scoring range (metres)
    min_scoring_distance: float = 0.5
    max_scoring_distance: float = 5.0

    # Newton solver tuning
    max_iterations: int = 25
    convergence_tolerance: float = 0.001  # seconds
    tof_min: float = 0.05
    tof_max: float = 5.0

    # Below this speed (m/s) skip SOTM and aim straight
    min_sotm_speed: float = 0.1
    # Above this speed (m/s) suppress firing (outside calibration range)
    max_sotm_speed: float = 3.0

    # Latency compensation (milliseconds)
    phase_delay_ms: float = 30.0  # vision pipeline lag
    mech_latency_ms: float = 20.0  # mechanism response time

    # Drag coefficient for inherited-velocity decay.  Set 0 to disable.
    sotm_drag_coeff: float = 0.47

    # Confidence scoring weights (weighted geometric mean)
    w_convergence: float = 1.0
    w_velocity_stability: float = 0.8
    w_vision_confidence: float = 1.2
    w_heading_accuracy: float = 1.5
    w_distance_in_range: float = 0.5
    heading_max_error_rad: float = math.radians(15)

    # Heading tolerance tightens with speed: base / (1 + scalar * speed)
    heading_speed_scalar: float = 1.0
    # Heading tolerance scales with distance (reference point)
    # Farther = tighter because the same angle error produces a larger miss at long range.
    heading_reference_distance: float = 2.5  # metres

    # Suppress firing when pitch or roll exceeds this (degrees)
    max_tilt_deg: float = 5.0


class LaunchParameters(NamedTuple):
    """Result of :meth:`ShotCalculator.calculate`."""

    rpm: float
    time_of_flight_sec: float
    drive_angle: Rotation2d
    drive_angular_velocity_rad_per_sec: float
    is_valid: bool
    confidence: float
    solved_distance_m: float
    iterations_used: int
    warm_start_used: bool


INVALID = LaunchParameters(
    rpm=0.0,
    time_of_flight_sec=0.0,
    drive_angle=Rotation2d(),
    drive_angular_velocity_rad_per_sec=0.0,
    is_valid=False,
    confidence=0.0,
    solved_distance_m=0.0,
    iterations_used=0,
    warm_start_used=False,
)


@dataclass
class ShotInputs:
    """All state the solver needs from your robot each cycle."""

    robot_pose: Pose2d
    field_velocity: ChassisSpeeds
    robot_velocity: ChassisSpeeds
    hub_center: Translation2d
    hub_forward: Translation2d
    vision_confidence: float = 1.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0


"""Solver"""

_DERIV_H = 0.01  # 1 cm finite-difference step


class ShotCalculator:
    """Shoot-on-the-move fire control solver.

    Call :meth:`calculate` once per robot cycle to get the current firing
    solution.  Fill the lookup tables first with :meth:`load_lut_entry`.
    """

    def __init__(self, config: ShotConfig | None = None) -> None:
        self.config = config or ShotConfig()

        self._rpm_map = _InterpolatingMap()
        self._tof_map = _InterpolatingMap()
        self._correction_rpm_map = _InterpolatingMap()
        self._correction_tof_map = _InterpolatingMap()

        self._rpm_offset: float = 0.0

        # Solver warm-start state
        self._previous_tof: float = -1.0
        self._previous_speed: float = 0.0

        # Previous-cycle velocities for acceleration estimation
        self._prev_robot_vx: float = 0.0
        self._prev_robot_vy: float = 0.0
        self._prev_robot_omega: float = 0.0

    """LUT management"""

    def load_lut_entry(self, distance_m: float, rpm: float, tof: float) -> None:
        """Add a distance / RPM / TOF point to the lookup table."""
        self._rpm_map.put(distance_m, rpm)
        self._tof_map.put(distance_m, tof)

    def effective_rpm(self, distance: float) -> float:
        base = self._rpm_map.get(distance)
        correction = (
            self._correction_rpm_map.get(distance)
            if len(self._correction_rpm_map)
            else 0.0
        )
        return base + correction + self._rpm_offset

    def effective_tof(self, distance: float) -> float:
        base = self._tof_map.get(distance)
        correction = (
            self._correction_tof_map.get(distance)
            if len(self._correction_tof_map)
            else 0.0
        )
        return base + correction

    """Drag"""

    def _drag_compensated_tof(self, tof: float) -> float:
        c = self.config.sotm_drag_coeff
        if c < 1e-6:
            return tof
        return (1.0 - math.exp(-c * tof)) / c

    """TOF derivative (central finite difference)"""

    def _tof_map_derivative(self, d: float) -> float:
        t_high = self.effective_tof(d + _DERIV_H)
        t_low = self.effective_tof(d - _DERIV_H)
        return (t_high - t_low) / (2.0 * _DERIV_H)

    """Main solver"""

    def get_distance_from_hub(
        self,
        inputs: ShotInputs,
        *,
        use_launcher_offset: bool = True,
        latency_compensated: bool = True,
    ) -> float:
        """Return planar distance to hub independent of shot validity gates.

        This can be used for telemetry/UI even when :meth:`calculate` would
        return an invalid shot (tilt, behind hub, speed limits, etc.).
        Returns 0.0 only when pose/hub inputs are missing or non-finite.
        """
        if inputs is None or inputs.robot_pose is None or inputs.hub_center is None:
            return 0.0

        pose = inputs.robot_pose
        pose_x = pose.x
        pose_y = pose.y
        if not _is_finite(pose_x) or not _is_finite(pose_y):
            return 0.0

        cfg = self.config

        if latency_compensated:
            robot_vel = inputs.robot_velocity
            dt = cfg.phase_delay_ms / 1000.0
            ax = (robot_vel.vx - self._prev_robot_vx) / 0.02
            ay = (robot_vel.vy - self._prev_robot_vy) / 0.02
            a_omega = (robot_vel.omega - self._prev_robot_omega) / 0.02
            pose = pose.exp(
                Twist2d(
                    robot_vel.vx * dt + 0.5 * ax * dt * dt,
                    robot_vel.vy * dt + 0.5 * ay * dt * dt,
                    robot_vel.omega * dt + 0.5 * a_omega * dt * dt,
                )
            )

        origin_x = pose.x
        origin_y = pose.y

        if use_launcher_offset:
            heading = pose.rotation().radians()
            cos_h = math.cos(heading)
            sin_h = math.sin(heading)
            origin_x += cfg.launcher_offset_x * cos_h - cfg.launcher_offset_y * sin_h
            origin_y += cfg.launcher_offset_x * sin_h + cfg.launcher_offset_y * cos_h

        hub_x = inputs.hub_center.x
        hub_y = inputs.hub_center.y
        if not _is_finite(hub_x) or not _is_finite(hub_y):
            return 0.0

        dx = hub_x - origin_x
        dy = hub_y - origin_y
        return math.hypot(dx, dy)

    def calculate(self, inputs: ShotInputs) -> LaunchParameters:
        """Solve for the firing solution.  Call once per cycle."""
        if inputs is None or inputs.robot_pose is None:
            return INVALID

        cfg = self.config
        raw_pose = inputs.robot_pose
        field_vel = inputs.field_velocity
        robot_vel = inputs.robot_velocity

        if (
            inputs.hub_center is None
            or inputs.hub_forward is None
            or field_vel is None
            or robot_vel is None
        ):
            return INVALID

        pose_x = raw_pose.x
        pose_y = raw_pose.y
        if not _is_finite(pose_x) or not _is_finite(pose_y):
            return INVALID

        # Second-order pose prediction:  v*dt + 0.5*a*dt^2
        dt = cfg.phase_delay_ms / 1000.0
        ax = (robot_vel.vx - self._prev_robot_vx) / 0.02
        ay = (robot_vel.vy - self._prev_robot_vy) / 0.02
        a_omega = (robot_vel.omega - self._prev_robot_omega) / 0.02

        compensated_pose = raw_pose.exp(
            Twist2d(
                robot_vel.vx * dt + 0.5 * ax * dt * dt,
                robot_vel.vy * dt + 0.5 * ay * dt * dt,
                robot_vel.omega * dt + 0.5 * a_omega * dt * dt,
            )
        )
        self._prev_robot_vx = robot_vel.vx
        self._prev_robot_vy = robot_vel.vy
        self._prev_robot_omega = robot_vel.omega

        robot_x = compensated_pose.x
        robot_y = compensated_pose.y
        heading = compensated_pose.rotation().radians()

        hub_x = inputs.hub_center.x
        hub_y = inputs.hub_center.y
        if not _is_finite(hub_x) or not _is_finite(hub_y):
            return INVALID

        # Behind-hub detection
        hf = inputs.hub_forward
        if not _is_finite(hf.x) or not _is_finite(hf.y):
            return INVALID
        dot = (hub_x - robot_x) * hf.x + (hub_y - robot_y) * hf.y
        if dot < 0:
            return INVALID

        # Tilt gate
        if (
            abs(inputs.pitch_deg) > cfg.max_tilt_deg
            or abs(inputs.roll_deg) > cfg.max_tilt_deg
        ):
            return INVALID

        # Transform robot centre > launcher position
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        launcher_x = (
            robot_x + cfg.launcher_offset_x * cos_h - cfg.launcher_offset_y * sin_h
        )
        launcher_y = (
            robot_y + cfg.launcher_offset_x * sin_h + cfg.launcher_offset_y * cos_h
        )

        # Launcher velocity includes rotational component
        launcher_field_off_x = (
            cfg.launcher_offset_x * cos_h - cfg.launcher_offset_y * sin_h
        )
        launcher_field_off_y = (
            cfg.launcher_offset_x * sin_h + cfg.launcher_offset_y * cos_h
        )
        omega = field_vel.omega
        vx = field_vel.vx + (-launcher_field_off_y) * omega
        vy = field_vel.vy + launcher_field_off_x * omega

        # Displacement launcher > hub
        rx = hub_x - launcher_x
        ry = hub_y - launcher_y
        distance = math.hypot(rx, ry)

        if distance < cfg.min_scoring_distance or distance > cfg.max_scoring_distance:
            return INVALID

        robot_speed = math.hypot(vx, vy)

        if robot_speed > cfg.max_sotm_speed:
            return INVALID

        velocity_filtered = robot_speed < cfg.min_sotm_speed

        """Solve TOF"""
        if velocity_filtered:
            solved_tof = self.effective_tof(distance)
            proj_dist = distance
            iterations_used = 0
            warm_start_used = False
        else:
            max_iter = cfg.max_iterations
            conv_tol = cfg.convergence_tolerance

            # Warm start from previous cycle
            if self._previous_tof > 0:
                tof = self._previous_tof
                warm_start_used = True
            else:
                tof = self.effective_tof(distance)
                warm_start_used = False

            proj_dist = distance
            iterations_used = 0

            _hypot = math.hypot
            _abs = abs

            for i in range(max_iter):
                prev_tof = tof

                c = self.config.sotm_drag_coeff
                drag_exp = 1.0 if c < 1e-6 else math.exp(-c * tof)
                drift_tof = tof if c < 1e-6 else (1.0 - drag_exp) / c

                prx = rx - vx * drift_tof
                pry = ry - vy * drift_tof
                proj_dist = _hypot(prx, pry)

                # Degenerate guard
                if proj_dist < 0.01:
                    tof = self.effective_tof(distance)
                    iterations_used = max_iter + 1
                    break

                lookup_tof = self.effective_tof(proj_dist)

                # Derivative for Newton step (chain rule: d/dt of dragCompensatedTOF = e^(-ct))
                d_prime = -drag_exp * (prx * vx + pry * vy) / proj_dist
                g_prime = self._tof_map_derivative(proj_dist)
                f = lookup_tof - tof
                f_prime = g_prime * d_prime - 1.0

                if (
                    not _is_finite(lookup_tof)
                    or not _is_finite(g_prime)
                    or not _is_finite(f)
                    or not _is_finite(f_prime)
                ):
                    tof = self.effective_tof(distance)
                    iterations_used = max_iter + 1
                    break

                # Newton step with near-zero denominator guard
                if _abs(f_prime) > 0.01:
                    tof = tof - f / f_prime
                else:
                    tof = lookup_tof  # fixed-point fallback

                tof = _clamp(tof, cfg.tof_min, cfg.tof_max)
                iterations_used = i + 1

                if _abs(tof - prev_tof) < conv_tol:
                    break

            # Divergence guard
            if tof > cfg.tof_max or tof < 0.0 or not _is_finite(tof):
                tof = self.effective_tof(distance)
                iterations_used = max_iter + 1

            solved_tof = tof

        # Save for warm start
        self._previous_tof = solved_tof

        effective_tof_val = _clamp(
            solved_tof + cfg.mech_latency_ms / 1000.0, cfg.tof_min, cfg.tof_max
        )

        # RPM from LUT at solved distance
        effective_rpm_val = self.effective_rpm(proj_dist)

        # Drive angle: aim at velocity-compensated target
        if velocity_filtered:
            comp_target_x = hub_x
            comp_target_y = hub_y
        else:
            heading_drift_tof = self._drag_compensated_tof(solved_tof)
            comp_target_x = hub_x - vx * heading_drift_tof
            comp_target_y = hub_y - vy * heading_drift_tof

        aim_x = comp_target_x - robot_x
        aim_y = comp_target_y - robot_y
        if abs(aim_x) < 1e-9 and abs(aim_y) < 1e-9:
            drive_angle = compensated_pose.rotation()
        else:
            drive_angle = Rotation2d(aim_x, aim_y)

        heading_error_rad = _angle_modulus(drive_angle.radians() - heading)

        # Angular velocity feedforward
        drive_angular_velocity = 0.0
        if not velocity_filtered and distance > 0.1:
            tangential_vel = (ry * vx - rx * vy) / distance
            drive_angular_velocity = tangential_vel / distance

        # Solver convergence quality
        if velocity_filtered:
            solver_quality = 1.0
        else:
            max_iter = cfg.max_iterations
            if iterations_used > max_iter:
                solver_quality = 0.0
            elif iterations_used <= 3:
                solver_quality = 1.0
            else:
                solver_quality = _interpolate(
                    1.0, 0.1, (iterations_used - 3) / (max_iter - 3)
                )

        confidence = self._compute_confidence(
            solver_quality,
            robot_speed,
            heading_error_rad,
            distance,
            inputs.vision_confidence,
        )
        self._previous_speed = robot_speed

        return LaunchParameters(
            rpm=effective_rpm_val,
            time_of_flight_sec=effective_tof_val,
            drive_angle=drive_angle,
            drive_angular_velocity_rad_per_sec=drive_angular_velocity,
            is_valid=True,
            confidence=confidence,
            solved_distance_m=distance,
            iterations_used=iterations_used,
            warm_start_used=warm_start_used,
        )

    """Confidence"""

    def _compute_confidence(
        self,
        solver_quality: float,
        current_speed: float,
        heading_error_rad: float,
        distance: float,
        vision_confidence: float,
    ) -> float:
        """0–100 weighted geometric mean of 5 factors.  One zero kills it."""
        cfg = self.config

        convergence_quality = solver_quality

        if current_speed < cfg.min_sotm_speed:
            velocity_stability = 1.0
        else:
            speed_delta = abs(current_speed - self._previous_speed)
            velocity_stability = _clamp(1.0 - speed_delta / 0.5, 0, 1)

        vision_conf = _clamp(vision_confidence, 0, 1)

        safe_distance = max(distance, 1e-6)
        distance_scale = _clamp(
            cfg.heading_reference_distance / safe_distance, 0.5, 2.0
        )
        speed_scale = 1.0 / (1.0 + cfg.heading_speed_scalar * current_speed)
        scaled_max_error = max(
            cfg.heading_max_error_rad * distance_scale * speed_scale,
            1e-6,
        )
        heading_err = abs(heading_error_rad)
        heading_accuracy = _clamp(1.0 - heading_err / scaled_max_error, 0, 1)

        range_span = cfg.max_scoring_distance - cfg.min_scoring_distance
        if range_span <= 1e-9:
            dist_in_range = 1.0
        else:
            range_fraction = (distance - cfg.min_scoring_distance) / range_span
            dist_in_range = _clamp(1.0 - 2.0 * abs(range_fraction - 0.5), 0, 1)

        components = (
            convergence_quality,
            velocity_stability,
            vision_conf,
            heading_accuracy,
            dist_in_range,
        )
        weights = (
            cfg.w_convergence,
            cfg.w_velocity_stability,
            cfg.w_vision_confidence,
            cfg.w_heading_accuracy,
            cfg.w_distance_in_range,
        )

        sum_w = 0.0
        log_sum = 0.0
        for c, w in zip(components, weights):
            if c <= 0:
                return 0.0
            log_sum += w * math.log(c)
            sum_w += w

        if sum_w <= 0:
            return 0.0

        composite = math.exp(log_sum / sum_w) * 100.0
        return _clamp(composite, 0, 100)

    """Corrections & offset"""

    def add_rpm_correction(self, distance: float, delta_rpm: float) -> None:
        self._correction_rpm_map.put(distance, delta_rpm)

    def add_tof_correction(self, distance: float, delta_tof: float) -> None:
        self._correction_tof_map.put(distance, delta_tof)

    def clear_corrections(self) -> None:
        self._correction_rpm_map.clear()
        self._correction_tof_map.clear()

    def adjust_offset(self, delta: float) -> None:
        """Bump RPM offset (bind to copilot D-pad).  Clamped ±200."""
        self._rpm_offset = _clamp(self._rpm_offset + delta, -200, 200)

    def reset_offset(self) -> None:
        self._rpm_offset = 0.0

    @property
    def offset(self) -> float:
        return self._rpm_offset

    def get_time_of_flight(self, distance_m: float) -> float:
        return self.effective_tof(distance_m)

    def get_base_rpm(self, distance: float) -> float:
        return self._rpm_map.get(distance)

    def reset_warm_start(self) -> None:
        """Call after a pose reset so the solver doesn't use stale data."""
        self._previous_tof = -1.0
        self._previous_speed = 0.0
        self._prev_robot_vx = 0.0
        self._prev_robot_vy = 0.0
        self._prev_robot_omega = 0.0
