"""
RK4 projectile physics with drag and Magnus lift.

Translated from ProjectileSimulator.java (FRC Team 5962 perSEVERE, MIT License).

Simulates a ball flying through the air with drag and Magnus lift. Uses RK4
integration in the vertical plane (x, z). For each distance, binary-searches
RPM until the ball arrives at the target height. Then generates a lookup table
from 0.50 m to 5.00 m in 5 cm steps (91 entries).

Usage::

    params = SimParameters(
        ball_mass_kg=0.215,
        ball_diameter_m=0.1501,
        drag_coeff=0.47,
        magnus_coeff=0.2,
        air_density=1.225,
        exit_height_m=0.43,
        wheel_diameter_m=0.1016,
        target_height_m=1.83,
        slip_factor=0.6,
        fixed_launch_angle_deg=45.0,
        dt=0.001,
        rpm_min=1500,
        rpm_max=6000,
        binary_search_iters=25,
        max_sim_time=5.0,
    )
    sim = ProjectileSimulator(params)
    lut = sim.generate_lut()
    for entry in lut.entries:
        if entry.reachable:
            print(f"{entry.distance_m:.2f}m -> {entry.rpm:.0f} RPM, {entry.tof:.3f}s TOF")
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import NamedTuple


@dataclass
class SimParameters:
    """Physical measurements from CAD and the game manual."""

    ball_mass_kg: float
    ball_diameter_m: float
    drag_coeff: float
    magnus_coeff: float
    air_density: float
    exit_height_m: float
    wheel_diameter_m: float
    target_height_m: float
    slip_factor: float
    fixed_launch_angle_deg: float
    dt: float
    rpm_min: float
    rpm_max: float
    binary_search_iters: int
    max_sim_time: float


class TrajectoryResult(NamedTuple):
    z_at_target: float
    tof: float
    reached_target: bool
    max_height: float
    apex_x: float


class LUTEntry(NamedTuple):
    """One row: distance -> RPM that lands it, TOF, reachable flag."""

    distance_m: float
    rpm: float
    tof: float
    reachable: bool


class GeneratedLUT(NamedTuple):
    """Full LUT with generation stats."""

    entries: list[LUTEntry]
    params: SimParameters
    reachable_count: int
    unreachable_count: int
    max_range_m: float
    generation_time_ms: float


class ProjectileSimulator:
    """RK4 projectile simulator with drag and Magnus lift.

    Plug in your robot's measurements from CAD, call :meth:`generate_lut`,
    and get a complete shooter table.
    """

    def __init__(self, params: SimParameters) -> None:
        self.params = params

        radius = params.ball_diameter_m / 2.0
        area = math.pi * radius * radius
        two_mass = 2.0 * params.ball_mass_kg

        self._k_drag = (params.air_density * params.drag_coeff * area) / two_mass
        self._k_magnus = (params.air_density * params.magnus_coeff * area) / two_mass

    """Conversions"""

    def exit_velocity(self, rpm: float) -> float:
        """RPM to ball exit speed (m/s).  Accounts for slip."""
        return (
            self.params.slip_factor
            * rpm
            * math.pi
            * self.params.wheel_diameter_m
            / 60.0
        )

    """Simulation"""

    def simulate(self, rpm: float, target_distance_m: float) -> TrajectoryResult:
        """Launch at *rpm* and see where the ball is at *target_distance_m*."""
        p = self.params
        v0 = self.exit_velocity(rpm)
        launch_rad = math.radians(p.fixed_launch_angle_deg)
        vx = v0 * math.cos(launch_rad)
        vz = v0 * math.sin(launch_rad)

        x = 0.0
        z = p.exit_height_m
        dt = p.dt
        max_height = z
        apex_x = 0.0
        t = 0.0
        max_time = p.max_sim_time

        _derivs = self._derivatives
        _hypot = math.hypot

        while t < max_time:
            # RK4 step
            state = (x, z, vx, vz)
            k1 = _derivs(state)
            s2 = _add_scaled(state, k1, dt / 2.0)
            k2 = _derivs(s2)
            s3 = _add_scaled(state, k2, dt / 2.0)
            k3 = _derivs(s3)
            s4 = _add_scaled(state, k3, dt)
            k4 = _derivs(s4)

            x += dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
            z += dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
            vx += dt / 6.0 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])
            vz += dt / 6.0 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3])
            t += dt

            if z > max_height:
                max_height = z
                apex_x = x

            # Passed or reached target distance
            if x >= target_distance_m:
                prev_x = x - vx * dt
                prev_z = z - vz * dt
                frac = (
                    (target_distance_m - prev_x) / (x - prev_x) if x != prev_x else 0.0
                )
                z_at_target = prev_z + frac * (z - prev_z)
                tof_at_target = t - dt + frac * dt
                return TrajectoryResult(
                    z_at_target, tof_at_target, True, max_height, apex_x
                )

            # Ball hit the ground
            if z < 0:
                return TrajectoryResult(0.0, t, False, max_height, apex_x)

        # Timed out
        return TrajectoryResult(0.0, max_time, False, max_height, apex_x)

    def _derivatives(
        self, state: tuple[float, float, float, float]
    ) -> tuple[float, float, float, float]:
        """state = (x, z, vx, vz) → (dx, dz, ax, az)."""
        _, _, svx, svz = state
        speed = math.hypot(svx, svz)

        ax = -self._k_drag * speed * svx
        az = -9.81 - self._k_drag * speed * svz + self._k_magnus * speed * speed

        return (svx, svz, ax, az)

    """LUT generation"""

    def find_rpm_for_distance(self, distance_m: float) -> LUTEntry:
        """Binary search for the RPM that puts the ball at the target height."""
        p = self.params
        height_tolerance = 0.02  # 2 cm
        lo = p.rpm_min
        hi = p.rpm_max

        # Quick feasibility check
        max_check = self.simulate(hi, distance_m)
        if not max_check.reached_target:
            return LUTEntry(distance_m, 0.0, 0.0, False)

        best_rpm = hi
        best_tof = max_check.tof
        best_error = abs(max_check.z_at_target - p.target_height_m)

        for _ in range(p.binary_search_iters):
            mid = (lo + hi) / 2.0
            result = self.simulate(mid, distance_m)

            if not result.reached_target:
                lo = mid
                continue

            error = result.z_at_target - p.target_height_m
            abs_error = abs(error)

            if abs_error < best_error:
                best_rpm = mid
                best_tof = result.tof
                best_error = abs_error

            if abs_error < height_tolerance:
                return LUTEntry(distance_m, mid, result.tof, True)

            if error > 0:
                hi = mid
            else:
                lo = mid

        return LUTEntry(distance_m, best_rpm, best_tof, best_error < 0.10)

    def generate_lut(self) -> GeneratedLUT:
        """Generate the full lookup table: 0.50 m to 5.00 m in 5 cm steps (91 entries)."""
        start_ns = time.monotonic_ns()
        entries: list[LUTEntry] = []
        reachable = 0
        unreachable = 0
        max_range = 0.0

        for i in range(91):
            distance = round(0.50 + i * 0.05, 2)
            entry = self.find_rpm_for_distance(distance)
            entries.append(entry)

            if entry.reachable:
                reachable += 1
                max_range = distance
            else:
                unreachable += 1

        elapsed_ms = (time.monotonic_ns() - start_ns) / 1_000_000
        return GeneratedLUT(
            entries, self.params, reachable, unreachable, max_range, elapsed_ms
        )

    """Internals (exposed for testing)"""

    @property
    def k_drag(self) -> float:
        return self._k_drag

    @property
    def k_magnus(self) -> float:
        return self._k_magnus


def _add_scaled(
    base: tuple[float, float, float, float],
    delta: tuple[float, float, float, float],
    scale: float,
) -> tuple[float, float, float, float]:
    return (
        base[0] + delta[0] * scale,
        base[1] + delta[1] * scale,
        base[2] + delta[2] * scale,
        base[3] + delta[3] * scale,
    )
