import math

from magicbot import StateMachine, state, will_reset_to
from wpilib import DriverStation

from components.drive_control import DriveControl
from components.shooter import Shooter
from components.swerve_drive import SwerveDrive
from game import get_hub_pos
from lemonlib import fms_feedback


class ShooterController(StateMachine):
    drive_control: DriveControl
    shooter: Shooter
    swerve_drive: SwerveDrive

    at_speed = will_reset_to(False)
    shooting = will_reset_to(False)

    def setup(self):

        # Meters
        self.distance_lookup = [1.597,4.597]  # TODO Tune these values

        # RPS
        self.speed_lookup = [43.0, 53.0]  # TODO Tune these values

        # Seconds — measured flight times at each distance
        self.time_lookup = [0.45]  # TODO Tune these values

        self.target_rps = 0.0
        self.target_angle = 0.0
        self.distance = 0.0
        self.valid_shot = False

        self.phase_delay = 0.03
        self.lead_iterations = 15

        self.shooter_offsetX = 0.25  # meters forward of robot center
        self.shooter_offsetY = 0.0  # meters left (+) / right (-)

        self.min_distance = 1.0
        self.max_distance = 6.0

        self.idle_speed_scalar = 0.8
        self.kicker_duty = 8  # Volts
        self.angle_tolerance = 0.035  # radians (~2 deg)
        self.speed_tolerance = 0.05  # 5%

    """
    CONTROL METHODS
    """

    def request_shoot(self):
        self.shooting = True

    def _update_target(self):
        pose = self.swerve_drive.get_estimated_pose()
        chassis = self.swerve_drive.get_velocity()

        is_red = DriverStation.getAlliance() == DriverStation.Alliance.kRed
        hub_pos = get_hub_pos(is_red)

        future_x = pose.x + chassis.vx * self.phase_delay
        future_y = pose.y + chassis.vy * self.phase_delay
        future_heading = pose.rotation().radians() + chassis.omega * self.phase_delay

        cos_h = math.cos(future_heading)
        sin_h = math.sin(future_heading)

        launcher_x = (
            future_x + self.shooter_offsetX * cos_h - self.shooter_offsetY * sin_h
        )

        launcher_y = (
            future_y + self.shooter_offsetX * sin_h + self.shooter_offsetY * cos_h
        )

        rot_vx = -chassis.omega * self.shooter_offsetY
        rot_vy = chassis.omega * self.shooter_offsetX

        launcher_vx = chassis.vx + rot_vx
        launcher_vy = chassis.vy + rot_vy

        predicted_x = hub_pos.x
        predicted_y = hub_pos.y

        lookahead_distance = math.hypot(
            predicted_x - launcher_x,
            predicted_y - launcher_y,
        )

        for _ in range(self.lead_iterations):
            time_of_flight = self._linear_interp(
                lookahead_distance,
                self.distance_lookup,
                self.time_lookup,
            )

            offset_x = launcher_vx * time_of_flight
            offset_y = launcher_vy * time_of_flight

            predicted_x = hub_pos.x - offset_x
            predicted_y = hub_pos.y - offset_y

            lookahead_distance = math.hypot(
                predicted_x - launcher_x,
                predicted_y - launcher_y,
            )

        dx = predicted_x - launcher_x
        dy = predicted_y - launcher_y

        self.distance = lookahead_distance
        self.target_angle = math.atan2(dy, dx)

        self.target_rps = self._linear_interp(
            self.distance,
            self.distance_lookup,
            self.speed_lookup,
        )

        self.valid_shot = self.min_distance <= self.distance <= self.max_distance

    def _linear_interp(self, x, xp, fp):
        if x <= xp[0]:
            return fp[0]
        if x >= xp[-1]:
            return fp[-1]

        for i in range(len(xp) - 1):
            if xp[i] <= x <= xp[i + 1]:
                t = (x - xp[i]) / (xp[i + 1] - xp[i])
                return fp[i] + t * (fp[i + 1] - fp[i])

        return fp[-1]

    """
    INFORMATIONAL METHODS
    """

    @fms_feedback
    def get_target_rps(self):
        return self.target_rps

    @fms_feedback
    def get_distance(self):
        return self.distance

    @fms_feedback
    def is_at_speed(self):
        return self.at_speed

    """
    STATES
    """

    @state(first=True)
    def idle(self):
        self._update_target()

        if self.valid_shot:
            self.shooter.set_velocity(self.target_rps * self.idle_speed_scalar)
        else:
            self.shooter.set_velocity(0)

        if self.shooting:
            self.next_state("spin_up")

    def _is_aimed(self):
        heading = self.swerve_drive.get_estimated_pose().rotation().radians()
        error = math.atan2(
            math.sin(self.target_angle - heading),
            math.cos(self.target_angle - heading),
        )
        return abs(error) <= self.angle_tolerance

    @state
    def spin_up(self):
        self._update_target()

        if not self.valid_shot:
            self.next_state("idle")
            return

        self.shooter.set_velocity(self.target_rps)
        self.drive_control.point_to(self.target_angle)

        tolerance = self.speed_tolerance * self.target_rps
        speed_ready = abs(self.shooter.get_velocity() - self.target_rps) <= tolerance
        aim_ready = self._is_aimed()

        self.at_speed = speed_ready and aim_ready

        if not self.shooting:
            self.next_state("idle")
        elif self.at_speed:
            self.next_state("shoot")

    @state
    def shoot(self):
        self._update_target()

        if not self.valid_shot:
            self.next_state("idle")
            return

        self.drive_control.point_to(self.target_angle)
        self.shooter.set_velocity(self.target_rps)

        tolerance = self.speed_tolerance * self.target_rps
        speed_ready = abs(self.shooter.get_velocity() - self.target_rps) <= tolerance
        aim_ready = self._is_aimed()

        self.at_speed = speed_ready and aim_ready

        if not self.shooting:
            self.next_state("idle")
        elif not self.at_speed:
            self.next_state("spin_up")
        else:
            self.shooter.set_kicker(self.kicker_duty)
