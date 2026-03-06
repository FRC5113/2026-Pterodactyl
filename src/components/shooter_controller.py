import math

from magicbot import StateMachine, state, will_reset_to
from wpilib import DriverStation

from components.drive_control import DriveControl
from components.shooter import Shooter
from components.swerve_drive import SwerveDrive
from game import get_hub_pos

_RED = DriverStation.Alliance.kRed


class ShooterController(StateMachine):
    drive_control: DriveControl
    shooter: Shooter
    swerve_drive: SwerveDrive

    at_speed = will_reset_to(False)
    shooting = will_reset_to(False)
    unjamming = will_reset_to(False)
    force_shoot_req = will_reset_to(False)
    force_shoot_rps = will_reset_to(0.0)

    def setup(self):

        # Meters
        self.distance_lookup = [1.597, 2.597, 3.597, 4.597]  # TODO Tune these values

        # RPS
        self.speed_lookup = [41.95, 45.8, 48.9, 53.0]  # TODO Tune these values

        # Seconds — measured flight times at each distance
        self.time_lookup = [0.97, 1.21, 1.2, 1.2]  # TODO Tune these values

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

    def request_unjam(self):
        self.unjamming = True

    def request_force_shoot(self, rps: float):
        self.force_shoot_req = True
        self.force_shoot_rps = rps

    def _update_target(self):
        pose = self.swerve_drive.get_estimated_pose()
        chassis = self.swerve_drive.get_velocity()

        is_red = DriverStation.getAlliance() == _RED
        hub_pos = get_hub_pos(is_red)

        phase_delay = self.phase_delay
        future_x = pose.x + chassis.vx * phase_delay
        future_y = pose.y + chassis.vy * phase_delay
        future_heading = pose.rotation().radians() + chassis.omega * phase_delay

        cos_h = math.cos(future_heading)
        sin_h = math.sin(future_heading)

        offsetX = self.shooter_offsetX
        offsetY = self.shooter_offsetY

        launcher_x = future_x + offsetX * cos_h - offsetY * sin_h
        launcher_y = future_y + offsetX * sin_h + offsetY * cos_h

        omega = chassis.omega
        rot_vx = -omega * offsetY
        rot_vy = omega * offsetX

        launcher_vx = chassis.vx + rot_vx
        launcher_vy = chassis.vy + rot_vy

        hub_x = hub_pos.x
        hub_y = hub_pos.y
        predicted_x = hub_x
        predicted_y = hub_y

        _hypot = math.hypot
        _interp = self._linear_interp
        dist_lut = self.distance_lookup
        time_lut = self.time_lookup

        lookahead_distance = _hypot(
            predicted_x - launcher_x,
            predicted_y - launcher_y,
        )

        for _ in range(self.lead_iterations):
            time_of_flight = _interp(
                lookahead_distance,
                dist_lut,
                time_lut,
            )

            predicted_x = hub_x - launcher_vx * time_of_flight
            predicted_y = hub_y - launcher_vy * time_of_flight

            lookahead_distance = _hypot(
                predicted_x - launcher_x,
                predicted_y - launcher_y,
            )

        self.distance = lookahead_distance
        self.target_angle = math.atan2(
            predicted_y - launcher_y, predicted_x - launcher_x
        )

        self.target_rps = _interp(
            lookahead_distance,
            dist_lut,
            self.speed_lookup,
        )

        self.valid_shot = self.min_distance <= lookahead_distance <= self.max_distance

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

    # @feedback
    def get_target_rps(self):
        return self.target_rps

    # @feedback
    def get_distance(self):
        return self.distance

    # @feedback
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

        if self.unjamming:
            self.next_state("unjam")
        if self.force_shoot_req:
            self.next_state("force_shoot")

        if self.shooting:
            self.next_state("spin_up")

    def _is_aimed(self):
        heading = self.swerve_drive.get_estimated_pose().rotation().radians()
        diff = self.target_angle - heading
        error = math.atan2(math.sin(diff), math.cos(diff))
        return abs(error) <= self.angle_tolerance

    @state
    def unjam(self):
        self.shooter.set_kicker(-self.kicker_duty)
        self.shooter.set_voltage(-self.kicker_duty)
        if not self.unjamming:
            self.next_state("idle")

    @state
    def force_shoot(self):
        self.shooter.set_velocity(self.force_shoot_rps)
        if abs(self.shooter.get_velocity() - self.target_rps) <= (10.0):
            self.shooter.set_kicker(self.kicker_duty)
        if not self.force_shoot_req:
            self.next_state("idle")

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
