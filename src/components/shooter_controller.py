import math

from magicbot import StateMachine, feedback, state, will_reset_to
from wpilib import DriverStation

from components.drive_control import DriveControl
from components.shooter import Shooter
from components.swerve_drive import SwerveDrive
from game import get_hub_pos, is_alliance_hub_active

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

    forceshoottolgood = will_reset_to(False)

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

        self.min_distance = 1.597
        self.max_distance = 4.597

        self.idle_speed_scalar = 0.5
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
        pose = self.swerve_drive.get_estimated_pose().translation()
        is_red = DriverStation.getAlliance() == _RED
        hub_pos = get_hub_pos(is_red)

        distance = hub_pos.distance(pose)
        self.distance = distance

        self.target_angle = math.atan2(hub_pos.y - pose.y, hub_pos.x - pose.x)

        self.target_rps = self._linear_interp(
            distance, self.distance_lookup, self.speed_lookup
        )
        self.valid_shot = self.min_distance <= distance <= self.max_distance

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

    @feedback
    def get_target_rps(self):
        return self.target_rps

    @feedback
    def get_distance(self):
        return self.distance

    # @feedback
    def is_at_speed(self):
        return self.at_speed

    @feedback
    def is_valid_shot(self) -> bool:
        return self.valid_shot

    def get_force_good(self):
        return self.forceshoottolgood

    """
    STATES
    """

    @state(first=True)
    def idle(self):
        self._update_target()
        # if self.valid_shot:
        self.shooter.set_velocity(self.target_rps * self.idle_speed_scalar)
        # else:
        #     self.shooter.set_velocity(0.0)

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
            self.forceshoottolgood = True
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

        self.at_speed = speed_ready and aim_ready and is_alliance_hub_active()

        if not self.shooting:
            self.next_state("idle")
        elif not self.at_speed:
            self.next_state("spin_up")
        else:
            self.shooter.set_kicker(self.kicker_duty)
