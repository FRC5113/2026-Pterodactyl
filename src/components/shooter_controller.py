import math

from magicbot import StateMachine, feedback, state, will_reset_to
from wpilib import DriverStation, Field2d
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds

from components.drive_control import DriveControl
from components.indexer import Indexer
from components.intake import Intake
from components.shooter import Shooter
from components.shot_calculator import (
    INVALID,
    LaunchParameters,
    ShotCalculator,
    ShotConfig,
    ShotInputs,
)
from components.swerve_drive import SwerveDrive
from game import get_hub_pos
from lemonlib.smart import SmartPreference

_RED = DriverStation.Alliance.kRed

# Hub forward vectors — points from behind the hub toward the field.
# Used by ShotCalculator to reject shots from behind the hub.
_RED_HUB_FORWARD = Translation2d(-1.0, 0.0)
_BLUE_HUB_FORWARD = Translation2d(1.0, 0.0)


class ShooterController(StateMachine):
    drive_control: DriveControl
    shooter: Shooter
    indexer: Indexer
    swerve_drive: SwerveDrive
    intake: Intake

    estimated_field: Field2d

    at_speed = will_reset_to(False)
    shooting = will_reset_to(False)
    unjamming = will_reset_to(False)
    force_shoot_req = will_reset_to(False)
    force_shoot_rps = will_reset_to(0.0)
    shoot_noncalc = will_reset_to(False)

    forceshoottolgood = will_reset_to(False)

    flywheel_speed = SmartPreference(30.0)

    # Minimum confidence (0-100) from the solver to accept a shot
    min_confidence = SmartPreference(40.0)

    def setup(self):
        self.distance_lookup = [1.597, 2.597, 3.597, 4.597]  # TODO Tune these values

        # RPS
        self.speed_lookup = [41.95, 45.8, 48.9, 53.0]  # TODO Tune these values

        # Seconds — measured flight times at each distance
        self.time_lookup = [0.97, 1.21, 1.2, 1.2]  # TODO Tune these values

        # Build the shot calculator
        min_d = self.distance_lookup[0]
        max_d = self.distance_lookup[-1]

        config = ShotConfig(
            launcher_offset_x=0.0,
            launcher_offset_y=0.0,
            min_scoring_distance=min_d,
            max_scoring_distance=max_d,
            phase_delay_ms=30.0,
            mech_latency_ms=20.0,
        )
        self.calculator = ShotCalculator(config)

        # Load every reachable LUT entry.  RPM from the sim is wheel RPM;
        # the shooter LUT expects flywheel RPS (= RPM / 60 * gear_ratio,
        for dis, rps, tof in zip(
            self.distance_lookup, self.speed_lookup, self.time_lookup
        ):
            self.calculator.load_lut_entry(dis, rps, tof)

        # Cached solver output
        self.launch: LaunchParameters = INVALID
        self.target_rps = 0.0
        self.target_angle = 0.0
        self.distance = 0.0
        self.valid_shot = False
        self.shot_confidence = 0.0

        # Tuning constants
        self.idle_speed_scalar = 0.5
        self.kicker_volts = 10  # Volts
        self.conveyor_volts = -9  # Volts
        self.angle_tolerance = 0.035  # radians (~2 deg)
        self.speed_tolerance = 0.1  # 10 %
        self.idle_accerlation = 300.0  # RPS

        self.vx = 0.0
        self.vy = 0.0

    """
    CONTROL METHODS
    """

    def set_drive_velocity(self, vx: float, vy: float, omega: float):
        self.vx = vx
        self.vy = vy
        self.omega = omega

    def request_shoot(self):
        self.shooting = True

    def request_unjam(self):
        self.unjamming = True

    def request_force_shoot(self, rps: float):
        self.force_shoot_req = True
        self.force_shoot_rps = rps

    def request_shoot_noncalc(self):
        self.shoot_noncalc = True
        self.shooting = True

    def adjust_rpm_offset(self, delta: float):
        """Copilot D-pad trim.  Persists until reset."""
        self.calculator.adjust_offset(delta)

    def reset_rpm_offset(self):
        self.calculator.reset_offset()

    def _build_shot_inputs(self) -> ShotInputs:
        pose = self.swerve_drive.get_estimated_pose()
        robot_vel = self.swerve_drive.get_velocity()  # robot-relative

        # Convert robot-relative velocity > field-relative
        heading = pose.rotation().radians()
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        field_vx = robot_vel.vx * cos_h - robot_vel.vy * sin_h
        field_vy = robot_vel.vx * sin_h + robot_vel.vy * cos_h
        field_vel = ChassisSpeeds(field_vx, field_vy, robot_vel.omega)

        is_red = DriverStation.getAlliance() == _RED
        hub_pos = get_hub_pos(is_red)
        hub_forward = _RED_HUB_FORWARD if is_red else _BLUE_HUB_FORWARD

        return ShotInputs(
            robot_pose=pose,
            field_velocity=field_vel,
            robot_velocity=robot_vel,
            hub_center=hub_pos,
            hub_forward=hub_forward,
        )

    def _update_target(self):
        inputs = self._build_shot_inputs()
        self.distance = self.calculator.get_distance_from_hub(inputs)
        if self.shoot_noncalc:
            # Bypass the solver and just do a linear interpolation based on
            # distance.
            self.valid_shot = True
            self.target_rps = self._linear_interp(
                self.distance, self.distance_lookup, self.speed_lookup
            )
            self.target_angle = math.atan2(
                inputs.hub_center.y - inputs.robot_pose.Y(),
                inputs.hub_center.x - inputs.robot_pose.X(),
            )
            self.shot_confidence = 100.0  # Fake it
        else:
            self.launch = self.calculator.calculate(inputs)

            if self.launch.is_valid:
                self.valid_shot = True
                self.target_rps = self.launch.rpm
                self.target_angle = self.launch.drive_angle.radians()
                self.shot_confidence = self.launch.confidence
            else:
                self.valid_shot = False
                self.target_rps = 0.0
                self.shot_confidence = 0.0
                # Keep target_angle from last valid solution so idle spin-up
                # doesn't jump around.

    def _linear_interp(self, x, xp, fp):
        """Fast linear interpolation"""
        if x <= xp[0]:
            return fp[0]
        if x >= xp[-1]:
            return fp[-1]

        for i in range(len(xp) - 1):
            if xp[i] <= x <= xp[i + 1]:
                # Linear interpolation formula
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
    def get_confidence(self):
        return self.shot_confidence

    # @feedback
    def is_at_speed(self):
        return self.at_speed

    # @feedback
    def is_valid_shot(self) -> bool:
        return self.valid_shot

    # @feedback
    def get_force_good(self):
        return self.forceshoottolgood

    """
    STATES
    """

    @state(first=True)
    def idle(self):
        self._update_target()
        # self.shooter.set_velocity(self.target_rps * self.idle_speed_scalar)
        # self.shooter.set_acceleration(self.idle_accerlation)

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
        self.indexer.set_kicker(-self.kicker_volts)
        self.indexer.set_conveyor(-self.conveyor_volts)
        self.shooter.set_voltage(-self.kicker_volts)
        if not self.unjamming:
            self.next_state("idle")

    @state
    def force_shoot(self):
        self.shooter.set_velocity(self.force_shoot_rps)
        tolerance = self.speed_tolerance * self.force_shoot_rps
        speed_ready = (
            abs(self.shooter.get_velocity() - self.force_shoot_rps) <= tolerance
        )
        if speed_ready:
            self.forceshoottolgood = True
            self.indexer.set_kicker(self.kicker_volts)
            self.indexer.set_conveyor(self.conveyor_volts)
            # self.intake.set_voltage(-10)
        if not self.force_shoot_req:
            self.next_state("idle")

    @state
    def spin_up(self):
        self._update_target()

        # Keep spinning and aiming even if the shot is momentarily invalid
        # (e.g. heading error spike) — don't drop back to idle and stop
        # pointing, which causes drive-state flutter.
        self.shooter.set_velocity(self.target_rps)
        self.drive_control.point_to_field(self.vx, self.vy, self.target_angle)

        tolerance = self.speed_tolerance * self.target_rps
        speed_ready = abs(self.shooter.get_velocity() - self.target_rps) <= tolerance

        # Gate on the SOTM solver's confidence instead of a raw heading
        # check — the solver already penalises heading error, so this
        # lets us shoot while moving without flickering on/off.
        confident = self.shot_confidence >= self.min_confidence

        self.at_speed = speed_ready and confident

        if not self.shooting:
            self.next_state("idle")
        if self.at_speed:
            self.next_state("shoot")

    @state
    def shoot(self):
        self._update_target()

        # Always keep aiming and spinning — don't bail on momentary invalid
        self.drive_control.point_to_field(self.vx, self.vy, self.target_angle)
        self.shooter.set_velocity(self.target_rps)

        tolerance = self.speed_tolerance * self.target_rps
        speed_ready = abs(self.shooter.get_velocity() - self.target_rps) <= tolerance
        confident = self.valid_shot and self.shot_confidence >= self.min_confidence

        self.at_speed = speed_ready and confident

        if not self.shooting:
            self.next_state("idle")
        elif not speed_ready:
            # Only drop back to spin_up when the flywheel actually loses
            # speed — NOT for heading drift.  The SOTM solver continuously
            # updates the aim, so small heading errors are compensated.
            self.next_state("spin_up")
        elif self.at_speed:
            self.indexer.set_kicker(self.kicker_volts)
            self.indexer.set_conveyor(self.conveyor_volts)
