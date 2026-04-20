import math

from magicbot import StateMachine, feedback, state, will_reset_to
from wpilib import DriverStation, Field2d
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds

from choreo.trajectory import SwerveSample
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

    transx = will_reset_to(0.0)
    transy = will_reset_to(0.0)
    omega = will_reset_to(0.0)
    sample = will_reset_to(None)
    drive_auto_mode = will_reset_to(False)

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
        self.kicker_volts = 12  # Volts
        self.conveyor_volts = -12  # Volts
        self.angle_tolerance = 0.035  # radians (~2 deg)
        self.speed_tolerance = 0.1  # 5 %
        self.idle_accerlation = 300.0  # RPS

    """
    CONTROL METHODS
    """

    def drive(self, transx: float, transy: float, omega: float):
        self.transx = transx
        self.transy = transy
        self.omega = omega

    def drive_auto(self, sample: SwerveSample):
        self.drive_auto_mode = True
        self.sample = sample

    def request_shoot(self):
        self.shooting = True

    def request_unjam(self):
        self.unjamming = True

    def request_force_shoot(self, rps: float):
        self.force_shoot_req = True
        self.force_shoot_rps = rps

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
        if self.drive_auto_mode and self.sample is not None:
            self.drive_control.drive_auto(self.sample)
        else:
            self.drive_control.drive_manual(self.transx, self.transy, self.omega, True)

        self._update_target()
        self.shooter.set_velocity(self.target_rps * self.idle_speed_scalar)
        self.shooter.set_acceleration(self.idle_accerlation)

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
            self.intake.set_voltage(-10)
        if not self.force_shoot_req:
            self.next_state("idle")

    @state
    def spin_up(self):
        self._update_target()

        # Keep spinning and aiming even if the shot is momentarily invalid
        # (e.g. heading error spike) — don't drop back to idle and stop
        # pointing, which causes drive-state flutter.
        self.shooter.set_velocity(self.target_rps)
        self.drive_control.point_to(self.target_angle)

        tolerance = self.speed_tolerance * self.target_rps
        speed_ready = abs(self.shooter.get_velocity() - self.target_rps) <= tolerance

        # Gate on the SOTM solver's confidence instead of a raw heading
        # check — the solver already penalises heading error, so this
        # lets us shoot while moving without flickering on/off.
        confident = self.shot_confidence >= self.min_confidence

        self.at_speed = speed_ready and confident

        if self.at_speed:
            self.next_state("shoot")
        if not self.shooting:
            self.next_state("idle")

    @state
    def shoot(self):
        self._update_target()

        # Always keep aiming and spinning — don't bail on momentary invalid
        self.drive_control.drive_point_field(
            self.transx, self.transy, self.target_angle
        )
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
