import math

from magicbot import StateMachine, feedback, state, will_reset_to
from wpilib import DriverStation
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds

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
_RED_HUB_FORWARD = Translation2d(-1.0, 0.0)
_BLUE_HUB_FORWARD = Translation2d(1.0, 0.0)


class ShooterController(StateMachine):
    """Self-contained shoot subsystem.

    Owns the SOTM solver and exposes the solved target (RPS, angle,
    confidence) as plain attributes so BigBoy can read them and decide
    when to override drive rotation. This component does NOT touch the
    drivetrain — see BigBoy for aim-assist."""

    shooter: Shooter
    indexer: Indexer
    intake: Intake
    swerve_drive: SwerveDrive

    at_speed = will_reset_to(False)
    shooting = will_reset_to(False)
    unjamming = will_reset_to(False)
    force_shoot_req = will_reset_to(False)
    force_shoot_rps = will_reset_to(0.0)
    forceshoottolgood = will_reset_to(False)
    # Set by BigBoy each cycle. The indexer only fires when this is True,
    # so aim correctness lives in BigBoy while the solver lives here.
    aim_ok = will_reset_to(False)

    flywheel_speed = SmartPreference(30.0)
    min_confidence = SmartPreference(40.0)

    def setup(self):
        self.distance_lookup = [1.597, 2.597, 3.597, 4.597]  # TODO Tune
        self.speed_lookup = [41.95, 45.8, 48.9, 53.0]  # RPS, TODO Tune
        self.time_lookup = [0.97, 1.21, 1.2, 1.2]  # seconds, TODO Tune

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
        for dis, rps, tof in zip(
            self.distance_lookup, self.speed_lookup, self.time_lookup
        ):
            self.calculator.load_lut_entry(dis, rps, tof)

        # Cached solver output — read by BigBoy each cycle.
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
        self.speed_tolerance = 0.1  # fraction of target
        self.idle_accerlation = 300.0  # RPS

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

    def set_aim_ok(self, ok: bool):
        """Called by BigBoy with the result of its heading-error check.
        Auto-shoot won't fire unless this is True."""
        self.aim_ok = ok

    def adjust_rpm_offset(self, delta: float):
        self.calculator.adjust_offset(delta)

    def reset_rpm_offset(self):
        self.calculator.reset_offset()

    """
    INFORMATIONAL METHODS
    """

    @feedback
    def is_valid_shot(self) -> bool:
        return self.valid_shot

    @feedback
    def get_target_rps(self) -> float:
        return self.target_rps

    @feedback
    def get_distance(self) -> float:
        return self.distance

    @feedback
    def get_confidence(self) -> float:
        return self.shot_confidence

    def get_force_good(self):
        return self.forceshoottolgood

    """
    INTERNAL
    """

    def _build_shot_inputs(self) -> ShotInputs:
        pose = self.swerve_drive.get_estimated_pose()
        robot_vel = self.swerve_drive.get_velocity()  # robot-relative

        # Convert robot-relative > field-relative
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
            # Keep last target_angle so idle spin-up doesn't jitter.

    """
    STATES
    """

    @state(first=True)
    def idle(self):
        self._update_target()
        self.shooter.set_velocity(self.target_rps * self.idle_speed_scalar)
        self.shooter.set_acceleration(self.idle_accerlation)

        if self.unjamming:
            self.next_state("unjam")
        if self.force_shoot_req:
            self.next_state("force_shoot")
        if self.shooting:
            self.next_state("spin_up")

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
        self.shooter.set_velocity(self.target_rps)

        tolerance = self.speed_tolerance * self.target_rps
        speed_ready = abs(self.shooter.get_velocity() - self.target_rps) <= tolerance

        confident = self.valid_shot and self.shot_confidence >= self.min_confidence

        # Auto-shoot requires explicit aim-ok from BigBoy in addition to
        # speed + solver confidence.
        self.at_speed = speed_ready and confident and self.aim_ok

        if self.at_speed:
            self.next_state("shoot")
        if not self.shooting:
            self.next_state("idle")

    @state
    def shoot(self):
        self._update_target()
        self.shooter.set_velocity(self.target_rps)

        tolerance = self.speed_tolerance * self.target_rps
        speed_ready = abs(self.shooter.get_velocity() - self.target_rps) <= tolerance
        confident = self.valid_shot and self.shot_confidence >= self.min_confidence

        self.at_speed = speed_ready and confident and self.aim_ok

        if not self.shooting:
            self.next_state("idle")
        elif not speed_ready:
            # Only drop back to spin_up when the flywheel actually loses
            # speed. Heading drift is handled by BigBoy's point_field call;
            # if heading is briefly off we just hold fire (aim_ok=False)
            # rather than re-cycling the state machine.
            self.next_state("spin_up")
        elif self.at_speed:
            self.indexer.set_kicker(self.kicker_volts)
            self.indexer.set_conveyor(self.conveyor_volts)
