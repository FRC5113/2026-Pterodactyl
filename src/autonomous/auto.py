from enum import Enum
from typing import List
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import meters, degrees, milliseconds
from components.swerve_drive import SwerveDrive
from components.shooter import Shooter
from components.intake import Intake
from components.shooter_controller import ShooterController
from magicbot import AutonomousStateMachine, state
import time
class StepStatus(Enum):
    RUNNING = 1
    DONE = 2
class AnnoyingGenericMagicBotAutoWrapperBecauseMagicBotIsPoorlyBuilt(AutonomousStateMachine):
    MODE_NAME = "I mean seriously who's dumb idea was it to lock teams into this half baked excuse for an Auto framework"
    DEFAULT = True
    swerve_drive: SwerveDrive
    shooter: Shooter
    intake: Intake
    shooter_controller: ShooterController

    def on_enable(self):
        self.runner = tempAutoRoutine
        self.ctx = AutoContext(self.swerve_drive, self.shooter, self.intake, self.shooter_controller)
        return super().on_enable()
    def on_disable(self) -> None:
        return super().on_disable()
    def on_iteration(self, tm: float) -> None:
        return super().on_iteration(tm)
    @state(first=True)
    def THE_ONLY_STATE(self):
        self.runner.run(self.ctx)
class AutoContext:
    sd: SwerveDrive
    sh: Shooter
    it: Intake
    sc: ShooterController

    def __init__(self, sd: SwerveDrive, sh: Shooter, it: Intake, sc: ShooterController):
        self.sd = sd
        self.sh = sh
        self.it = it
        self.sc = sc

class AutoStep:
    """Base class for all auto steps"""

    def execute(self, ctx: AutoContext) -> StepStatus:
        raise NotImplementedError


class AutoRunner:
    def __init__(self, steps: List[AutoStep]):
        self.steps = steps
        self.ctx = None
        self.index = 0
        print("\x1b[31m[DEBUG] AUTO RUNNER CREATED\x1b[0m")

    def reset(self):
        self.index = 0

    def run(self, ctx: AutoContext):
        print("\x1b[31m[DEBUG] AUTO RUNNER Running\x1b[0m")

        if self.index >= len(self.steps):
            return

        status = self.steps[self.index].execute(ctx)

        if status == StepStatus.DONE:
            self.index += 1


class ParallelStep(AutoStep):
    """
    Runs multiple steps at the same time.
    Finishes when ALL steps are DONE.
    """

    def __init__(self, *steps: AutoStep):
        self.steps = steps
        # Track which sub-steps have completed so we don't re-execute them
        self._done = [False] * len(steps)

    def execute(self, ctx: AutoContext) -> StepStatus:
        all_done = True
        for i in range(len(self.steps)):
            if self._done[i]:
                continue
            status = self.steps[i].execute(ctx)
            if status == StepStatus.DONE:
                self._done[i] = True
            else:
                all_done = False
        # If any step is not done yet, we're still running
        if not all_done:
            return StepStatus.RUNNING
        return StepStatus.DONE


class SwerveDriveAuto(AutoStep):
    """
    Field-oriented drive: takes absolute field coordinates (x, y in meters, heading in degrees).
    Uses WPILib coordinate system where (0,0) is at field corner from blue alliance perspective.
    """

    POSITION_TOLERANCE = 0.02  # meters

    def __init__(self, x: meters, y: meters, heading: degrees):
        self.x = x
        self.y = y
        self.heading_deg = heading
        self.target_pose = None

    def execute(self, ctx: AutoContext) -> StepStatus:
        print("\x1b[34m[DEBUG] Swerve drive executing\x1b[0m")

        # TODO: PID - it should do pid not just naivley drive there and dead stop
        # TODO: Angle tolerance
        if self.target_pose is None:
            self.target_pose = Pose2d(
                self.x, self.y, Rotation2d.fromDegrees(self.heading_deg)
            )
            ctx.sd.set_desired_pose(self.target_pose)

        distance = ctx.sd.get_distance_from_pose(self.target_pose)
        if distance <= self.POSITION_TOLERANCE:
            return StepStatus.DONE
        return StepStatus.RUNNING

class SwerveDriveBotRelativeAuto(AutoStep):
    def __init__(self, x: meters, y: meters, heading: degrees):
        self.x = x
        self.y = y
        self.heading = heading
        self.target_pose = None
    def execute(self, ctx: AutoContext) -> StepStatus:
        raise Exception("TODO")
class IntakeAuto(AutoStep):
    """Lets you turn on or off the intake with the boolean parameter"""

    def __init__(self, is_on: bool):
        self.is_on = is_on
        self.applied = False

    def execute(self, ctx: AutoContext) -> StepStatus:
        if not self.applied:
            ctx.it.set_voltage(12 if self.is_on else 0)
            self.applied = True
        return StepStatus.DONE


class ShootAuto(AutoStep):
    STATIC_ANGLE = 78  # Not sure this is right

    def __init__(self):
        self.started = False
        self.durration = 5.0
        # TODO: This should have a durration parameter
    def execute(self, ctx: AutoContext) -> StepStatus:
        if self.start == 0:
            self.start = time.perf_counter()
        if time.perf_counter() - self.start > self.duration:
            return StepStatus.DONE
        ctx.sc.request_shoot()
        return StepStatus.RUNNING


# THESE ARE ALL MADE UP NUMBERS!!!!!!!!!
tempAutoRoutine = AutoRunner(
    [
        SwerveDriveAuto(0.63, 0.04, -213.81),
        SwerveDriveAuto(0.76, 0.04, 0.47),
        SwerveDriveAuto(0.82, 0.04, 0.18),
        SwerveDriveAuto(0.82, 0.04, 0.00),
        SwerveDriveAuto(0.76, 0.04, -0.18),
        SwerveDriveAuto(0.63, 0.04, 182.00),
        IntakeAuto(True),
        SwerveDriveAuto(0.22, -0.00, -178.70),
        SwerveDriveAuto(0.25, 0.01, -1.33),
        SwerveDriveAuto(0.27, 0.01, -0.53),
        SwerveDriveAuto(0.27, 0.01, -0.00),
        SwerveDriveAuto(0.25, 0.01, 0.53),
        SwerveDriveAuto(0.22, -0.00, 85.71),
        SwerveDriveAuto(0.01, -0.26, 3.38),
        SwerveDriveAuto(0.01, -0.31, 0.00),
        SwerveDriveAuto(0.01, -0.33, 0.00),
        SwerveDriveAuto(0.01, -0.33, -0.00),
        SwerveDriveAuto(0.01, -0.31, 0.00),
        SwerveDriveAuto(0.01, -0.26, -99.82),
        SwerveDriveAuto(-0.20, 0.01, -165.52),
        SwerveDriveAuto(-0.24, 0.01, -0.52),
        SwerveDriveAuto(-0.25, 0.01, 0.09),
        SwerveDriveAuto(-0.26, 0.01, 0.57),
        SwerveDriveAuto(-0.24, 0.02, 1.16),
        SwerveDriveAuto(-0.21, 0.02, 139.84),
        IntakeAuto(False),
        SwerveDriveAuto(-0.62, 0.34, -114.70),
        SwerveDriveAuto(-0.89, 0.21, -15.44),
        SwerveDriveAuto(-1.03, 0.09, -8.34),
        SwerveDriveAuto(-1.03, -0.02, 353.79),
        SwerveDriveAuto(-0.91, -0.12, -6.56),
        SwerveDriveAuto(-0.65, -0.21, -24.36),
        ShootAuto(),
    ],
)
