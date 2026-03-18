from enum import Enum
from typing import Sequence, cast
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import meters, degrees, milliseconds
from components.drive_control import DriveControl
from components.shooter import Shooter
from components.intake import Intake
from components.shooter_controller import ShooterController
import time
import math
class StepStatus(Enum):
    RUNNING = 1
    DONE = 2



class AutoContext:
    dc: DriveControl
    sh: Shooter
    it: Intake
    sc: ShooterController

    def __init__(self, dc: DriveControl, sh: Shooter, it: Intake, sc: ShooterController):
        self.dc = dc
        self.sh = sh
        self.it = it
        self.sc = sc

class AutoStep:
    """Base class for all auto steps"""

    def execute(self, ctx: AutoContext) -> StepStatus:
        raise NotImplementedError


class AutoRunner:
    def __init__(self, steps: Sequence[AutoStep]):
        self.steps = steps
        self.ctx = AutoContext(cast(DriveControl, DriveControl()), Shooter(), Intake(), cast(ShooterController, ShooterController())) # Placeholder
        self.index = 0

    def reset(self):
        self.index = 0

    def run(self, ctx: AutoContext):
        if self.index >= len(self.steps):
            return

        status = self.steps[self.index].execute(self.ctx)

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
        # TODO: PID - it should do pid not just naivley drive there and dead stop
        # TODO: Angle tolerance
        if self.target_pose is None:
            self.target_pose = Pose2d(
                self.x, self.y, Rotation2d.fromDegrees(self.heading_deg)
            )
            ctx.dc.request_pose(self.target_pose)

        distance = ctx.dc.get_distance_from_target_pose()
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
        if self.target_pose is None:
            est = ctx.dc.get_estimated_pose()
            est.rotateBy(Rotation2d((self.heading * math.pi) / 180))
            new = Pose2d(est.x + self.x, est.y + self.y, est.rotation())
            ctx.dc.request_pose(new)
            return StepStatus.RUNNING
        else:
            DISTANCE_TOLLERANCE = meters(0.01)
            if ctx.dc.get_distance_from_target_pose() > DISTANCE_TOLLERANCE:
                return StepStatus.RUNNING
            else:
                return StepStatus.DONE
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
    def __init__(self, durration: milliseconds):
        self.started = False
        self.durration = durration
    def execute(self, ctx: AutoContext) -> StepStatus:
        if self.start == 0:
            self.start = time.perf_counter()
        if time.perf_counter() - self.start > self.durration:
            return StepStatus.DONE
        ctx.sc.request_shoot()
        return StepStatus.RUNNING