import math

from magicbot import feedback, will_reset_to

from components.drive_control import DriveControl
from components.shooter_controller import ShooterController
from components.swerve_drive import SwerveDrive
from lemonlib.smart import SmartPreference


class BigBoy:
    """Aim-assist + shoot-gate coordinator.

    Each cycle BigBoy:
      1. Reads the solver output from ShooterController (target_angle,
         valid_shot).
      2. Computes whether the robot's actual heading is within tolerance
         of the target — the "aim correctness" check — and pushes the
         result to ShooterController via set_aim_ok().  The shooter only
         fires its indexer when aim_ok is True.
      3. If a shoot/aim request is active, calls drive_control.point_field
         to override ONLY the drivetrain rotation.  Translation continues
         to come from whatever called drive_control.drive_manual this
         cycle (the driver in robot.py, or auto in auto_base).
    """

    drive_control: DriveControl
    shooter_controller: ShooterController
    swerve_drive: SwerveDrive

    aim_assist_req = will_reset_to(False)
    shoot_req = will_reset_to(False)

    # ~2 deg.  Tightenable per-driver preference.
    angle_tolerance = SmartPreference(0.035)

    def setup(self):
        pass

    """
    CONTROL METHODS
    """

    def request_shoot(self):
        """SOTM shot: aim, spin up, fire when aimed and at speed."""
        self.aim_assist_req = True
        self.shoot_req = True
        # Forward immediately so the flywheel spins up DURING the aim
        # transit — we don't want to wait until aimed to start spinning.
        self.shooter_controller.request_shoot()

    def request_aim(self):
        """Aim-assist only — no fire request."""
        self.aim_assist_req = True

    def request_force_shoot(self, rps: float):
        """Fixed-RPS shot, no aim-assist."""
        self.shooter_controller.request_force_shoot(rps)

    def request_unjam(self):
        self.shooter_controller.request_unjam()

    """
    INFORMATIONAL METHODS
    """

    @feedback
    def is_aim_active(self) -> bool:
        return self.aim_assist_req

    @feedback
    def is_aimed(self) -> bool:
        return self._is_aimed()

    """
    INTERNAL
    """

    def _heading_error(self) -> float:
        """Wrapped heading error in radians, target − current."""
        heading = self.swerve_drive.get_estimated_pose().rotation().radians()
        diff = self.shooter_controller.target_angle - heading
        return math.atan2(math.sin(diff), math.cos(diff))

    def _is_aimed(self) -> bool:
        if not self.shooter_controller.valid_shot:
            return False
        return abs(self._heading_error()) <= self.angle_tolerance

    def execute(self):
        # 1. Tell the shooter whether it's aimed.  Auto-shoot's indexer is
        #    gated on this, so the bot can't fire while still rotating.
        self.shooter_controller.set_aim_ok(self._is_aimed())

        # 2. If we're aiming (shoot or aim-only request), override only
        #    drivetrain rotation — translation stays under driver control.
        if (
            (self.aim_assist_req or self.shoot_req)
            and self.shooter_controller.valid_shot
        ):
            self.drive_control.point_field(self.shooter_controller.target_angle)
