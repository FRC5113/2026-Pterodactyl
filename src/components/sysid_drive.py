"""Module for sysid drive."""

from commands2.sysid import SysIdRoutine
from wpimath import units

from components.drive_control import DriveControl
from components.swerve_drive import SwerveDrive
from lemonlib.util import MagicSysIdRoutine


class SysIdDriveLinear(MagicSysIdRoutine):
    """SysIdDriveLinear class."""
    drive_control: DriveControl
    swerve_drive: SwerveDrive
    period: units.seconds = 0.02

    def setup(self):
        """Execute setup."""
        self.setup_sysid(
            SysIdRoutine.Config(
                rampRate=1,
                stepVoltage=7.0
            ),
            SysIdRoutine.Mechanism(
                self.drive_sysid,
                self.swerve_drive.log,
                self.swerve_drive, # type: ignore
                "Drive Linear",
            ),
        )

    def drive_sysid(self, voltage: units.volts) -> None:
        """Execute drive_sysid."""
        self.drive_control.drive_sysid_manual(voltage)


class SysIdDriveRotation(MagicSysIdRoutine):
    """SysIdDriveRotation class."""
    drive_control: DriveControl
    swerve_drive: SwerveDrive
    period: units.seconds = 0.02

    def setup(self):
        """Execute setup."""
        self.setup_sysid(
            SysIdRoutine.Config(rampRate=0.2, stepVoltage=7.0),
            SysIdRoutine.Mechanism(
                self.drive_sysid,
                self.swerve_drive.log,
                self.swerve_drive, # type: ignore
                "Drive Rotatinal",
            ),
        )

    def drive_sysid(self, voltage: units.volts) -> None:
        """Execute drive_sysid."""
        self.drive_control.drive_sysid_manual(voltage)
