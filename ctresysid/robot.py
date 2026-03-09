import math

from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger, swerve
from wpilib.sysid import SysIdRoutineLog
from wpimath import units

from components.swerve_drive import SwerveDrive
from components.sysid import SysIdDrive
from generated.tuner_constants import TunerConstants
from lemonlib import LemonInput, LemonRobot
from lemonlib.smart import SmartProfile


class MyRobot(LemonRobot):
    sysid: SysIdDrive
    swerve_drive: SwerveDrive

    def createObjects(self):
        self.tuning_enabled = False

        self.max_speed: units.meters_per_second = TunerConstants.speed_at_12_volts

        self.translation_profile = SmartProfile(
            "translation",
            {
                "kP": 5.0,
                "kI": 0.0,
                "kD": 0.0,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )
        self.rotation_profile = SmartProfile(
            "rotation",
            {
                "kP": 7.0,
                "kI": 0.0,
                "kD": 0.1,
                "kMaxV": 8.0,
                "kMaxA": 40.0,
                "kMinInput": -math.pi,
                "kMaxInput": math.pi,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )

        self.steer_profile = SmartProfile(
            "swerve_steer",
            {
                "kP": TunerConstants._steer_gains.k_p,
                "kI": TunerConstants._steer_gains.k_i,
                "kD": TunerConstants._steer_gains.k_d,
                "kS": TunerConstants._steer_gains.k_s,
                "kV": TunerConstants._steer_gains.k_v,
                "kA": TunerConstants._steer_gains.k_a,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )

        self.drive_profile = SmartProfile(
            "swerve_drive",
            {
                "kP": TunerConstants._drive_gains.k_p,
                "kI": TunerConstants._drive_gains.k_i,
                "kD": TunerConstants._drive_gains.k_d,
                "kS": TunerConstants._drive_gains.k_s,
                "kV": TunerConstants._drive_gains.k_v,
                "kA": TunerConstants._drive_gains.k_a,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )

    def teleopInit(self):
        self.primary = LemonInput(0)

    def teleopPeriodic(self):
        if self.primary.getAButton():
            self.sysid.quasistatic_forward()
        elif self.primary.getBButton():
            self.sysid.quasistatic_reverse()
        elif self.primary.getXButton():
            self.sysid.dynamic_forward()
        elif self.primary.getYButton():
            self.sysid.dynamic_reverse()
