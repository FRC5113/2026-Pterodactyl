import math

from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger, swerve
from wpilib.sysid import SysIdRoutineLog
from wpimath import units

from components.swerve_drive import SwerveDrive
from generated.tuner_constants import TunerConstants
from lemonlib import LemonInput, LemonRobot
from lemonlib.smart import SmartProfile


class MyRobot(LemonRobot):
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

        self.translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self.steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self.rotation_characterization = swerve.requests.SysIdSwerveRotation()

        self.sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0,
                # Log state with SignalLogger class
                recordState=lambda state: (
                    SignalLogger.write_string(
                        "SysIdTranslation_State",
                        SysIdRoutineLog.stateEnumToString(state),
                    )
                    and None
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.swerve_drive.apply_control(
                    self.translation_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
                name="SysIdTranslation",
            ),
        )
        """SysId routine for characterizing translation. This is used to find PID gains for the drive motors."""

        self.sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Log state with SignalLogger class
                recordState=lambda state: (
                    SignalLogger.write_string(
                        "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                    )
                    and None
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.swerve_drive.apply_control(
                    self.steer_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
                name="SysIdSteer",
            ),
        )
        """SysId routine for characterizing steer. This is used to find PID gains for the steer motors."""

        self.sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                # This is in radians per second², but SysId only supports "volts per second"
                rampRate=math.pi / 6,
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Use default timeout (10 s)
                # Log state with SignalLogger class
                recordState=lambda state: (
                    SignalLogger.write_string(
                        "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                    )
                    and None
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: (
                    (
                        # output is actually radians per second, but SysId only supports "volts"
                        self.swerve_drive.apply_control(
                            self.rotation_characterization.with_rotational_rate(output)
                        ),
                        # also log the requested output for SysId
                        SignalLogger.write_double("Rotational_Rate", output),
                    )
                    and None
                ),
                lambda log: None,
                self,
                name="SysIdRotation",
            ),
        )

        self.sysid_to_apply = self.sys_id_routine_translation

    def teleopPeriodic(self):
        if self.primary.getPOV() == 180:
            if self.primary.getAButton():
                self.sysid_to_apply.dynamic(SysIdRoutine.Direction.kForward)
            elif self.primary.getBButton():
                self.sysid_to_apply.dynamic(SysIdRoutine.Direction.kReverse)
            elif self.primary.getXButton():
                self.sysid_to_apply.quasistatic(SysIdRoutine.Direction.kForward)
            elif self.primary.getYButton():
                self.sysid_to_apply.quasistatic(SysIdRoutine.Direction.kReverse)
