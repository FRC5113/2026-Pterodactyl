from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger,swerve
from wpilib.sysid import SysIdRoutineLog
from wpimath import units
import math
from components.swerve_drive import SwerveDrive
from lemonlib.util import MagicSysIdRoutine


class SysIdDrive(MagicSysIdRoutine):
    swerve_drive: SwerveDrive
    period: units.seconds = 0.02

    def setup(self):
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
                rampRate=2.0,
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

        self.sys_id_routine_rotation_rate = SysIdRoutine(
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
        self.sys_id_routine_rotation_volts = SysIdRoutine(
            SysIdRoutine.Config(
                # This is in radians per second², but SysId only supports "volts per second"
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
                name="SysIdRotation_volts",
            ),
        )

        self.sysid_to_apply = self.sys_id_routine_rotation_rate
        self.setup_sysid(
            self.sysid_to_apply.config,self.sysid_to_apply.mechanism
        )