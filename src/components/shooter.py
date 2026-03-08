from magicbot import feedback, will_reset_to
from phoenix6 import controls
from phoenix6.configs import (
    FeedbackConfigs,
    TalonFXConfiguration,
    TalonFXSConfiguration,
)
from phoenix6.hardware import TalonFX, TalonFXS
from phoenix6.signals import (
    FeedbackSensorSourceValue,
    MotorAlignmentValue,
    MotorArrangementValue,
    NeutralModeValue,
)
from wpimath import units
import enum
from lemonlib.smart import SmartProfile

class ShooterSettings(enum.IntEnum):
    HIGH = 1
    MIDDLE = 2
    LOW = 3
    OFF = 4
class Shooter:
    """low level component that directly manages the shooter motors and their configuration. controlled by the shooter controller component, but can also be directly controlled for testing purposes"""

    right_motor: TalonFX
    left_motor: TalonFX
    left_kicker_motor: TalonFXS
    right_kicker_motor: TalonFXS
    vibrator_motor: TalonFXS
    vibrator_volts = 0
    shooter_profile: SmartProfile
    shooter_gear_ratio: float
    shooter_amps: units.amperes
    shooter_setting: ShooterSettings
    setting_to_volt = {
        ShooterSettings.LOW: 42,
        ShooterSettings.MIDDLE: 46,
        ShooterSettings.HIGH: 50
    }
    kicker_volts = 0
    def setup(self):
        self._cached_velocity = 0.0
        self.shooter_motors_config = TalonFXConfiguration()
        self.shooter_motors_config.motor_output.neutral_mode = NeutralModeValue.COAST
        self.shooter_motors_config.feedback = (
            FeedbackConfigs()
            .with_feedback_sensor_source(FeedbackSensorSourceValue.ROTOR_SENSOR)
            .with_sensor_to_mechanism_ratio(self.shooter_gear_ratio)
        )

        self.shooter_motors_config.current_limits.stator_current_limit = (
            self.shooter_amps
        )
        self.shooter_motors_config.current_limits.stator_current_limit_enable = True

        self.shooter_motors_config.slot0 = (
            self.shooter_profile.create_ctre_flywheel_controller()
        )

        self.left_motor.configurator.apply(self.shooter_motors_config)
        self.right_motor.configurator.apply(self.shooter_motors_config)

        self.shooter_control = (
            controls.VelocityVoltage(0).with_enable_foc(True).with_slot(0)
        )
        self.shooter_follower = controls.Follower(
            self.right_motor.device_id, MotorAlignmentValue.OPPOSED
        )

        self.kicker_motor_configs = TalonFXSConfiguration()
        self.kicker_motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.kicker_motor_configs.commutation.motor_arrangement = (
            MotorArrangementValue.NEO550_JST
        )

        self.left_kicker_motor.configurator.apply(self.kicker_motor_configs)
        self.right_kicker_motor.configurator.apply(self.kicker_motor_configs)

        self.conveyor_motor_configs = TalonFXSConfiguration()
        self.conveyor_motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.conveyor_motor_configs.commutation.motor_arrangement = (
            MotorArrangementValue.NEO550_JST
        )

        self.vibrator_motor.configurator.apply(self.conveyor_motor_configs)



    def set_shooter(self, setting: ShooterSettings):
        self.shooter_setting = setting
    def kicker_on(self):
        self.kicker_volts = 8
    def kicker_off(self):
        self.kicker_volts = 0
    def vibrator_on(self):
        self.vibrator_volts = 8
    def vibrator_off(self):
        self.vibrator_volts = 0
    def execute(self):
        self.right_motor.set_control(
            self.shooter_control.with_velocity(self.setting_to_volt[self.shooter_setting])
        )
        self.left_motor.set_control(
            self.shooter_control.with_velocity(-self.setting_to_volt[self.shooter_setting])
        )
        self.right_kicker_motor.set_control(
            controls.VoltageOut(self.kicker_volts)
        )
        self.left_kicker_motor.set_control(
            controls.VoltageOut(-self.kicker_volts)
        )
        self.vibrator_motor.set_control(
            controls.VoltageOut(self.vibrator_volts)
        )