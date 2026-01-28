import math

from phoenix6 import controls, StatusSignal
from phoenix6.configs import (
    ClosedLoopGeneralConfigs,
    FeedbackConfigs,
    TalonFXConfiguration,
    TalonFXSConfiguration,
    CANcoderConfiguration,
)
from phoenix6.hardware import CANcoder, TalonFX, TalonFXS
from phoenix6.signals import (
    FeedbackSensorSourceValue,
    NeutralModeValue,
    SensorDirectionValue,
    MotorAlignmentValue,
    MotorArrangementValue,
    ExternalFeedbackSensorSourceValue,
)
from wpimath import units
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpiutil import Sendable

from magicbot import will_reset_to
from lemonlib.smart import SmartNT, SmartPreference, SmartProfile


class Shooter:
    right_motor: TalonFX
    left_motor: TalonFX
    hood_motor: TalonFXS
    hood_profile: SmartProfile
    shooter_profile: SmartProfile
    shooter_gear_ratio: float
    shooter_hood_gear_ratio: float
    shooter_amps: units.amperes
    tuning_enabled: bool

    shooter_velocity = will_reset_to(0.0)
    shooter_angle = will_reset_to(0.0)

    def setup(self):
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

        self.left_motor.configurator.apply(self.shooter_motors_config)
        self.right_motor.configurator.apply(self.shooter_motors_config)

        self.shooter_control = (
            controls.VelocityVoltage(0).with_enable_foc(True).with_slot(0)
        )
        self.shooter_follower = controls.Follower(
            self.left_motor.device_id, MotorAlignmentValue.OPPOSED
        )

        self.hood_motor_config = TalonFXSConfiguration()
        self.hood_motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.hood_motor_config.commutation.motor_arrangement = (
            MotorArrangementValue.NEO550_JST
        )
        self.hood_motor_config.external_feedback.external_feedback_sensor_source = (
            ExternalFeedbackSensorSourceValue.QUADRATURE
        )

        self.hood_motor.configurator.apply(self.hood_motor_config)

    def set_velocity(self, speed: float):
        self.shooter_velocity = speed

    def set_shoot_angle(self, angle: units.degrees):
        self.shooter_angle = angle

    def on_enable(self):
        self.hood_controller = self.hood_profile.create_turret_controller("Hood")
        if self.tuning_enabled:
            self.shooter_controller = (
                self.shooter_profile.create_ctre_flywheel_controller()
            )
            self.left_motor.configurator.apply(
                self.shooter_motors_config.with_slot0(self.shooter_controller)
            )
            self.right_motor.configurator.apply(
                self.shooter_motors_config.with_slot0(self.shooter_controller)
            )

    def execute(self):
        hood_output = self.hood_controller.calculate(
            self.shooter_angle, self.hood_motor.get_position().value
        )
        self.hood_motor.set_control(
            controls.VoltageOut(hood_output).with_enable_foc(True)
        )

        self.left_motor.set_control(
            self.shooter_control.with_velocity(self.shooter_velocity)
        )
        self.right_motor.set_control(self.shooter_follower)
