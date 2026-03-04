from magicbot import will_reset_to
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
from wpilib import RobotBase
from wpimath import units

from lemonlib import fms_feedback # type: ignore Lemon
from lemonlib.smart import SmartProfile


class Shooter:
    right_motor: TalonFX
    left_motor: TalonFX
    left_kicker_motor: TalonFXS
    right_kicker_motor: TalonFXS
    conveyor_motor: TalonFXS

    shooter_profile: SmartProfile
    shooter_gear_ratio: float
    shooter_amps: units.amperes
    tuning_enabled: bool

    shooter_velocity = will_reset_to(0.0)
    shooter_voltage = will_reset_to(0.0)
    kicker_duty = will_reset_to(0.0)
    manual_control = will_reset_to(False)

    def setup(self):
        """Initialize shooter hardware and configuration."""
        self._cached_velocity = 0.0
        
        # Use short timeout in simulation to avoid blocking
        timeout = 0.05 if RobotBase.isSimulation() else 0.5

        self.shooter_motors_config = TalonFXConfiguration()
        self.shooter_motors_config.motor_output.neutral_mode = NeutralModeValue.COAST

        self.shooter_motors_config.feedback = (
            FeedbackConfigs()
            .with_feedback_sensor_source(FeedbackSensorSourceValue.ROTOR_SENSOR)
            .with_sensor_to_mechanism_ratio(self.shooter_gear_ratio)
        )

        self.shooter_motors_config.current_limits.stator_current_limit = self.shooter_amps
        self.shooter_motors_config.current_limits.stator_current_limit_enable = True

        self.left_motor.configurator.apply(self.shooter_motors_config, timeout)
        self.right_motor.configurator.apply(self.shooter_motors_config, timeout)

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

        self.left_kicker_motor.configurator.apply(self.kicker_motor_configs, timeout)
        self.right_kicker_motor.configurator.apply(self.kicker_motor_configs, timeout)

        self.conveyor_motor_configs = TalonFXSConfiguration()
        self.conveyor_motor_configs.motor_output.neutral_mode = NeutralModeValue.COAST
        self.conveyor_motor_configs.commutation.motor_arrangement = (
            MotorArrangementValue.NEO550_JST
        )

        self.conveyor_motor.configurator.apply(self.conveyor_motor_configs, timeout)

        self.voltage_control = controls.VoltageOut(0).with_enable_foc(True)
        self.duty_control = controls.DutyCycleOut(0).with_enable_foc(True)
        self.kicker_follower = controls.Follower(
            self.right_kicker_motor.device_id, MotorAlignmentValue.OPPOSED
        )

    def on_enable(self):
        """Apply tuned PID/feedforward gains when robot is enabled."""
        timeout = 0.05 if RobotBase.isSimulation() else 0.5
        if self.tuning_enabled:
            self.shooter_controller = (
                self.shooter_profile.create_ctre_flywheel_controller()
            )
            self.left_motor.configurator.apply(
                self.shooter_motors_config.with_slot0(self.shooter_controller), timeout
            )
            self.right_motor.configurator.apply(
                self.shooter_motors_config.with_slot0(self.shooter_controller), timeout
            )

    def set_velocity(self, speed: float):
        """Set target flywheel velocity with closed-loop control."""
        self.manual_control = False
        self.shooter_velocity = speed

    def set_voltage(self, volts: units.volts):
        """Set flywheel voltage directly for open-loop control."""
        self.manual_control = True
        self.shooter_voltage = volts

    def set_kicker(self, value: float):
        """Set kicker/indexer motor voltage."""
        self.kicker_duty = value

    def get_velocity(self) -> float:
        """Get current flywheel velocity."""
        return self._cached_velocity

    @fms_feedback
    def get_target_velocity(self) -> float:
        """Get target flywheel velocity."""
        return self.shooter_velocity

    def execute(self):
        """Execute shooter control loop."""
        self._cached_velocity = self.left_motor.get_velocity().value

        self.right_kicker_motor.set_control(
            self.voltage_control.with_output(self.kicker_duty)
        )
        self.left_kicker_motor.set_control(self.kicker_follower)
        self.conveyor_motor.set_control(self.kicker_follower)

        if self.manual_control:
            self.right_motor.set_control(
                self.voltage_control.with_output(self.shooter_voltage)
            )
        else:
            self.right_motor.set_control(
                self.shooter_control.with_velocity(self.shooter_velocity)
            )
        self.left_motor.set_control(self.shooter_follower)
