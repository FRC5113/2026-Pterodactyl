from magicbot import feedback, will_reset_to
from phoenix6 import controls
from phoenix6.configs import (
    FeedbackConfigs,
    TalonFXConfiguration,
)
from phoenix6.hardware import TalonFX
from phoenix6.signals import (
    FeedbackSensorSourceValue,
    MotorAlignmentValue,
    NeutralModeValue,
)
from wpimath import units

from lemonlib.smart import SmartProfile


class Shooter:
    """low level component that directly manages the shooter motors and their configuration. controlled by the shooter controller component, but can also be directly controlled for testing purposes"""

    right_motor: TalonFX
    left_motor: TalonFX

    shooter_profile: SmartProfile
    shooter_gear_ratio: float
    shooter_amps: units.amperes
    tuning_enabled: bool

    shooter_velocity = will_reset_to(0.0)
    shooter_voltage = will_reset_to(0.0)

    manual_control = will_reset_to(False)

    def setup(self):
        self._cached_velocity = 0.0
        self._shooter_follower_set = False

        self._configure_motors()

        self.shooter_control = (
            controls.VelocityVoltage(0).with_slot(0).with_enable_foc(True)
        )
        self.shooter_follower = controls.Follower(
            self.right_motor.device_id, MotorAlignmentValue.OPPOSED
        )

        self.voltage_control = controls.VoltageOut(0)
        self.coast_control = controls.CoastOut()

        self.prev_shooter_control = 0.0

        self.component_enabled = True

    def _configure_motors(self):
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

    def on_enable(self):
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

    """
    CONTROL METHODS
    """

    def set_velocity(self, speed: float):
        self.manual_control = False
        self.shooter_velocity = speed

    def set_voltage(self, volts: units.volts):
        self.manual_control = True
        self.shooter_voltage = volts

    def turn_off_component(self):
        self.component_enabled = False

    def turn_on_component(self):
        self.component_enabled = True

    """
    INFORMATIONAL METHODS
    """

    @feedback
    def get_velocity(self) -> float:
        return self._cached_velocity

    # @feedback
    def get_target_velocity(self) -> float:
        return self.shooter_velocity

    def execute(self):
        # thing so that if batt low we can turn off to save energy
        if not self.component_enabled:
            self.right_motor.set_control(self.coast_control)
            self.left_motor.set_control(self.coast_control)
            return

        # Cache velocity once per cycle for feedback and shooter_controller use
        self._cached_velocity = self.left_motor.get_velocity().value

        if self.manual_control:
            shooter_voltage = self.shooter_voltage
            if shooter_voltage != self.prev_shooter_control:
                self.prev_shooter_control = shooter_voltage
                self.right_motor.set_control(
                    self.voltage_control.with_output(shooter_voltage)
                )
        else:
            shooter_velocity = self.shooter_velocity
            if shooter_velocity != self.prev_shooter_control:
                self.prev_shooter_control = shooter_velocity
                self.right_motor.set_control(
                    self.shooter_control.with_velocity(shooter_velocity)
                )
                self.left_motor.set_control(self.shooter_follower)
