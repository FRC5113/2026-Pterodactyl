from magicbot import feedback, will_reset_to
from phoenix6 import controls
from phoenix6.configs import (
    FeedbackConfigs,
    Slot1Configs,
    TalonFXConfiguration,
)
from phoenix6.hardware import TalonFX
from phoenix6.signals import (
    FeedbackSensorSourceValue,
    MotorAlignmentValue,
    NeutralModeValue,
    InvertedValue,
)
from wpimath import units

from lemonlib.smart import SmartProfile


class Shooter:
    """Low-level shooter motor control with fast recovery tuning"""

    right_motor: TalonFX
    left_motor: TalonFX

    shooter_profile: SmartProfile
    shooter_gear_ratio: float
    shooter_stator_amps: units.amperes
    shooter_supply_amps: units.amperes
    shooter_peak_amps: units.amperes
    tuning_enabled: bool

    shooter_velocity = will_reset_to(0.0)
    shooter_voltage = will_reset_to(0.0)
    shooter_acceleration = will_reset_to(0.0)
    shooter_slot = will_reset_to(0)

    manual_control = will_reset_to(False)

    def setup(self):
        self._cached_velocity = 0.0
        self._last_velocity = 0.0
        self._boost_timer = 0

        self._configure_motors()

        self.shooter_control = controls.VelocityVoltage(0).with_slot(0)

        # follower (set once)
        self.shooter_follower = controls.Follower(
            self.left_motor.device_id, MotorAlignmentValue.OPPOSED
        )
        self.right_motor.set_control(self.shooter_follower)

        self.voltage_control = controls.VoltageOut(0)
        self.coast_control = controls.CoastOut()

        self.prev_shooter_control = None
        self.component_enabled = True

    def _configure_motors(self):
        self.slot0 = self.shooter_profile.create_ctre_flywheel_controller()
        self.slot1 = (
            Slot1Configs()
            .with_k_p(self.slot0.k_p + 0.1)
            .with_k_v(self.slot0.k_v)
            .with_k_a(self.slot0.k_a)
        )

        config = TalonFXConfiguration()

        config.motor_output.neutral_mode = NeutralModeValue.COAST
        config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        config.feedback = (
            FeedbackConfigs()
            .with_feedback_sensor_source(FeedbackSensorSourceValue.ROTOR_SENSOR)
            .with_sensor_to_mechanism_ratio(self.shooter_gear_ratio)
        )

        # Current limits
        config.current_limits.stator_current_limit = self.shooter_stator_amps
        config.current_limits.stator_current_limit_enable = True

        config.current_limits.supply_current_limit = self.shooter_supply_amps
        config.current_limits.supply_current_limit_enable = True

        config.torque_current.peak_forward_torque_current = self.shooter_peak_amps
        config.torque_current.peak_reverse_torque_current = -self.shooter_peak_amps

        config.slot0 = self.slot0
        config.slot1 = self.slot1

        # mm = config.motion_magic
        # mm.motion_magic_acceleration = 1000
        # mm.motion_magic_jerk = 15000

        self.right_motor.configurator.apply(config)
        self.left_motor.configurator.apply(config)

        self._base_config = config  # save for dynamic updates

    def on_enable(self):
        if self.tuning_enabled:
            self.slot0 = self.shooter_profile.create_ctre_flywheel_controller()
            self.slot1 = (
                Slot1Configs()
                .with_k_p(self.slot0.k_p + 0.1)
                .with_k_v(self.slot0.k_v)
                .with_k_a(self.slot0.k_a)
            )

            self._base_config.slot0 = self.slot0
            self._base_config.slot1 = self.slot1
            self.right_motor.configurator.apply(self._base_config)
            self.left_motor.configurator.apply(self._base_config)

    """
    CONTROL METHODS
    """

    def set_velocity(self, speed: float):
        self.manual_control = False
        self.shooter_velocity = speed

    def set_voltage(self, volts: units.volts):
        self.manual_control = True
        self.shooter_voltage = volts

    def set_acceleration(self, accel: float):
        self.shooter_acceleration = accel

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

    def get_target_velocity(self) -> float:
        return self.shooter_velocity

    def execute(self):
        if not self.component_enabled:
            self.right_motor.set_control(self.coast_control)
            self.left_motor.set_control(self.coast_control)
            return

        # cache velocity
        self._cached_velocity = self.left_motor.get_velocity().value
        # velocity = self._cached_velocity

        # # Shot detection (velocity drop)
        # if velocity < self._last_velocity - 5:  # tune threshold
        #     self._boost_timer = 10  # ~200ms boost

        # self._last_velocity = velocity
        # if self._boost_timer > 0:
        #     self._boost_timer -= 1
        #     self.shooter_slot = 1

        if self.manual_control:
            self.left_motor.set_control(
                self.voltage_control.with_output(self.shooter_voltage)
            )
        else:
            self.left_motor.set_control(
                self.shooter_control.with_velocity(self.shooter_velocity)
            )
