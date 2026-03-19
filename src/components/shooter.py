from magicbot import feedback, will_reset_to
from phoenix6 import controls
from phoenix6.configs import (
    FeedbackConfigs,
    TalonFXConfiguration,
    TorqueCurrentConfigs,
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

    manual_control = will_reset_to(False)

    def setup(self):
        self._cached_velocity = 0.0
        self._last_velocity = 0.0
        self._boost_timer = 0

        self._configure_motors()

        self.shooter_control = controls.MotionMagicVelocityTorqueCurrentFOC(
            0
        ).with_slot(0)

        # follower (set once)
        self.shooter_follower = controls.Follower(
            self.right_motor.device_id, MotorAlignmentValue.OPPOSED
        )
        self.left_motor.set_control(self.shooter_follower)

        self.voltage_control = controls.VoltageOut(0)
        self.coast_control = controls.CoastOut()

        self.prev_shooter_control = None
        self.component_enabled = True

    def _configure_motors(self):
        config = TalonFXConfiguration()

        config.motor_output.neutral_mode = NeutralModeValue.COAST

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

        config.slot0 = self.shooter_profile.create_ctre_flywheel_controller()

        mm = config.motion_magic
        mm.motion_magic_acceleration = 1000
        mm.motion_magic_jerk = 15000

        self.right_motor.configurator.apply(config)
        self.left_motor.configurator.apply(config)

        self._base_config = config  # save for dynamic updates

    def on_enable(self):
        if self.tuning_enabled:
            controller = self.shooter_profile.create_ctre_flywheel_controller()

            self.right_motor.configurator.apply(
                self._base_config.with_slot0(controller)
            )
            self.left_motor.configurator.apply(self._base_config.with_slot0(controller))

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
        self._cached_velocity = self.right_motor.get_velocity().value
        velocity = self._cached_velocity

        # Shot detection (velocity drop)
        if velocity < self._last_velocity - 5:  # tune threshold
            self._boost_timer = 10  # ~200ms boost

        self._last_velocity = velocity

        # Dynamic torque boost
        if self._boost_timer > 0:
            self._boost_timer -= 1
            boosted_torque = TorqueCurrentConfigs()
            boosted_torque.peak_forward_torque_current = self.shooter_peak_amps + 20
            boosted_torque.peak_reverse_torque_current = -(self.shooter_peak_amps + 20)

            self.right_motor.configurator.apply(
                self._base_config.with_torque_current(boosted_torque)
            )
        else:
            normal_torque = TorqueCurrentConfigs()
            normal_torque.peak_forward_torque_current = self.shooter_peak_amps
            normal_torque.peak_reverse_torque_current = -self.shooter_peak_amps

            self.right_motor.configurator.apply(
                self._base_config.with_torque_current(normal_torque)
            )

        if self.manual_control:
            if self.shooter_voltage != self.prev_shooter_control:
                self.prev_shooter_control = self.shooter_voltage
                self.right_motor.set_control(
                    self.voltage_control.with_output(self.shooter_voltage)
                )
        else:
            if self.shooter_velocity != self.prev_shooter_control:
                self.prev_shooter_control = self.shooter_velocity
                self.right_motor.set_control(
                    self.shooter_control.with_velocity(
                        self.shooter_velocity
                    ).with_acceleration(self.shooter_acceleration)
                )
