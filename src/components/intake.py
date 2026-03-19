import enum

from magicbot import will_reset_to
from phoenix6 import controls
from phoenix6.configs import (
    CANcoderConfiguration,
    TalonFXConfiguration,
    TalonFXSConfiguration,
)
from phoenix6.hardware import TalonFX, TalonFXS
from phoenix6.signals import (
    MotorAlignmentValue,
    MotorArrangementValue,
    NeutralModeValue,
)
from wpimath import units

from lemonlib.smart import SmartProfile
from lemonlib.util import Alert, AlertType


class IntakeAngle(enum.Enum):
    UP = 0.8
    INTAKING = 0.55
    HARDUP = 0.85
    HARDDOWN = 0.5
    LED_DOWN = 0.6


class Intake:
    spin_motor: TalonFX

    left_motor: TalonFXS
    right_motor: TalonFXS

    # encoder: CANcoder

    profile: SmartProfile

    spin_amps: units.amperes
    arm_amps: units.amperes
    hard_stop_amps: units.amperes

    encoder_offset: float

    tuning_enabled: bool

    spin_voltage = will_reset_to(0.0)
    arm_voltage = will_reset_to(0.0)
    arm_manual_control = will_reset_to(False)
    arm_angle = will_reset_to(IntakeAngle.UP.value)

    rev_lim = will_reset_to(False)
    fwd_lim = will_reset_to(False)

    bypass_limits = will_reset_to(False)

    INTAKEUP = IntakeAngle.UP.value
    INTAKING = IntakeAngle.INTAKING.value
    HARDUP = IntakeAngle.HARDUP.value
    HARDDOWN = IntakeAngle.HARDDOWN.value

    def setup(self):
        self._cached_angle = 0.0

        self._config_cancoder()
        self._config_spin_motor()
        self._config_arm_motors()

        # Controls
        self.spin_control = controls.VoltageOut(0)
        self.arm_voltage_control = controls.VoltageOut(0)
        self.arm_position_control = controls.PositionVoltage(0).with_slot(0)
        self.arm_follower = controls.Follower(
            self.right_motor.device_id, MotorAlignmentValue.ALIGNED
        )
        self.coast_control = controls.CoastOut()

        # Alerts
        self.hinge_alert = Alert(
            "intake hinge has rotated too far!", type=AlertType.WARNING
        )

        self.break_alert = Alert(
            "intake arm may be breaking! Check for mechanical issues.",
            type=AlertType.ERROR,
        )

        self.bypass_alert = Alert(
            "Bypassing intake limits!",
            type=AlertType.WARNING,
        )

        self.prev_arm_voltage = 0.0
        self.prev_spin_voltage = 0.0

        self.component_enabled = True

    def _config_cancoder(self):
        encoder_config = CANcoderConfiguration()
        encoder_config.magnet_sensor.absolute_sensor_discontinuity_point = 0.5
        encoder_config.magnet_sensor.magnet_offset = self.encoder_offset

    def _config_spin_motor(self):
        # Configure motors
        spin_config = TalonFXConfiguration()
        spin_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        spin_config.current_limits.supply_current_limit = self.spin_amps
        spin_config.current_limits.supply_current_limit_enable = True
        self.spin_motor.configurator.apply(spin_config)

    def _config_arm_motors(self):
        self.arm_motor_config = TalonFXSConfiguration()
        self.arm_motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE

        self.arm_motor_config.current_limits.stator_current_limit = self.arm_amps
        self.arm_motor_config.current_limits.stator_current_limit_enable = True

        self.arm_motor_config.commutation.motor_arrangement = (
            MotorArrangementValue.BRUSHED_DC
        )
        # self.arm_motor_config.external_feedback.external_feedback_sensor_source = (
        #     ExternalFeedbackSensorSourceValue.REMOTE_CANCODER
        # )
        # self.arm_motor_config.external_feedback.feedback_remote_sensor_id = self.encoder.device_id

        self.arm_motor_config.closed_loop_general.continuous_wrap = True

        self.arm_motor_config.software_limit_switch.forward_soft_limit_threshold = (
            self.HARDUP
        )
        self.arm_motor_config.software_limit_switch.reverse_soft_limit_threshold = (
            self.HARDDOWN
        )
        self.arm_motor_config.software_limit_switch.forward_soft_limit_enable = False
        self.arm_motor_config.software_limit_switch.reverse_soft_limit_enable = False

        self.arm_motor_config.slot0 = self.profile.create_ctre_arm_controller()

        self.left_motor.configurator.apply(self.arm_motor_config)
        self.right_motor.configurator.apply(self.arm_motor_config)

    def on_enable(self):
        if self.tuning_enabled:
            self.arm_motor_config.slot0 = self.profile.create_ctre_arm_controller()
            self.left_motor.configurator.apply(self.arm_motor_config)
            self.right_motor.configurator.apply(self.arm_motor_config)

    """
    INFORMATIONAL METHODS
    """

    def get_encoder_reg(self):
        return self.right_motor.get_position().value

    # @feedback
    def get_position(self) -> float:
        """Return the pos of the hinge normalized to [-0.5,0.5].
        An angle of 0 refers to the intake in the up/stowed position.
        """
        pos = self.get_encoder_reg()
        if pos > 0.5:
            pos -= 1
        return pos

    """
    CONTROL METHODS
    """

    def set_voltage(self, voltage: units.volts):
        self.spin_voltage = voltage

    def set_arm_voltage(self, volts: units.volts):
        self.arm_voltage = volts
        self.arm_manual_control = True

    def set_arm_angle(self, angle: units.degrees):
        self.arm_angle = angle
        self.arm_manual_control = False

    def set_bypass_limits(self):
        self.bypass_limits = True

    def turn_off_component(self):
        self.component_enabled = False

    def turn_on_component(self):
        self.component_enabled = True

    """
    INFORMATIONAL METHODS
    """

    def get_left_current(self):
        return self.left_motor.get_stator_current()

    def get_right_current(self):
        return self.right_motor.get_stator_current()

    def get_avg_current(self):
        return (self.get_left_current().value + self.get_right_current().value) / 2

    def execute(self):
        # thing so that if batt low we can turn off to save energy
        if not self.component_enabled:
            self.spin_motor.set_control(self.coast_control)
            self.right_motor.set_control(self.coast_control)
            self.left_motor.set_control(self.coast_control)
            return

        if self.spin_voltage != self.prev_spin_voltage:
            self.prev_spin_voltage = self.spin_voltage
            self.spin_motor.set_control(
                self.spin_control.with_output(self.spin_voltage)
            )

        # making sure we don't try to move the arm past its limits or break
        if (
            self.right_motor.get_fault_forward_soft_limit()
            or self.right_motor.get_fault_reverse_soft_limit()
        ):
            self.hinge_alert.set_text(
                f"Intake hinge has rotated too far! Position: {self.get_position():.2f}"
            )
            self.hinge_alert.enable()

        if (
            not self.arm_manual_control
            and self.arm_angle != self.prev_arm_voltage
            and False
        ):
            self.prev_arm_voltage = self.arm_angle
            self.right_motor.set_control(
                self.arm_position_control.with_position(
                    self.arm_angle
                ).with_ignore_software_limits(self.bypass_limits)
            )
            self.left_motor.set_control(self.arm_follower)

        if self.get_avg_current() > self.hard_stop_amps and self.arm_voltage != 0.0:
            if self.arm_voltage > 0.0:
                self.fwd_lim = True
            else:
                self.rev_lim = True

        if self.arm_manual_control and self.arm_voltage != self.prev_arm_voltage:
            self.prev_arm_voltage = self.arm_voltage
            self.right_motor.set_control(
                self.arm_voltage_control.with_output(self.arm_voltage)
                .with_limit_forward_motion(self.fwd_lim)
                .with_limit_reverse_motion(self.rev_lim)
            )
            self.left_motor.set_control(self.arm_follower)
