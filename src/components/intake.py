import enum

from magicbot import will_reset_to
from phoenix6 import controls
from phoenix6.configs import (
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


class Intake:
    spin_motor: TalonFX

    left_motor: TalonFXS
    right_motor: TalonFXS

    # left_encoder: SparkAbsoluteEncoder
    # right_encoder: SparkAbsoluteEncoder

    profile: SmartProfile

    spin_amps: units.amperes
    arm_amps: units.amperes

    spin_voltage = will_reset_to(0.0)
    arm_voltage = will_reset_to(0.0)
    arm_manual_control = will_reset_to(False)
    arm_angle = will_reset_to(IntakeAngle.UP.value)

    bypass_limits = will_reset_to(False)

    INTAKEUP = IntakeAngle.UP.value
    INTAKING = IntakeAngle.INTAKING.value

    def setup(self):
        self._cached_angle = 0.0
        spin_config = TalonFXConfiguration()
        spin_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        spin_config.current_limits.supply_current_limit = self.spin_amps
        spin_config.current_limits.supply_current_limit_enable = True
        self.spin_motor.configurator.apply(spin_config)

        self.arm_motor_config = TalonFXSConfiguration()
        self.arm_motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE

        self.arm_motor_config.current_limits.stator_current_limit = self.arm_amps
        self.arm_motor_config.current_limits.stator_current_limit_enable = True
        self.arm_motor_config.slot0 = self.profile.create_ctre_pid_controller()

        self.arm_motor_config.commutation.motor_arrangement = (
            MotorArrangementValue.NEO550_JST
        )

        self.left_motor.configurator.apply(self.arm_motor_config)
        self.right_motor.configurator.apply(self.arm_motor_config)

        self.spin_control = controls.VoltageOut(0)
        self.arm_voltage_control = controls.VoltageOut(0)
        self.arm_position_control = controls.PositionVoltage(0).with_slot(0)
        self.arm_follower = controls.Follower(
            self.right_motor.device_id, MotorAlignmentValue.OPPOSED
        )
        self.coast_control = controls.CoastOut()

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

    def on_enable(self):
        self.arm_motor_config.slot0 = self.profile.create_ctre_pid_controller()
        self.left_motor.configurator.apply(self.arm_motor_config)
        self.right_motor.configurator.apply(self.arm_motor_config)

    """
    INFORMATIONAL METHODS
    """

    def get_left_angle(self) -> units.degrees:
        return self.left_motor.get_position().value

    def get_right_angle(self) -> units.degrees:
        return self.right_motor.get_position().value

    # @feedback
    def get_position(self) -> float:
        """Return the pos of the hinge normalized to [-0.5,0.5].
        An angle of 0 refers to the intake in the up/stowed position.
        """
        right_angle = self.get_right_angle()
        # left_angle = self.get_left_angle()
        pos = right_angle
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

    def execute(self):
        # thing so that if batt low we can turn off to save energy
        if not self.component_enabled:
            self.spin_motor.set_control(self.coast_control)
            self.right_motor.set_control(self.coast_control)
            self.left_motor.set_control(self.coast_control)
            return

        pos = self.get_position()

        if not self.arm_manual_control and self.arm_angle != self.prev_arm_voltage:
            self.prev_arm_voltage = self.arm_angle
            self.right_motor.set_control(
                self.arm_position_control.with_position(self.arm_angle)
            )
            self.left_motor.set_control(self.arm_follower)

        # making sure we don't try to move the arm past its limits or break
        if pos > (self.INTAKEUP + 0.1):
            if not self.bypass_limits:
                self.arm_voltage = max(self.arm_voltage, 0)

            self.hinge_alert.set_text(
                f"Intake hinge has rotated too far! Position: {pos:.2f}"
            )
            self.hinge_alert.enable()
        elif pos < (self.INTAKING - 0.1):
            if not self.bypass_limits:
                self.arm_voltage = min(self.arm_voltage, 0)

            self.hinge_alert.set_text(
                f"Intake hinge has rotated too far! Position: {pos:.2f}"
            )
            self.hinge_alert.enable()

        if self.arm_manual_control and self.arm_voltage != self.prev_arm_voltage:
            self.prev_arm_voltage = self.arm_voltage
            self.right_motor.set_control(
                self.arm_voltage_control.with_output(self.arm_voltage)
            )
            self.left_motor.set_control(self.arm_follower)

        if self.spin_voltage != self.prev_spin_voltage:
            self.prev_spin_voltage = self.spin_voltage
            self.spin_motor.set_control(
                self.spin_control.with_output(self.spin_voltage)
            )
