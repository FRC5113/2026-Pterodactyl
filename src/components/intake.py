import enum

from magicbot import will_reset_to
from phoenix6 import controls
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.signals import (
    NeutralModeValue,
)
from rev import PersistMode, ResetMode, SparkAbsoluteEncoder, SparkMax, SparkMaxConfig
from wpimath import units

from lemonlib.smart import SmartProfile
from lemonlib.util import Alert, AlertType


class IntakeAngle(enum.Enum):
    UP = 0.8
    INTAKING = 0.55


class Intake:
    spin_motor: TalonFX

    left_motor: SparkMax
    right_motor: SparkMax

    left_encoder: SparkAbsoluteEncoder
    right_encoder: SparkAbsoluteEncoder

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
        self.spin_motor.configurator.apply(spin_config)

        self.arm_config = SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        self.right_motor.configure(
            self.arm_config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        self.left_motor.configure(
            self.arm_config.follow(self.right_motor, True),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        self.spin_control = controls.VoltageOut(0)

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

    def on_enable(self):
        self.controller = self.profile.create_arm_controller("intake_arm")

    """
    INFORMATIONAL METHODS
    """

    def get_left_angle(self) -> units.degrees:
        return self.left_encoder.getPosition()

    def get_right_angle(self) -> units.degrees:
        return self.right_encoder.getPosition()

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

    def set_arm_voltage(self, volts):
        self.arm_voltage = volts
        self.arm_manual_control = True

    def set_arm_angle(self, angle: units.degrees):
        self.arm_angle = angle

    def set_bypass_limits(self):
        self.bypass_limits = True

    def execute(self):
        pos = self.get_position()

        # if not self.arm_manual_control:
        #     self.arm_voltage = self.controller.calculate(pos, self.arm_angle)

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

        if self.arm_voltage != self.prev_arm_voltage:
            self.prev_arm_voltage = self.arm_voltage
            self.right_motor.setVoltage(self.arm_voltage)

        if self.spin_voltage != self.prev_spin_voltage:
            self.prev_spin_voltage = self.spin_voltage
            self.spin_motor.set_control(
                self.spin_control.with_output(self.spin_voltage)
            )
