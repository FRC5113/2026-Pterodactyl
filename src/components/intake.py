import enum

from magicbot import will_reset_to
from phoenix6 import controls
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.signals import (
    MotorAlignmentValue,
    NeutralModeValue,
)
from wpilib import DutyCycleEncoder
from wpimath import units

from lemonlib import fms_feedback
from lemonlib.smart import SmartProfile


class IntakeAngle(enum.Enum):
    UP = 0.8
    INTAKING = 0.55


class Intake:
    spin_motor: TalonFX

    left_motor: TalonFX
    right_motor: TalonFX

    left_encoder: DutyCycleEncoder
    right_encoder: DutyCycleEncoder

    profile: SmartProfile

    spin_amps: units.amperes
    arm_amps: units.amperes

    spin_voltage = will_reset_to(0.0)
    arm_voltage = will_reset_to(0.0)
    arm_angle = will_reset_to(IntakeAngle.UP.value)

    INTAKEUP = IntakeAngle.UP.value
    INTAKING = IntakeAngle.INTAKING.value

    def setup(self):
        self._cached_angle = 0.0
        spin_config = TalonFXConfiguration()
        spin_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        spin_config.current_limits.supply_current_limit = self.spin_amps
        self.spin_motor.configurator.apply(spin_config)

        arm_config = TalonFXConfiguration()
        arm_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        arm_config.current_limits.supply_current_limit = self.arm_amps
        self.left_motor.configurator.apply(arm_config)
        self.right_motor.configurator.apply(arm_config)

        self.arm_control = controls.VoltageOut(0).with_enable_foc(True)
        self.arm_follower = controls.Follower(
            self.right_motor.device_id, MotorAlignmentValue.OPPOSED
        )

        self.spin_control = controls.VoltageOut(0).with_enable_foc(True)

        self.right_encoder.setInverted(True)

        self.right_offset = 0.0#-0.21
        self.left_offset = 0.0#0.48

    def on_enable(self):
        self.controller = self.profile.create_arm_controller("intake_arm")

    """
    INFORMATIONAL METHODS
    """
    @fms_feedback
    def get_left_angle(self) -> units.degrees:
        pos = (self.left_encoder.get() - self.left_offset) 
        if pos < 0.0:
            pos += 1.0
        elif pos >= 1.0:
            pos -= 1.0
        return pos
    @fms_feedback
    def get_right_angle(self) -> units.degrees:
        pos = (self.right_encoder.get() - self.right_offset)
        if pos < 0.0:
            pos += 1.0
        elif pos >= 1.0:
            pos -= 1.0
        return pos

    @fms_feedback
    def get_position(self) -> float:
        # return (self.get_left_angle() + self.get_right_angle()) / 2
        return self.get_left_angle()

    """
    CONTROL METHODS
    """

    def set_voltage(self, voltage: units.volts):
        self.spin_voltage = voltage

    def set_arm_voltage(self, volts):
        self.arm_voltage = volts

    def set_arm_angle(self, angle: units.degrees):
        self.arm_angle = angle

    def execute(self):
        # Cache angle once per cycle for feedback and control use
        pos = self.get_position()
        # self.arm_voltage = self.controller.calculate(angle, self.arm_angle)

        # making sure we don't try to move the arm past its limits
        if pos > self.INTAKEUP:
            self.arm_voltage = max(self.arm_voltage, 0)
        elif pos < self.INTAKING:
            self.arm_voltage = min(self.arm_voltage, 0)

        self.right_motor.set_control(self.arm_control.with_output(self.arm_voltage))
        self.left_motor.set_control(self.arm_follower)
        self.spin_motor.set_control(self.spin_control.with_output(self.spin_voltage))
