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
from wpilib import DutyCycleEncoder, RobotBase
from wpimath import units

from lemonlib import fms_feedback # type: ignore
from lemonlib.smart import SmartPreference, SmartProfile
from lemonlib.util import Alert, AlertType


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
    arm_manual_control = will_reset_to(False)
    arm_angle = will_reset_to(IntakeAngle.UP.value)

    INTAKEUP = IntakeAngle.UP.value
    INTAKING = IntakeAngle.INTAKING.value

    right_offset = SmartPreference(0.79)
    left_offset = SmartPreference(0.48)

    def setup(self):
        """Initialize intake hardware and configuration."""
        self._cached_angle = 0.0
        
        # Use short timeout in simulation to avoid blocking
        timeout = 0.05 if RobotBase.isSimulation() else 0.5

        spin_config = TalonFXConfiguration()
        spin_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        spin_config.current_limits.supply_current_limit = self.spin_amps
        self.spin_motor.configurator.apply(spin_config, timeout)

        arm_config = TalonFXConfiguration()
        arm_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        arm_config.current_limits.supply_current_limit = self.arm_amps
        self.left_motor.configurator.apply(arm_config, timeout)
        self.right_motor.configurator.apply(arm_config, timeout)

        self.arm_control = controls.VoltageOut(0).with_enable_foc(True)
        self.arm_follower = controls.Follower(
            self.right_motor.device_id, MotorAlignmentValue.OPPOSED
        )
        self.spin_control = controls.VoltageOut(0).with_enable_foc(True)

        self.right_encoder.setInverted(True)

        self.hinge_alert = Alert(
            "intake hinge has rotated too far!", type=AlertType.ERROR
        )
        self.break_alert = Alert(
            "intake arm may be breaking! Check for mechanical issues.",
            type=AlertType.ERROR,
        )

    def on_enable(self):
        """Create arm controller when robot is enabled."""
        self.controller = self.profile.create_arm_controller("intake_arm")

    def get_left_angle(self) -> units.degrees:
        """Get left arm encoder position adjusted by calibration offset."""
        pos = self.left_encoder.get() - float(self.left_offset) # type: ignore
        if pos < 0.0:
            pos += 1.0
        elif pos >= 1.0:
            pos -= 1.0
        return pos

    def get_right_angle(self) -> units.degrees:
        """Get right arm encoder position adjusted by calibration offset."""
        pos = self.right_encoder.get() - float(self.right_offset) # type: ignore
        if pos < 0.0:
            pos += 1.0
        elif pos >= 1.0:
            pos -= 1.0
        return pos

    @fms_feedback
    def get_position(self) -> float:
        """Get average arm position from both encoders."""
        return (self.get_left_angle() + self.get_right_angle()) / 2

    def zero_encoders(self):
        """Recalibrate encoder zero positions to current arm position."""
        self.left_offset = self.left_encoder.get() + 0.5
        self.right_offset = self.right_encoder.get() + 0.5

    def set_voltage(self, voltage: units.volts) -> None:
        """Set spin motor voltage directly."""
        self.spin_voltage = voltage

    def set_arm_voltage(self, voltage: units.volts) -> None:
        """Manually control arm position with voltage bypassing PID."""
        self.arm_voltage = voltage
        self.arm_manual_control = True

    def set_arm_angle(self, angle: units.degrees):
        """Set target arm angle for PID control."""
        self.arm_angle = angle

    def execute(self):
        """Execute intake control loop with safety checks."""
        pos = float(self.get_position()) # type: ignore

        if not self.arm_manual_control:
            self.arm_voltage = self.controller.calculate(pos, self.arm_angle) # type: ignore

        if abs(self.get_right_angle() - self.get_left_angle()) > 0.2:
            self.arm_voltage = 0.0
            self.break_alert.enable()
            self.break_alert.set_text(
                f"Intake arm may be breaking! Left: {self.get_left_angle():.2f}, Right: {self.get_right_angle():.2f}"
            )

        if pos > self.INTAKEUP:
            self.arm_voltage = max(self.arm_voltage, 0) # type: ignore
            self.hinge_alert.enable()
            self.hinge_alert.set_text(
                f"Intake hinge has rotated too far! Position: {pos:.2f}"
            )
        elif pos < self.INTAKING:
            self.arm_voltage = min(self.arm_voltage, 0) # type: ignore
            self.hinge_alert.enable()
            self.hinge_alert.set_text(
                f"Intake hinge has rotated too far! Position: {pos:.2f}"
            )

        self.right_motor.set_control(self.arm_control.with_output(self.arm_voltage)) # type: ignore
        self.left_motor.set_control(self.arm_follower)
        self.spin_motor.set_control(self.spin_control.with_output(self.spin_voltage))
