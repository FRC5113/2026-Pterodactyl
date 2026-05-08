import enum

from magicbot import will_reset_to
from wpimath import units

from lemonlib.smart import SmartProfile
from lemonlib.util import Alert, AlertType
from phoenix6 import controls
from phoenix6.configs import (
    TalonFXConfiguration,
    TalonFXSConfiguration,
)
from phoenix6.hardware import TalonFX, TalonFXS
from phoenix6.signals import (
    MotorArrangementValue,
    NeutralModeValue,
)


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

    spin_amps: units.amperes
    arm_amps: units.amperes

    spin_voltage = will_reset_to(0.0)
    arm_voltage = will_reset_to(0.0)
    arm_manual_control = will_reset_to(False)
    arm_angle = will_reset_to(IntakeAngle.UP.value)

    def setup(self):
        self._cached_angle = 0.0

        # self._config_cancoder()
        self._config_spin_motor()
        self._config_arm_motors()

        # Controls
        self.spin_control = controls.VoltageOut(0)
        self.arm_voltage_control = controls.VoltageOut(0)
        self.arm_position_control = controls.PositionVoltage(0).with_slot(0)
        self.arm_follower = controls.Follower(self.right_motor.device_id, False)
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

    def _config_spin_motor(self):
        # Configure motors
        spin_config = TalonFXConfiguration()
        spin_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        spin_config.current_limits.supply_current_limit = self.spin_amps
        spin_config.current_limits.supply_current_limit_enable = True
        self.spin_motor.configurator.apply(spin_config)

    def _config_arm_motors(self):
        self.arm_motor_config = TalonFXSConfiguration()

        self.arm_motor_config.current_limits.stator_current_limit = self.arm_amps

        self.arm_motor_config.commutation.motor_arrangement = (
            MotorArrangementValue.BRUSHED_DC
        )

        self.left_motor.configurator.apply(self.arm_motor_config)
        self.right_motor.configurator.apply(self.arm_motor_config)

    def on_enable(self):
        self.left_motor.set_control(self.arm_follower)

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

    def execute(self):

        if self.spin_voltage != self.prev_spin_voltage:
            self.prev_spin_voltage = self.spin_voltage
            self.spin_motor.set_control(
                self.spin_control.with_output(self.spin_voltage)
            )

        self.right_motor.set_control(
            self.arm_voltage_control.with_output(self.arm_voltage)
        )
        self.arm_voltage = 0.0
