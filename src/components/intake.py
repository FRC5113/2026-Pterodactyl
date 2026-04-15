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


class Intake:
    spin_motor: TalonFX

    left_motor: TalonFXS
    right_motor: TalonFXS

    spin_amps: units.amperes
    arm_amps: units.amperes

    tuning_enabled: bool

    spin_voltage = will_reset_to(0.0)
    arm_voltage = will_reset_to(0.0)

    def setup(self):
        self._cached_angle = 0.0

        # self._config_cancoder()
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

        self.prev_arm_voltage = 0.0
        self.prev_spin_voltage = 0.0

        self.component_enabled = True

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

        self.left_motor.configurator.apply(self.arm_motor_config)
        self.right_motor.configurator.apply(self.arm_motor_config)

    def on_enable(self):
        self.left_motor.set_control(self.arm_follower)

    """
    CONTROL METHODS
    """

    def set_voltage(self, voltage: units.volts):
        self.spin_voltage = voltage

    def set_arm_voltage(self, volts: units.volts):
        self.arm_voltage = volts
        self.arm_manual_control = True

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

        if self.spin_voltage != self.prev_spin_voltage:
            self.prev_spin_voltage = self.spin_voltage
            self.spin_motor.set_control(
                self.spin_control.with_output(self.spin_voltage)
            )

        self.right_motor.set_control(
            self.arm_voltage_control.with_output(self.arm_voltage)
        )
        self.left_motor.set_control(
            self.arm_voltage_control.with_output(self.arm_voltage)
        )
        self.arm_voltage = 0.0
