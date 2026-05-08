from magicbot import will_reset_to
from phoenix6 import controls
from phoenix6.configs import (
    TalonFXSConfiguration,
)
from phoenix6.hardware import TalonFXS
from phoenix6.signals import (
    MotorArrangementValue,
    NeutralModeValue,
)
from wpimath import units


class Indexer:
    left_kicker_motor: TalonFXS
    right_kicker_motor: TalonFXS
    conveyor_motor: TalonFXS

    conveyor_amps: units.amperes
    kicker_amps: units.amperes
    tuning_enabled: bool

    kicker_duty = will_reset_to(0.0)
    conveyor_volt = will_reset_to(0.0)
    manual_control = will_reset_to(False)

    def setup(self):
        self._kicker_follower_set = False

        self._config_kicker_motors()
        self._config_conveyor_motor()

        self.voltage_control = controls.VoltageOut(0)
        self.duty_control = controls.DutyCycleOut(0)
        self.kicker_follower = controls.Follower(
            self.right_kicker_motor.device_id, True
        )
        self.coast_control = controls.CoastOut()

        self.prev_kicker_control = 0.0
        self.prev_conveyor_control = 0.0

        self.component_enabled = True

    def _config_kicker_motors(self):
        self.kicker_motor_configs = TalonFXSConfiguration()
        self.kicker_motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.kicker_motor_configs.commutation.motor_arrangement = (
            MotorArrangementValue.NEO550_JST
        )
        self.kicker_motor_configs.current_limits.supply_current_limit = self.kicker_amps

        self.left_kicker_motor.configurator.apply(self.kicker_motor_configs)
        self.right_kicker_motor.configurator.apply(self.kicker_motor_configs)

    def _config_conveyor_motor(self):
        self.conveyor_motor_configs = TalonFXSConfiguration()
        self.conveyor_motor_configs.motor_output.neutral_mode = NeutralModeValue.COAST
        self.conveyor_motor_configs.commutation.motor_arrangement = (
            MotorArrangementValue.BRUSHED_DC
        )
        self.conveyor_motor_configs.current_limits.supply_current_limit = (
            self.conveyor_amps
        )

        self.conveyor_motor.configurator.apply(self.conveyor_motor_configs)

    """
    CONTROL METHODS
    """

    def set_kicker(self, volts: units.volts):
        self.kicker_duty = volts  # ha duty thats funny right there

    def set_conveyor(self, volts: units.volts):
        self.conveyor_volt = volts

    def turn_off_component(self):
        self.component_enabled = False

    def turn_on_component(self):
        self.component_enabled = True

    def execute(self):
        # thing so that if batt low we can turn off to save energy

        self.right_kicker_motor.set_control(
            self.voltage_control.with_output(self.kicker_duty)
        )
        self.left_kicker_motor.set_control(self.kicker_follower)

        self.conveyor_motor.set_control(
            self.voltage_control.with_output(self.conveyor_volt)
        )
