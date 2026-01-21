from phoenix6.hardware import TalonFX
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from magicbot import will_reset_to
from wpimath.units import volts
class Intake:
    motor_1: TalonFX #should be front motor
    motor_2: TalonFX #should be back motor
    motor_volts = will_reset_to(0.0)

    """
    Takes the ammount the bumpers are down 
    """
    def setup(self):
        config = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.motor_1.configurator.apply(config)
        self.motor_2.configurator.apply(config)
    def applyVolts(self, v: volts):
        self.motor_volts = v
    def applyReverseVolts(self, v: volts):
        self.motor_volts = -v
    def execute(self):
        self.motor_1.setVoltage(self.motor_volts)
        self.motor_2.setVoltage(self.motor_volts)