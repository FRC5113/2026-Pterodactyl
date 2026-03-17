from phoenix6.hardware import TalonFX, TalonFXS
from phoenix6.configs import TalonFXSConfiguration, TalonFXConfiguration
from phoenix6.signals import NeutralModeValue, InvertedValue
from phoenix6.controls import VoltageOut
from wpilib import DutyCycleEncoder
from magicbot import feedback
import enum


class IntakeAngle(enum.IntEnum):
    # these values might be (are) wrong
    UP = 85
    DOWN = 5


class Intake:
    left_rollout_motor: TalonFXS
    right_rollout_motor: TalonFXS
    spin_motor: TalonFX
    left_encoder: DutyCycleEncoder
    right_encoder: DutyCycleEncoder

    is_lowered = False
    is_on = False

    def setup(self) -> None:
        left_rollout_config = TalonFXSConfiguration()
        left_rollout_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        left_rollout_config.motor_output.inverted = (
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.left_rollout_motor.configurator.apply(left_rollout_config)

        right_rollout_config = TalonFXSConfiguration()
        right_rollout_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        right_rollout_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.right_rollout_motor.configurator.apply(right_rollout_config)

        spin_motor_config = TalonFXConfiguration()
        spin_motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.spin_motor.configurator.apply(spin_motor_config)

    @feedback
    def get_angle(self) -> float:
        """NOTE: They might change the encoder rotations, so if there is a bug look here"""
        left = self.left_encoder.get()
        right = self.right_encoder.get()

        return (left + right) / 2

    def down(self):
        self.is_lowered = True

    def up(self):
        self.is_lowered = False

    def on(self):
        self.is_on = True

    def off(self):
        self.is_on = False

    def teleop_periodic(self):
        TOLERANCE = 0.001
        POS_VOLTAGE_CONTROL = VoltageOut(8)
        NEG_VOLTAGE_CONTROL = VoltageOut(-8)
        STOP_VOLTAGE_CONTROL = VoltageOut(0)
        changeNeeded = 0
        if self.is_lowered:
            changeNeeded = IntakeAngle.DOWN - self.get_angle()
        else:
            changeNeeded = IntakeAngle.UP - self.get_angle()

        if changeNeeded > 0 and changeNeeded > TOLERANCE:
            # to low
            self.left_rollout_motor.set_control(NEG_VOLTAGE_CONTROL)
            self.right_rollout_motor.set_control(NEG_VOLTAGE_CONTROL)
        elif changeNeeded < 0 and abs(changeNeeded) > TOLERANCE:
            # to high
            self.left_rollout_motor.set_control(POS_VOLTAGE_CONTROL)
            self.right_rollout_motor.set_control(POS_VOLTAGE_CONTROL)
        else:
            self.left_rollout_motor.set_control(STOP_VOLTAGE_CONTROL)
            self.right_rollout_motor.set_control(STOP_VOLTAGE_CONTROL)

        INTAKE_ON = VoltageOut(8)  # you can tune this
        INTAKE_OFF = VoltageOut(0)
        self.spin_motor.set_control(INTAKE_ON if self.is_on else INTAKE_OFF)
