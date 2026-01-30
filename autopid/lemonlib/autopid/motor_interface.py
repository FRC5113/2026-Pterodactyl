"""
Generic motor interface for tuning different motor controllers.
Abstracts TalonFX and SparkMax control methods.
"""

from abc import ABC, abstractmethod
from enum import Enum
from typing import Optional, Callable
import math
from phoenix6.hardware import TalonFX, TalonFXS
from phoenix6 import controls
import wpilib
from phoenix6.configs import Slot0Configs
from phoenix6.signals import GravityTypeValue, StaticFeedforwardSignValue
from rev import SparkMax, SparkMaxConfig, ResetMode, PersistMode
from wpimath.controller import (
    ArmFeedforward,
    SimpleMotorFeedforwardMeters,
    SimpleMotorFeedforwardRadians,
    ElevatorFeedforward,
)
from .tuning_data import GravityType


class MotorType(Enum):
    """Supported motor controller types"""

    TALON_FX = "TalonFX"
    TALON_FXS = "TalonFX"
    SPARK_MAX = "SparkMax"


class ControlMode(Enum):
    """Motor control modes"""

    VOLTAGE = "Voltage"
    VELOCITY = "Velocity"
    POSITION = "Position"
    PERCENT = "Percent"
    BRAKE = "Brake"


class MotorInterface(ABC):
    """Abstract interface for motor control during tuning"""

    @abstractmethod
    def set_control(self, mode: ControlMode, value: float = 0.0) -> None:
        """
        Set motor control mode and value.

        Args:
            mode: Control mode (voltage, velocity, position, etc.)
            value: Setpoint value (depends on mode)
        """
        pass

    @abstractmethod
    def get_position(self) -> float:
        """Get current position in mechanism units (radians, meters, etc.)"""
        pass

    @abstractmethod
    def get_velocity(self) -> float:
        """Get current velocity in mechanism units/sec"""
        pass

    @abstractmethod
    def apply_gains(
        self,
        kS: float = 0.0,
        kV: float = 0.0,
        kA: float = 0.0,
        kP: float = 0.0,
        kI: float = 0.0,
        kD: float = 0.0,
        kG: float = 0.0,
        slot: int = 0,
    ) -> bool:
        """
        Apply PID gains to motor controller.

        Args:
            kS: Static friction feedforward
            kV: Velocity feedforward
            kA: Acceleration feedforward
            kP: Proportional gain
            kI: Integral gain
            kD: Derivative gain
            kG: Gravity feedforward
            slot: PID slot number

        Returns:
            True if successful
        """
        pass

    @abstractmethod
    def get_motor_id(self) -> int:
        """Get motor device ID"""
        pass

    @abstractmethod
    def get_motor_type(self) -> MotorType:
        """Get motor controller type"""
        pass


class TalonFXInterface(MotorInterface):
    """TalonFX motor controller interface"""

    def __init__(
        self, motor: TalonFX, position_getter: Optional[Callable[[], float]] = None
    ):
        """
        Initialize with a TalonFX motor object.

        Args:
            motor: phoenix6.hardware.TalonFX instance
            position_getter: Optional custom position getter (for mechanisms with gearing)
        """
        self.motor = motor
        self.position_getter = position_getter
        self._last_config_time = 0.0
        self._config_delay = 0.1  # 100ms between configs

    def set_control(self, mode: ControlMode, value: float = 0.0) -> None:
        if mode == ControlMode.VOLTAGE:
            self.motor.set_control(controls.VoltageOut(value))
        elif mode == ControlMode.VELOCITY:
            self.motor.set_control(controls.VelocityVoltage(value))
        elif mode == ControlMode.POSITION:
            # TalonFX uses rotations, convert from radians
            rotations = value / math.tau
            self.motor.set_control(controls.PositionVoltage(rotations))
        elif mode == ControlMode.PERCENT:
            self.motor.set_control(controls.DutyCycleOut(value))
        elif mode == ControlMode.BRAKE:
            self.motor.set_control(controls.StaticBrake())

    def get_position(self) -> float:
        """Returns position in radians or custom units"""
        if self.position_getter:
            return self.position_getter()
        rotations = self.motor.get_position().value
        return rotations * math.tau

    def get_velocity(self) -> float:
        """Returns velocity in radians/sec"""
        rps = self.motor.get_velocity().value
        return rps * math.tau

    def apply_gains(
        self,
        kS: float = 0.0,
        kV: float = 0.0,
        kA: float = 0.0,
        kP: float = 0.0,
        kI: float = 0.0,
        kD: float = 0.0,
        kG: float = 0.0,
        gravity_type: Optional[GravityType] = None,
        static_feedforward_sign: Optional[StaticFeedforwardSignValue] = None,
        slot: int = 0,
    ) -> bool:

        current_time = wpilib.Timer.getFPGATimestamp()
        if current_time - self._last_config_time < self._config_delay:
            return False

        slot_config = Slot0Configs()
        slot_config.k_s = kS
        slot_config.k_v = kV
        slot_config.k_a = kA
        slot_config.k_p = kP
        slot_config.k_i = kI
        slot_config.k_d = kD
        slot_config.k_g = kG
        if gravity_type is not None:
            slot_config.gravity_type = (
                GravityTypeValue.ARM_COSINE
                if gravity_type == GravityType.COSINE
                else GravityTypeValue.ELEVATOR_STATIC
            )
        if static_feedforward_sign is not None:
            slot_config.static_feedforward_sign = (
                StaticFeedforwardSignValue.USE_VELOCITY_SIGN
                if kS >= 0
                else StaticFeedforwardSignValue.NEGATIVE_ONLY
            )

        status = self.motor.configurator.apply(slot_config, 0.050)

        if status.is_ok():
            self._last_config_time = current_time
            return True
        return False

    def get_motor_id(self) -> int:
        return self.motor.device_id

    def get_motor_type(self) -> MotorType:
        return MotorType.TALON_FX


class TalonFXSInterface(MotorInterface):
    """TalonFXS motor controller interface"""

    def __init__(
        self, motor: TalonFXS, position_getter: Optional[Callable[[], float]] = None
    ):
        """
        Initialize with a TalonFXS motor object.

        Args:
            motor: phoenix6.hardware.TalonFXS instance
            position_getter: Optional custom position getter (for mechanisms with gearing)
        """
        self.motor = motor
        self.position_getter = position_getter
        self._last_config_time = 0.0
        self._config_delay = 0.1  # 100ms between configs

    def set_control(self, mode: ControlMode, value: float = 0.0) -> None:
        if mode == ControlMode.VOLTAGE:
            self.motor.set_control(controls.VoltageOut(value))
        elif mode == ControlMode.VELOCITY:
            self.motor.set_control(controls.VelocityVoltage(value))
        elif mode == ControlMode.POSITION:
            # TalonFXS uses rotations, convert from radians
            rotations = value / math.tau
            self.motor.set_control(controls.PositionVoltage(rotations))
        elif mode == ControlMode.PERCENT:
            self.motor.set_control(controls.DutyCycleOut(value))
        elif mode == ControlMode.BRAKE:
            self.motor.set_control(controls.StaticBrake())

    def get_position(self) -> float:
        """Returns position in radians or custom units"""
        if self.position_getter:
            return self.position_getter()
        rotations = self.motor.get_position().value
        return rotations * math.tau

    def get_velocity(self) -> float:
        """Returns velocity in radians/sec"""
        rps = self.motor.get_velocity().value
        return rps * math.tau

    def apply_gains(
        self,
        kS: float = 0.0,
        kV: float = 0.0,
        kA: float = 0.0,
        kP: float = 0.0,
        kI: float = 0.0,
        kD: float = 0.0,
        kG: float = 0.0,
        slot: int = 0,
    ) -> bool:

        current_time = wpilib.Timer.getFPGATimestamp()
        if current_time - self._last_config_time < self._config_delay:
            return False

        slot_config = Slot0Configs()
        slot_config.k_s = kS
        slot_config.k_v = kV
        slot_config.k_a = kA
        slot_config.k_p = kP
        slot_config.k_i = kI
        slot_config.k_d = kD
        slot_config.k_g = kG

        status = self.motor.configurator.apply(slot_config, 0.050)

        if status.is_ok():
            self._last_config_time = current_time
            return True
        return False

    def get_motor_id(self) -> int:
        return self.motor.device_id

    def get_motor_type(self) -> MotorType:
        return MotorType.TALON_FXS


class SparkMaxInterface(MotorInterface):
    """SparkMax motor controller interface\n
    Note:
    This does not currently work cause im or rev is stupid
    """

    def __init__(
        self,
        motor: SparkMax,
        conversion_factor: float = 1.0,
        position_getter: Optional[Callable[[], float]] = None,
    ):
        """
        Initialize with a SparkMax motor object.

        Args:
            motor: rev.CANSparkMax instance
            conversion_factor: Position conversion factor (e.g., for encoder to mechanism units)
            position_getter: Optional custom position getter
        """
        self.motor = motor
        self.encoder = motor.getEncoder()
        self.pid_controller = motor.getClosedLoopController()
        self.conversion_factor = conversion_factor
        self.position_getter = position_getter
        self._last_config_time = 0.0
        self._config_delay = 0.1

        # Store kG separately since SparkMax doesn't have native support
        self._kG = 0.0

    def set_control(self, mode: ControlMode, value: float = 0.0) -> None:

        if mode == ControlMode.VOLTAGE:
            self.motor.setVoltage(value)
        elif mode == ControlMode.VELOCITY:
            # Convert radians/sec to RPM if needed
            self.pid_controller.setReference(value, SparkMax.ControlType.kVelocity)
        elif mode == ControlMode.POSITION:
            self.pid_controller.setReference(value, SparkMax.ControlType.kPosition)
        elif mode == ControlMode.PERCENT:
            self.motor.set(value)
        elif mode == ControlMode.BRAKE:
            self.motor.set(0.0)
            self.motor.configure(
                SparkMaxConfig.setIdleMode(SparkMaxConfig.IdleMode.kBrake),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
            )

    def get_position(self) -> float:
        """Returns position in mechanism units (based on conversion factor)"""
        if self.position_getter:
            return self.position_getter()
        return self.encoder.getPosition() * self.conversion_factor

    def get_velocity(self) -> float:
        """Returns velocity in mechanism units/sec (based on conversion factor)"""
        return (
            self.encoder.getVelocity() * self.conversion_factor / 60.0
        )  # RPM to units/sec

    def apply_gains(
        self,
        kS: float = 0.0,
        kV: float = 0.0,
        kA: float = 0.0,
        kP: float = 0.0,
        kI: float = 0.0,
        kD: float = 0.0,
        kG: float = 0.0,
        slot: int = 0,
    ) -> bool:
        import wpilib

        current_time = wpilib.Timer.getFPGATimestamp()
        if current_time - self._last_config_time < self._config_delay:
            return False

        # SparkMax doesn't have kS, kA, kG in the same way
        # Use FF for kV, store kG separately for manual application
        self.pid_controller.setP(kP, slot)
        self.pid_controller.setI(kI, slot)
        self.pid_controller.setD(kD, slot)
        self.pid_controller.setFF(kV, slot)  # Use kV as feedforward

        self._kG = kG  # Store for manual gravity compensation

        # Note: kS and kA need to be applied manually in user code
        # You may want to use WPILib's SimpleMotorFeedforward

        self._last_config_time = current_time
        return True

    def get_motor_id(self) -> int:
        return self.motor.getDeviceId()

    def get_motor_type(self) -> MotorType:
        return MotorType.SPARK_MAX
