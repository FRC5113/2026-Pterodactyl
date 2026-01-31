import wpilib
from magicbot import MagicRobot
from wpimath.geometry import Rotation2d
from wpimath import units
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6 import CANBus

from components.swerve_wheel import SwerveWheel
from lemonlib.smart import SmartProfile
from lemonlib.autopid.generic_motor_tuner import (
    GenericMotorTuner,
    tune_swerve_module,
    tune_flywheel,
    tune_hood,
    tune_elevator,
    tune_arm,
)
from lemonlib.autopid.tuning_data import ControlType, GravityType, MotorGains
from lemonlib.autopid.motor_interface import TalonFXInterface, SparkMaxInterface


class MyRobot(MagicRobot):
    rear_left: SwerveWheel

    def createObjects(self):
        """Initialize all robot objects here"""
        # Detect if we're in simulation
        self.is_simulation = wpilib.RobotBase.isSimulation()

        # Create joystick for manual control
        self.joystick = wpilib.XboxController(0)

        # Flag to track if tuning is active
        self.tuning_active = False
        self.tuning_enabled = True
        self.tuner_instance = None  # Will hold the active tuner

        self.canicore_canbus = CANBus("can0")
        # Front Left Module - CAN IDs: Speed=1, Direction=2, CANcoder=3
        self.rear_left_speed_motor = TalonFX(41, self.canicore_canbus)
        self.rear_left_direction_motor = TalonFX(42, self.canicore_canbus)
        self.rear_left_cancoder = CANcoder(43, self.canicore_canbus)

        # Configure module parameters (adjust these for your robot)
        self.wheel_radius = 0.05
        self.drive_gear_ratio = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0))
        self.direction_gear_ratio = 150.0 / 7.0
        self.direction_amps: units.amperes = 40.0
        self.speed_amps: units.amperes = 60.0

        self.direction_profile = SmartProfile(
            "direction",
            {
                "kP": 0.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.0,
            },
            True,
        )
        self.speed_profile = SmartProfile(
            "speed",
            {
                "kP": 0.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.0,
                "kA": 0.0,
            },
            False,
        )
        self.previous_state = None
        self.state = None

    def teleopPeriodic(self):
        """Called periodically during teleop mode"""

        # Start tuning when back button is pressed
        if self.joystick.getBackButtonPressed() and self.tuner_instance is None:
            print("Starting Swerve Tuning...")
            self.tuner_instance = tune_swerve_module(
                self.rear_left.direction_motor,
                "rear_left_direction_motor",
            )
            self.tuner_instance.start_tuning(
                use_analytical=True, use_trial=True, timeout_seconds=180.0
            )

        # Run tuner periodic if active
        if self.tuner_instance is not None:
            if not self.tuner_instance.is_complete():
                self.tuner_instance.periodic()
            else:
                # Tuning complete - get results
                gains = self.tuner_instance.get_final_gains()
                if gains:
                    print(f"Tuning complete! Final gains: {gains}")
                self.tuner_instance = None  # Clear tuner


if __name__ == "__main__":
    wpilib.run(MyRobot)
