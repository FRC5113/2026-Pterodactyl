"""
Generic motor tuner that can tune any FRC mechanism.
Supports TalonFX and SparkMax motors, position and velocity control,
and various gravity compensation types.
"""

from typing import Optional, Callable
import wpilib

from .motor_interface import (
    MotorInterface,
    TalonFXInterface,
    SparkMaxInterface,
    ControlMode,
)
from .tuning_data import ControlType, GravityType, MotorGains, MechanismTuningResults
from .analytical_tuner import AnalyticalTuner
from .trial_error_tuner import TrialErrorTuner


class GenericMotorTuner:
    """
    High-level tuner for any FRC mechanism.

    Example usage:
        ```python
        # Swerve module
        tuner = GenericMotorTuner(
            motor=swerve_module.direction_motor,
            control_type=ControlType.POSITION,
            name="Swerve Module FL"
        )
        gains = tuner.tune()

        # Flywheel
        tuner = GenericMotorTuner(
            motor=shooter.flywheel_motor,
            control_type=ControlType.VELOCITY,
            name="Shooter Flywheel"
        )
        gains = tuner.tune(use_trial=False)

        # Elevator
        tuner = GenericMotorTuner(
            motor=elevator.motor,
            control_type=ControlType.POSITION,
            gravity_type=GravityType.CONSTANT,
            name="Elevator"
        )
        gains = tuner.tune()

        # Arm
        tuner = GenericMotorTuner(
            motor=arm.pivot_motor,
            control_type=ControlType.POSITION,
            gravity_type=GravityType.COSINE,
            name="Arm Pivot"
        )
        gains = tuner.tune()
        ```
    """

    def __init__(
        self,
        motor,
        control_type: ControlType,
        name: str,
        gravity_type: GravityType = GravityType.NONE,
        position_getter: Optional[Callable[[], float]] = None,
        conversion_factor: float = 1.0,
    ):
        """
        Initialize the generic motor tuner.

        Args:
            motor: TalonFX or SparkMax motor object
            control_type: POSITION or VELOCITY control
            name: Human-readable name for the mechanism
            gravity_type: Type of gravity compensation (NONE, CONSTANT, COSINE)
            position_getter: Optional custom position getter function
            conversion_factor: For SparkMax, conversion from encoder units to mechanism units
        """
        # Create motor interface
        self.motor_interface = self._create_motor_interface(
            motor, position_getter, conversion_factor
        )

        self.control_type = control_type
        self.gravity_type = gravity_type
        self.name = name

        # Create tuning components
        self.analytical_tuner = AnalyticalTuner()
        self.trial_tuner = TrialErrorTuner()

        # Initialize components
        self.analytical_tuner.setup()
        self.trial_tuner.setup()

        # Results
        self.results: Optional[MechanismTuningResults] = None

    def _create_motor_interface(
        self, motor, position_getter, conversion_factor
    ) -> MotorInterface:
        """Create appropriate motor interface based on motor type"""
        # Try to detect motor type
        motor_type_name = type(motor).__name__

        if "TalonFX" in motor_type_name:
            return TalonFXInterface(motor, position_getter)
        elif "SparkMax" in motor_type_name or "CANSparkMax" in motor_type_name:
            return SparkMaxInterface(motor, conversion_factor, position_getter)
        else:
            # Default to TalonFX interface
            print(
                f"Warning: Unknown motor type {motor_type_name}, defaulting to TalonFX interface"
            )
            return TalonFXInterface(motor, position_getter)

    def tune(
        self,
        use_analytical: bool = True,
        use_trial: bool = True,
        timeout_seconds: float = 180.0,
    ) -> MotorGains:
        """
        Run the complete tuning process.

        Args:
            use_analytical: Run analytical tuning phase
            use_trial: Run trial-and-error tuning phase
            timeout_seconds: Maximum time for entire tuning process

        Returns:
            MotorGains object with final tuned gains
        """
        print(f"\n{'='*70}")
        print(f"Starting Auto-Tune for: {self.name}")
        print(f"Control Type: {self.control_type.value}")
        print(f"Gravity Type: {self.gravity_type.value}")
        print(
            f"Motor: {self.motor_interface.get_motor_type().value} (ID: {self.motor_interface.get_motor_id()})"
        )
        print(f"{'='*70}\n")

        start_time = wpilib.Timer.getFPGATimestamp()

        # Phase 1: Analytical tuning
        if use_analytical:
            print(f"[{self.name}] Starting analytical tuning...")
            self.analytical_tuner.start(
                self.motor_interface, self.control_type, self.gravity_type, self.name
            )

            # Run analytical tuner
            while not self.analytical_tuner.is_complete():
                if wpilib.Timer.getFPGATimestamp() - start_time > timeout_seconds:
                    print(f"[{self.name}] Analytical tuning timed out")
                    break

                self.analytical_tuner.execute()
                wpilib.wait(0.02)  # 20ms loop

            analytical_results = self.analytical_tuner.get_results()
            if analytical_results:
                self.results = analytical_results
                print(f"[{self.name}] Analytical tuning complete")
            else:
                print(f"[{self.name}] Analytical tuning failed")
                # Create empty results
                self.results = MechanismTuningResults(
                    mechanism_id=self.motor_interface.get_motor_id(),
                    mechanism_name=self.name,
                    control_type=self.control_type,
                    gravity_type=self.gravity_type,
                )

        # Phase 2: Trial-and-error tuning
        if use_trial:
            print(f"\n[{self.name}] Starting trial-and-error tuning...")

            # Get analytical gains as starting point
            analytical_gains = (
                self.analytical_tuner.get_analytical_gains() if use_analytical else None
            )

            # Pass results to trial tuner if we have them
            if self.results:
                self.trial_tuner.results = self.results

            self.trial_tuner.start(
                self.motor_interface,
                self.control_type,
                self.gravity_type,
                self.name,
                analytical_gains,
            )

            # Run trial tuner
            while not self.trial_tuner.is_complete():
                if wpilib.Timer.getFPGATimestamp() - start_time > timeout_seconds:
                    print(f"[{self.name}] Trial tuning timed out")
                    break

                self.trial_tuner.execute()
                wpilib.wait(0.02)  # 20ms loop

            trial_results = self.trial_tuner.get_results()
            if trial_results:
                self.results = trial_results
                print(f"[{self.name}] Trial-and-error tuning complete")
            else:
                print(f"[{self.name}] Trial-and-error tuning failed")

        # Finalize results
        if self.results:
            # Select final gains (prefer trial if available, otherwise analytical)
            self.results.select_final_gains(use_trial=use_trial)

            # Print comparison
            self.results.print_comparison()

            # Apply final gains to motor
            self._apply_final_gains()

            # Stop motor
            self.motor_interface.set_control(ControlMode.BRAKE)

            return self.results.final_gains
        else:
            print(f"[{self.name}] Tuning failed - no results available")
            return MotorGains()

    def _apply_final_gains(self):
        """Apply final tuned gains to the motor"""
        if self.results and self.results.final_gains:
            gains = self.results.final_gains
            success = self.motor_interface.apply_gains(
                kS=gains.kS,
                kV=gains.kV,
                kA=gains.kA,
                kP=gains.kP,
                kI=gains.kI,
                kD=gains.kD,
                kG=gains.kG,
            )

            if success:
                print(f"[{self.name}] Final gains applied to motor")
            else:
                print(f"[{self.name}] Warning: Could not apply final gains to motor")

    def get_results(self) -> Optional[MechanismTuningResults]:
        """Get the complete tuning results"""
        return self.results

    def export_gains_to_file(self, filename: str):
        """Export tuning results to a JSON file"""
        if self.results:
            import json

            with open(filename, "w") as f:
                json.dump(self.results.to_dict(), f, indent=2)
            print(f"[{self.name}] Gains exported to {filename}")

    def apply_gains(self, gains: MotorGains) -> bool:
        """Apply a set of gains to the motor"""
        return self.motor_interface.apply_gains(
            kS=gains.kS,
            kV=gains.kV,
            kA=gains.kA,
            kP=gains.kP,
            kI=gains.kI,
            kD=gains.kD,
            kG=gains.kG,
        )


# Convenience functions for common mechanisms


def tune_swerve_module(
    motor,
    name: str = "Swerve Module",
    position_getter: Optional[Callable[[], float]] = None,
) -> MotorGains:
    """
    Tune a swerve steering module.

    Args:
        motor: TalonFX or SparkMax motor
        name: Name for the module
        position_getter: Optional custom position getter

    Returns:
        Tuned MotorGains
    """
    tuner = GenericMotorTuner(
        motor=motor,
        control_type=ControlType.POSITION,
        name=name,
        position_getter=position_getter,
    )
    return tuner.tune(use_analytical=True, use_trial=True)


def tune_flywheel(motor, name: str = "Flywheel") -> MotorGains:
    """
    Tune a shooter flywheel.

    Args:
        motor: TalonFX or SparkMax motor
        name: Name for the flywheel

    Returns:
        Tuned MotorGains
    """
    tuner = GenericMotorTuner(motor=motor, control_type=ControlType.VELOCITY, name=name)
    return tuner.tune(use_analytical=True, use_trial=True)


def tune_hood(
    motor, name: str = "Hood", position_getter: Optional[Callable[[], float]] = None
) -> MotorGains:
    """
    Tune a shooter hood or similar mechanism.

    Args:
        motor: TalonFX or SparkMax motor
        name: Name for the hood
        position_getter: Optional custom position getter

    Returns:
        Tuned MotorGains
    """
    tuner = GenericMotorTuner(
        motor=motor,
        control_type=ControlType.POSITION,
        name=name,
        position_getter=position_getter,
    )
    return tuner.tune(use_analytical=True, use_trial=False)  # Analytical only


def tune_elevator(
    motor, name: str = "Elevator", position_getter: Optional[Callable[[], float]] = None
) -> MotorGains:
    """
    Tune an elevator with constant gravity compensation.

    Args:
        motor: TalonFX or SparkMax motor
        name: Name for the elevator
        position_getter: Optional custom position getter (e.g., for height in meters)

    Returns:
        Tuned MotorGains
    """
    tuner = GenericMotorTuner(
        motor=motor,
        control_type=ControlType.POSITION,
        gravity_type=GravityType.CONSTANT,
        name=name,
        position_getter=position_getter,
    )
    return tuner.tune(use_analytical=True, use_trial=True)


def tune_arm(
    motor,
    name: str = "Arm Pivot",
    position_getter: Optional[Callable[[], float]] = None,
) -> MotorGains:
    """
    Tune an arm pivot with cosine gravity compensation.

    Args:
        motor: TalonFX or SparkMax motor
        name: Name for the arm
        position_getter: Optional custom position getter (e.g., for angle in radians)

    Returns:
        Tuned MotorGains
    """
    tuner = GenericMotorTuner(
        motor=motor,
        control_type=ControlType.POSITION,
        gravity_type=GravityType.COSINE,
        name=name,
        position_getter=position_getter,
    )
    return tuner.tune(use_analytical=True, use_trial=True)
