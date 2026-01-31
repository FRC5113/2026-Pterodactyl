"""
Comprehensive examples for using the Generic Motor Tuner.

This module demonstrates how to tune various FRC mechanisms including:
- Swerve modules
- Flywheels
- Hoods
- Elevators
- Arms
- And more!
"""

from .generic_motor_tuner import (
    GenericMotorTuner,
    tune_swerve_module,
    tune_flywheel,
    tune_hood,
    tune_elevator,
    tune_arm,
)
from .tuning_data import ControlType, GravityType, MotorGains
from .motor_interface import TalonFXInterface, SparkMaxInterface


def example_tune_swerve_module_simple(swerve_module):
    """
    Simple example: Tune a swerve module using the convenience function.
    """
    # Assuming swerve_module has a direction_motor (TalonFX)
    gains = tune_swerve_module(
        motor=swerve_module.direction_motor,
        name=f"Swerve Module {swerve_module.direction_motor.device_id}",
    )

    print(f"Tuned gains: {gains}")
    return gains


def example_tune_swerve_module_custom(swerve_module):
    """
    Advanced example: Tune swerve module with custom position getter.
    Useful when swerve module has complex gearing or uses a custom position method.
    """
    tuner = GenericMotorTuner(
        motor=swerve_module.direction_motor,
        control_type=ControlType.POSITION,
        name=f"Swerve Module FL",
        position_getter=lambda: swerve_module.getPosition().angle.radians(),
    )

    # Run both analytical and trial tuning
    gains = tuner.tune(use_analytical=True, use_trial=True)

    # Export results to file
    tuner.export_gains_to_file(
        f"swerve_module_{swerve_module.direction_motor.device_id}_gains.json"
    )

    return gains


def example_tune_flywheel_simple(shooter):
    """
    Simple example: Tune a shooter flywheel.
    """
    gains = tune_flywheel(motor=shooter.flywheel_motor, name="Shooter Flywheel")

    print(f"Flywheel tuned gains: {gains}")
    return gains


def example_tune_flywheel_analytical_only(shooter):
    """
    Tune flywheel using only analytical methods (faster, but potentially less accurate).
    """
    tuner = GenericMotorTuner(
        motor=shooter.flywheel_motor,
        control_type=ControlType.VELOCITY,
        name="Shooter Flywheel",
    )

    # Only analytical tuning
    gains = tuner.tune(use_analytical=True, use_trial=False, timeout_seconds=60.0)

    return gains


def example_tune_dual_flywheels(shooter):
    """
    Example: Tune two flywheels (top and bottom).
    """
    # Tune top flywheel
    top_gains = tune_flywheel(motor=shooter.top_flywheel_motor, name="Top Flywheel")

    # Tune bottom flywheel
    bottom_gains = tune_flywheel(
        motor=shooter.bottom_flywheel_motor, name="Bottom Flywheel"
    )

    return top_gains, bottom_gains


def example_tune_hood(shooter):
    """
    Tune a shooter hood (position control, no gravity compensation needed).
    """
    gains = tune_hood(motor=shooter.hood_motor, name="Shooter Hood")

    return gains


def example_tune_hood_with_custom_units(shooter):
    """
    Tune hood with custom position getter (e.g., returning angle in degrees).
    """
    import math

    tuner = GenericMotorTuner(
        motor=shooter.hood_motor,
        control_type=ControlType.POSITION,
        name="Shooter Hood",
        position_getter=lambda: shooter.get_hood_angle_degrees()
        * math.pi
        / 180.0,  # Convert to radians
    )

    gains = tuner.tune(use_analytical=True, use_trial=False)
    return gains


def example_tune_elevator(elevator):
    """
    Tune an elevator with constant gravity compensation.
    """
    gains = tune_elevator(
        motor=elevator.motor,
        name="Elevator",
        position_getter=lambda: elevator.get_height(),  # Returns height in meters
    )

    print(f"Elevator gains (with kG for gravity): {gains}")
    print(f"  kG (gravity compensation): {gains.kG:.4f} V")

    return gains


def example_tune_elevator_sparkmax(elevator):
    """
    Tune an elevator using a SparkMax motor controller.
    """
    tuner = GenericMotorTuner(
        motor=elevator.spark_max_motor,
        control_type=ControlType.POSITION,
        gravity_type=GravityType.CONSTANT,
        name="Elevator (SparkMax)",
        conversion_factor=0.1,  # Convert encoder units to meters
        position_getter=lambda: elevator.get_height_meters(),
    )

    gains = tuner.tune(use_analytical=True, use_trial=True)
    return gains


def example_tune_arm_pivot(arm):
    """
    Tune an arm pivot with cosine gravity compensation.
    """
    gains = tune_arm(
        motor=arm.pivot_motor,
        name="Arm Pivot",
        position_getter=lambda: arm.get_angle(),  # Returns angle in radians
    )

    print(f"Arm gains (with kG for gravity): {gains}")
    print(f"  kG (gravity at horizontal): {gains.kG:.4f} V")

    return gains


def example_tune_arm_with_prep(arm):
    """
    Tune arm with manual preparation step.

    Important: For cosine gravity compensation, the arm should be
    positioned horizontally before tuning begins.
    """
    # Move arm to horizontal position first
    print("Moving arm to horizontal position for gravity measurement...")
    arm.move_to_horizontal()  # Your robot-specific method

    # Wait for arm to settle
    import wpilib

    wpilib.Wait(2.0)

    # Now tune
    gains = tune_arm(
        motor=arm.pivot_motor,
        name="Arm Pivot",
        position_getter=lambda: arm.get_angle_radians(),
    )

    return gains


def example_tune_custom_mechanism(mechanism):
    """
    Tune a custom mechanism with full control over all parameters.
    """
    tuner = GenericMotorTuner(
        motor=mechanism.motor,
        control_type=ControlType.POSITION,  # or ControlType.VELOCITY
        gravity_type=GravityType.NONE,  # or CONSTANT, or COSINE
        name="Custom Mechanism",
        position_getter=lambda: mechanism.get_custom_position(),
        conversion_factor=1.0,  # Only used for SparkMax
    )

    # Tune with custom timeout
    gains = tuner.tune(
        use_analytical=True, use_trial=True, timeout_seconds=120.0  # 2 minute timeout
    )

    # Get detailed results
    results = tuner.get_results()
    if results:
        print(f"\nDetailed Results for {mechanism}:")
        print(f"  Static Friction: {results.static_friction_voltage:.3f} V")
        print(f"  Max Velocity: {results.max_velocity_pos:.3f} units/s")
        print(f"  Time Constant: {results.time_constant_pos:.3f} s")
        results.print_comparison()

    return gains


def example_batch_tune_all_swerve_modules(drivetrain):
    """
    Tune all four swerve modules in sequence.
    """
    modules = [
        (drivetrain.front_left, "Front Left"),
        (drivetrain.front_right, "Front Right"),
        (drivetrain.rear_left, "Rear Left"),
        (drivetrain.rear_right, "Rear Right"),
    ]

    all_gains = {}

    for module, name in modules:
        print(f"\n{'='*70}")
        print(f"Tuning {name} swerve module...")
        print(f"{'='*70}\n")

        gains = tune_swerve_module(
            motor=module.direction_motor,
            name=f"Swerve {name}",
            position_getter=lambda m=module: m.getPosition().angle.radians(),
        )

        all_gains[name] = gains

    # Print summary
    print("\n" + "=" * 70)
    print("ALL SWERVE MODULES TUNED")
    print("=" * 70)
    for name, gains in all_gains.items():
        print(f"\n{name}:")
        print(f"  {gains}")

    return all_gains


def example_tune_complete_robot(robot):
    """
    Tune all mechanisms on a robot.
    """
    results = {}

    # Tune drivetrain
    print("\n### TUNING DRIVETRAIN ###")
    results["swerve"] = example_batch_tune_all_swerve_modules(robot.drivetrain)

    # Tune shooter
    print("\n### TUNING SHOOTER ###")
    results["flywheel_top"] = tune_flywheel(robot.shooter.top_motor, "Top Flywheel")
    results["flywheel_bottom"] = tune_flywheel(
        robot.shooter.bottom_motor, "Bottom Flywheel"
    )
    results["hood"] = tune_hood(robot.shooter.hood_motor, "Hood")

    # Tune elevator
    print("\n### TUNING ELEVATOR ###")
    results["elevator"] = tune_elevator(robot.elevator.motor, "Elevator")

    # Tune arm
    print("\n### TUNING ARM ###")
    results["arm"] = tune_arm(robot.arm.pivot_motor, "Arm Pivot")

    print("\n" + "=" * 70)
    print("COMPLETE ROBOT TUNING FINISHED")
    print("=" * 70)

    return results


def example_apply_saved_gains(motor, gains: MotorGains):
    """
    Apply previously tuned gains to a motor.
    """
    tuner = GenericMotorTuner(
        motor=motor, control_type=ControlType.POSITION, name="Mechanism"  # Or VELOCITY
    )

    success = tuner.apply_gains(gains)

    if success:
        print("Gains applied successfully!")
    else:
        print("Failed to apply gains")

    return success


def example_load_and_apply_gains(motor, filename: str):
    """
    Load gains from a JSON file and apply them.
    """
    import json

    with open(filename, "r") as f:
        data = json.load(f)

    gains = MotorGains(
        kS=data["final_gains"]["kS"],
        kV=data["final_gains"]["kV"],
        kA=data["final_gains"]["kA"],
        kP=data["final_gains"]["kP"],
        kI=data["final_gains"]["kI"],
        kD=data["final_gains"]["kD"],
        kG=data["final_gains"]["kG"],
    )

    return example_apply_saved_gains(motor, gains)


def example_incremental_tuning(mechanism):
    """
    Start with analytical tuning, then refine with trial-and-error.
    """
    # First pass: Analytical only (quick)
    print("Phase 1: Analytical tuning (fast)...")
    tuner = GenericMotorTuner(
        motor=mechanism.motor, control_type=ControlType.POSITION, name="Mechanism"
    )

    analytical_gains = tuner.tune(
        use_analytical=True, use_trial=False, timeout_seconds=60.0
    )

    # Test the analytical gains...
    print(
        "Test the robot with analytical gains. If not satisfied, continue to trial tuning."
    )

    # Second pass: Trial-and-error refinement
    print("\nPhase 2: Trial-and-error refinement...")
    final_gains = tuner.tune(
        use_analytical=False, use_trial=True, timeout_seconds=120.0
    )

    return final_gains


def example_mixed_controllers(robot):
    """
    Tune mechanisms with different motor controllers.
    """
    # TalonFX swerve modules
    swerve_gains = tune_swerve_module(
        motor=robot.drivetrain.front_left.direction_motor, name="Swerve FL"  # TalonFX
    )

    # SparkMax elevator
    elevator_gains = GenericMotorTuner(
        motor=robot.elevator.spark_motor,  # SparkMax
        control_type=ControlType.POSITION,
        gravity_type=GravityType.CONSTANT,
        name="Elevator (SparkMax)",
        conversion_factor=0.1,  # Encoder units to meters
    ).tune()

    # TalonFX arm
    arm_gains = tune_arm(motor=robot.arm.talon_motor, name="Arm Pivot")  # TalonFX

    return {"swerve": swerve_gains, "elevator": elevator_gains, "arm": arm_gains}


def integrate_tuner_into_robot():
    """
    Example of how to integrate tuner into your robot.py or commands.
    """

    example_robot_py = '''
from generic_motor_tuner import tune_swerve_module, tune_elevator, tune_arm
from magicbot import MagicRobot

class MyRobot(MagicRobot):
    # ... your robot components ...
    
    def teleopInit(self):
        """When teleop starts, optionally run auto-tune"""
        # Check if tuning is requested (e.g., via shuffleboard button)
        if self.should_run_tuning:
            self.run_auto_tune()
    
    def run_auto_tune(self):
        """Run auto-tune for all mechanisms"""
        print("Starting auto-tune sequence...")
        
        # Tune swerve modules
        for module in [self.front_left, self.front_right, self.rear_left, self.rear_right]:
            gains = tune_swerve_module(
                motor=module.direction_motor,
                name=f"Swerve {module.direction_motor.device_id}"
            )
            # Gains are automatically applied
        
        # Tune elevator
        elevator_gains = tune_elevator(
            motor=self.elevator.motor,
            name="Elevator"
        )
        
        # Tune arm
        arm_gains = tune_arm(
            motor=self.arm.pivot_motor,
            name="Arm"
        )
        
        print("Auto-tune complete!")
'''

    print(example_robot_py)


if __name__ == "__main__":
    print(__doc__)
    print(
        "\nThis file contains comprehensive examples for using the Generic Motor Tuner."
    )
    print("Import the functions you need in your robot code.")
