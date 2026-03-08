import math
from typing import List

import choreo
import choreo.util
from choreo.trajectory import SwerveTrajectory
from magicbot import AutonomousStateMachine, state, timed_state
from wpilib import Field2d, RobotBase, SmartDashboard
from wpimath.geometry import Pose2d

from components.drive_control import DriveControl
from components.intake import Intake, IntakeAngle
from components.swerve_drive import SwerveDrive

# from components.odometry import Odometry
from lemonlib.util import is_red


class AutoBase(AutonomousStateMachine):
    drive_control: DriveControl
    swerve_drive: SwerveDrive
    intake: Intake
    # odometry: Odometry
    estimated_field: Field2d

    DISTANCE_TOLERANCE = 0.05
    ANGLE_TOLERANCE = math.radians(1)  # radians
    TRANSLATIONAL_SPEED_TOLERANCE = 0.2  # m/s
    ROTATIONAL_SPEED_TOLERANCE = 0.1  # rad/s

    def __init__(self, sequence: List[str]) -> None:
        super().__init__()

        self.sequence = sequence  # List of trajectories and states
        self.current_step = -1
        self.trajectory_index = -1
        self.trajectories: list[SwerveTrajectory] = []
        SmartDashboard.putNumber("Distance", 0)
        SmartDashboard.putString("Final Pose", "none")

        # separates sequence in states and trajectories (which are tagged accordingly. if not tagged, will throw error)
        for item in self.sequence:
            x = item.split(":")  # divides into tag and name
            assert len(x) == 2  # asserts there were not multiple :'s in item
            match x[0]:
                case "state":
                    pass
                case "trajectory":
                    try:
                        self.trajectories.append(choreo.load_swerve_trajectory(x[1]))
                    except ValueError:
                        print(f"WARNING: TRAJECTORY {x[1]} NOT FOUND")
                case _:
                    print(
                        'WARNING:Elements in sequence must be tagged with either "state:" or "trajectory"'
                    )

    def on_enable(self) -> None:
        starting_pose = self.get_starting_pose()
        if starting_pose is not None and RobotBase.isSimulation():
            self.swerve_drive.set_starting_pose(starting_pose)

        self.current_step = -1  # Reset current step
        self.trajectory_index = -1  # Reset trajectory index

        super().on_enable()

    def _get_full_path_poses(self) -> list[Pose2d]:
        """Get a list of poses for the full path for display."""
        return [
            sample
            for trajectory in self.trajectories
            for sample in trajectory.get_poses()
        ]

    def get_starting_pose(self) -> Pose2d | None:
        """Get the initial pose of the first trajectory."""
        if not self.trajectories:
            return None
        return self.trajectories[0].get_initial_pose(is_red())

    def display_trajectory(self) -> None:
        """Display the trajectory on the estimated field."""
        self.estimated_field.getObject("trajectory").setPoses(
            self._get_full_path_poses()
        )

    def on_disable(self) -> None:
        """Clear the trajectory display when disabled."""
        super().on_disable()
        self.estimated_field.getObject("trajectory").setPoses(
            []
        )  # Clear trajectory display

    @state(first=True)
    def next_step(self):
        """Moves to the next step in the sequence, determining if it's a trajectory or a state."""
        self.current_step += 1
        if self.current_step >= len(self.sequence):
            self.done()
            return

        step = self.sequence[self.current_step]  # Get the current step
        if step.startswith("state:"):
            state = step.split("state:")[1]  # Extract state name
            if state not in self.state_names:
                print(f"WARNING: STATE {state} NOT DEFINED")
                # raise ReferenceError("State {state} not defined")
                self.next_state("next_step")
                return

            self.next_state(step.split("state:")[1])  # Go to the specified state
        else:
            self.trajectory_index += 1
            self.current_trajectory = self.trajectories[
                self.trajectory_index
            ]  # Get current trajectory
            if self.current_trajectory:
                self.next_state(
                    "tracking_trajectory"
                )  # Move to tracking trajectory state
            else:
                self.next_state("next_step")  # Skip invalid trajectory names

    @state
    def tracking_trajectory(self, state_tm):
        """Follows the current trajectory and transitions when done."""

        current_pose = self.swerve_drive.get_estimated_pose()
        final_pose = self.current_trajectory.get_final_pose(
            is_red()
        )  # Get final pose of trajectory
        if final_pose is None:
            final_pose = Pose2d()
        distance = current_pose.translation().distance(
            final_pose.translation()
        )  # Calculate distance to final pose
        angle_error = (
            final_pose.rotation() - current_pose.rotation()
        ).radians()  # Calculate angle error
        velocity = self.swerve_drive.get_velocity()
        speed = math.sqrt(
            math.pow(velocity.vx, 2.0) + math.pow(velocity.vy, 2.0)
        )  # Calculate speed
        SmartDashboard.putString("Final Pose", f"{final_pose}")

        in_distance_tolerance = distance < self.DISTANCE_TOLERANCE
        in_angle_tolerance = abs(angle_error) < self.ANGLE_TOLERANCE
        in_translational_speed_tolerance = speed < self.TRANSLATIONAL_SPEED_TOLERANCE
        in_rotational_speed_tolerance = (
            abs(velocity.omega) < self.ROTATIONAL_SPEED_TOLERANCE
        )
        is_in_second_half_of_trajectory = (
            state_tm > self.current_trajectory.get_total_time() / 2.0
        )
        if (
            in_distance_tolerance
            and in_angle_tolerance
            and in_translational_speed_tolerance
            and in_rotational_speed_tolerance
            and is_in_second_half_of_trajectory
        ):
            self.next_state("next_step")  # Move to next step if trajectory is complete
        sample = self.current_trajectory.sample_at(
            state_tm, is_red()
        )  # Sample trajectory at current time
        if sample is not None:
            self.drive_control.drive_auto(sample)  # Drive using the sampled trajectory

            SmartDashboard.putNumber("Distance", distance)

    """
    STATES
    """

    @timed_state(duration=5.0, next_state="next_step")
    def shoot(self):
        """Placeholder for shooting state."""
        #self.shooter_controller.request_shoot()

    @state
    def climb(self):
        """Placeholder for climbing state."""
        print("Climbing State Placeholder")
        self.next_state("next_step")

    @timed_state(duration=3.0, next_state="next_step")
    def go_forward_and_intake(self):
        self.drive_control.drive_auto_manual(1, 0.0, 0.0, False)
        self.intake.set_arm_angle(IntakeAngle.INTAKING.value)
        self.intake.set_voltage(8)

    @timed_state(duration=5.0, next_state="next_step")
    def outpost_wait(self):
        """Wait for 5 seconds before moving to the next step."""
        pass
