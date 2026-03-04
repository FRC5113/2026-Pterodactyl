from choreo.trajectory import SwerveSample
from magicbot import StateMachine, will_reset_to
from magicbot.state_machine import state
from wpilib import DriverStation
from wpimath import units
from wpimath.geometry import Pose2d

from components.swerve_drive import SwerveDrive


class DriveControl(StateMachine):
    swerve_drive: SwerveDrive

    go_to_pose = will_reset_to(False)
    desired_pose = Pose2d()
    period: units.seconds = 0.02
    drive_auto_man = will_reset_to(False)
    translationX = will_reset_to(0)
    translationY = will_reset_to(0)
    rotationX = will_reset_to(0)
    drive_sysid = will_reset_to(False)
    sysid_volts = will_reset_to(0.0)
    point_to_target = will_reset_to(False)
    point_target: units.degrees = 0.0
    point_joy_target = will_reset_to(False)
    point_joy_x = will_reset_to(0.0)
    point_joy_y = will_reset_to(0.0)
    sample = will_reset_to(None)

    def setup(self):
        pass

    def drive_manual(
        self,
        translationX: units.meters_per_second,
        translationY: units.meters_per_second,
        rotationX: units.radians_per_second,
        field_relative: bool,
    ):
        """Request manual drive control with joystick velocities."""
        if self.current_state in ("free", "point_towards_target", "point_towards_joy"):
            self.translationX = translationX
            self.translationY = translationY
            self.rotationX = rotationX
            self.field_relative = field_relative

    def drive_sysid_manual(self, volts: float):
        """Request sysID mode with specified voltage."""
        self.drive_sysid = True
        self.sysid_volts = volts

    def request_pose(self, pose: Pose2d):
        """Request autonomous traj following to a specific field pose."""
        self.go_to_pose = True
        self.desired_pose = pose

    def point_to(self, angle: units.radians):
        """Request robot to point towards a specific field-absolute angle while driving."""
        self.point_to_target = True
        self.point_target = angle

    def point_to_joy(self, rightX: float, rightY: float):
        """Request robot to point in the direction of the right joystick."""
        self.point_joy_target = True
        self.point_joy_x = rightX
        self.point_joy_y = rightY

    def drive_point(
        self,
        vx: units.meters_per_second,
        vy: units.meters_per_second,
        angle: units.radians,
    ):
        """Request driving with translation while facing a field-absolute angle."""
        self.translationX = vx
        self.translationY = vy
        self.point_to_target = True
        self.point_target = angle

    def drive_point_joy(
        self,
        vx: units.meters_per_second,
        vy: units.meters_per_second,
        joy_x: float,
        joy_y: float,
    ):
        """Request driving while pointing robot in joystick direction."""
        self.translationX = vx
        self.translationY = vy
        self.point_joy_target = True
        self.point_joy_x = joy_x
        self.point_joy_y = joy_y

    def drive_auto(self, sample: SwerveSample | None = None):
        """Provide trajectory sample for autonomous path following."""
        self.sample = sample

    def drive_auto_manual(
        self,
        translationX: units.meters_per_second,
        translationY: units.meters_per_second,
        rotationX: units.radians_per_second,
        field_relative: bool,
    ):
        """Manual drive override during autonomous mode."""
        self.translationX = translationX
        self.translationY = translationY
        self.rotationX = rotationX
        self.field_relative = field_relative
        self.drive_auto_man = True

    @state(first=True)
    def initialize(self):
        """Initial state - resets drive commands and determines next state."""
        self.translationX = 0
        self.translationY = 0
        self.rotationX = 0
        self.field_relative = False

        if self.go_to_pose:
            self.next_state("going_to_pose")
        elif DriverStation.isAutonomousEnabled():
            self.next_state("run_auton_routine")
        else:
            self.next_state("free")

    @state
    def free(self):
        """Default teleop state - accepts manual joystick input."""
        self.swerve_drive.drive(
            self.translationX,
            self.translationY,
            self.rotationX,
            self.field_relative,
        )

        if DriverStation.isAutonomousEnabled():
            self.next_state("run_auton_routine")
        elif self.go_to_pose:
            self.next_state("going_to_pose")
        elif self.point_to_target:
            self.next_state("point_towards_target")
        elif self.point_joy_target:
            self.next_state("point_towards_joy")
        elif self.drive_sysid:
            self.next_state("drive_sysid_state")

    @state
    def point_towards_target(self):
        """Drive while pointing robot at a specific target angle."""
        self.swerve_drive.drive_point(
            self.translationX,
            self.translationY,
            self.point_target,
        )

        if not self.point_to_target:
            self.next_state("free")

    @state
    def point_towards_joy(self):
        """Drive while pointing robot in the direction of the right joystick."""
        if self.point_to_target:
            self.next_state("point_towards_target")
            return

        self.swerve_drive.drive_point_joy(
            self.translationX,
            self.translationY,
            self.point_joy_x,
            self.point_joy_y,
        )

        if not self.point_joy_target:
            self.next_state("free")

    @state
    def drive_sysid_state(self):
        """System identification mode - applies constant voltage for characterization."""
        if not self.drive_sysid:
            self.next_state("free")
            return
        self.swerve_drive.sysid_drive(self.sysid_volts)

    @state
    def going_to_pose(self):
        """Autonomous pose targeting state - drives robot to desired pose."""
        if not self.go_to_pose:
            self.next_state("free")
            return
        self.swerve_drive.set_desired_pose(self.desired_pose)

    @state
    def run_auton_routine(self):
        """Autonomous routine state - follows trajectory samples or manual commands."""
        if DriverStation.isTeleop():
            self.next_state("free")
            return

        if self.drive_auto_man:
            self.swerve_drive.drive(
                self.translationX,
                self.translationY,
                self.rotationX,
                self.field_relative,
            )
        elif self.sample is not None:
            self.swerve_drive.follow_trajectory(self.sample)
