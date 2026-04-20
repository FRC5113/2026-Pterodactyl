from magicbot import timed_state

from autonomous.auto_base import AutoBase
from components.drive_control import DriveControl
from components.intake import Intake
from components.shooter_controller import ShooterController

"""
Trajectories: (start with trajectory:)
- outpost_shoot
- hub_shoot
- lt_intake
- rt_intake
- end_of_lt_intake_to_shoot
- end_of_rt_intake_to_shoot
- lt_outpost
- rt_outpost
- auto_test
- hard_shoot_outpost
"""

"""
States: (start with state:)
- shoot
- outpost_wait
- intake_down
"""


class hub_shoot_outpost_shoot(AutoBase):
    MODE_NAME = "H-shoot-outpost-shoot"
    DEFAULT = True

    drive_control: DriveControl
    shooter_controller: ShooterController
    intake: Intake

    def __init__(self):
        super().__init__(
            [
                "state:move_back",
                "state:intake_out",
                "state:hard_shoot",
                "trajectory:hard_shoot_outpost",
                "state:outpost_wait",
                "trajectory:outpost_shoot",
                "state:shoot",
            ]
        )

    @timed_state(duration=1.5, next_state="next_step")
    def move_back(self):
        self.drive_control.drive_auto_manual(
            translationX=-1, translationY=0.0, rotationX=0.0, field_relative=False
        )

    @timed_state(duration=1, next_state="next_step")
    def intake_out(self):
        self.intake.set_arm_voltage(-8)

    @timed_state(duration=5, next_state="next_step")
    def hard_shoot(self):
        self.shooter_controller.request_force_shoot(45.5)
        self.intake.set_voltage(-10)


class hard_code_shoot(AutoBase):
    MODE_NAME = "Hard Code Shoot"

    drive_control: DriveControl
    shooter_controller: ShooterController
    intake: Intake

    def __init__(self):
        super().__init__(
            [
                "state:move_back",
                "state:intake_out",
                "state:hard_shoot",
            ]
        )

    @timed_state(duration=1.5, next_state="next_step")
    def move_back(self):
        self.drive_control.drive_auto_manual(
            translationX=-1, translationY=0.0, rotationX=0.0, field_relative=False
        )

    @timed_state(duration=16.5, next_state="next_step")
    def hard_shoot(self):
        self.shooter_controller.request_force_shoot(45.5)
        self.intake.set_voltage(-10)


class hard_code_shoot_angled(AutoBase):
    MODE_NAME = "Hard Code Shoot angled"

    drive_control: DriveControl
    shooter_controller: ShooterController
    intake: Intake

    def __init__(self):
        super().__init__(
            [
                "state:move_back",
                "state:intake_out",
                "state:hard_shoot",
            ]
        )

    @timed_state(duration=1.5, next_state="next_step")
    def move_back(self):
        self.drive_control.drive_auto_manual(
            translationX=-1, translationY=0.0, rotationX=0.0, field_relative=False
        )

    @timed_state(duration=16.5, next_state="next_step")
    def hard_shoot(self):
        self.shooter_controller.request_force_shoot(46)
        self.intake.set_voltage(-10)


class hub_outpost_shoot(AutoBase):
    MODE_NAME = "H-Outpost>Shoot"

    def __init__(self):
        super().__init__(
            [
                "state:intake_down",
                "trajectory:hub_outpost",
                "state:outpost_wait",
                "trajectory:outpost_shoot",
                "state:shoot",
            ]
        )


class lt_intake_center_shoot(AutoBase):
    MODE_NAME = "Lt-Intake_Center>Shoot"

    def __init__(self):
        super().__init__(
            [
                "state:intake_down",
                "trajectory:lt_intake",
                "state:go_forward_and_intake",
                "trajectory:end_of_lt_intake_to_shoot",
                "state:shoot",
            ]
        )


class rt_intake_center_shoot(AutoBase):
    MODE_NAME = "Rt-Intake_Center>Shoot"

    def __init__(self):
        super().__init__(
            [
                "state:intake_down",
                "trajectory:rt_intake",
                "state:go_forward_and_intake",
                "trajectory:end_of_rt_intake_to_shoot",
                "state:shoot",
            ]
        )


class lt_outpost_shoot(AutoBase):
    MODE_NAME = "Lt-Outpost>Shoot"

    def __init__(self):
        super().__init__(
            [
                "state:intake_down",
                "trajectory:lt_outpost",
                "state:outpost_wait",
                "trajectory:outpost_shoot",
                "state:shoot",
            ]
        )


class rt_outpost_shoot(AutoBase):
    MODE_NAME = "Rt-Outpost>Shoot"

    def __init__(self):
        super().__init__(
            [
                "state:intake_down",
                "trajectory:rt_outpost",
                "state:outpost_wait",
                "trajectory:outpost_shoot",
                "state:shoot",
            ]
        )
