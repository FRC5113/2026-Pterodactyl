from magicbot import timed_state,AutonomousStateMachine

from autonomous.auto_base import AutoBase
from components.drive_control import DriveControl
from components.shooter_controller import ShooterController 

"""
Trajectories: (start with trajectory:)
- hub_climb
- outpost_shoot
- hub_outpost
- shoot_climb
- hub_shoot
"""

"""
States: (start with state:)
- shoot
- climb
- outpost_wait
"""


class hard_code_shoot(AutoBase):
    MODE_NAME = "Hard Code Shoot"
    DEFAULT = True

    drive_control: DriveControl
    shooter_controller: ShooterController

    def __init__(self):
        super().__init__(
            [
                "state:move_back",
                "state:shoot",
            ]
        )

    @timed_state(duration=1.5, next_state="next_step")
    def move_back(self):
        self.drive_control.drive_auto_manual(
            translationX=-1, translationY=0.0, rotationX=0.0, field_relative=False
        )

    @timed_state(duration=5.0, next_state="next_step")
    def shoot(self):
        self.shooter_controller.request_shoot()

class hard_code_spin_shoot(AutoBase):
    MODE_NAME = "Hard Code Spin Shoot"
    
    drive_control: DriveControl
    shooter_controller: ShooterController

    def __init__(self):
        super().__init__(
            [
                "state:move_back",
                "state:spin",
                "state:shoot",
            ]
        )

    @timed_state(duration=1.5, next_state="next_step")
    def move_back(self):
        self.drive_control.drive_auto_manual(
            translationX=-1, translationY=0.0, rotationX=0.0, field_relative=False
        )
        
    @timed_state(duration=5, next_state="next_step")
    def spin(self):
        self.drive_control.drive_auto_manual(
            translationX=0.0, translationY=0.0, rotationX=12.0, field_relative=False
        )

    @timed_state(duration=5.0, next_state="next_step")
    def shoot(self):
        self.shooter_controller.request_shoot()

class hub_outpost_shoot(AutoBase):
    MODE_NAME = "H-Outpost>Shoot"

    def __init__(self):
        super().__init__(
            [
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
                "trajectory:rt_outpost",
                "state:outpost_wait",
                "trajectory:outpost_shoot",
                "state:shoot",
            ]
        )
