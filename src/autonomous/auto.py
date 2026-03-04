from magicbot import AutonomousStateMachine, timed_state

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


class hard_code_shoot(AutonomousStateMachine):
    MODE_NAME = "Hard Code Shoot"
    DEFAULT = True

    shooter_controller: ShooterController
    drive_control: DriveControl

    @timed_state(first=True, duration=1.5, next_state="shoot")
    def move_back(self):
        self.drive_control.drive_auto_manual(
            translationX=-1, translationY=0.0, rotationX=0.0, field_relative=False
        )

    @timed_state(duration=5.0)
    def shoot(self):
        self.shooter_controller.request_shoot()


class hub_outpost_shoot(AutoBase):
    MODE_NAME = "Hub>Outpost>Shoot"

    def __init__(self):
        super().__init__(
            [
                "trajectory:hub_outpost",
                "state:outpost_wait",
                "trajectory:outpost_shoot",
                "state:shoot",
            ]
        )


class hub_shoot(AutoBase):
    MODE_NAME = "Hub>Shoot"

    def __init__(self):
        super().__init__(
            [
                "trajectory:hub_shoot",
                "state:shoot",
            ]
        )
class lt_immediate_shoot(AutoBase):
    MODE_NAME = "LtImmediateShoot"
    def __init__(self):
        super().__init__(
            [
                "trajectory:lt_immediate_shoot",
                "state:shoot"
            ]
        )

class rt_immediate_shoot(AutoBase):
    MODE_NAME = "RtImmediateShoot"
    def __init__(self):
        super().__init__(
            [
                "trajectory:rt_immediate_shoot",
                "state:shoot"
            ]
        )
class hub_immediate_shoot(AutoBase):
    MODE_NAME = "HubImmediateShoot"
    def __init__(self):
        super().__init__(
            [
                "trajectory:hub_immediate_shoot",
                "state:shoot"
            ]
        )

class lt_intake_center_shoot(AutoBase):
    MODE_NAME = "LtIntakeCenterShoot"
    def __init__(self):
        super().__init__(
            [
                "trajectory:lt_intake",
                "state:go_foward_and_intake_lt",
                "trajectory:end_of_lt_intake_to_shoot",
                "state:shoot"
            ]
        )
class rt_intake_center_shoot(AutoBase):
    MODE_NAME = "RtIntakeCenterShoot"
    def __init__(self):
        super().__init__(
            [
                "trajectory:rt_intake",
                "state:go_foward_and_intake_rt",
                "trajectory:end_of_rt_intake_to_shoot",
                "state:shoot"
            ]
        )

class lt_outpost_shoot(AutoBase):
    MODE_NAME = "LtOutpostShoot"
    def __init__(self):
        super().__init__(
            [
                "trajectory:lt_outpost",
                "state:outpost_wait",
                "trajectory:outpost_shoot",
                "state:shoot"
            ]
        )

class rt_outpost_shoot(AutoBase):
    MODE_NAME = "RtOutpostShoot"
    def __init__(self):
        super().__init__(
            [
                "trajectory:rt_outpost",
                "state:outpost_wait",
                "trajectory:outpost_shoot",
                "state:shoot"
            ]
        )

class hub__outpost_shoot(AutoBase):
    MODE_NAME = "HubOutpostShoot"
    def __init__(self):
        super().__init__(
            [
                "trajectory:hub_outpost",
                "state:outpost_wait",
                "trajectory:outpost_shoot",
                "state:shoot"
            ]
        )

class lt_outpost_depot_shoot(AutoBase):
    MODE_NAME = "LtOutpostDepotShoot"
    def __init__(self):
        super().__init__(
            [
                "trajectory:lt_outpost",
                "state:outpost_wait",
                "trajectory:outpost_depot_shoot",
                "state:shoot"
            ]
        )


class rt_outpost_depot_shoot(AutoBase):
    MODE_NAME = "RtOutpostDepotShoot"
    def __init__(self):
        super().__init__(
            [
                "trajectory:rt_outpost",
                "state:outpost_wait",
                "trajectory:outpost_depot_shoot",
                "state:shoot"
            ]
        )

class hub_outpost_depot_shoot(AutoBase):
    MODE_NAME = "HubOutpostDepotShoot"
    def __init__(self):
        super().__init__(
            [
                "trajectory:hub_outpost",
                "state:outpost_wait",
                "trajectory:outpost_depot_shoot",
                "state:shoot"
            ]
        )