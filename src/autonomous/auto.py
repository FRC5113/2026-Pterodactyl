"""Module for auto."""

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
    """hard_code_shoot class."""
    MODE_NAME = "Hard Code Shoot"
    DEFAULT = True

    shooter_controller: ShooterController
    drive_control: DriveControl

    @timed_state(first=True, duration=1.5, next_state="shoot")
    def move_back(self):
        """Execute move_back."""
        self.drive_control.drive_auto_manual(
            translationX=-1, translationY=0.0, rotationX=0.0, field_relative=False
        )

    @timed_state(duration=5.0)
    def shoot(self):
        """Execute shoot."""
        self.shooter_controller.request_shoot()


class hub_outpost_shoot(AutoBase):
    """hub_outpost_shoot class."""
    MODE_NAME = "Hub>Outpost>Shoot"

    def __init__(self):
        """Execute __init__."""
        super().__init__(
            [
                "trajectory:hub_outpost",
                "state:outpost_wait",
                "trajectory:outpost_shoot",
                "state:shoot",
            ]
        )


class hub_shoot(AutoBase):
    """hub_shoot class."""
    MODE_NAME = "Hub>Shoot"

    def __init__(self):
        """Execute __init__."""
        super().__init__(
            [
                "trajectory:hub_shoot",
                "state:shoot",
            ]
        )
