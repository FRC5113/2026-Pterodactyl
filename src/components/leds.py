from magicbot import will_reset_to
from wpilib import Color, DriverStation

from components.intake import Intake, IntakeAngle
from components.shooter_controller import ShooterController
from components.swerve_drive import SwerveDrive
from game import is_alliance_hub_active
from lemonlib.util import AlertManager, AlertType, LEDController


class LEDStrip:
    shooter_controller: ShooterController
    swerve_drive: SwerveDrive
    intake: Intake
    leds: LEDController

    justin_bool = will_reset_to(False)
    is_aligned = will_reset_to(False)

    """
    INITIALIZATION METHODS
    """

    def setup(self):
        self.error_color = (255, 0, 0)
        self.warning_color = (255, 255, 0)
        self.auton_color = (255, 50, 0)
        self.disabled = (10, 10, 10)
        self.idle = (77, 77, 0)

        self.intake_up = (141, 55, 255)

        self.zone_active = (0, 255, 255)

        self.leds.set_solid_color(self.disabled)

    def on_disable(self):
        self.leds.set_solid_color(self.disabled)

    """
    INFORMATIONAL METHODS
    """

    # @feedback
    def get_colors(self) -> list[str]:
        """Returns LED colors in list of hex strings"""
        return [Color(led.r, led.g, led.b).hexString() for led in self.leds.buffer]

    def has_warnings_present(self) -> bool:
        return len(AlertManager.get_strings(AlertType.WARNING)) > 0

    def has_errors_present(self) -> bool:
        return len(AlertManager.get_strings(AlertType.ERROR)) > 0

    """

    CONTROL METHODS
    """

    def justin_fun(self):
        # I miss you justin
        self.justin_bool = True

    """
    EXECUTE
    """

    def execute(self):
        if self.has_errors_present():
            self.leds.set_solid_color(self.error_color)

        elif self.has_warnings_present():
            self.leds.set_solid_color(self.warning_color)

        elif DriverStation.isAutonomousEnabled():
            self.leds.move_across(self.auton_color, 20, 50)

        elif self.intake.get_position() > IntakeAngle.LED_DOWN.value:
            self.leds.set_solid_color(self.intake_up)

        elif is_alliance_hub_active():
            self.leds.set_solid_color(self.zone_active)

        else:
            self.leds.move_across(self.idle)
