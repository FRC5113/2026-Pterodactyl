from magicbot import feedback, will_reset_to
from wpilib import Color, DriverStation, Timer

from components.swerve_drive import SwerveDrive
from lemonlib.util import AlertManager, AlertType, LEDController


class LEDStrip:
    swerve_drive: SwerveDrive
    leds: LEDController

    justin_bool = will_reset_to(False)
    is_aligned = will_reset_to(False)

    def setup(self):
        """Initialize LED strip with default colors."""
        self.leds.set_solid_color((10, 10, 10))

        self.error_color = (255, 0, 0)
        self.warning_color = (255, 255, 0)
        self.auton_color = (255, 50, 0)
        self.disabled = (10, 10, 10)
        self.idle = (50, 50, 50)

        self.animation_period = 0.05
        self.feedback_period = 0.2
        self._last_animation_time = 0.0
        self._last_feedback_time = 0.0
        self._cached_colors: list[str] = []

    def on_disable(self):
        """Set LEDs to dim disabled color when robot is disabled."""
        self.leds.set_solid_color(self.disabled)

    @feedback
    def get_colors(self) -> list[str]:
        """Get current LED colors for dashboard display."""
        now = Timer.getFPGATimestamp()
        if now - self._last_feedback_time >= self.feedback_period:
            self._last_feedback_time = now
            self._cached_colors = [
                Color(led.r, led.g, led.b).hexString() for led in self.leds.buffer
            ]
        return self._cached_colors

    def has_warnings_present(self) -> bool:
        """Check if any warning-level alerts are active."""
        return len(AlertManager.get_strings(AlertType.WARNING)) > 0

    def has_errors_present(self) -> bool:
        """Check if any error-level alerts are active."""
        return len(AlertManager.get_strings(AlertType.ERROR)) > 0

    def justin_fun(self):
        """Trigger special LED animation."""
        self.justin_bool = True

    def execute(self):
        """Update LED strip based on robot state with priority-based display."""
        now = Timer.getFPGATimestamp()

        if self.has_errors_present():
            self.leds.set_solid_color(self.error_color)
        elif self.has_warnings_present():
            self.leds.set_solid_color(self.warning_color)
        elif DriverStation.isAutonomousEnabled():
            if now - self._last_animation_time >= self.animation_period:
                self._last_animation_time = now
                self.leds.move_across(self.auton_color, 20, 50)
        else:
            if now - self._last_animation_time >= self.animation_period:
                self._last_animation_time = now
                self.leds.scolling_rainbow()
