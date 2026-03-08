# import cProfile

# Patch out the expensive traceback in Phoenix6 error reports (see _report_status_no_traceback).
import math

import wpilib
from magicbot import feedback
from phoenix6 import CANBus
from phoenix6.hardware import TalonFX, TalonFXS
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpilib import (
    DigitalInput,
    DriverStation,
    DutyCycleEncoder,
    Field2d,
)
from wpimath import units
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation3d, Transform3d

from autonomous.auto_base import AutoBase
from components.drive_control import DriveControl
from components.intake import Intake
from components.leds import LEDStrip
from components.odometry import Odometry
from components.shooter import Shooter, ShootSettings
from components.swerve_drive import SwerveDrive
from game import is_alliance_hub_active
from generated.tuner_constants import TunerConstants
from lemonlib import LemonCamera, LemonInput, LemonRobot
from lemonlib.smart import SmartPreference, SmartProfile
from lemonlib.util import (
    AlertManager,
    AlertType,
    LEDController,
    curve,
)


class MyRobot(LemonRobot):
    # led_strip: LEDStrip

    drive_control: DriveControl
    odometry: Odometry

    swerve_drive: SwerveDrive
    shooter: Shooter
    intake: Intake

    # greatest speed that chassis should move (not greatest possible speed)
    top_speed = SmartPreference(4.7)
    top_omega = SmartPreference(6.0)

    rasing_slew_rate= SmartPreference(8.0)
    # falling_slew_rate = SmartPreference(20.0)
    firstRun = True

    # flywheel_speed = SmartPreference(30.0)

    def createObjects(self):
        """This method is where all attributes to be injected are
        initialized. This is done here rather that inside the components
        themselves so that all constants and initialization parameters
        can be found in one place. Also, attributes shared by multiple
        components, such as the NavX, need only be created once.
        """
        self.tuning_enabled = False

        self.rio_canbus = CANBus.roborio()

        """
        SWERVE
        Swerve hardware (TalonFX drive/steer motors, CANcoders, Pigeon2)
        is now created internally by the Phoenix 6 SwerveDrivetrain via
        generated/tuner_constants.py.  Only the high-level constants needed
        by SwerveDrive are set here.
        """

        self.max_speed: units.meters_per_second = TunerConstants.speed_at_12_volts

        self.translation_profile = SmartProfile(
            "translation",
            {
                "kP": 5.0,
                "kI": 0.0,
                "kD": 0.0,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )
        self.rotation_profile = SmartProfile(
            "rotation",
            {
                "kP": 7.0,
                "kI": 0.0,
                "kD": 0.1,
                "kMaxV": 8.0,
                "kMaxA": 40.0,
                "kMinInput": -math.pi,
                "kMaxInput": math.pi,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )

        # Steer motor closed-loop gains (applied to all 4 steer TalonFXs)
        self.steer_profile = SmartProfile(
            "swerve_steer",
            {
                "kP": TunerConstants._steer_gains.k_p,
                "kI": TunerConstants._steer_gains.k_i,
                "kD": TunerConstants._steer_gains.k_d,
                "kS": TunerConstants._steer_gains.k_s,
                "kV": TunerConstants._steer_gains.k_v,
                "kA": TunerConstants._steer_gains.k_a,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )

        # Drive motor closed-loop gains (applied to all 4 drive TalonFXs)
        self.drive_profile = SmartProfile(
            "swerve_drive",
            {
                "kP": TunerConstants._drive_gains.k_p,
                "kI": TunerConstants._drive_gains.k_i,
                "kD": TunerConstants._drive_gains.k_d,
                "kS": TunerConstants._drive_gains.k_s,
                "kV": TunerConstants._drive_gains.k_v,
                "kA": TunerConstants._drive_gains.k_a,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )

        """
        INTAKE
        """

        self.intake_spin_motor = TalonFX(51)
        self.intake_left_motor = TalonFX(52)
        self.intake_right_motor = TalonFX(53)
        self.intake_left_encoder = DutyCycleEncoder(DigitalInput(0))
        self.intake_right_encoder = DutyCycleEncoder(DigitalInput(1))

        self.intake_spin_amps: units.amperes = 40.0
        self.intake_arm_amps: units.amperes = 20.0

        self.intake_profile = SmartProfile(
            "intake",
            {
                "kP": 0.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.0,
                "kG": 0.0,
                "kMaxV": 0.0,
                "kMaxA": 0.0,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )

        """
        SHOOTER
        """
        self.shooter_left_motor = TalonFX(2, self.rio_canbus)
        self.shooter_right_motor = TalonFX(3, self.rio_canbus)

        self.shooter_gear_ratio = 1.0
        self.shooter_amps: units.amperes = 60.0

        self.shooter_angle = 23  # degrees

        self.shooter_profile = SmartProfile(
            "shooter",
            {
                "kP": 0.3,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.11137,
                "kA": 0.29663,
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )

        """
        INDEXER
        """
        self.shooter_left_kicker_motor = TalonFXS(4, self.rio_canbus)
        self.shooter_right_kicker_motor = TalonFXS(5, self.rio_canbus)
        self.shooter_vibrator_motor = TalonFXS(6, self.rio_canbus)
        self.shooter_kicker_amps: units.amperes = 20.0
        self.shooter_conveyor_amps: units.amperes = 10.0
        """
        ODOMETRY
        """
        # Custom apriltag field layout
        # self.field_layout = AprilTagFieldLayout(
        #     str(Path(__file__).parent.resolve() / "2026_test_field.json")
        # )

        self.field_layout = AprilTagFieldLayout.loadField(
            AprilTagField.k2026RebuiltWelded
        )

        # Robot to Camera Transforms
        ox = 0.298
        oy = 0.298
        self.rtc_front_left = Transform3d(0.0, 0.0, 0.0, Rotation3d(0, 30, 45))
        self.rtc_front_right = Transform3d(0.0, 0.0, 0.0, Rotation3d(0, 30, -45))
        self.rtc_back_left = Transform3d(
            -ox, -oy, 0.21, Rotation3d(0, 1.0472, (5 * math.pi) / 4)
        )
        self.rtc_back_right = Transform3d(
            ox, -oy, 0.21, Rotation3d(0, 1.0472, (7 * math.pi) / 4)
        )

        self.camera_front_left = LemonCamera(
            "Front_Left", self.rtc_front_left, self.field_layout
        )
        self.camera_front_right = LemonCamera(
            "Front_Right", self.rtc_front_right, self.field_layout
        )

        self.camera_back_left = LemonCamera(
            "Back_Left", self.rtc_back_left, self.field_layout
        )
        self.camera_back_right = LemonCamera(
            "Back_Right", self.rtc_back_right, self.field_layout
        )

        """
        MISCELLANEOUS
        """

        self.fms = DriverStation.isFMSAttached()

        # driving curve
        self.sammi_curve = curve(
            lambda x: 1.89 * x**3 + 0.61 * x, 0.0, deadband=0.1, max_mag=1.0
        )

        self.led_length = 150
        self.leds = LEDController(2, self.led_length)

        # alerts
        AlertManager(self.logger)
        if self.low_bandwidth:
            AlertManager.instant_alert(
                "Low Bandwidth Mode is active! Tuning is disabled.", AlertType.INFO
            )

        self.estimated_field = Field2d()

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.alliance = True
        else:
            self.alliance = False

    def enabledperiodic(self):
        self.drive_control.engage()

    def autonomousInit(self):
        # globalProfiler.enable()
        pass

    def robotPeriodic(self) -> None:
        if self.tuning_enabled:
            watchdog = self.watchdog
            self.__sd_update()
            watchdog.addEpoch("SmartDashboard")
            self.__lv_update()
            watchdog.addEpoch("LiveWindow")

    def autonomousPeriodic(self):
        self._display_auto_trajectory()

    def teleopInit(self):
        # globalProfiler.enable()
        # initialize HIDs here in case they are changed after robot initializes
        self.primary = LemonInput(0)
        self.secondary = LemonInput(1)

        self.x_filter = SlewRateLimiter(
            self.rasing_slew_rate  # , self.falling_slew_rate
        )
        self.y_filter = SlewRateLimiter(
            self.rasing_slew_rate  # , self.falling_slew_rate
        )
        self.omega_filter = SlewRateLimiter(
            self.rasing_slew_rate  # , self.falling_slew_rate
        )
        # Cache inputs called multiple times

    def teleopPeriodic(self):
        primary_r2 = self.primary.getR2Axis()
        primary_l2 = self.primary.getL2Axis()
        primary_ly = self.primary.getLeftY()
        primary_lx = self.primary.getLeftX()
        primary_rx = self.primary.getRightX()
        primary_ry = self.primary.getRightY()

        secondary_left_bumper = self.secondary.getLeftBumper()
        secondary_right_bumper = self.secondary.getRightBumper()

        if self.secondary.getRightStickButton():
            self.intake.set_bypass_limits()

        """
        SWERVE
        """
        with self.consumeExceptions():
            # if both 25% else 50 or 75
            if (primary_r2 >= 0.8) and (primary_l2 >= 0.8):
                mult = 0.25
            elif primary_r2 >= 0.8:
                mult = 0.75
            elif primary_l2 >= 0.8:
                mult = 0.5
            else:
                mult = 1.0

            # only apply the curve and slew rate if the input is above the deadband, otherwise set to 0 to avoid useless math
            if abs(primary_ly) <= 0.0:
                vx = 0.0
            else:
                vx = self.x_filter.calculate(
                    self.sammi_curve(primary_ly) * mult * self.top_speed
                )
            if abs(primary_lx) <= 0.0:
                vy = 0.0
            else:
                vy = self.omega_filter.calculate(
                    self.sammi_curve(primary_lx) * mult * self.top_speed
                )

            # if self.primary.getLeftBumper():
            # if abs(primary_rx) <= 0.0:
            #     omega = 0.0
            # else:
            omega = self.y_filter.calculate(
                self.sammi_curve(primary_rx) * self.top_omega
            )
            self.drive_control.drive_manual(
                vx,
                vy,
                omega,
                not self.primary.getCreateButton(),  # temporary
            )
            # else:
            #     self.drive_control.drive_point_joy(  # Keaton mode
            #         vx, vy, primary_rx, primary_ry
            #     )

            if self.primary.getCrossButton():
                self.drive_control.Xbrake()

            if self.primary.getSquareButton():
                self.swerve_drive.reset_gyro()

        """
        INTAKE
        """
        with self.consumeExceptions():
            if (
                secondary_right_bumper
                and secondary_left_bumper
                and self.secondary.getAButton()
            ):
                self.intake.zero_encoders()
            elif self.secondary.getLeftTriggerAxis() >= 0.8:
                self.intake.set_voltage(8.0)
            elif secondary_left_bumper:
                self.intake.set_voltage(-8.0)

            if self.secondary.getBButton():
                self.intake.set_arm_voltage(-4.0)

            if self.secondary.getXButton():
                self.intake.set_arm_voltage(4.0)

        """
        SHOOTER
        """
        with self.consumeExceptions():
            "A - LOW"
            "X - MIDDLE"
            "B - HIGH"
            "Y - Vibrate and Index"
            if self.secondary.getAButton():
                self.shooter.set_shooter(ShootSettings.LOW)
            elif self.secondary.getXButton():
                self.shooter.set_shooter(ShootSettings.MIDDLE)
            elif self.secondary.getBButton():
                self.shooter.set_shooter(ShootSettings.HIGH)
            else:
                self.shooter.set_shooter(ShootSettings.OFF)

            if self.secondary.getYButton():
                self.shooter.vibrator_on()
                self.shooter.kicker_on
            else:
                self.shooter.vibrator_off()
                self.shooter.kicker_off() 

    def disabledPeriodic(self):
        # self.odometry.execute()
        pass

    def disabledInit(self):
        # if not self.firstRun:
        #     try:
        #         globalProfiler.dump_stats("/home/lvuser/teleop.prof")
        #         globalProfiler.disable()
        #     except FileNotFoundError:
        #         globalProfiler.dump_stats("./temp.prof")
        #     except Exception as e:
        # else:
        #     self.firstRun = False
        pass

    def _display_auto_trajectory(self) -> None:
        selected_auto = self._automodes.chooser.getSelected()
        if isinstance(selected_auto, AutoBase):
            selected_auto.display_trajectory()

    # @feedback
    def hub_status(self) -> bool:
        return is_alliance_hub_active()

    # @feedback
    def display_auto_state(self) -> None:
        selected_auto = self._automodes.chooser.getSelected()
        if isinstance(selected_auto, AutoBase):
            return selected_auto.current_state
        return "No Auto Selected"


if __name__ == "__main__":
    wpilib.run(MyRobot)
