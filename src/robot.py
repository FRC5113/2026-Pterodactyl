import math

import wpilib
from wpilib import (
    DigitalInput,
    DriverStation,
    DutyCycleEncoder,
    Field2d,
    RobotController,
)

from phoenix6 import CANBus
from phoenix6.hardware import TalonFX, TalonFXS

from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

from wpimath import units
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation3d, Transform3d

from autonomous.auto_base import AutoBase

from components.drive_control import DriveControl
from components.intake import Intake
from components.leds import LEDStrip
from components.odometry import Odometry
from components.shooter import Shooter
from components.shooter_controller import ShooterController
from components.swerve_drive import SwerveDrive

from generated.tuner_constants import TunerConstants

from lemonlib import LemonCamera, LemonInput, LemonRobot, fms_feedback # type: ignore (lemonlib needs strict typing)
from lemonlib.smart import SmartPreference, SmartProfile
from lemonlib.util import (
    AlertManager,
    AlertType,
    LEDController,
    curve,
)

from FuelSim.fuel_sim import FuelSim


class MyRobot(LemonRobot):
    led_strip: LEDStrip
    shooter_controller: ShooterController

    drive_control: DriveControl
    odometry: Odometry

    swerve_drive: SwerveDrive
    shooter: Shooter
    intake: Intake

    top_speed = SmartPreference(4.7)  # Maximum chassis speed (m/s)
    top_omega = SmartPreference(6.0)  # Maximum rotation speed (rad/s)

    raising_slew_rate: SmartPreference = SmartPreference(8.0)  # Rate of acceleration
    falling_slew_rate: SmartPreference = SmartPreference(20.0)  # Rate of deceleration

    flywheel_speed = SmartPreference(30.0)  # Target flywheel RPS

    def createObjects(self) -> None:
        """
        Initialize all robot hardware and configuration parameters.
        """
        self.tuning_enabled = True  # Enable NetworkTables tuning for PID values

        self.rio_canbus = CANBus.roborio()

        self.robot_width: units.meters = 0.5
        self.robot_length: units.meters = 0.5
        self.bumper_height: units.meters = 0.08

        """""""""""""""
        SWERVE CONFIG
        """""""""""""""

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

        self.steer_profile = SmartProfile(
            "swerve_steer",
            {
                "kP": TunerConstants._steer_gains.k_p, # type: ignore
                "kI": TunerConstants._steer_gains.k_i, # type: ignore
                "kD": TunerConstants._steer_gains.k_d, # type: ignore
                "kS": TunerConstants._steer_gains.k_s, # type: ignore
                "kV": TunerConstants._steer_gains.k_v, # type: ignore
                "kA": TunerConstants._steer_gains.k_a, # type: ignore
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )

        self.drive_profile = SmartProfile(
            "swerve_drive",
            {
                "kP": TunerConstants._drive_gains.k_p, # type: ignore
                "kI": TunerConstants._drive_gains.k_i, # type: ignore
                "kD": TunerConstants._drive_gains.k_d, # type: ignore
                "kS": TunerConstants._drive_gains.k_s, # type: ignore
                "kV": TunerConstants._drive_gains.k_v, # type: ignore
                "kA": TunerConstants._drive_gains.k_a, # type: ignore
            },
            (not self.low_bandwidth) and self.tuning_enabled,
        )

        """""""""""""""
        INTAKE CONFIG
        """""""""""""""
        
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

        """""""""""""""
        SHOOTER CONFIG
        """""""""""""""

        self.shooter_left_motor = TalonFX(2, self.rio_canbus)
        self.shooter_right_motor = TalonFX(3, self.rio_canbus)

        self.shooter_gear_ratio = 1.0
        self.shooter_amps: units.amperes = 60.0

        self.shooter_angle: units.degrees = 23 # desired shooter angle

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

        self.shooter_left_kicker_motor = TalonFXS(4, self.rio_canbus)
        self.shooter_right_kicker_motor = TalonFXS(5, self.rio_canbus)

        self.shooter_conveyor_motor = TalonFXS(6, self.rio_canbus)

        self.shooter_kicker_amps: units.amperes = 20.0
        self.shooter_conveyor_amps: units.amperes = 10.0

        """""""""""""""
        ODOMETRY N' VISION CONFIG
        """""""""""""""

        self.field_layout = AprilTagFieldLayout.loadField(
            AprilTagField.k2026RebuiltWelded
        )

        # Robot to cam transforms
        ox: units.meters = 0.298
        oy: units.meters = 0.298
        
        # Front
        self.rtc_front_left = Transform3d(0.0, 0.0, 0.0, Rotation3d(0, 30, 45))
        self.rtc_front_right = Transform3d(0.0, 0.0, 0.0, Rotation3d(0, 30, -45))
        
        # Back
        self.rtc_back_left = Transform3d(
            -ox, -oy, 0.21, Rotation3d(0, 1.0472, (5 * math.pi) / 4)  # 60 deg, 225 deg yaw
        )
        self.rtc_back_right = Transform3d(
            ox, -oy, 0.21, Rotation3d(0, 1.0472, (7 * math.pi) / 4)  # 60 deg, 315 deg yaw
        )

        # TODO: Add front LemonCameras

        self.camera_back_left = LemonCamera(
            "Arducam OV9281 USB Camera", self.rtc_back_left, self.field_layout
        )
        self.camera_back_right = LemonCamera(
            "PC_Camera", self.rtc_back_right, self.field_layout
        )

        """""""""""""""
        MISC CONFIG
        """""""""""""""

        self.fms = DriverStation.isFMSAttached()

        # Full speed at edges
        self.sammi_curve = curve(
            lambda x: 1.89 * x**3 + 0.61 * x, 0.0, deadband=0.1, max_mag=1.0
        )

        self.led_length = 112  # Number of LEDs
        self.leds = LEDController(2, self.led_length)

        AlertManager(self.logger)
        if self.low_bandwidth:
            AlertManager.instant_alert(
                "Low Bandwidth Mode is active! Tuning is disabled.", AlertType.INFO
            )

        self.estimated_field = Field2d()
        self._displayed_auto = None

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.alliance = True  # Red alliance
        else:
            self.alliance = False  # Blue alliance

        self.fuel_sim = FuelSim()
        self.fuel_sim.spawn_starting_fuel()

        self.fuel_sim.register_robot(
            self.robot_width,
            self.robot_length,
            self.bumper_height,
            lambda: self.swerve_drive.get_estimated_pose(),
            lambda: self.swerve_drive.get_field_speeds()
        )

        self.fuel_sim.register_intake(
            0, 12, 0, 12
        )

        self.fuel_sim.set_subticks(1)

        self.fuel_sim.start()
    
    def enabledperiodic(self) -> None:
        self.drive_control.engage()
        self.shooter_controller.engage()

    def robotPeriodic(self) -> None:
        # Adds simulationPeriodic - Magic does not natively support.
        if self.isSimulation():
            self.simulationPeriodic()

    def autonomousPeriodic(self) -> None:
        self._display_auto_trajectory()

    def teleopInit(self) -> None:
        self.primary = LemonInput(0)
        self.secondary = LemonInput(1)

        self.x_filter = SlewRateLimiter(self.raising_slew_rate) # type: ignore (assume float)
        self.y_filter = SlewRateLimiter(self.raising_slew_rate) # type: ignore (assume float)
        self.omega_filter = SlewRateLimiter(self.raising_slew_rate) # type: ignore (assume float)

    def teleopPeriodic(self):
        primary_r2 = self.primary.getR2Axis()  # Right trigger
        primary_l2 = self.primary.getL2Axis()  # Left trigger
        primary_ly = self.primary.getLeftY()  # Left stick Y (forward/back)
        primary_lx = self.primary.getLeftX()  # Left stick X (strafe)
        primary_rx = self.primary.getRightX()  # Right stick X (rotation)
        primary_ry = self.primary.getRightY()  # Right stick Y (rotation aiming)

        """""""""""""""
        SWERVE CONTROL
        """""""""""""""

        # Both triggers = 25% speed
        # R2 only = 75% speed
        # L2 only = 50% speed
        # Else = 100% speed
        if (primary_r2 >= 0.8) and (primary_l2 >= 0.8):
            mult = 0.25
        elif primary_r2 >= 0.8:
            mult = 0.75
        elif primary_l2 >= 0.8:
            mult = 0.5
        else:
            mult = 1.0

        if abs(primary_ly) <= 0.0:
            vx = 0.0
        else:
            vx = self.x_filter.calculate(
                self.sammi_curve(primary_ly) * mult * self.top_speed # type: ignore (assume float)
            )
        if abs(primary_lx) <= 0.0:
            vy = 0.0
        else:
            vy = self.omega_filter.calculate(
                self.sammi_curve(primary_lx) * mult * self.top_speed # type: ignore (assume float)
            )

        if self.primary.getLeftBumper():
            if abs(primary_rx) <= 0.0:
                omega = 0.0
            else:
                omega = self.y_filter.calculate(
                    self.sammi_curve(primary_rx) * self.top_omega # type: ignore (assume float)
                )
            self.drive_control.drive_manual(
                vx,
                vy,
                omega,
                not self.primary.getCreateButton(), # Temp field relative toggle
            )
        else:
            # Point robot in direction of right joystick
            self.drive_control.drive_point_joy(
                vx, vy, primary_rx, primary_ry
            )

        if self.primary.getSquareButton():
            self.swerve_drive.reset_gyro()

        """""""""""""""
        INTAKE CONTROL
        """""""""""""""

        if self.secondary.getLeftBumper():
            self.intake.set_voltage(6.0)

        if self.secondary.getBButton():
            self.intake.set_arm_voltage(-8.0)

        if self.secondary.getXButton():
            self.intake.set_arm_voltage(8.0)

        if (
            self.secondary.getRightBumper()
            and self.secondary.getLeftBumper()
            and self.secondary.getAButton()
        ):
            self.intake.zero_encoders()

        """""""""""""""
        SHOOTER CONTROL
        """""""""""""""

        if self.secondary.getRightTriggerAxis() >= 0.8:
            self.shooter.set_kicker(8)

        if self.secondary.getAButton():
            self.shooter.set_velocity(self.flywheel_speed) # type: ignore (assume float)

        if self.secondary.getYButton():
            self.shooter.set_velocity(15.0)

        if self.secondary.getStartButton():
            self.shooter.set_voltage(-6)
            self.shooter.set_kicker(-10)

    def simulationPeriodic(self) -> None:
        """
        Method called in robotPeriodic on simulation.
        """
        self.fuel_sim.step_sim()

    @fms_feedback
    def get_battery_voltage(self) -> units.volts:
        """
        Returns current battery voltage.
        """
        return RobotController.getBatteryVoltage()

    def _display_auto_trajectory(self) -> None:
        """
        Display the selected autonomous trajectory on dashboard.
        """
        selected_auto = self._automodes.chooser.getSelected()
        if isinstance(selected_auto, AutoBase) and selected_auto is not self._displayed_auto:
            selected_auto.display_trajectory()
            self._displayed_auto = selected_auto

    @fms_feedback
    def display_auto_state(self) -> str:
        """
        Display the current autonomous routine state on the dashboard.  
        """
        selected_auto = self._automodes.chooser.getSelected()
        if isinstance(selected_auto, AutoBase):
            return selected_auto.current_state
        return "No Auto Selected"


if __name__ == "__main__":
    wpilib.run(MyRobot) # type: ignore
