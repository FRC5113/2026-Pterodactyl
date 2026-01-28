import math
import wpilib
from wpilib import TimedRobot, Timer, Notifier, SmartDashboard
from magicbot import MagicRobot

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import (
    TalonFXConfiguration,
    FeedbackConfigs,
    ClosedLoopGeneralConfigs,
)
from phoenix6.signals import (
    FeedbackSensorSourceValue,
    NeutralModeValue,
    StaticFeedforwardSignValue,
)
from phoenix6 import controls, CANBus


class MyRobot(MagicRobot):
    """
    Test script to try and automate tuning of swerve steering PID.
    The robot will attempt to tune kP first by increasing it until oscillations
    are detected. Then, it will increase kD until overshoot
    """

    def createObjects(self):
        self.steer_gear_ratio = (
            150.0 / 7.0
        )  # Standard swerve module steering gear ratio
        self.target_angle = math.pi / 2  # 90 deg

        self.canbus = CANBus("can0")
        self.steer_motor = TalonFX(42, self.canbus)
        self.cancoder = CANcoder(43, self.canbus)

        motor_cfg = TalonFXConfiguration()

        motor_cfg.motor_output.neutral_mode = NeutralModeValue.BRAKE

        # Configure motor to use CANcoder as feedback sensor for closed-loop control
        motor_cfg.feedback = (
            FeedbackConfigs()
            .with_feedback_remote_sensor_id(self.cancoder.device_id)
            .with_feedback_sensor_source(
                FeedbackSensorSourceValue.FUSED_CANCODER
            )  # Fuses CANcoder with internal encoder
            .with_rotor_to_sensor_ratio(
                self.steer_gear_ratio
            )  # Accounts for gearing between motor and sensor
        )

        # Enable continuous wrap so position loops around at 0/360 degrees
        motor_cfg.closed_loop_general = ClosedLoopGeneralConfigs().with_continuous_wrap(
            True
        )

        motor_cfg.slot0.k_p = 0
        motor_cfg.slot0.k_i = 0
        motor_cfg.slot0.k_d = 0
        motor_cfg.slot0.k_s = 0
        motor_cfg.slot0.static_feedforward_sign = (
            StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
        )

        self.steer_motor.configurator.apply(motor_cfg)

        # Position control using voltage output, slot 0 gains, with Field Oriented Control enabled
        self.control = controls.PositionVoltage(0).with_slot(0).with_enable_foc(True)

        self.stage = (
            "s_sweep"  # State machine: "s_sweep" -> "p_sweep" -> "d_sweep" -> "done"
        )
        self.timer = Timer()
        self.timer.start()

        self.s_test = 0.0  # Starting kS value
        self.s_step = 0.001  # Amount to increment kS each iteration
        self.s_osc = None

        self.p_test = 1  # Starting kP value
        self.p_step = 1.0  # Amount to increment kP each iteration
        self.p_step_small = 0.1
        self.p_osc = None  # Stores kP value where oscillations were detected
        self.p_small = False

        self.d_test = 0.0  # Starting kD value
        self.d_step = 0.01  # Amount to increment kD each iteration
        self.d_step_small = 0.001
        self.d_small = False

        self.crossings = 0  # Counts how many times error crosses zero (sign changes)
        self.last_sign = 0  # Tracks previous error sign to detect zero crossings

        self.notifier = Notifier(self.pidPeriodic)
        self.notifier.setName("PID Notifier")

        self.loop_time = 0.02  # Initial loop time

    def teleopInit(self):
        self.starting_angle = self.cancoder.get_absolute_position().value * math.tau
        self.notifier.startPeriodic(1.2)  # Run pidPeriodic every 1.2 seconds

    def disabledInit(self):
        self.notifier.stop()

    def pidPeriodic(self):
        # Get current angle in radians (CANcoder returns rotations, multiply by tau)
        current_angle = self.cancoder.get_absolute_position().value * math.tau
        # Normalize error to [-pi, pi] range for proper wraparound handling
        error = (self.target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

        if self.stage == "s_sweep":
            if (current_angle - self.starting_angle) > 0.001:
                self.s_osc = self.s_test
                self.s_test *= 0.9  # Back off to 90% of move point
                self._apply_gains(self.p_test, 0, self.s_test)
                self.crossings = 0
                print(f"S Movement at {self.s_osc:.3f}, using {self.s_test:.3f}")
            else:
                self.s_test += self.s_step
                self._apply_gains(0, 0, self.s_test)
                print(f"Testing S = {self.s_test:.3f}")

        # p_sweep: Increase P until oscillations detected (3+ zero crossings = oscillating)
        if self.stage == "p_sweep":
            print(f"Crossings: {self.crossings}")
            if self.crossings >= 3:
                self.p_osc = self.p_test
                self.p_test *= 0.7  # Back off to 70% of oscillation point
                self._apply_gains(self.p_test, 0, self.s_test)
                self.crossings = 0
                print(f"P oscillation at {self.p_osc:.3f}, using {self.p_test:.3f}")
                if not self.p_small:
                    self.p_small = True
                    self.p_step = self.p_step_small
                else:
                    self.stage = "d_sweep"
            else:
                self.p_test += self.p_step
                self._apply_gains(self.p_test, 0, self.s_test)
                print(f"Testing P = {self.p_test:.2f}")

        # d_sweep: Increase D until overshoot < 2 degrees (D dampens oscillations)
        elif self.stage == "d_sweep":
            overshoot_deg = abs(error) * 180 / math.pi  # Convert error to degrees

            if overshoot_deg < 2.0:
                if not self.d_small:
                    self.d_small = True
                    self.d_test *= 0.9
                    self.d_step = self.d_step_small
                else:
                    self.stage = "done"
                    print("\nAUTO TUNE COMPLETE")
                    print(f"FINAL kP = {self.p_test:.2f}")
                    print(f"FINAL kD = {self.d_test:.2f}")
            else:
                self.d_test += self.d_step
                self._apply_gains(self.p_test, self.d_test, self.s_test)
                print(f"Testing D = {self.d_test:.2f}")

        elif self.stage == "done":
            self.steer_motor.set_control(
                controls.static_brake.StaticBrake()
            )  # Hold position when tuning complete

    def teleopPeriodic(self):
        # Get current angle in radians (CANcoder returns rotations, multiply by tau)
        current_angle = (
            (self.cancoder.get_absolute_position().value * math.tau) * 180 / math.pi
        )
        # Normalize error to [-pi, pi] range for proper wraparound handling
        error = (self.target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

        # Detect zero crossings - when error changes sign, motor has passed target
        sign = 1 if error > 0 else -1
        if sign != self.last_sign:
            self.crossings += 1
        self.last_sign = sign

        # Command motor to target position (divide by tau to convert radians to rotations)
        self.steer_motor.set_control(
            self.control.with_position(self.target_angle / math.tau)
        )

        SmartDashboard.putString("Tuning Stage", self.stage)
        SmartDashboard.putNumber("kP Test", self.p_test)
        SmartDashboard.putNumber("kD Test", self.d_test)
        SmartDashboard.putNumber("kS Test", self.s_test)
        SmartDashboard.putNumber(
            "Current Angle (deg)",
            self.cancoder.get_absolute_position().value * 360,
        )
        SmartDashboard.putNumber(
            "Error (deg)",
            (self.target_angle - self.cancoder.get_absolute_position().value * math.tau)
            * 180
            / math.pi,
        )
        SmartDashboard.putNumber("loop time (s)", self.loop_time)
        SmartDashboard.putNumber(
            "output voltage (V)", self.steer_motor.get_closed_loop_output().value
        )

    def _apply_gains(self, p, d, s):
        """Apply new PID gains to motor controller"""
        motor_cfg = TalonFXConfiguration()
        motor_cfg.slot0.k_p = p
        motor_cfg.slot0.k_i = 0
        motor_cfg.slot0.k_d = d
        motor_cfg.slot0.k_s = s
        self.steer_motor.configurator.apply(motor_cfg)

    def _do_periodics(self):
        super()._do_periodics()

        self.loop_time = max(self.control_loop_wait_time, self.watchdog.getTime())


if __name__ == "__main__":
    wpilib.run(MyRobot)
