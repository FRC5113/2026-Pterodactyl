"""
Generic trial-and-error tuning component using empirical methods.
Incrementally tunes kS, kV, kA, kP, kI, kD, and kG gains through observation.
"""

import math
from typing import List, Optional
import wpilib
from wpilib import RobotBase, RuntimeType

from ..smart import SmartNT
from .motor_interface import MotorInterface, ControlMode
from .tuning_data import MechanismTuningResults, MotorGains, ControlType, GravityType


class TrialErrorTuner:
    """
    Component that performs trial-and-error (empirical) tuning.
    Incrementally adjusts gains based on system response.
    Works with any mechanism type.
    """

    def setup(self):
        self.nt = SmartNT("Trial Error Tuner")

        # Tuning increments (will be adjusted based on mechanism)
        self.kS_increment = 0.05  # Volts
        self.kV_increment = 0.01  # V/(unit/s)
        self.kA_increment = 0.005  # V/(unit/s²)
        self.kP_increment = 0.5
        self.kI_increment = 0.01
        self.kD_increment = 0.01
        self.kG_increment = 0.05  # Volts

        # State tracking
        self.is_running = False
        self.is_done = False
        self.current_motor: Optional[MotorInterface] = None
        self.results: Optional[MechanismTuningResults] = None

        # Current gain values being tested
        self.kS_trial = 0.0
        self.kV_trial = 0.0
        self.kA_trial = 0.0
        self.kP_trial = 0.0
        self.kI_trial = 0.0
        self.kD_trial = 0.0
        self.kG_trial = 0.0

        # Data collection
        self.position_samples: List[float] = []
        self.velocity_samples: List[float] = []
        self.time_samples: List[float] = []

        # Timing
        self.timer = wpilib.Timer()

        # Test state
        self.current_test = "idle"  # idle, kG, kS, kV, kA, kP, kI, kD

        # Tuning tracking
        self.velocity_setpoint = 2.0  # units/s for velocity tuning
        self.position_setpoint = 0.0  # For position tuning
        self.best_kV = 0.0
        self.best_kV_error = float("inf")
        self.best_kA = 0.0
        self.best_kA_error = float("inf")

    """
    CONTROL METHODS
    """

    def start(
        self,
        motor: MotorInterface,
        control_type: ControlType,
        gravity_type: GravityType,
        name: str,
        initial_gains: Optional[MotorGains] = None,
    ) -> None:
        """
        Start trial-and-error tuning for a mechanism.

        Args:
            motor: MotorInterface wrapping the motor controller
            control_type: Position or Velocity control
            gravity_type: Type of gravity compensation needed
            name: Human-readable name for the mechanism
            initial_gains: Optional analytical gains to use as starting point
        """
        self.current_motor = motor
        self.is_running = True
        self.is_done = False

        motor_id = motor.get_motor_id()

        # Get or create results
        if self.results and self.results.mechanism_id == motor_id:
            # Continue with existing results
            pass
        else:
            self.results = MechanismTuningResults(
                mechanism_id=motor_id,
                mechanism_name=name,
                control_type=control_type,
                gravity_type=gravity_type,
            )

        # Initialize trial gains from analytical if provided
        if initial_gains:
            self.kS_trial = initial_gains.kS * 0.5  # Start conservative
            self.kV_trial = initial_gains.kV * 0.5
            self.kA_trial = initial_gains.kA * 0.3
            self.kP_trial = initial_gains.kP * 0.3
            self.kI_trial = 0.0  # Always start kI from zero
            self.kD_trial = 0.0  # Always start kD from zero
            self.kG_trial = (
                initial_gains.kG * 0.8 if gravity_type != GravityType.NONE else 0.0
            )
        else:
            self.kS_trial = 0.0
            self.kV_trial = 0.0
            self.kA_trial = 0.0
            self.kP_trial = 0.0
            self.kI_trial = 0.0
            self.kD_trial = 0.0
            self.kG_trial = 0.0

        # Start with gravity if needed, otherwise kS
        if gravity_type != GravityType.NONE:
            self.current_test = "kG"
        else:
            self.current_test = "kS"

        self.best_kV = 0.0
        self.best_kV_error = float("inf")
        self.best_kA = 0.0
        self.best_kA_error = float("inf")
        self.timer.restart()
        self._reset_data()

        # Apply initial zero gains
        self.current_motor.apply_gains(
            kS=0.0, kV=0.0, kA=0.0, kP=0.0, kI=0.0, kD=0.0, kG=0.0
        )

        self.nt.put(f"{self.results.mechanism_name}/Trial kS", 0.0)
        self.nt.put(f"{self.results.mechanism_name}/Trial kV", 0.0)
        self.nt.put(f"{self.results.mechanism_name}/Trial kA", 0.0)
        self.nt.put(f"{self.results.mechanism_name}/Trial kP", 0.0)
        self.nt.put(f"{self.results.mechanism_name}/Trial kI", 0.0)
        self.nt.put(f"{self.results.mechanism_name}/Trial kD", 0.0)
        self.nt.put(f"{self.results.mechanism_name}/Trial kG", 0.0)

        print(f"Trial-and-error tuning started for {name}")

    def stop(self) -> None:
        """Stop the current tuning process"""
        self.is_running = False
        if self.current_motor:
            self.current_motor.set_control(ControlMode.BRAKE)

    def is_complete(self) -> bool:
        """Check if tuning is complete"""
        return self.is_done

    def get_results(self) -> Optional[MechanismTuningResults]:
        """Get the tuning results"""
        return self.results

    def get_trial_gains(self) -> Optional[MotorGains]:
        """Get just the trial gains"""
        if self.results:
            return self.results.trial_gains
        return None

    """
    HELPER METHODS
    """

    def _reset_data(self):
        """Clear data collection arrays"""
        self.position_samples.clear()
        self.velocity_samples.clear()
        self.time_samples.clear()
        self.timer.restart()

    def _collect_data(self, elapsed: float):
        """Collect position and velocity data"""
        if self.current_motor:
            position = self.current_motor.get_position()
            velocity = self.current_motor.get_velocity()

            self.position_samples.append(position)
            self.velocity_samples.append(velocity)
            self.time_samples.append(elapsed)

    def _calculate_velocity_error(self, target_velocity: float) -> float:
        """Calculate velocity tracking error"""
        if len(self.velocity_samples) > 10:
            avg_velocity = sum(abs(v) for v in self.velocity_samples[-10:]) / 10.0
            return abs(target_velocity - avg_velocity)
        return float("inf")

    def _detect_oscillation(self) -> bool:
        """Detect position oscillation"""
        if len(self.position_samples) < 30:
            return False

        recent_samples = self.position_samples[-30:]
        target = sum(recent_samples) / len(recent_samples)
        crossings = 0

        for i in range(1, len(recent_samples)):
            if (recent_samples[i - 1] < target and recent_samples[i] > target) or (
                recent_samples[i - 1] > target and recent_samples[i] < target
            ):
                crossings += 1

        return crossings >= 4

    def _detect_jitter(self) -> bool:
        """Detect velocity jitter"""
        if len(self.velocity_samples) < 30:
            return False

        recent_velocities = self.velocity_samples[-30:]
        mean_vel = sum(recent_velocities) / len(recent_velocities)
        variance = sum((v - mean_vel) ** 2 for v in recent_velocities) / len(
            recent_velocities
        )

        return variance > 0.5

    """
    TEST METHODS
    """

    def _tune_kG(self, elapsed: float):
        """Tune kG (gravity compensation)"""
        self._collect_data(elapsed)

        # Apply kG voltage (with sign depending on gravity type)
        self.current_motor.set_control(ControlMode.VOLTAGE, self.kG_trial)
        self.nt.put("Trial kG", self.kG_trial)

        # Check if position is stable (mechanism not falling/rising)
        if len(self.position_samples) > 20:
            recent_positions = self.position_samples[-20:]
            position_variance = sum(
                (p - sum(recent_positions) / len(recent_positions)) ** 2
                for p in recent_positions
            ) / len(recent_positions)

            if position_variance < 0.0001:  # Very stable
                self.results.trial_gains.kG = self.kG_trial
                self.nt.put(f"{self.results.mechanism_name}/Trial kG", self.kG_trial)
                print(f"{self.results.mechanism_name} - Trial kG: {self.kG_trial:.4f}")

                self.current_test = "kS"
                self._reset_data()
                return

        # Increment every 0.5 seconds
        if elapsed > 0.5 and int(elapsed / 0.5) != int((elapsed - 0.02) / 0.5):
            self.kG_trial += self.kG_increment

        # Timeout or simulation
        if elapsed > 20.0 or (
            RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0
        ):
            if RobotBase.getRuntimeType() == RuntimeType.kSimulation:
                self.kG_trial = 0.5
            self.results.trial_gains.kG = self.kG_trial
            print(
                f"{self.results.mechanism_name} - Trial kG (timeout/sim): {self.kG_trial:.4f}"
            )
            self.current_test = "kS"
            self._reset_data()

    def _tune_kS(self, elapsed: float):
        """Tune kS by increasing until movement detected"""
        self._collect_data(elapsed)

        # Apply kS voltage plus gravity compensation
        voltage = self.kG_trial + self.kS_trial
        self.current_motor.set_control(ControlMode.VOLTAGE, voltage)
        self.nt.put("Trial kS", self.kS_trial)

        # Check for movement
        if len(self.position_samples) > 10:
            recent_movement = abs(
                self.position_samples[-1] - self.position_samples[-10]
            )

            if recent_movement > 0.01:  # Movement detected
                self.kS_trial = max(0.0, self.kS_trial - self.kS_increment)
                self.results.trial_gains.kS = self.kS_trial
                self.nt.put(f"{self.results.mechanism_name}/Trial kS", self.kS_trial)
                print(f"{self.results.mechanism_name} - Trial kS: {self.kS_trial:.4f}")

                self.current_test = "kV"
                self._reset_data()
                return

        # Increment every 0.5 seconds
        if elapsed > 0.5 and int(elapsed / 0.5) != int((elapsed - 0.02) / 0.5):
            self.kS_trial += self.kS_increment

        # Timeout or simulation
        if elapsed > 15.0 or (
            RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0
        ):
            if RobotBase.getRuntimeType() == RuntimeType.kSimulation:
                self.kS_trial = 0.5
            self.results.trial_gains.kS = self.kS_trial
            print(
                f"{self.results.mechanism_name} - Trial kS (timeout/sim): {self.kS_trial:.4f}"
            )
            self.current_test = "kV"
            self._reset_data()

    def _tune_kV(self, elapsed: float):
        """Tune kV by minimizing velocity tracking error"""
        self._collect_data(elapsed)

        # Apply current gains and use velocity control
        self.current_motor.apply_gains(
            kS=self.kS_trial,
            kV=self.kV_trial,
            kA=0.0,
            kP=0.0,
            kI=0.0,
            kD=0.0,
            kG=self.kG_trial,
        )
        self.current_motor.set_control(ControlMode.VELOCITY, self.velocity_setpoint)
        self.nt.put("Trial kV", self.kV_trial)

        # Check error after settling
        if elapsed > 1.0 and len(self.velocity_samples) > 20:
            error = self._calculate_velocity_error(self.velocity_setpoint)
            self.nt.put("kV Error", error)

            if error < self.best_kV_error:
                self.best_kV_error = error
                self.best_kV = self.kV_trial

            # If error increasing, we passed optimum
            if (
                error > self.best_kV_error * 1.5
                and self.kV_trial > self.best_kV + self.kV_increment
            ):
                self.kV_trial = self.best_kV
                self.results.trial_gains.kV = self.kV_trial
                self.nt.put(f"{self.results.mechanism_name}/Trial kV", self.kV_trial)
                print(
                    f"{self.results.mechanism_name} - Trial kV: {self.kV_trial:.4f} (error: {self.best_kV_error:.4f})"
                )

                # For velocity control, skip to kP
                if self.results.control_type == ControlType.VELOCITY:
                    self.current_test = "kP"
                else:
                    self.current_test = (
                        "kP"  # Skip kA for now (complex to tune empirically)
                    )
                self._reset_data()
                return

        # Increment every 1.0 seconds
        if elapsed > 1.0 and int(elapsed) != int(elapsed - 0.02):
            self.kV_trial += self.kV_increment

        # Timeout or simulation
        if elapsed > 20.0 or (
            RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0
        ):
            if RobotBase.getRuntimeType() == RuntimeType.kSimulation:
                self.kV_trial = 0.12
            else:
                self.kV_trial = self.best_kV if self.best_kV > 0 else self.kV_trial
            self.results.trial_gains.kV = self.kV_trial
            print(
                f"{self.results.mechanism_name} - Trial kV (timeout/sim): {self.kV_trial:.4f}"
            )
            self.current_test = "kP"
            self._reset_data()

    def _tune_kP(self, elapsed: float):
        """Tune kP by increasing until oscillation"""
        self._collect_data(elapsed)

        # Apply current gains
        self.current_motor.apply_gains(
            kS=self.kS_trial,
            kV=self.kV_trial,
            kA=self.kA_trial,
            kP=self.kP_trial,
            kI=0.0,
            kD=0.0,
            kG=self.kG_trial,
        )

        # Use appropriate control mode
        if self.results.control_type == ControlType.POSITION:
            self.current_motor.set_control(ControlMode.POSITION, self.position_setpoint)
        else:
            self.current_motor.set_control(ControlMode.VELOCITY, self.velocity_setpoint)

        self.nt.put("Trial kP", self.kP_trial)

        # Check for oscillation after settling
        if elapsed > 1.0 and len(self.position_samples) > 30:
            if self._detect_oscillation():
                self.kP_trial = max(0.0, self.kP_trial - self.kP_increment)
                self.results.trial_gains.kP = self.kP_trial
                self.nt.put(f"{self.results.mechanism_name}/Trial kP", self.kP_trial)
                print(f"{self.results.mechanism_name} - Trial kP: {self.kP_trial:.4f}")

                # For velocity control, may want kI
                if self.results.control_type == ControlType.VELOCITY:
                    self.current_test = "kI"
                else:
                    self.current_test = "kD"
                self._reset_data()
                return

        # Increment every 1.0 seconds
        if elapsed > 1.0 and int(elapsed) != int(elapsed - 0.02):
            self.kP_trial += self.kP_increment

        # Timeout or simulation
        if elapsed > 20.0 or (
            RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0
        ):
            if RobotBase.getRuntimeType() == RuntimeType.kSimulation:
                self.kP_trial = 10.0
            self.results.trial_gains.kP = self.kP_trial
            print(
                f"{self.results.mechanism_name} - Trial kP (timeout/sim): {self.kP_trial:.4f}"
            )
            if self.results.control_type == ControlType.VELOCITY:
                self.current_test = "kI"
            else:
                self.current_test = "kD"
            self._reset_data()

    def _tune_kI(self, elapsed: float):
        """Tune kI for velocity control (helps with steady-state error)"""
        self._collect_data(elapsed)

        # Apply current gains
        self.current_motor.apply_gains(
            kS=self.kS_trial,
            kV=self.kV_trial,
            kA=self.kA_trial,
            kP=self.kP_trial,
            kI=self.kI_trial,
            kD=0.0,
            kG=self.kG_trial,
        )
        self.current_motor.set_control(ControlMode.VELOCITY, self.velocity_setpoint)
        self.nt.put("Trial kI", self.kI_trial)

        # Check for instability or oscillation
        if elapsed > 2.0 and len(self.velocity_samples) > 30:
            if self._detect_jitter():
                self.kI_trial = max(0.0, self.kI_trial - self.kI_increment)
                self.results.trial_gains.kI = self.kI_trial
                self.nt.put(f"{self.results.mechanism_name}/Trial kI", self.kI_trial)
                print(f"{self.results.mechanism_name} - Trial kI: {self.kI_trial:.4f}")

                # Done!
                self.is_done = True
                self.is_running = False
                self.current_test = "idle"
                return

        # Increment every 1.0 seconds
        if elapsed > 1.0 and int(elapsed) != int(elapsed - 0.02):
            self.kI_trial += self.kI_increment

        # Timeout or simulation
        if elapsed > 20.0 or (
            RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0
        ):
            if RobotBase.getRuntimeType() == RuntimeType.kSimulation:
                self.kI_trial = 0.5
            self.results.trial_gains.kI = self.kI_trial
            print(
                f"{self.results.mechanism_name} - Trial kI (timeout/sim): {self.kI_trial:.4f}"
            )
            self.is_done = True
            self.is_running = False
            self.current_test = "idle"

    def _tune_kD(self, elapsed: float):
        """Tune kD by increasing until jitter (position control)"""
        self._collect_data(elapsed)

        # Apply current gains with full PD
        self.current_motor.apply_gains(
            kS=self.kS_trial,
            kV=self.kV_trial,
            kA=self.kA_trial,
            kP=self.kP_trial,
            kI=0.0,
            kD=self.kD_trial,
            kG=self.kG_trial,
        )
        self.current_motor.set_control(ControlMode.POSITION, self.position_setpoint)
        self.nt.put("Trial kD", self.kD_trial)

        # Check for jitter after settling
        if elapsed > 1.0 and len(self.velocity_samples) > 30:
            if self._detect_jitter():
                self.kD_trial = max(0.0, self.kD_trial - self.kD_increment)
                self.results.trial_gains.kD = self.kD_trial
                self.nt.put(f"{self.results.mechanism_name}/Trial kD", self.kD_trial)
                print(f"{self.results.mechanism_name} - Trial kD: {self.kD_trial:.4f}")

                # Done!
                self.is_done = True
                self.is_running = False
                self.current_test = "idle"
                return

        # Increment every 1.0 seconds
        if elapsed > 1.0 and int(elapsed) != int(elapsed - 0.02):
            self.kD_trial += self.kD_increment

        # Timeout or simulation
        if elapsed > 20.0 or (
            RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0
        ):
            if RobotBase.getRuntimeType() == RuntimeType.kSimulation:
                self.kD_trial = 0.1
            self.results.trial_gains.kD = self.kD_trial
            print(
                f"{self.results.mechanism_name} - Trial kD (timeout/sim): {self.kD_trial:.4f}"
            )
            self.is_done = True
            self.is_running = False
            self.current_test = "idle"

    """
    EXECUTE METHOD
    """

    def execute(self):
        """Called every robot loop - runs the current test"""
        if not self.is_running or self.current_motor is None:
            return

        elapsed = self.timer.get()

        # Run the current test
        if self.current_test == "kG":
            self._tune_kG(elapsed)
        elif self.current_test == "kS":
            self._tune_kS(elapsed)
        elif self.current_test == "kV":
            self._tune_kV(elapsed)
        elif self.current_test == "kP":
            self._tune_kP(elapsed)
        elif self.current_test == "kI":
            self._tune_kI(elapsed)
        elif self.current_test == "kD":
            self._tune_kD(elapsed)
