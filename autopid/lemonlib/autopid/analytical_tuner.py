"""
Generic analytical tuning component using physics-based methods.
Performs static friction measurement, step response tests, and Ziegler-Nichols tuning.
Supports position and velocity control, with optional gravity compensation.
"""

import math
from typing import List, Optional
import wpilib
from wpilib import RobotBase, RuntimeType

from ..smart import SmartNT
from .motor_interface import MotorInterface, ControlMode
from .tuning_data import MechanismTuningResults, MotorGains, ControlType, GravityType


class AnalyticalTuner:
    """
    Component that performs analytical (physics-based) tuning.
    Uses step response and oscillation tests to calculate optimal PID gains.
    Works with any mechanism type (swerve, flywheel, hood, elevator, arm, etc.)
    """

    def setup(self):
        self.nt = SmartNT("Analytical Tuner")

        # Tuning parameters
        self.test_voltage = 2.0  # Volts for step response
        self.oscillation_voltage = 1.5  # Volts for oscillation test

        # State tracking
        self.is_running = False
        self.is_done = False
        self.current_motor: Optional[MotorInterface] = None
        self.results: Optional[MechanismTuningResults] = None

        # Data collection
        self.position_samples: List[float] = []
        self.time_samples: List[float] = []
        self.velocity_samples: List[float] = []
        self.acceleration_samples: List[float] = []

        # Timing
        self.timer = wpilib.Timer()

        # Test state
        self.current_test = (
            "idle"  # idle, gravity, static_friction, step_pos, step_neg, oscillation
        )
        self.processing_positive_step = True

    """
    CONTROL METHODS
    """

    def start(
        self,
        motor: MotorInterface,
        control_type: ControlType,
        gravity_type: GravityType,
        name: str,
    ) -> None:
        """
        Start analytical tuning for a mechanism.

        Args:
            motor: MotorInterface wrapping the motor controller
            control_type: Position or Velocity control
            gravity_type: Type of gravity compensation needed
            name: Human-readable name for the mechanism
        """
        self.current_motor = motor
        self.is_running = True
        self.is_done = False

        motor_id = motor.get_motor_id()
        self.results = MechanismTuningResults(
            mechanism_id=motor_id,
            mechanism_name=name,
            control_type=control_type,
            gravity_type=gravity_type,
        )

        # Start with gravity measurement if needed
        if gravity_type != GravityType.NONE:
            self.current_test = "gravity"
        else:
            self.current_test = "static_friction"

        self.processing_positive_step = True
        self.timer.restart()
        self._reset_data()

        # self.nt.put(f"{self.results.mechanism_name}/Analytical kS", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Analytical kV", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Analytical kA", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Analytical kP", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Analytical kI", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Analytical kD", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Analytical kG", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Static Voltage", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Step Voltage", 0.0)
        # self.nt.put(
        #     f"{self.results.mechanism_name}/Time Constant pos",
        #     0.0,
        # )
        # self.nt.put(f"{self.results.mechanism_name}/Max Velocity pos", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Max Accel pos", 0.0)
        # self.nt.put(
        #     f"{self.results.mechanism_name}/Time Constant neg",
        #     0.0,
        # )
        # self.nt.put(f"{self.results.mechanism_name}/Max Velocity neg", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Max Accel neg", 0.0)
        # self.nt.put("Oscillation Voltage", 0.0)
        # self.nt.put("Static Voltage", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Gravity V", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Static Friction V", 0.0)
        # self.nt.put("Step Voltage", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Oscillation Period", 0.0)
        # self.nt.put(f"{self.results.mechanism_name}/Oscillation Amplitude", 0.0)

        print(f"Analytical tuning started for {name}")

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

    def get_analytical_gains(self) -> Optional[MotorGains]:
        """Get just the analytical gains"""
        if self.results:
            return self.results.analytical_gains
        return None

    """
    HELPER METHODS
    """

    def _reset_data(self):
        """Clear data collection arrays"""
        self.position_samples.clear()
        self.time_samples.clear()
        self.velocity_samples.clear()
        self.acceleration_samples.clear()
        self.timer.restart()

    def _collect_data(self, elapsed: float):
        """Collect position, velocity, and acceleration data"""
        if self.current_motor:
            position = self.current_motor.get_position()
            velocity = self.current_motor.get_velocity()

            self.position_samples.append(position)
            self.velocity_samples.append(velocity)
            self.time_samples.append(elapsed)

            # Calculate acceleration from velocity samples
            if len(self.velocity_samples) >= 2:
                dt = self.time_samples[-1] - self.time_samples[-2]
                if dt > 0:
                    accel = (self.velocity_samples[-1] - self.velocity_samples[-2]) / dt
                    self.acceleration_samples.append(accel)
                else:
                    self.acceleration_samples.append(0.0)
            else:
                self.acceleration_samples.append(0.0)

    """
    TEST METHODS
    """

    def _test_gravity(self, elapsed: float):
        """
        Measure gravity compensation voltage.
        For CONSTANT: voltage to hold position against gravity
        For COSINE: voltage at horizontal position (90 degrees)
        """
        self._collect_data(elapsed)

        if self.results.gravity_type == GravityType.CONSTANT:
            # For elevators, slowly increase voltage until no movement
            voltage = min(elapsed * 0.2, 2.0)  # Ramp up to 2V over 10 seconds
            self.current_motor.set_control(ControlMode.VOLTAGE, voltage)

            # Check if position is stable
            if len(self.position_samples) > 20:
                recent_positions = self.position_samples[-20:]
                position_variance = sum(
                    (p - sum(recent_positions) / len(recent_positions)) ** 2
                    for p in recent_positions
                ) / len(recent_positions)

                if position_variance < 0.0001:  # Very stable
                    self.results.gravity_voltage = voltage
                    self.nt.put(f"{self.results.mechanism_name}/Gravity V", voltage)
                    print(
                        f"{self.results.mechanism_name} - Gravity voltage: {voltage:.3f}V"
                    )
                    self.current_test = "static_friction"
                    self._reset_data()
                    return

        elif self.results.gravity_type == GravityType.COSINE:
            # For arms, measure voltage at horizontal position
            # Assuming mechanism is already at horizontal (user responsibility)
            voltage = min(elapsed * 0.2, 2.0)
            self.current_motor.set_control(ControlMode.VOLTAGE, voltage)

            # Similar stability check
            if len(self.position_samples) > 20:
                recent_positions = self.position_samples[-20:]
                position_variance = sum(
                    (p - sum(recent_positions) / len(recent_positions)) ** 2
                    for p in recent_positions
                ) / len(recent_positions)

                if position_variance < 0.0001:
                    self.results.gravity_voltage = voltage
                    self.nt.put(f"{self.results.mechanism_name}/Gravity V", voltage)
                    print(
                        f"{self.results.mechanism_name} - Gravity voltage (horizontal): {voltage:.3f}V"
                    )
                    self.current_test = "static_friction"
                    self._reset_data()
                    return

        # Timeout or simulation
        if elapsed > 15.0 or (
            RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0
        ):
            if RobotBase.getRuntimeType() == RuntimeType.kSimulation:
                self.results.gravity_voltage = 0.5
            print(
                f"{self.results.mechanism_name} - Gravity voltage (timeout/sim): {self.results.gravity_voltage:.3f}V"
            )
            self.current_test = "static_friction"
            self._reset_data()

    def _test_static_friction(self, elapsed: float):
        """Measure static friction by gradually increasing voltage"""
        self._collect_data(elapsed)

        # Apply gravity compensation if needed
        gravity_comp = 0.0
        if self.results.gravity_type != GravityType.NONE:
            if self.results.gravity_type == GravityType.CONSTANT:
                gravity_comp = self.results.gravity_voltage
            elif self.results.gravity_type == GravityType.COSINE:
                # At horizontal, full gravity compensation
                gravity_comp = self.results.gravity_voltage

        # Gradually increase voltage
        friction_voltage = min(elapsed * 0.5, 3.0)  # Ramp up to 3V over 6 seconds
        total_voltage = gravity_comp + friction_voltage

        self.nt.put("Static Voltage", total_voltage)
        self.current_motor.set_control(ControlMode.VOLTAGE, total_voltage)

        # Check if mechanism started moving
        if len(self.position_samples) > 10:
            recent_movement = abs(
                self.position_samples[-1] - self.position_samples[-10]
            )
            if recent_movement > 0.01:  # Movement detected
                # Found static friction point
                self.results.static_friction_voltage = friction_voltage
                self.nt.put(
                    f"{self.results.mechanism_name}/Static Friction V", friction_voltage
                )
                print(
                    f"{self.results.mechanism_name} - Static friction: {friction_voltage:.3f}V"
                )

                # Move to step response
                self.current_test = "step_pos"
                self.processing_positive_step = True
                self._reset_data()
                return

        # Timeout or simulation mode
        if elapsed > 10.0 or (
            RobotBase.getRuntimeType() == RuntimeType.kSimulation and elapsed > 2.0
        ):
            self.results.static_friction_voltage = friction_voltage
            self.nt.put(
                f"{self.results.mechanism_name}/Static Friction V", friction_voltage
            )
            print(
                f"{self.results.mechanism_name} - Static friction (timeout/sim): {friction_voltage:.3f}V"
            )
            self.current_test = "step_pos"
            self.processing_positive_step = True
            self._reset_data()

    def _test_step_response(self, elapsed: float, positive: bool):
        """Execute step response test"""
        self._collect_data(elapsed)

        # Apply gravity compensation
        gravity_comp = 0.0
        if self.results.gravity_type == GravityType.CONSTANT:
            gravity_comp = self.results.gravity_voltage
        elif self.results.gravity_type == GravityType.COSINE:
            # Approximate - assumes near horizontal
            gravity_comp = self.results.gravity_voltage

        # Apply step input
        step_voltage = self.test_voltage if positive else -self.test_voltage
        total_voltage = gravity_comp + step_voltage

        self.current_motor.set_control(ControlMode.VOLTAGE, total_voltage)
        self.nt.put("Step Voltage", total_voltage)

        # Duration: 3 seconds
        if elapsed > 3.0:
            # Analyze the step response
            self._analyze_step_response(positive)

            # Move to next test
            if positive:
                self.current_test = "step_neg"
                self.processing_positive_step = False
            else:
                # For velocity control, skip oscillation test
                if self.results.control_type == ControlType.VELOCITY:
                    self._calculate_analytical_gains()
                    self.is_done = True
                    self.is_running = False
                    self.current_test = "idle"
                else:
                    self.current_test = "oscillation"
                    self.processing_positive_step = True

            self._reset_data()

    def _analyze_step_response(self, positive: bool):
        """Analyze collected step response data"""
        if len(self.position_samples) < 10 or len(self.velocity_samples) < 10:
            return

        # Calculate max velocity
        max_velocity = max(abs(v) for v in self.velocity_samples)

        # Calculate max acceleration
        max_acceleration = (
            max(abs(a) for a in self.acceleration_samples)
            if self.acceleration_samples
            else 0.0
        )

        # Find time constant (63.2% of final value)
        final_position = self.position_samples[-1]
        initial_position = self.position_samples[0]
        delta = final_position - initial_position

        time_constant = None
        if abs(delta) > 0.001:
            target_63_2 = initial_position + 0.632 * delta

            for i, pos in enumerate(self.position_samples):
                if abs(pos - target_63_2) < abs(delta * 0.05):  # Within 5%
                    time_constant = self.time_samples[i]
                    break

        # Store results
        suffix = "pos" if positive else "neg"
        if positive:
            self.results.time_constant_pos = time_constant
            self.results.max_velocity_pos = max_velocity
            self.results.max_acceleration_pos = max_acceleration
        else:
            self.results.time_constant_neg = time_constant
            self.results.max_velocity_neg = max_velocity
            self.results.max_acceleration_neg = max_acceleration

        self.nt.put(
            f"{self.results.mechanism_name}/Time Constant {suffix}",
            time_constant or 0.0,
        )
        self.nt.put(
            f"{self.results.mechanism_name}/Max Velocity {suffix}", max_velocity
        )
        self.nt.put(
            f"{self.results.mechanism_name}/Max Accel {suffix}", max_acceleration
        )

        print(
            f"{self.results.mechanism_name} - Step response ({suffix}): "
            f"tau={time_constant:.3f}s, max_vel={max_velocity:.3f}, max_accel={max_acceleration:.3f}"
            if time_constant
            else f"max_vel={max_velocity:.3f}, max_accel={max_acceleration:.3f}"
        )

    def _test_oscillation(self, elapsed: float):
        """Test for oscillations using relay feedback (position control only)"""
        self._collect_data(elapsed)

        # Simple relay: switch voltage based on error from setpoint
        current_position = self.position_samples[-1] if self.position_samples else 0.0
        target_position = 0.0
        error = target_position - current_position

        # Apply gravity compensation
        gravity_comp = 0.0
        if self.results.gravity_type == GravityType.CONSTANT:
            gravity_comp = self.results.gravity_voltage
        elif self.results.gravity_type == GravityType.COSINE:
            gravity_comp = self.results.gravity_voltage

        # Relay with hysteresis
        if error > 0.05:
            voltage = gravity_comp + self.oscillation_voltage
        elif error < -0.05:
            voltage = gravity_comp - self.oscillation_voltage
        else:
            voltage = gravity_comp

        self.current_motor.set_control(ControlMode.VOLTAGE, voltage)
        self.nt.put("Oscillation Voltage", voltage)

        # Duration: 3 seconds
        if elapsed > 3.0:
            self._analyze_oscillations()
            self._calculate_analytical_gains()
            self.is_done = True
            self.is_running = False
            self.current_test = "idle"

    def _analyze_oscillations(self):
        """Analyze oscillation data to find period and amplitude"""
        if len(self.position_samples) < 20:
            return

        # Find peaks in position data
        peaks = []
        for i in range(1, len(self.position_samples) - 1):
            if (
                self.position_samples[i] > self.position_samples[i - 1]
                and self.position_samples[i] > self.position_samples[i + 1]
            ):
                peaks.append(i)

        if len(peaks) >= 2:
            # Calculate average period
            periods = []
            for i in range(1, len(peaks)):
                period = self.time_samples[peaks[i]] - self.time_samples[peaks[i - 1]]
                periods.append(period)

            avg_period = sum(periods) / len(periods)

            # Calculate amplitude
            amplitudes = [self.position_samples[p] for p in peaks]
            avg_amplitude = sum(amplitudes) / len(amplitudes)

            self.results.oscillation_period = avg_period
            self.results.oscillation_amplitude = avg_amplitude

            self.nt.put(f"{self.results.mechanism_name}/Oscillation Period", avg_period)
            self.nt.put(
                f"{self.results.mechanism_name}/Oscillation Amplitude", avg_amplitude
            )

            print(
                f"{self.results.mechanism_name} - Oscillations: "
                f"period={avg_period:.3f}s, amplitude={avg_amplitude:.3f}"
            )

    def _calculate_analytical_gains(self):
        """Calculate PID gains using system identification"""
        gains = self.results.analytical_gains

        # Estimate kS from static friction
        gains.kS = self.results.static_friction_voltage

        # Estimate kV from max velocity
        max_vel = max(self.results.max_velocity_pos, self.results.max_velocity_neg)
        gains.kV = self.test_voltage / max_vel if max_vel > 0.1 else 0.12

        # Estimate kA from max acceleration
        max_accel = max(
            self.results.max_acceleration_pos, self.results.max_acceleration_neg
        )
        gains.kA = self.test_voltage / max_accel if max_accel > 0.1 else 0.01

        # Set kG from gravity measurement
        gains.kG = self.results.gravity_voltage

        # Calculate feedback gains based on control type
        if self.results.control_type == ControlType.POSITION:
            # Use oscillation data if available
            if self.results.oscillation_period and self.results.oscillation_amplitude:
                amplitude = self.results.oscillation_amplitude
                if amplitude > 0.001:
                    Ku = 4.0 * self.oscillation_voltage / (math.pi * amplitude)
                    Tu = self.results.oscillation_period

                    # Modified Ziegler-Nichols for PD control
                    gains.kP = 0.45 * Ku
                    gains.kI = 0.0
                    gains.kD = 0.1 * Ku * Tu
                else:
                    gains.kP = 10.0
                    gains.kI = 0.0
                    gains.kD = 0.1
            else:
                # Fallback to step response
                tau = self.results.time_constant_pos or 0.1
                if tau > 0:
                    gains.kP = 1.2 / tau
                    gains.kD = gains.kP * tau / 1.5
                else:
                    gains.kP = 10.0
                    gains.kD = 0.1
                gains.kI = 0.0

        else:  # VELOCITY control
            # For velocity control, use simpler tuning
            tau = self.results.time_constant_pos or 0.1
            if tau > 0:
                gains.kP = 1.0 / tau
                gains.kI = gains.kP / (10.0 * tau)  # Small integral for velocity
                gains.kD = 0.0
            else:
                gains.kP = 5.0
                gains.kI = 0.5
                gains.kD = 0.0

        # Output to NetworkTables
        self.nt.put(f"{self.results.mechanism_name}/Analytical kS", gains.kS)
        self.nt.put(f"{self.results.mechanism_name}/Analytical kV", gains.kV)
        self.nt.put(f"{self.results.mechanism_name}/Analytical kA", gains.kA)
        self.nt.put(f"{self.results.mechanism_name}/Analytical kP", gains.kP)
        self.nt.put(f"{self.results.mechanism_name}/Analytical kI", gains.kI)
        self.nt.put(f"{self.results.mechanism_name}/Analytical kD", gains.kD)
        self.nt.put(f"{self.results.mechanism_name}/Analytical kG", gains.kG)

        print(f"{self.results.mechanism_name} - Analytical gains: {gains}")

    """
    EXECUTE METHOD
    """

    def execute(self):
        """Called every robot loop - runs the current test"""
        if not self.is_running or self.current_motor is None:
            return

        elapsed = self.timer.get()

        # Run the current test
        if self.current_test == "gravity":
            self._test_gravity(elapsed)
        elif self.current_test == "static_friction":
            self._test_static_friction(elapsed)
        elif self.current_test == "step_pos":
            self._test_step_response(elapsed, positive=True)
        elif self.current_test == "step_neg":
            self._test_step_response(elapsed, positive=False)
        elif self.current_test == "oscillation":
            self._test_oscillation(elapsed)
