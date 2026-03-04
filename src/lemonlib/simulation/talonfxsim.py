"""Module for talonfxsim."""

from phoenix6.hardware.talon_fx import TalonFX
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor, LinearSystemId


class FalconSim:
    """FalconSim class."""
    def __init__(self, motor: TalonFX, moi: float, gearing: float):
        """Execute __init__."""
        self.gearbox = DCMotor.falcon500(1)
        self.plant = LinearSystemId.DCMotorSystem(self.gearbox, moi, gearing)
        self.gearing = gearing
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(self.plant, self.gearbox)

    def getSetpoint(self) -> float:
        """Execute getSetpoint."""
        return self.sim_state.motor_voltage

    def update(self, dt: float):
        """Execute update."""
        voltage = self.sim_state.motor_voltage
        self.motor_sim.setInputVoltage(voltage)
        self.motor_sim.update(dt)
        self.sim_state.set_raw_rotor_position(
            self.motor_sim.getAngularPositionRotations() * self.gearing
        )
        self.sim_state.set_rotor_velocity(
            self.motor_sim.getAngularVelocityRPM() / 60 * self.gearing
        )


class FalconSimFOC:
    """FalconSimFOC class."""
    def __init__(self, motor: TalonFX, moi: float, gearing: float):
        """Execute __init__."""
        self.gearbox = DCMotor.falcon500FOC(1)
        self.plant = LinearSystemId.DCMotorSystem(self.gearbox, moi, gearing)
        self.gearing = gearing
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(self.plant, self.gearbox)

    def getSetpoint(self) -> float:
        """Execute getSetpoint."""
        return self.sim_state.motor_voltage

    def update(self, dt: float):
        """Execute update."""
        voltage = self.sim_state.motor_voltage
        self.motor_sim.setInputVoltage(voltage)
        self.motor_sim.update(dt)
        self.sim_state.set_raw_rotor_position(
            self.motor_sim.getAngularPositionRotations() * self.gearing
        )
        self.sim_state.set_rotor_velocity(
            self.motor_sim.getAngularVelocityRPM() / 60 * self.gearing
        )


class KrakenSim:
    """KrakenSim class."""
    def __init__(self, motor: TalonFX, moi: float, gearing: float):
        """Execute __init__."""
        self.gearbox = DCMotor.krakenX60(1)
        self.plant = LinearSystemId.DCMotorSystem(self.gearbox, moi, gearing)
        self.gearing = gearing
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(self.plant, self.gearbox)

    def getSetpoint(self) -> float:
        """Execute getSetpoint."""
        return self.sim_state.motor_voltage

    def update(self, dt: float):
        """Execute update."""
        voltage = self.sim_state.motor_voltage
        self.motor_sim.setInputVoltage(voltage)
        self.motor_sim.update(dt)
        self.sim_state.set_raw_rotor_position(
            self.motor_sim.getAngularPositionRotations() * self.gearing
        )
        self.sim_state.set_rotor_velocity(
            self.motor_sim.getAngularVelocityRPM() / 60 * self.gearing
        )


class KrakenSimFOC:
    """KrakenSimFOC class."""
    def __init__(self, motor: TalonFX, moi: float, gearing: float):
        """Execute __init__."""
        self.gearbox = DCMotor.krakenX60FOC(1)
        self.plant = LinearSystemId.DCMotorSystem(self.gearbox, moi, gearing)
        self.gearing = gearing
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(self.plant, self.gearbox)

    def getSetpoint(self) -> float:
        """Execute getSetpoint."""
        return self.sim_state.motor_voltage

    def update(self, dt: float):
        """Execute update."""
        voltage = self.sim_state.motor_voltage
        self.motor_sim.setInputVoltage(voltage)
        self.motor_sim.update(dt)
        self.sim_state.set_raw_rotor_position(
            self.motor_sim.getAngularPositionRotations() * self.gearing
        )
        self.sim_state.set_rotor_velocity(
            self.motor_sim.getAngularVelocityRPM() / 60 * self.gearing
        )
