#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import math
from pyfrc.physics.core import PhysicsInterface
from phoenix6.hardware.talon_fx import TalonFX
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor, LinearSystemId

from phoenix6 import unmanaged
from phoenix6.hardware.talon_fx import TalonFX
from pyfrc.physics.core import PhysicsInterface
from wpimath.system.plant import DCMotor, LinearSystemId
from robot import MyRobot


import typing

if typing.TYPE_CHECKING:
    from .robot import MyRobot


class KrakenSimFOC:
    def __init__(self, motor: TalonFX, moi: float, gearing: float):
        self.gearbox = DCMotor.krakenX60FOC(1)
        self.plant = LinearSystemId.DCMotorSystem(self.gearbox, moi, gearing)
        self.gearing = gearing
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(self.plant, self.gearbox)

    def getSetpoint(self) -> float:
        return self.sim_state.motor_voltage

    def update(self, dt: float):
        voltage = self.sim_state.motor_voltage
        self.motor_sim.setInputVoltage(voltage)
        self.motor_sim.update(dt)
        self.sim_state.set_raw_rotor_position(
            self.motor_sim.getAngularPositionRotations() * self.gearing
        )
        self.sim_state.set_rotor_velocity(
            self.motor_sim.getAngularVelocityRPM() / 60 * self.gearing
        )


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """

        self.physics_controller = physics_controller

        print("TODO: modify simulation for my robot")

        self.robot = robot
        self.encoder_sim = robot.cancoder.sim_state
        self.kraken_sim = KrakenSimFOC(robot.steer_motor, 0.01, robot.steer_gear_ratio)

    def update_sim(self, now: float, tm_diff: float) -> None:
        unmanaged.feed_enable(100)
        self.encoder_sim.add_position(
            -self.kraken_sim.motor_sim.getAngularVelocity() / (2 * math.pi) * tm_diff
        )
        self.kraken_sim.update(tm_diff)
