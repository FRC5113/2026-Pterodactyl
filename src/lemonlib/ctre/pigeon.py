"""Module for pigeon."""

import math

from phoenix6.hardware.pigeon2 import Pigeon2


class LemonPigeon:
    "Gyro class that creates an instance of the Pigeon 2.0 Gyro"

    def __init__(self, can_id: int):
        """Execute __init__."""
        self.gyro = Pigeon2(can_id)
        self.gyro.reset()

    def getAngleCCW(self):

        """Execute getAngleCCW."""
        return self.gyro.get_yaw().value

    def getRoll(self):
        """Execute getRoll."""
        return self.gyro.get_roll().value

    def getPitch(self):
        """Execute getPitch."""
        return self.gyro.get_pitch().value

    def getDegreesPerSecCCW(self):
        """Execute getDegreesPerSecCCW."""
        return self.gyro.get_angular_velocity_z_world().value

    def getRadiansPerSecCCW(self):
        """Execute getRadiansPerSecCCW."""
        return math.radians(self.getDegreesPerSecCCW())

    def getRotation2d(self):
        """Execute getRotation2d."""
        return self.gyro.getRotation2d()

    def setAngleAdjustment(self, angle):
        """Execute setAngleAdjustment."""
        self.gyro.set_yaw(angle)
