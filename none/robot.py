import wpilib
import phoenix6

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.motor = phoenix6.hardware.TalonFX(42,phoenix6.CANBus("can_s5"))

    def teleopPeriodic(self):
        self.motor.set_control(phoenix6.controls.CoastOut())