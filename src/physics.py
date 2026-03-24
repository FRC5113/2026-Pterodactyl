import typing

from phoenix6 import unmanaged
from pyfrc.physics.core import PhysicsInterface
from wpilib import RobotController

# from lemonlib.simulation import LemonCameraSim, LemonVisionSim

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller
        self.robot = robot

        # # Shared vision simulation for all cameras
        # self.vision_sim = LemonVisionSim(robot.field_layout)

        # # Vision camera simulations
        # self.vision_sim_front_left = LemonCameraSim(
        #     robot.camera_front_left, robot.field_layout, fov=70.0, fps=60.0
        # )
        # self.vision_sim_front_right = LemonCameraSim(
        #     robot.camera_front_right, robot.field_layout, fov=70.0, fps=60.0
        # )
        # self.vision_sim_back_left = LemonCameraSim(
        #     robot.camera_back_left, robot.field_layout, fov=70.0, fps=60.0
        # )
        # self.vision_sim_back_right = LemonCameraSim(
        #     robot.camera_back_right, robot.field_layout, fov=70.0, fps=60.0
        # )

        # # Register all cameras with the shared vision sim
        # self.vision_sim.add_camera(self.vision_sim_front_left)
        # self.vision_sim.add_camera(self.vision_sim_front_right)
        # self.vision_sim.add_camera(self.vision_sim_back_left)
        # self.vision_sim.add_camera(self.vision_sim_back_right)
        # self.robot.sim_intake_left_motor = FalconSim(
        #     self.robot.intake_left_motor, 0.1, 100
        # )
        # self.robot.sim_intake_spin_motor = FalconSim(
        #     self.robot.intake_spin_motor, 0.1, 100
        # )
        # self.robot.sim_intake_right_motor = FalconSim(
        #     self.robot.intake_right_motor, 0.1, 100
        # )

    def update_sim(self, now, tm_diff):
        # Keep Phoenix 6 devices enabled in sim
        unmanaged.feed_enable(100)

        # Handle starting pose resets from SwerveDrive
        if self.robot.swerve_drive.starting_pose is not None:
            self.robot.swerve_drive.set_starting_pose(None)

        # Phoenix 6 SwerveDrivetrain handles ALL motor, encoder, and Pigeon2
        # simulation internally — drive motors, steer motors, CANcoders, and
        # Pigeon2 yaw are all updated in one call.
        self.robot.swerve_drive.drivetrain.update_sim_state(
            tm_diff,
            RobotController.getBatteryVoltage(),
        )

        # Sync pyfrc field pose from the drivetrain's odometry
        pose = self.robot.swerve_drive.get_estimated_pose()
        self.physics_controller.field.setRobotPose(pose)

        # Update all vision cameras in one call
        # self.vision_sim.update(pose)

        # """Intake"""
        # self.robot.sim_intake_left_motor.update(tm_diff)
        # self.robot.sim_intake_right_motor.update(tm_diff)
        # self.robot.sim_intake_spin_motor.update(tm_diff)
