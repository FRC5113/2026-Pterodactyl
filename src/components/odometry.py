from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from robotpy_apriltag import AprilTagFieldLayout
from wpilib import Field2d
from wpimath.geometry import Pose3d, Rotation3d
from ntcore import NetworkTableInstance

from components.swerve_drive import SwerveDrive
from lemonlib.vision import LemonCamera


class Odometry:
    camera_back_left: LemonCamera
    camera_back_right: LemonCamera
    field_layout: AprilTagFieldLayout
    swerve_drive: SwerveDrive
    estimated_field: Field2d

    def setup(self):
        """Initialize pose estimators for all cameras."""
        self.camera_pose_estimator_back_left = PhotonPoseEstimator(
            self.field_layout,
            self.camera_back_left.camera_to_bot,
        )
        self.camera_pose_estimator_back_right = PhotonPoseEstimator(
            self.field_layout,
            self.camera_back_right.camera_to_bot,
        )

        self.robot_pose_publisher = NetworkTableInstance.getDefault() \
            .getStructTopic("/Odometry/RobotPose3d", Pose3d) \
            .publish()

    def execute(self):
        
        """Process camera results and update pose estimates."""
        self._process_latest_result(
            self.camera_back_left, self.camera_pose_estimator_back_left
        )
        self._process_latest_result(
            self.camera_back_right, self.camera_pose_estimator_back_right
        )

        pose_2d = self.swerve_drive.get_estimated_pose()
        self.estimated_field.setRobotPose(pose_2d)
        
        pose_3d = Pose3d(
            pose_2d.X(),
            pose_2d.Y(),
            0.0,
            Rotation3d(0, 0, pose_2d.rotation().radians())
        )
        self.robot_pose_publisher.set(pose_3d)

    def _process_latest_result(
        self, camera: LemonCamera, estimator: PhotonPoseEstimator
    ):
        """Process vision results from a single camera."""
        results = camera.getAllUnreadResults()
        if not results:
            return

        result = results[-1]

        camEstPose = estimator.estimateCoprocMultiTagPose(result)
        if camEstPose is None:
            return

        self.swerve_drive.addVisionPoseEstimate(
            camEstPose.estimatedPose.toPose2d(), camEstPose.timestampSeconds
        )
