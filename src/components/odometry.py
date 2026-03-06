from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from robotpy_apriltag import AprilTagFieldLayout
from wpilib import DriverStation, Field2d, SmartDashboard

from components.swerve_drive import SwerveDrive
from lemonlib.vision import LemonCamera


class Odometry:
    camera_front_left: LemonCamera
    camera_front_right: LemonCamera
    camera_back_left: LemonCamera
    camera_back_right: LemonCamera

    field_layout: AprilTagFieldLayout
    swerve_drive: SwerveDrive
    estimated_field: Field2d

    def setup(self):
        self._fms = DriverStation.isFMSAttached()
        cameras = (
            self.camera_front_left,
            self.camera_front_right,
            self.camera_back_left,
            self.camera_back_right,
        )
        self._camera_estimator_pairs = tuple(
            (cam, PhotonPoseEstimator(self.field_layout, cam.camera_to_bot))
            for cam in cameras
        )
        if not DriverStation.isFMSAttached():
            SmartDashboard.putData("Estimated Field", self.estimated_field)

    def execute(self):
        add = self.swerve_drive.addVisionPoseEstimate
        for cam, estimator in self._camera_estimator_pairs:
            try:
                results = cam.getAllUnreadResults()
            except Exception:
                continue
            if not results:
                continue
            pose = estimator.estimateCoprocMultiTagPose(results[-1])
            if pose is not None:
                add(pose.estimatedPose.toPose2d(), pose.timestampSeconds)
        if not self._fms:
            self.estimated_field.setRobotPose(self.swerve_drive.cached_pose)
