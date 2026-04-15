import math
from collections import deque

import numpy as np
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from robotpy_apriltag import AprilTagFieldLayout
from wpilib import DriverStation, Field2d, SmartDashboard
from wpimath.geometry import Pose2d, Pose3d, Rotation2d

from components.swerve_drive import SwerveDrive
from game import is_disabled, is_sim
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
            # self.camera_back_left,
            # self.camera_back_right,
        )
        self._camera_estimator_pairs = tuple(
            (cam, PhotonPoseEstimator(self.field_layout, cam.camera_to_bot))
            for cam in cameras
        )

        self.linear_baseline_std = 0.10  # meters
        self.angular_baseline_std = math.radians(10)
        self.angular_baseline_std_sim = math.radians(30)

        # Stale timestamp tracking per camera
        self._last_timestamps: list[float] = [0.0] * len(self._camera_estimator_pairs)

        # Gyro-fused fallback: track recent yaw rates for stability check
        self._yaw_rate_history: deque[float] = deque(maxlen=15)
        SmartDashboard.putData("Estimated Field", self.estimated_field)

    def _compute_std_devs(
        self, avg_dist: float, tag_count: int, is_single_tag_gyro_fused: bool = False
    ) -> tuple[float, float, float]:
        """Compute standard deviations for a vision measurement."""
        angular_baseline = (
            self.angular_baseline_std_sim if is_sim() else self.angular_baseline_std
        )
        std_factor = (avg_dist**2) / tag_count
        std_xy = self.linear_baseline_std * std_factor
        if is_single_tag_gyro_fused:
            # Gyro heading is far more trustworthy; tell Kalman filter to ignore vision heading
            std_rot = 1e6
        else:
            std_rot = angular_baseline * std_factor
        return (std_xy, std_xy, std_rot)

    def _reject_measurement(
        self,
        pose3d: Pose3d,
        twod_pose: Pose2d,
        ts: float,
        cam_idx: int,
        targets: list,
        robot_pose: Pose2d,
        disabled: bool,
    ) -> bool:
        """Return True if the measurement should be rejected."""
        # Z-height check: reject poses that claim robot is far off ground
        if abs(pose3d.Z()) > 0.2:
            return True

        # Stale timestamp check
        if ts <= self._last_timestamps[cam_idx]:
            return True

        # Distance from current pose check (existing)
        dist = robot_pose.relativeTo(twod_pose).translation().norm()
        if dist > 1.0 and not disabled:
            return True

        # Yaw consistency check for weak observations

        # tag_count = len(targets)
        # total_area = sum(t.getArea() for t in targets)
        # avg_area = total_area / tag_count if tag_count > 0 else 0
        # if tag_count == 1 or avg_area < 2.0:
        #     # Compare vision heading to gyro heading
        #     gyro_heading = self.drivetrain.get_rotation().radians()
        #     vision_heading = twod_pose.rotation().radians()
        #     heading_diff = abs(math.atan2(
        #         math.sin(vision_heading - gyro_heading),
        #         math.cos(vision_heading - gyro_heading),
        #     ))
        #     if heading_diff > math.radians(5):
        #         return True

        return False

    def _fuse_estimates(
        self, estimates: list[tuple[Pose2d, float, tuple[float, float, float]]]
    ) -> tuple[Pose2d, float, tuple[float, float, float]]:
        """Fuse multiple vision estimates using inverse-variance weighting.

        Each estimate is (pose2d, timestamp, (std_x, std_y, std_rot)).
        Returns fused (pose2d, timestamp, std_devs).
        """
        if len(estimates) == 1:
            return estimates[0]

        # Use the most recent timestamp
        estimates.sort(key=lambda e: e[1])
        newest_ts = estimates[-1][1]

        speeds = self.swerve_drive.get_velocity()
        vx = speeds.vx
        vy = speeds.vy
        omega = speeds.omega

        # Time-align older estimates to newest using odometry delta
        aligned_poses: list[tuple[float, float, float]] = []
        aligned_stds: list[tuple[float, float, float]] = []
        for pose, ts, stds in estimates:
            if ts < newest_ts:
                # Approximate alignment: use the pose estimator's odometry
                # The time difference should be small (< 1 frame), so we use
                # the raw pose offset as a simple approximation
                dt = newest_ts - ts
                dx = vx * dt
                dy = vy * dt
                dtheta = omega * dt
                cos_t = math.cos(dtheta)
                sin_t = math.sin(dtheta)
                new_x = pose.x + dx * cos_t - dy * sin_t
                new_y = pose.y + dx * sin_t + dy * cos_t
                new_rot = pose.rotation().radians() + dtheta
                pose = Pose2d(new_x, new_y, Rotation2d(new_rot))
            aligned_poses.append((pose.x, pose.y, pose.rotation().radians()))
            aligned_stds.append(stds)

        poses = np.asarray(aligned_poses, dtype=float)
        stds = np.asarray(aligned_stds, dtype=float)

        inv_var = np.where(stds > 0.0, 1.0 / (stds**2), 1e6)
        sum_inv_var = inv_var.sum(axis=0)

        weighted_xy = (poses[:, :2] * inv_var[:, :2]).sum(axis=0)
        rot = poses[:, 2]
        weighted_sin = np.sin(rot) @ inv_var[:, 2]
        weighted_cos = np.cos(rot) @ inv_var[:, 2]

        fused_x = weighted_xy[0] / sum_inv_var[0]
        fused_y = weighted_xy[1] / sum_inv_var[1]
        fused_rot = math.atan2(weighted_sin, weighted_cos)

        fused_std_x = math.sqrt(1.0 / sum_inv_var[0])
        fused_std_y = math.sqrt(1.0 / sum_inv_var[1])
        fused_std_rot = math.sqrt(1.0 / sum_inv_var[2])

        fused_pose = Pose2d(fused_x, fused_y, Rotation2d(fused_rot))
        return (fused_pose, newest_ts, (fused_std_x, fused_std_y, fused_std_rot))

    def execute(self):
        disabled = is_disabled()

        # Update yaw rate history (deque auto-evicts oldest)
        yaw_rate = self.swerve_drive.get_velocity().omega
        self._yaw_rate_history.append(yaw_rate)

        # Collect valid estimates from all cameras for fusion
        valid_estimates: list[tuple[Pose2d, float, tuple[float, float, float]]] = []

        for cam_idx, (cam, pose_est) in enumerate(self._camera_estimator_pairs):
            cam.update()

            for res in cam.results:
                targets = res.getTargets()
                if not targets:
                    continue

                best_target = res.getBestTarget()
                if best_target and best_target.poseAmbiguity > 0.2:
                    continue

                # Try multi-tag estimation first (most accurate)
                pupdate = pose_est.estimateCoprocMultiTagPose(res)

                if pupdate is None:
                    pupdate = pose_est.estimateCoprocMultiTagPose(res)
                    if pupdate is None:
                        continue

                is_gyro_fused = False

                pose3d = pupdate.estimatedPose
                twod_pose = pose3d.toPose2d()
                ts = pupdate.timestampSeconds

                # Compute std devs
                tag_count = len(targets)
                total_dist = sum(
                    t.getBestCameraToTarget().translation().norm() for t in targets
                )
                avg_dist = total_dist / tag_count
                if avg_dist > 2.0 and not disabled:
                    continue

                std_devs = self._compute_std_devs(avg_dist, tag_count, is_gyro_fused)

                # Record timestamp
                self._last_timestamps[cam_idx] = ts

                valid_estimates.append((twod_pose, ts, std_devs))

        # Fuse and apply
        if valid_estimates:
            if len(valid_estimates) == 1:
                pose, ts, stds = valid_estimates[0]
            else:
                pose, ts, stds = self._fuse_estimates(valid_estimates)

            self.swerve_drive.addVisionPoseEstimate(pose, ts, stds)
            self.estimated_field.setRobotPose(self.swerve_drive.cached_pose)
