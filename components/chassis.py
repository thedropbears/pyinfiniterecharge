import math

import rev
import wpilib
import wpilib.geometry
import wpilib.kinematics
from wpilib.drive import DifferentialDrive

from utilities.navx import NavX


class Chassis:
    chassis_left_rear: rev.CANSparkMax
    chassis_left_front: rev.CANSparkMax
    chassis_right_rear: rev.CANSparkMax
    chassis_right_front: rev.CANSparkMax

    imu: NavX

    WHEEL_RADIUS = 0.076
    GEAR_REDUCTION = 10.75
    COUNTS_TO_METERS = GEAR_REDUCTION * WHEEL_RADIUS * math.tau

    def setup(self) -> None:
        self.chassis_left_rear.setInverted(False)
        self.chassis_left_front.setInverted(False)
        self.chassis_right_rear.setInverted(False)
        self.chassis_right_front.setInverted(False)

        self.chassis_left_rear.follow(self.chassis_left_front)
        self.chassis_right_rear.follow(self.chassis_right_front)

        self.left_encoder: rev.CANEncoder = self.chassis_left_front.getEncoder()
        self.right_encoder: rev.CANEncoder = self.chassis_right_front.getEncoder()
        for encoder in (self.left_encoder, self.right_encoder):
            # measure encoder outputs in m and m/s
            encoder.setPositionConversionFactor(self.COUNTS_TO_METERS)
            encoder.setVelocityConversionFactor(self.COUNTS_TO_METERS * 60)

        rotation = wpilib.geometry.Rotation2d(0)

        self.odometry_tracker = wpilib.kinematics.DifferentialDriveOdometry(rotation)
        # if reset_pose is not called, the robot assumes it started at 0, 0

        self.diff_drive = DifferentialDrive(
            self.chassis_left_front, self.chassis_right_front
        )

    def execute(self) -> None:
        rotation = wpilib.geometry.Rotation2d(self.imu.getAngle())
        left_travel = self.left_encoder.getPosition()
        right_travel = self.right_encoder.getPosition()
        self.diff_drive.arcadeDrive(self.vx, self.vz, squareInputs=False)
        self.odometry_tracker.update(rotation, left_travel, right_travel)

    def drive(self, vx: float, vz: float) -> None:
        """Drive the robot with forwards velocity and rotational velocity
            vx: Forward is positive.
            vz: Clockwise is negative
        """
        self.vx, self.vz = vx, -vz

    def get_pose(self) -> wpilib.geometry.Pose2d:
        """Return our position and heading as a pose."""
        return self.odometry_tracker.getPose()

    def reset_pose(self, pose: wpilib.geometry.Pose2d) -> None:
        """To be called by autonomous to set our start position"""
        self.left_encoder.setPosition(0)
        self.right_encoder.setPosition(0)
        self.imu.resetHeading()
        self.odometry_tracker.resetPosition(pose, 0)
