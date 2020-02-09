import math

import magicbot
import navx
import rev

from wpilib.geometry import Pose2d, Rotation2d
from wpilib.kinematics import (
    ChassisSpeeds,
    DifferentialDriveKinematics,
    DifferentialDriveOdometry,
)

GEAR_RATIO = 10.75

# measurements in metres
TRACK_WIDTH = 0.630  # theoretical as measured
WHEEL_CIRCUMFERENCE = 0.0254 * 6 * math.pi


class Chassis:
    left_rear: rev.CANSparkMax
    left_front: rev.CANSparkMax
    right_rear: rev.CANSparkMax
    right_front: rev.CANSparkMax

    imu: navx.AHRS

    vx = magicbot.will_reset_to(0.0)
    vy = magicbot.will_reset_to(0.0)

    def setup(self) -> None:
        self.left_front.setInverted(False)
        self.right_front.setInverted(True)

        self.left_rear.follow(self.left_front)
        self.right_rear.follow(self.right_front)

        self.left_encoder: rev.CANEncoder = self.left_front.getEncoder()
        self.right_encoder: rev.CANEncoder = self.right_front.getEncoder()

        for enc in (self.left_encoder, self.right_encoder):
            enc.setPositionConversionFactor(GEAR_RATIO * WHEEL_CIRCUMFERENCE)
            enc.setVelocityConversionFactor(GEAR_RATIO * WHEEL_CIRCUMFERENCE / 60)

        self.left_pid: rev.CANPIDController = self.left_front.getPIDController()
        self.right_pid: rev.CANPIDController = self.right_front.getPIDController()

        for pid in (self.left_pid, self.right_pid):
            # TODO: needs tuning
            pid.setP(6e-4)
            pid.setI(1e-6)
            pid.setD(0)
            pid.setIZone(0)
            pid.setFF(0.000156)
            pid.setOutputRange(-1, 1)

        self.kinematics = DifferentialDriveKinematics(TRACK_WIDTH)
        self.odometry = DifferentialDriveOdometry(self.get_heading())

    def execute(self) -> None:
        # XXX: https://github.com/robotpy/robotpy-wpilib/issues/635
        chassis_speeds = ChassisSpeeds()
        chassis_speeds.vx = self.vx
        chassis_speeds.omega = self.vz

        speeds = self.kinematics.toWheelSpeeds(chassis_speeds)

        # TODO: use characterisation suite to find appropriate feedforward
        self.left_pid.setReference(speeds.left, rev.ControlType.kVelocity)
        self.right_pid.setReference(speeds.right, rev.ControlType.kVelocity)

        self.odometry.update(
            self.get_heading(),
            self.left_encoder.getPosition(),
            self.right_encoder.getPosition(),
        )

    def drive(self, vx: float, vz: float) -> None:
        """Sets the desired robot velocity.

        Arguments:
            vx: Forwards linear velocity in m/s.
            vz: Angular velocity in rad/s, anticlockwise positive.
        """
        self.vx = vx
        self.vz = vz

    def get_heading(self) -> Rotation2d:
        """Get the current heading of the robot, anticlockwise positive."""
        return Rotation2d.fromDegrees(-self.imu.getYaw())

    def get_pose(self) -> Pose2d:
        """Get the current position of the robot on the field."""
        return self.odometry.getPose()

    def reset_odometry(self, pose: Pose2d) -> None:
        """Resets the odometry to start with the given pose.

        This is intended to be called at the start of autonomous.
        """
        self.left_encoder.setPosition(0)
        self.right_encoder.setPosition(0)
        self.odometry.resetPosition(pose, self.get_heading())
