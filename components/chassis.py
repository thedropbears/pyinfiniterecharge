import math

import magicbot
import rev

from utilities.nav_x import NavX

from wpilib.controller import SimpleMotorFeedforwardMeters
from wpilib.geometry import Pose2d, Rotation2d
from wpilib.kinematics import (
    ChassisSpeeds,
    DifferentialDriveKinematics,
    DifferentialDriveOdometry,
)

GEAR_RATIO = 10.75

# measurements in metres
TRACK_WIDTH = 0.579  # measured by characterisation
WHEEL_CIRCUMFERENCE = 0.0254 * 6 * math.pi


class Chassis:
    left_rear: rev.CANSparkMax
    left_front: rev.CANSparkMax
    right_rear: rev.CANSparkMax
    right_front: rev.CANSparkMax

    imu: NavX

    open_loop = magicbot.tunable(False)
    vx = magicbot.will_reset_to(0.0)
    vz = magicbot.will_reset_to(0.0)

    def setup(self) -> None:
        self.left_front.setInverted(False)
        self.right_front.setInverted(True)

        for motor in (
            self.left_front,
            self.left_rear,
            self.right_front,
            self.right_rear,
        ):
            motor.setIdleMode(rev.IdleMode.kCoast)

        self.left_rear.follow(self.left_front)
        self.right_rear.follow(self.right_front)

        self.left_encoder = rev.CANEncoder(self.left_front)
        self.right_encoder = rev.CANEncoder(self.right_front)

        rev_to_m = WHEEL_CIRCUMFERENCE / GEAR_RATIO

        for enc in (self.left_encoder, self.right_encoder):
            enc.setPositionConversionFactor(rev_to_m)
            enc.setVelocityConversionFactor(rev_to_m / 60)

        self.left_pid: rev.CANPIDController = self.left_front.getPIDController()
        self.right_pid: rev.CANPIDController = self.right_front.getPIDController()

        self.ff_calculator = SimpleMotorFeedforwardMeters(kS=0.194, kV=2.79, kA=0.457)
        for pid in (self.left_pid, self.right_pid):
            # TODO: needs tuning
            pid.setP(5.19 / 10)
            pid.setI(0)
            pid.setD(0)
            pid.setIZone(0)
            pid.setFF(0)
            pid.setOutputRange(-1, 1)

        self.kinematics = DifferentialDriveKinematics(TRACK_WIDTH)
        self.odometry = DifferentialDriveOdometry(self._get_heading())

    def execute(self) -> None:
        # XXX: https://github.com/robotpy/robotpy-wpilib/issues/635
        chassis_speeds = ChassisSpeeds()
        chassis_speeds.vx = self.vx
        chassis_speeds.omega = self.vz

        speeds = self.kinematics.toWheelSpeeds(chassis_speeds)

        left_ff = self.ff_calculator.calculate(speeds.left)
        right_ff = self.ff_calculator.calculate(speeds.right)

        if self.open_loop:
            self.left_front.setVoltage(left_ff)
            self.right_front.setVoltage(right_ff)
        else:
            self.left_pid.setReference(
                speeds.left, rev.ControlType.kVelocity, arbFeedforward=left_ff,
            )
            self.right_pid.setReference(
                speeds.right, rev.ControlType.kVelocity, arbFeedforward=right_ff,
            )

        self.odometry.update(
            self._get_heading(),
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

    def disable_closed_loop(self) -> None:
        """Run the drivetrain in open loop mode (feedforward only)."""
        self.open_loop = True

    def enable_closed_loop(self) -> None:
        """Run the drivetrain in velocity closed loop mode."""
        self.open_loop = False

    def _get_heading(self) -> Rotation2d:
        """Get the current heading of the robot from the IMU, anticlockwise positive."""
        return Rotation2d(self.imu.getYaw())

    def get_pose(self) -> Pose2d:
        """Get the current position of the robot on the field."""
        return self.odometry.getPose()

    @magicbot.feedback
    def get_left_velocity(self) -> float:
        return self.left_encoder.getVelocity()

    @magicbot.feedback
    def get_right_velocity(self) -> float:
        return self.right_encoder.getVelocity()

    def reset_odometry(self, pose: Pose2d) -> None:
        """Resets the odometry to start with the given pose.

        This is intended to be called at the start of autonomous.
        """
        self.left_encoder.setPosition(0)
        self.right_encoder.setPosition(0)
        self.odometry.resetPosition(pose, self._get_heading())
