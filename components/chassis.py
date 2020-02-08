import math

import rev
import wpilib
from wpilib.drive import DifferentialDrive
from typing import Tuple


class Chassis:
    chassis_left_rear: rev.CANSparkMax
    chassis_left_front: rev.CANSparkMax
    chassis_right_rear: rev.CANSparkMax
    chassis_right_front: rev.CANSparkMax

    WHEEL_RADIUS = 0.076
    GEAR_REDUCTION = 1/10.75
    COUNTS_TO_METERS = GEAR_REDUCTION * WHEEL_RADIUS * math.tau

    def setup(self) -> None:
        self.chassis_left_rear.setInverted(False)
        self.chassis_left_front.setInverted(False)
        self.chassis_right_rear.setInverted(False)
        self.chassis_right_front.setInverted(False)

        self.chassis_left_rear.follow(self.chassis_left_front)
        self.chassis_right_rear.follow(self.chassis_right_front)

        self.left_encoder = self.chassis_left_front.getEncoder()
        self.right_encoder = self.chassis_right_front.getEncoder()

        self.odometry_tracker = wpilib.kinematics.DifferentialDriveOdometry()

        self.diff_drive = DifferentialDrive(
            self.chassis_left_front, self.chassis_right_front
        )

    def execute(self) -> None:
        heading = self.get_heading()
        left_travel = self.left_encoder.getPosition() * self.COUNTS_TO_METERS
        right_travel = self.right_encoder.getPosition() * self.COUNTS_TO_METERS
        self.diff_drive.arcadeDrive(self.vx, self.vz, squareInputs=False)
        self.odometry_tracker.update(heading, left_travel, right_travel)

    def drive(self, vx: float, vz: float) -> None:
        """Drive the robot with forwards velocity and rotational velocity
            vx: Forward is positive.
            vz: Clockwise is negative
        """
        self.vx, self.vz = vx, -vz

    def get_heading(self) -> float:
        # TODO
        return 0.0

    def get_position(self) -> Tuple[float, float]:
        # TODO
        return (0.0, 0.0)
