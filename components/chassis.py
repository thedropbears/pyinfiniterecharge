import rev
import wpilib
from wpilib.drive import DifferentialDrive
from typing import Tuple


class Chassis:
    chassis_left_rear: rev.CANSparkMax
    chassis_left_front: rev.CANSparkMax
    chassis_right_rear: rev.CANSparkMax
    chassis_right_front: rev.CANSparkMax

    def setup(self) -> None:
        self.chassis_left_rear.setInverted(False)
        self.chassis_left_front.setInverted(False)
        self.chassis_right_rear.setInverted(False)
        self.chassis_right_front.setInverted(False)

        self.chassis_left_rear.follow(self.chassis_left_front)
        self.chassis_right_rear.follow(self.chassis_right_front)

        self.diff_drive = DifferentialDrive(
            self.chassis_left_front, self.chassis_right_front
        )

    def execute(self) -> None:
        self.diff_drive.arcadeDrive(self.vx, self.vz)

    def drive(self, vx: float, vz: float) -> None:
        """Drive the robot with forwards velocity and rotational velocity
            vx: Forward is positive.
            vz: Clockwise is negative
        """
        self.vx, self.vz = vx, vz

    def get_heading(self) -> float:
        # TODO
        return 0.0

    def get_position(self) -> Tuple[float, float]:
        # TODO
        return (0.0, 0.0)
