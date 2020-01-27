import rev
import wpilib
from wpilib.drive import DifferentialDrive


class Chassis:
    chassis_left_rear: rev.CANSparkMax
    chassis_left_front: rev.CANSparkMax
    chassis_right_rear: rev.CANSparkMax
    chassis_right_front: rev.CANSparkMax

    def setup(self) -> None:
        self.chassis_left_rear.setInverted(True)
        self.chassis_left_front.setInverted(True)
        self.chassis_right_rear.setInverted(True)
        self.chassis_right_front.setInverted(True)

        self.left = wpilib.SpeedControllerGroup(
            self.chassis_left_front, self.chassis_left_rear
        )
        self.right = wpilib.SpeedControllerGroup(
            self.chassis_right_front, self.chassis_right_rear
        )

        self.diff_drive = DifferentialDrive(self.left, self.right)

    def execute(self) -> None:
        pass

    def drive(self, vx: float, vz: float) -> None:
        """Drive the robot with forwards velocity and rotational velocity
            vx: Forward is positive.
            vz: Clockwise is negative, inverted below because arcadeDrive's positive is clockwise.
        """
        self.diff_drive.arcadeDrive(-vx, vz)

    def get_heading(self) -> float:
        # TODO
        return 0.0

    def get_position(self) -> tuple:
        # TODO
        return (0.0, 0.0)
