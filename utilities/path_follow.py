from typing import List

from wpilib import controller
from wpilib import trajectory
from wpilib import Timer
from wpilib.trajectory import constraint
from wpilib.geometry import Pose2d

from components.chassis import Chassis


class Path:
    start: Pose2d
    waypoints: List(Pose2d)
    end: Pose2d
    reversed: bool

    def __init__(self, poses, reversed) -> None:
        self.start, *self.waypoints, self.end = poses


class PathFollow:
    """
    A class to follow a given class, run must be called in each loop where
    an output is wanted
    """

    def __init__(self, chassis: Chassis, tolerance: Pose2d) -> None:
        self.chassis = chassis
        self.controller = controller.RamseteController()
        self.controller.setTolerance(tolerance)
        self.trajectory_config = trajectory.TrajectoryConfig(
            maxVelocity=1, maxAcceleration=1
        )
        self.gen = trajectory.TrajectoryGenerator()
        self.trajectory_config.setKinematics(self.chassis.kinematics)
        self.trajectory_config.addConstraint(
            constraint.DifferentialDriveVoltageConstraint(
                self.chassis.ff_calculator, self.chassis.kinematics, maxVoltage=10
            )
        )
        self.trajectory: trajectory.Trajectory
        self.timer: Timer = Timer()
        self.start_time: float

    def new_path(self, path: Path) -> None:
        """
        Give the path follower a new path, it will abandon the current one and
        follow it instead
        """
        self.trajectory_config.setReversed(path.reversed)
        self.trajectory = self.gen.generateTrajectory(
            path.start,
            path.waypoints,
            path.end,
            self.trajectory_config,
        )
        self.start_time = Timer.getFPGATimestamp()

    def run(self) -> None:
        """
        Send the chassis control inputs for this loop
        """
        if self.path_done():
            self.chassis.drive(0, 0)
        else:
            pos = self.chassis.get_pose()
            goal = self.trajectory.sample(self.current_path_time)
            speeds = self.controller.calculate(pos, goal)
            self.chassis.drive(speeds.vx, speeds.omega)

    def path_done(self) -> bool:
        """
        Check to see if enough time has passed to complete the path
        """
        # TODO investigate closing the loop here
        self.current_path_time = Timer.getFPGATimestamp() - self.start_time
        return self.current_path_time > self.trajectory.totalTime()
