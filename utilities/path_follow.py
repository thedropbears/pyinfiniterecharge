from typing import List

from wpilib import controller
from wpilib import trajectory
from wpilib import Timer
from wpilib.trajectory import constraint
from wpilib.trajectory import TrajectoryUtil
from wpilib.geometry import Pose2d, Translation2d

from components.chassis import Chassis


class Path:
    start: Pose2d
    waypoints: List[Translation2d]
    end: Pose2d
    reversed: bool

    def __init__(self, points, reversed) -> None:
        self.start, *self.waypoints, self.end = points
        self.reversed = reversed

    def getTrajectory(self, config: trajectory.TrajectoryConfig):
        return trajectory.TrajectoryGenerator().generateTrajectory(
            self.start, self.waypoints, self.end, config
        )


class LoadPath:
    start: Pose2d
    filename: str
    reversed: bool

    def __init__(self, filename, reversed):
        self.filename = filename
        self.reversed = reversed
        self.trajectory = TrajectoryUtil.fromPathweaverJson(self.filename)
        self.start = self.trajectory.initialPose()  # is used to reset odometry
        print(f"loaded file {filename}")
        print(f"states num: {len(self.trajectory.states())}")

    def getTrajectory(self, *_: trajectory.TrajectoryConfig):
        # dosent care about the config you give it, set config through pathweaver
        return self.trajectory


class PathFollow:
    """
    A class to follow a given class, run must be called in each loop where
    an output is wanted
    """

    def __init__(self, chassis: Chassis) -> None:
        self.chassis = chassis
        self.controller = controller.RamseteController()
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
        self.trajectory_config.addConstraint(
            constraint.CentripetalAccelerationConstraint(1.5)  # m/s^2
        )
        self.trajectory: trajectory.Trajectory
        self.start_time: float

    def new_path(self, path: Path) -> None:
        """
        Give the path follower a new path, it will abandon the current one and
        follow it instead
        """
        self.trajectory_config.setReversed(path.reversed)
        self.trajectory = path.getTrajectory(self.trajectory_config)
        self.start_time = Timer.getFPGATimestamp()
        print("total path time: ", self.trajectory.totalTime())

    def new_quintic_path(self, waypoints: List[Pose2d], reversed: bool):
        """
        Give the path follower a new path, it will abandon the current one and
        follow it instead. This method specifies the heading at each waypoint.
        """
        self.trajectory_config.setReversed(reversed)
        self.trajectory = self.gen.generateTrajectory(
            waypoints,
            self.trajectory_config,
        )
        self.start_time = Timer.getFPGATimestamp()

    def new_quintic_path(self, waypoints: List[Pose2d], reversed: bool):
        """
        Give the path follower a new path, it will abandon the current one and
        follow it instead. This method specifies the heading at each waypoint.
        """
        self.trajectory_config.setReversed(reversed)
        self.trajectory = self.gen.generateTrajectory(
            waypoints,
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
