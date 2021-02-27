from typing import List

from wpilib import controller
from wpilib import trajectory
from wpilib import Timer
from wpilib.trajectory import constraint
from wpilib.geometry import Pose2d, Translation2d

from components.chassis import Chassis


class Path:
    start: Pose2d
    waypoints: List[Translation2d]
    end: Pose2d
    reversed: bool
    shoot: bool
    intake: bool
    stop_to_fire: bool

    def __init__(self, points, reversed, shoot=False, intake=False, stop_to_fire=True) -> None:
        self.start, *self.waypoints, self.end = points
        self.reversed = reversed
        self.shoot = shoot
        self.intake = intake
        self.stop_to_fire = stop_to_fire


class PathFollow:
    """
    A class to follow a given path, run must be called in each loop where
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
        self.cached_path: Path
        self.trajectory: trajectory.Trajectory
        self.start_time: float

    def set_path(self, path: Path) -> None:
        """
        Give the path follower a new path, it will abandon the current one and
        follow it instead
        """
        self.trajectory_config.setReversed(path.reversed)
        self.cached_path = path
        self.trajectory = self.new_path(path)
        self.trajectory_config.setStartVelocity(0)
        self.start_time = Timer.getFPGATimestamp()

    # def set_quintic_path(self, waypoints: List[Pose2d], reversed: bool):
    #     """
    #     Give the path follower a new path, it will abandon the current one and
    #     follow it instead. This method specifies the heading at each waypoint.
    #     """
    #     self.trajectory_config.setReversed(reversed)
    #     self.trajectory = self.gen.generateTrajectory(
    #         waypoints,
    #         self.trajectory_config,
    #     )
    #     self.start_time = Timer.getFPGATimestamp()

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

    def new_path(self, path: Path) -> trajectory.Trajectory:
        """
        Returns a trajectory from the path with the current constraints
        """
        return self.gen.generateTrajectory(
            path.start,
            path.waypoints,
            path.end,
            self.trajectory_config,
        )

    def get_total_path_time(self, path: Path) -> float:
        """
        Returns the amount of time expected to complete the given path
        """
        trajectory = self.new_path(path)
        return trajectory.totalTime

    def set_transition_speed(self, speed: float) -> None:
        """
        A rather hacky way of transitioning at a given speed at runtime between two paths
        """
        # there is an edge case where this makes us exceed acceleration constraints
        # TODO handle it
        self.trajectory_config.setEndVelocity(speed)
        self.set_path(self.cached_path)
        self.trajectory_config.setEndVelocity(0)
        self.trajectory_config.setStartVelocity(speed)
