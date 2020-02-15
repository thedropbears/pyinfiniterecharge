import math
from typing import Optional

from wpilib import controller
from wpilib import geometry
from wpilib import trajectory
from magicbot import AutonomousStateMachine, state

from components.chassis import Chassis
from components.indexer import Indexer
from controllers.shooter import ShooterController


def to_pose(x: float, y: float, heading: float) -> geometry.Pose2d:
    """
    Convert inputs into a wpilib pose object
    """
    rotation = geometry.Rotation2d(heading)
    return geometry.Pose2d(x, y, rotation)


class ShootMoveShootBase(AutonomousStateMachine):

    shooter_controller: ShooterController

    chassis: Chassis
    indexer: Indexer

    def __init__(self) -> None:
        super().__init__()
        self.controller = controller.RamseteController(1, 0.7)
        tolerance = to_pose(0.1, 0.1, math.pi / 18)
        self.controller.setTolerance(tolerance)
        self.trajectory_config = trajectory.TrajectoryConfig(
            maxVelocity=1, maxAcceleration=1
        )
        self.gen = trajectory.TrajectoryGenerator()

    def setup(self):
        self.trajectory_config.setKinematics(self.chassis.kinematics)

    def on_enable(self) -> None:
        self.chassis.reset_odometry(geometry.Pose2d())
        super().on_enable()

    @state
    def shoot(self) -> None:
        """
        Shoot all balls that we currently have
        """
        self.shooter_controller.engage()
        self.shooter_controller.fire_command()
        if self.indexer.balls_loaded == 0:
            self.next_state("move")

    @state(first=True)
    def move(self, initial_call) -> None:
        """
        Follow the trajectory defined by our waypoints
        """
        pos = self.chassis.get_pose()
        speeds = self.controller.calculate(pos, self.goal)
        self.chassis.drive(speeds.vx, speeds.omega)
        if self.controller.atReference():
            self.current_waypoint_id += 1
            if self.current_waypoint_id >= self.number_of_waypoints:
                self.done()
                return
            self.goal = self.get_waypoint(self.current_waypoint_id)
            print(self.goal.velocity)

    def get_waypoint(self, waypoint_id: int) -> Optional[trajectory.Trajectory.State]:
        """Get waypoint by a zero indexed id"""
        if waypoint_id >= self.number_of_waypoints:
            print("Attempted to track waypoint beyond number in trajectory.")
            return
        time = self.time_per_waypoint * (waypoint_id + 1)
        waypoint = self.path.sample(time)
        return waypoint


class test(ShootMoveShootBase):
    MODE_NAME = "Test"
    DEFAULT = True

    def __init__(self) -> None:
        super().__init__()
        self.start_pose = to_pose(0, 0, 0)
        self.end_pose = to_pose(4, 0, 0)
        self.waypoints = [geometry.Translation2d(2, 0), geometry.Translation2d(3, 0)]
        self.trajectory_config = trajectory.TrajectoryConfig(
            maxVelocity=1, maxAcceleration=1
        )
        self.path = self.gen.generateTrajectory(
            self.start_pose, self.waypoints, self.end_pose, self.trajectory_config
        )

        self.number_of_waypoints = 5
        self.total_time = self.path.totalTime()
        self.time_per_waypoint = self.total_time / self.number_of_waypoints

        self.current_waypoint_id = 0
        self.goal = self.get_waypoint(self.current_waypoint_id)
        self.current_waypoint_id += 1
