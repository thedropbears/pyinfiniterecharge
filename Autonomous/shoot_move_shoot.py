import math
import time
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
        self.controller = controller.RamseteController()
        tolerance = to_pose(0.1, 0.1, math.pi / 18)
        self.controller.setTolerance(tolerance)
        self.trajectory_config = trajectory.TrajectoryConfig(
            maxVelocity=1, maxAcceleration=1
        )
        self.gen = trajectory.TrajectoryGenerator()
        self.trajectory_num = 0
        self.trajectory_max = 1

    def setup(self):
        self.trajectory_config.setKinematics(self.chassis.kinematics)

    def on_enable(self) -> None:
        self.chassis.reset_odometry(geometry.Pose2d())
        super().on_enable()

    @state(first=True)
    def shoot(self) -> None:
        """
        Shoot all balls that we currently have
        """
        self.shooter_controller.engage()
        self.shooter_controller.fire_input()
        if self.indexer.balls_loaded() == 0:
            if self.trajectory_num >= self.trajectory_max:
                self.done()
            else:
                self.next_state("move")

    @state
    def move(self, initial_call, state_tm) -> None:
        """
        Follow the trajectory defined by our waypoints
        """
        if state_tm > self.path.totalTime():
            print(f"Calculated path time: {self.path.totalTime()}")
            self.chassis.drive(0, 0)
            self.next_state("shoot")
            self.trajectory_num += 1
            return
        pos = self.chassis.get_pose()
        goal = self.path.sample(state_tm)
        speeds = self.controller.calculate(pos, goal)
        self.chassis.drive(speeds.vx, speeds.omega)


class test(ShootMoveShootBase):
    MODE_NAME = "Test"
    DEFAULT = True

    def __init__(self) -> None:
        super().__init__()

    def setup(self):
        super().setup()
        self.start_pose = to_pose(0, 0, 0)
        self.chassis.reset_odometry(self.start_pose)
        self.end_pose = to_pose(2, 0, 0)
        self.waypoints = [geometry.Translation2d(1, 0)]
        self.trajectory_config = trajectory.TrajectoryConfig(
            maxVelocity=1, maxAcceleration=1
        )
        self.path = self.gen.generateTrajectory(
            self.start_pose, self.waypoints, self.end_pose, self.trajectory_config
        )
