import math
from typing import Optional

import wpilib.geometry
import wpilib.controller
from magicbot import AutonomousStateMachine, state

from components.chassis import Chassis
from components.indexer import Indexer
from controllers.shooter import ShooterController


def to_pose(x: float, y: float, heading: float) -> wpilib.geometry.Pose2d:
    """
    Convert inputs into a wpilib pose object
    """
    rotation = wpilib.geometry.Rotation2d(heading)
    return wpilib.geometry.Pose2d(x, y, rotation)


class ShootMoveShootBase(AutonomousStateMachine):

    shooter_controller: ShooterController

    chassis: Chassis
    indexer: Indexer

    def __init__(self) -> None:
        super().__init__()
        self.controller = wpilib.controller.RamseteController()
        tolerance = to_pose(0.1, 0.1, math.pi/18)
        self.controller.setTolerance(tolerance)
        self.pos_tolerance = 0.1  # m
        self.ang_tolerance = math.pi / 18  # rad
        # TODO tune these
        # waypoints is defined in each subclass
        # super().__init__() must be called after this
        # otherwise the below line will throw an error
        self.current_waypoint = self.waypoints[0]
        self.waypoint_index = 0

    def on_enable(self) -> None:
        self.chassis.reset_odometry(wpilib.geometry.Pose2d())
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
    def move(self, initial_call):
        """
        Follow the trajectory defined by our waypoints
        """
        if initial_call:
            if self.current_waypoint is None:
                # end the statemachine if we re-enter move
                # with no new trajectory
                self.done()
                return
        pos = self.chassis.get_pose()
        if self.current_waypoint is None:
            self.chassis.drive(0, 0)
            self.next_state("shoot")
            return
        speeds = self.controller.calculate(pos, self.current_waypoint, 1, 0)
        self.chassis.drive(speeds.vx, speeds.omega)
        if self.controller.atReference():
            self.current_waypoint = self.get_next_waypoint()

    def get_next_waypoint(self) -> Optional[wpilib.geometry.Pose2d]:
        """
        Return the next waypoint in the que, returns none if there is none
        """
        if len(self.waypoints)-1 > self.waypoint_index:
            self.waypoint_index += 1
            return self.waypoints[self.waypoint_index]
        else:
            return None


class test(ShootMoveShootBase):
    MODE_NAME = "Test"
    DEFAULT = True

    def __init__(self) -> None:
        self.waypoints = [to_pose(2, 0.5, 0)]
        super().__init__()
