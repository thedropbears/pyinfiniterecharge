import math
from typing import Optional

import wpilib
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

    shootercontroller: ShooterController

    chassis: Chassis
    indexer: Indexer

    def __init__(self) -> None:
        super().__init__()
        self.controller = wpilib.RamseteController()
        self.pos_tolerance = 0.1  # m
        self.ang_tolerance = math.pi / 18  # rad
        # TODO tune these
        # waypoints is defined in each subclass
        # super().__init__() must be called after this
        # otherwise the below line will throw an error
        self.current_waypoint = self.waypoints[0]
        self.waypoint_index = 0

    @state(first=True)
    def shoot(self) -> None:
        """
        Shoot all balls that we currently have
        """
        self.shootercontroller.engage()
        self.shootercontroller.fire_command()
        if self.indexer.balls_loaded == 0:
            self.next_state("move")

    @state
    def move(self, initial_call):
        """
        Follow the trajectory defined by our waypoints
        """
        if initial_call:
            if self.current_waypoint is None:
                # end the statemachine if we re-enter move
                # with no new trajectory
                self.done()
        pos = self.chassis.get_pose()
        if self.current_waypoint is None:
            self.chassis.drive(0, 0)
            self.next_state("shoot")
        vx, _, vz = self.controller.calculate(pos, self.current_waypoint, 3, 0)
        self.chassis.drive(vx, vz)
        error = self.current_waypoint.relativeTo(pos)
        if (
            abs(error.translation) < self.pos_tolerance
            and abs(error.rotation) < self.ang_tolerance
        ):
            self.current_waypoint = self.get_next_waypoint()

    def get_next_waypoint(self) -> Optional(wpilib.geometry.Pose2d):
        """
        Return the next waypoint in the que, returns none if there is none
        """
        if len(self.waypoints) > self.waypoint_index:
            self.waypoint_index += 1
            return self.waypoints[self.waypoint_index]
        else:
            return None


class Basic3Forward3(ShootMoveShootBase):
    def __init__(self) -> None:
        self.waypoints = (to_pose(1, 0, 0), to_pose(2, 1, -math.pi / 4))
        super().__init__()
