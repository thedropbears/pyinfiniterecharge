import math
from typing import List

from wpilib.geometry import Pose2d, Translation2d
from magicbot import AutonomousStateMachine, state

from components.chassis import Chassis
from components.indexer import Indexer
from controllers.shooter import ShooterController
from utilities.path_follow import Path, PathFollow


class RunAndGun(AutonomousStateMachine):

    shooter_controller: ShooterController

    chassis: Chassis
    indexer: Indexer

    def __init__(self) -> None:
        super().__init__()
        self.path_num = 0
        self.paths: List(Path)

    def setup(self) -> None:
        self.path_follow: PathFollow = PathFollow(self.chassis)
        self.indexer.auto_retract = True

    def on_enable(self) -> None:
        self.chassis.reset_odometry(self.paths[0].start)
        self.path_num = 0
        super().on_enable()

    @state(first=True)
    def move(self, initial_call) -> None:
        """
        Follow the trajectory defined by our waypoints
        """
        if initial_call:
            path = self.paths[self.path_num]
            self.path_follow.new_path(path)
        self.path_follow.run()
        if self.path_follow.path_done():
            self.next_state("done_move")

    @state
    def shoot(self) -> None:
        """
        Shoot while stationary before moving on the current trajectory
        """
        self.shooter_controller.engage()
        self.shooter_controller.fire_input()
        if self.indexer.balls_loaded() <= 0:
            self.next_state("move")

    @state(first=True)
    def run_and_gun(self, initial_call) -> None:
        """
        Follow the trajectory defined by our waypoints shooting until we run out of balls
        """
        if initial_call:
            path = self.paths[self.path_num]
            self.path_follow.new_path(path)
        self.path_follow.run()
        self.shooter_controller.engage()
        self.shooter_controller.fire_input()
        if self.path_follow.path_done() and self.indexer.balls_loaded() <= 0:
            self.next_state("done_move")

    @state
    def done_move(self):
        self.path_num += 1
        if self.path_num >= len(self.paths):
            self.done()
        else:
            path = self.paths[self.pathnum]

            if path.intake:
                self.indexer.lower_intake()
            else:
                # The intake is also raised if we reach 5 balls in the indexer
                self.indexer.raise_intake()

            if path.shoot:
                if path.stop_to_fire:
                    self.next_state("shoot")
                else:
                    self.next_state("run_and_gun")
            else:
                self.next_state("move")
