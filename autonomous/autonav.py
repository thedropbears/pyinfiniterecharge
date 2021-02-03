from typing import List

from wpilib.geometry import Pose2d, Translation2d
from magicbot import AutonomousStateMachine, state

from components.chassis import Chassis
from utilities.path_follow import Path, LoadPath, PathFollow
import math
import os


class AutoNavBase(AutonomousStateMachine):

    chassis: Chassis

    def __init__(self) -> None:
        super().__init__()
        self.path_num = 0
        self.paths: List(Path)

    def setup(self) -> None:
        self.path_follow: PathFollow = PathFollow(self.chassis)

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
    def done_move(self):
        self.path_num += 1
        if self.path_num >= len(self.paths):
            self.done()
        else:
            self.next_state("move")


class test(AutoNavBase):
    MODE_NAME = "Test"
    DEFAULT = True

    def setup(self):
        self.paths = [
            Path(
                [
                    Pose2d(0, 0, 0),
                    Translation2d(1, 1),
                    Translation2d(0, 1),
                    Translation2d(-1, 0),
                    Pose2d(0, 0, 0),
                ],
                reversed=False,
            )
        ]
        super().setup()


class BarrelRacing(AutoNavBase):
    MODE_NAME = "Barrel Racing"

    def setup(self):
        self.paths = [
            Path(
                [
                    Pose2d(1.044, 2.139, 0),
                    Translation2d(4.031, 2.25),
                    Translation2d(4.718, 1.483),
                    Translation2d(3.585, 0.689),
                    Translation2d(2.925, 1.991),
                    Translation2d(4.985, 2.330),
                    Translation2d(4.985, 2.330),
                    Translation2d(6.903, 2.562),
                    Translation2d(6.456, 3.7),
                    Translation2d(5.368, 3.346),
                    Translation2d(6.126, 1.269),
                    Translation2d(7.250, 0.921),
                    Translation2d(8.0, 1.0),
                    Translation2d(7.8, 2.0),
                    Translation2d(6.5, 2.2),
                    Pose2d(1.044, 2.2, math.pi),
                ],
                reversed=False,
            )
        ]
        super().setup()


PATHWEAVER_PATH = os.path.join( os.path.dirname(os.path.abspath(__file__)) , "pathweaver_paths", "output")
class LoadTest(AutoNavBase):
    MODE_NAME = "Pathweaver Test"

    def setup(self):
        self.paths = [
            LoadPath(os.path.join(PATHWEAVER_PATH, "Barrel_Racing.wpilib.json"), reversed=False)
        ]
        super().setup()
