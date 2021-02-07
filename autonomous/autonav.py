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
                    Translation2d(3, 0),
                    Pose2d(4, 1, math.pi/2),
                ],
                reversed=False,
            ),
            Path(
                [
                    Pose2d(4, 1, math.pi/2),
                    Translation2d(3, 0),
                    Pose2d(0, 0, 0),
                ],
                reversed=True
            ),
            Path(
                [
                    Pose2d(0, 0, 0),
                    Translation2d(3, 0),
                    Pose2d(4, 1, math.pi/2),
                ],
                reversed=False,
            ),
            Path(
                [
                    Pose2d(4, 1, math.pi/2),
                    Translation2d(3, 0),
                    Pose2d(0, 0, 0),
                ],
                reversed=True
            ),
            Path(
                [
                    Pose2d(0, 0, 0),
                    Translation2d(3, 0),
                    Pose2d(4, 1, math.pi/2),
                ],
                reversed=False,
            ),
            Path(
                [
                    Pose2d(4, 1, math.pi/2),
                    Translation2d(3, 0),
                    Pose2d(0, 0, 0),
                ],
                reversed=True
            ),
        ]
        super().setup()


PATHWEAVER_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "pathweaver_paths", "output"
)


class BarrelRacing(AutoNavBase):
    MODE_NAME = "Barrel Racing"

    def setup(self):
        self.paths = [
            LoadPath(
                os.path.join(PATHWEAVER_PATH, "Barrel_Racing.wpilib.json"),
                reversed=False,
            )
        ]
        super().setup()

class Slalom(AutoNavBase):
    MODE_NAME = "Slalom"

    def setup(self):
        self.paths = [
            LoadPath(
                os.path.join(PATHWEAVER_PATH, "Slalom.wpilib.json"),
                reversed=False,
            )
        ]
        super().setup()
