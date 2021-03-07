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

    def done(self) -> None:
        super().done()
        self.chassis.disable_brake_mode()


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


class Slalom(AutoNavBase):
    MODE_NAME = "Slalom"

    def setup(self):
        self.paths = [
            Path(
                [
                    Pose2d(1.056, 0.618, 0),
                    Translation2d(2.188, 0.908),
                    Translation2d(2.696, 2.269),
                    Translation2d(4.596, 2.737),
                    Translation2d(5.757, 2.522),
                    Translation2d(6.509, 2.120),
                    Translation2d(7.111, 1.088),
                    Translation2d(8.151, 0.681),
                    Translation2d(8.464, 1.543),
                    Translation2d(7.955, 2.522),
                    Translation2d(7.194, 2.323),
                    Translation2d(6.531, 0.952),
                    Translation2d(4.473, 0.572),
                    Translation2d(2.382, 0.912),
                    Translation2d(2.277, 2.335),
                    Pose2d(0.946, 2.346, math.pi),
                ],
                reversed=False,
            ),
        ]
        super().setup()


class Bounce(AutoNavBase):
    MODE_NAME = "Bounce"

    def setup(self):
        self.paths = [
            Path(
                [
                    Pose2d(1.019, 2.139, 0),
                    Translation2d(1.830, 2.359),
                    Pose2d(2.250, 3.406, math.pi / 2),
                ],
                reversed=False,
            ),
            Path(
                [
                    Pose2d(2.250, 3.406, math.pi / 2),
                    Translation2d(2.686, 2.169),
                    Translation2d(3.164, 1.137),
                    Translation2d(3.795, 0.786),
                    Translation2d(4.443, 1.186),
                    Pose2d(4.557, 3.429, -math.pi / 2),
                ],
                reversed=True,
            ),
            Path(
                [
                    Pose2d(4.557, 3.429, -math.pi / 2),
                    Translation2d(4.668, 1.020),
                    Translation2d(5.873, 0.766),
                    Translation2d(6.776, 1.068),
                    Pose2d(6.856, 3.414, math.pi / 2),
                ],
                reversed=False,
            ),
            Path(
                [
                    Pose2d(6.856, 3.414, math.pi / 2),
                    Translation2d(7.157, 2.328),
                    Pose2d(8.352, 2.168, math.pi),
                ],
                reversed=True,
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
            Path(
                [
                    Pose2d(1.019, 2.139, 0),
                    Translation2d(4.1, 2.35),
                    Translation2d(4.72, 1.5),
                    Translation2d(3.6, 0.7),
                    Translation2d(3.1, 1.9),
                    Translation2d(7, 2.6),
                    Translation2d(6.3, 3.7),
                    Translation2d(5.2, 3.2),
                    Translation2d(6.1, 1.25),
                    Translation2d(7.3, 0.96),
                    Translation2d(8.2, 1.15),
                    Translation2d(8, 2),
                    Translation2d(6, 2.346),
                    Pose2d(0.946, 2.45, math.pi),
                ],
                reversed=False,
            ),
        ]
        super().setup()
