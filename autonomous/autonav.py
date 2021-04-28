import math
from typing import List

from wpimath.geometry import Pose2d, Translation2d
from magicbot import AutonomousStateMachine, state

from components.chassis import Chassis
from utilities.path_follow import Path, PathFollow


class AutoNavBase(AutonomousStateMachine):

    chassis: Chassis

    def __init__(self) -> None:
        super().__init__()
        self.path_num = 0
        self.paths: List[Path]

    def setup(self) -> None:
        self.path_follow: PathFollow = PathFollow(self.chassis)

    def on_enable(self) -> None:
        self.chassis.reset_odometry(self.paths[0].start)
        self.path_num = 0
        super().on_enable()

    def done(self) -> None:
        super().done()
        self.chassis.disable_brake_mode()


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


class Slalom(AutoNavBase):
    MODE_NAME = "Slalom"

    def setup(self):
        self.paths = [
            Path(
                [
                    Pose2d(1.056, 0.618, 0),
                    Translation2d(2.188, 0.908),
                    Translation2d(2.678, 2.127),
                    Translation2d(3.560, 2.579),
                    Translation2d(5.600, 2.557),
                    Translation2d(6.700, 2.120),
                    Translation2d(7.150, 0.872),
                    Translation2d(8.056, 0.834),
                    Translation2d(8.464, 1.543),
                    Translation2d(8.012, 2.304),
                    Translation2d(7.194, 2.323),
                    Translation2d(6.404, 0.834),
                    Translation2d(4.537, 0.711),
                    Translation2d(2.755, 0.747),
                    Translation2d(2.087, 1.769),
                    Pose2d(1.263, 2.479, math.radians(139.25)),
                ],
                reversed=False,
            ),
        ]
        super().setup()


class Bounce(AutoNavBase):
    MODE_NAME = "Bounce"

    def setup(self):
        path_offset = 0.5
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
                    Pose2d(4.557, 3.529 + path_offset/2, -math.pi / 2),
                ],
                reversed=True,
            ),
            Path(
                [
                    Pose2d(4.557, 3.529 + path_offset/3, -math.pi / 2),
                    Translation2d(4.668, 1.020 + path_offset),
                    Translation2d(5.673, 0.666 + path_offset),
                    Translation2d(6.800, 0.768 + path_offset),
                    Pose2d(6.856, 3.614 + path_offset, math.pi / 2),
                ],
                reversed=False,
            ),
            Path(
                [
                    Pose2d(6.856, 3.614 + path_offset, math.pi / 2),
                    Translation2d(7.157, 2.328 + 1.5 * path_offset),
                    Pose2d(8.352, 2.168 + 1.5 * path_offset, math.pi),
                ],
                reversed=True,
            ),
        ]
        super().setup()
