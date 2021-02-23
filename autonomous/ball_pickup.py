import math

from wpilib import controller
from wpimath import geometry
from wpimath import trajectory
from wpimath.trajectory import constraint
from wpilib.geometry import Pose2d, Translation2d
from magicbot import AutonomousStateMachine, state

from components.chassis import Chassis
from components.indexer import Indexer
from components.shooter import Shooter
from controllers.shooter import ShooterController
from components.vision import Vision

from utilities.path_follow import Path, LoadPath, PathFollow


def to_pose(x: float, y: float, heading: float) -> geometry.Pose2d:
    """
    Convert inputs into a wpilib pose object
    """
    rotation = geometry.Rotation2d(heading)
    return geometry.Pose2d(x, y, rotation)


class BallPickup(AutonomousStateMachine):
    MODE_NAME = "Ball Pickup"

    shooter_controller: ShooterController
    shooter: Shooter

    chassis: Chassis
    indexer: Indexer
    vision: Vision

    TARGET_POSITION = geometry.Translation2d(0, 0)

    path_names = ["None", "A1", "A2", "B1", "B2"]
    # 1 is for red and 2 is for blue
    all_paths = {}
    all_paths["A1"] = Path( # red A
        [Pose2d(2.286, 0.282, 0),
        Translation2d(2.286, 2.286),
        Translation2d(1.524, 3.810),
        Translation2d(3.810, 4.572),
        Pose2d(3.810, 9.144, 0),
        ],
        reversed=False)
    all_paths["A2"] = Path( # blue A
        [Pose2d(2.286, 0.282, 0),
        Translation2d(0.762, 4.572),
        Translation2d(3.048, 5.334),
        Translation2d(2.286, 6.858),
        Pose2d(2.286, 9.144, 0),
        ],
        reversed=False)
    all_paths["B1"] = Path( # red B
        [Pose2d(2.286, 0.282, 0),
        Translation2d(3.048, 2.286),
        Translation2d(1.524, 3.810),
        Translation2d(3.048, 5.334),
        Pose2d(3.048, 9.144, 0),
        ],
        reversed=False)
    all_paths["B2"] = Path( # blue B
        [Pose2d(2.286, 0.282, 0),
        Translation2d(1.524, 4.572),
        Translation2d(3.048, 6.096),
        Translation2d(1.524, 7.620),
        Pose2d(1.524, 9.144, 0),
        ],
        reversed=False)

    expected_balls = 3

    def __init__(self) -> None:
        super().__init__()

        self.path_name = None

    def setup(self):
        self.path_follow: PathFollow = PathFollow(self.chassis)
        self.indexer.set_max_balls(3)
        self.shooter.toggle() # turns fly wheels off

    @state(first=True)
    def findPath(self, initial_call, state_tm):
        # wait for vision to know which path its on
        self.vision_data = self.vision.get_data()
        # self.vision_data.distance is actually the path num not the distance
        print("vison data", self.vision_data)
        if self.vision_data != None:
            if self.vision_data.distance % 1 == 0: # check if its the balls vision which will only return whole numbers
                if self.vision_data.distance != 0: # if it knowns which path its on
                    self.path_name = self.path_names[int(self.vision_data.distance)]
                    self.next_state("move")
                else:
                    print("dosent know which path yet")
            else:
                print("wrong vision data")
        else:
            print("no vision data")

    @state
    def move(self, initial_call, state_tm):
        if initial_call:
            path = self.all_paths[self.path_name]
            self.path_follow.new_path(path)
        self.path_follow.run()
        if self.path_follow.path_done():
            self.done()

    def has_collected_balls(self) -> bool:
        """
        Has the robot collected all the balls for the current trajectory?
        """
        ball_target = self.expected_balls
        if ball_target == 0:
            return False
        if self.indexer.balls_loaded() >= ball_target:
            return True
        else:
            return False