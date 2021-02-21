import math

from wpilib import controller
from wpimath import geometry
from wpimath import trajectory
from wpimath.trajectory import constraint
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


class BallPickupBase(AutonomousStateMachine):

    shooter_controller: ShooterController
    shooter: Shooter

    chassis: Chassis
    indexer: Indexer
    vision: Vision

    TARGET_POSITION = geometry.Translation2d(0, 0)

    path_names = ["None", "A1", "A2", "B1", "B2"]
    all_paths = {"A1" : Path([], reversed=False),
    "A2" : Path([], reversed=False),
    "B1" : Path([], reversed=False),
    "B2" : Path([], reversed=False),
    }

    expected_balls = 3

    def __init__(self) -> None:
        super().__init__()

        self.path_name = None

        self.shooter.toggle() # turns fly wheels off

    def setup(self):
        self.path_follow: PathFollow = PathFollow(self.chassis)

    @state(first=True)
    def findPath(self):
    	# wait for vision to know which path its on
    	self.vision_data = self.vision.get_data()
    	if self.vision_data[0] % 1 == 0: # check if its the balls vision which will only return whole numbers
    		if self.vision_data[0] != 0: # if it knowns which path its on
    			self.path_name = self.path_names[self.vision_data[0]]
    			self.next_state("move")
    	else:
    		print("wrong vision data")

    @state
    def move(self):
    	if initial_call:
            path = self.all_paths[self.path_name]
            self.path_follow.new_path(path)
        self.path_follow.run()
        if self.path_follow.path_done():
            self.done()

        if self.has_collected_balls():
        	

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