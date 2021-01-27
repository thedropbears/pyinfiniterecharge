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
    shooter: Shooter

    # has_zeroed: bool

    TARGET_POSITION = geometry.Translation2d(0, 0)

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

    def setup(self):
        self.trajectory_config.setKinematics(self.chassis.kinematics)
        self.trajectory_config.addConstraint(
            constraint.DifferentialDriveVoltageConstraint(
                self.chassis.ff_calculator, self.chassis.kinematics, 10
            )
        )
        self.end_ranges = []
        self.paths = []
        for i in range(len(self.start_poses)):
            self.trajectory_config.setReversed(self.reversed[i])
            path = self.gen.generateTrajectory(
                self.start_poses[i],
                self.waypoints[i],
                self.end_poses[i],
                self.trajectory_config,
            )
            self.paths.append(path)
            self.end_ranges.append(
                (self.TARGET_POSITION - self.end_poses[i].translation()).norm()
            )
        self.path = self.paths[0]

    def on_enable(self) -> None:
        self.chassis.reset_odometry(self.start_poses[0])
        self.indexer.lower_intake()
        self.indexer.enable_intaking()
        self.trajectory_num = 0
        # self.has_zeroed = True
        super().on_enable()

    @state(first=True)
    def shoot(self, initial_call, state_tm) -> None:
        """
        Shoot all balls that we currently have
        """
        if initial_call:
            if self.trajectory_num == 0:
                self.shooter.set_range(3)
            self.shooter_controller.fired_count = 0
        self.shooter_controller.engage()
        self.shooter_controller.fire_input()
        if self.has_fired_balls():
            if self.trajectory_num >= len(self.paths):
                self.done()
            else:
                self.next_state("move")

    @state
    def move(self, initial_call, state_tm) -> None:
        """
        Follow the trajectory defined by our waypoints
        """
        if initial_call:
            self.path = self.paths[self.trajectory_num]
            self.shooter.set_range(self.end_ranges[self.trajectory_num])
        if (
            state_tm
            > self.path.totalTime()
            # or self.has_collected_balls()
        ):
            # this needs to be overidden in the subclasses
            print(f"Calculated path time: {self.path.totalTime()}")
            self.chassis.drive(0, 0)
            self.next_state("shoot")
            self.trajectory_num += 1
            return
        pos = self.chassis.get_pose()
        goal = self.path.sample(state_tm)
        speeds = self.controller.calculate(pos, goal)
        self.chassis.drive(speeds.vx, speeds.omega)

    def has_collected_balls(self) -> bool:
        """
        Has the robot collected all the balls for the current trajectory?
        """
        ball_target = self.expected_balls[self.trajectory_num]
        if ball_target == 0:
            return False
        if self.indexer.balls_loaded() >= ball_target:
            return True
        else:
            return False

    def has_fired_balls(self) -> bool:
        balls_to_fire = self.expected_balls[self.trajectory_num - 1]
        if self.trajectory_num == 0:
            balls_to_fire = 3
        # print(f"Expecting to fire {balls_to_fire} balls")
        return self.shooter_controller.fired_count >= balls_to_fire


class test(ShootMoveShootBase):
    MODE_NAME = "Test"
    DEFAULT = True

    def setup(self):
        self.start_poses = [to_pose(0, 0, math.pi)]
        self.end_poses = [to_pose(-2, 0, math.pi)]
        self.waypoints = [[geometry.Translation2d(-1, math.pi)]]
        self.trajectory_config = trajectory.TrajectoryConfig(
            maxVelocity=1, maxAcceleration=1
        )
        self.expected_balls = [0]
        self.reversed = [False]
        self.trajectory_max = 1
        super().setup()


class _3Right3(ShootMoveShootBase):
    MODE_NAME = "3RIGHT3"

    def setup(self):
        self.start_poses = [to_pose(-3.459, -1.7, math.pi)]
        self.end_poses = [to_pose(-8.163, -1.7, math.pi)]
        self.waypoints = [
            [geometry.Translation2d(-7.077, -1.7), geometry.Translation2d(-7.992, -1.7)]
        ]
        self.expected_balls = [3]
        self.reversed = [False]
        self.trajectory_config = trajectory.TrajectoryConfig(
            maxVelocity=2.5, maxAcceleration=1
        )
        self.trajectory_max = 1
        super().setup()


class _3Right23(ShootMoveShootBase):
    MODE_NAME = "3RIGHT23"

    def setup(self):
        # TODO add correct headings
        # empty waypoints are used for trajectories with only two points
        self.start_poses = [
            to_pose(-3.459, -1.7, math.pi),
            to_pose(-6.165, 0.582, 0),
            to_pose(-4.698, 2.359, 0),
        ]
        self.end_poses = [
            to_pose(-6.179, 2.981, 0),
            to_pose(-4.698, 2.359, 0),
            to_pose(-8.163, 0.705, 0),
        ]
        self.waypoints = [
            [
                geometry.Translation2d(-4.971, 1.154),
                geometry.Translation2d(-5.616, 1.978),
            ],
            [],
            [geometry.Translation2d(-5.668, 0.988)],
        ]
        self.trajectory_config = trajectory.TrajectoryConfig(
            maxVelocity=1.5, maxAcceleration=1
        )
        self.expected_balls = [5, 0, 3]
        self.reversed = [False, True, False]
        self.trajectory_max = 3
        super().setup()


class _3Right32(ShootMoveShootBase):
    MODE_NAME = "3RIGHT32"

    def setup(self):
        # empty waypoints are used for trajectories with only two points
        self.start_poses = [
            to_pose(-3.459, -1.7, math.pi),
            to_pose(-8.163, -1.7, math.pi),
            to_pose(-4.168, -1.7, math.pi),
        ]
        self.end_poses = [
            to_pose(-8.163, -1.7, math.pi),
            to_pose(-4.168, -1.7, math.pi),
            to_pose(-6.062 - 1, 0.248, -1.178),
        ]
        self.waypoints = [
            [
                geometry.Translation2d(-7.077, -1.7),
                geometry.Translation2d(-7.992, -1.7),
            ],
            [],
            [],
        ]
        self.expected_balls = [3, 0, 2]
        self.reversed = [False, True, False]
        self.trajectory_config = trajectory.TrajectoryConfig(
            maxVelocity=2.5, maxAcceleration=1
        )
        self.trajectory_max = 3
        super().setup()
