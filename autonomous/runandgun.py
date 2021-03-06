import math
from typing import List

from wpilib.geometry import Pose2d, Translation2d
from wpilib import trajectory
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
        self.has_transitioned = False
        self.path_num = 0
        self.paths: List[Path]

    def setup(self) -> None:
        self.path_follow: PathFollow = PathFollow(self.chassis)
        self.populate_transition_speeds()
        self.indexer.auto_retract = True

    def on_enable(self) -> None:
        self.chassis.reset_odometry(self.paths[0].start)
        self.path_num = -1  # we initialise right into the first increment
        super().on_enable()

    @state
    def move(self, initial_call) -> None:
        """
        Follow the trajectory defined by our waypoints
        """
        if initial_call:
            path = self.paths[self.path_num]
            self.path_follow.set_transition_speed(path.transition_velocity)
            self.path_follow.set_path(path)
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
            self.shooter_controller.stop()
            self.next_state("move")

    @state
    def run_and_gun(self, initial_call) -> None:
        """
        Follow the trajectory defined by our waypoints shooting until we run out of balls
        """
        if initial_call:
            path = self.paths[self.path_num]
            if self.indexer.balls_loaded() <= 0:
                self.path_follow.set_transition_speed(path.transition_velocity)
                self.has_transitioned = True
            else:
                self.has_transitioned = False
            self.path_follow.set_path(path)
        self.path_follow.run()
        self.shooter_controller.engage()
        self.shooter_controller.fire_input()
        if self.indexer.balls_loaded() <= 0:
            if self.path_follow.path_done():
                self.shooter_controller.stop()
                self.next_state("done_move")
            else:
                if not self.has_transitioned:
                    # This should return 0 for the final path
                    self.has_transitioned = True
                    path = self.paths[self.path_num]
                    print(f"Transitioning at speed {path.transition_velocity}")
                    self.path_follow.set_transition_speed(path.transition_velocity)
                    self.path_follow.new_path(path)

    @state(first=True)
    def done_move(self):
        self.path_num += 1
        if self.path_num >= len(self.paths):
            self.chassis.drive(0, 0)
            self.done()
        else:
            path = self.paths[self.path_num]

            if path.intake:
                self.indexer.lower_intake()
                self.indexer.enable_intaking()
            else:
                # The intake is also raised if we reach 5 balls in the indexer
                self.indexer.raise_intake()
                self.indexer.disable_intaking()

            if path.shoot:
                if path.stop_to_fire:
                    self.next_state("shoot")
                else:
                    self.next_state("run_and_gun")
            else:
                self.next_state("move")

    def generate_unified_trajectory(self):
        """
        Generate a trajectory as if all of the paths were a single one. Intended to be used
        for calculating transition speeds
        """
        start = None
        end = None
        waypoints = []
        for i, path in enumerate(self.paths):
            if i <= len(self.paths) - 1:
                end = path.end
            if i == 0:
                start = path.start
                continue
            else:
                waypoints.append(path.start.translation())
                waypoints += path.waypoints

        return self.path_follow.gen.generateTrajectory(
            start, waypoints, end, self.path_follow.trajectory_config
        )

    def cumulative_time_to_path(self, target_path: Path) -> float:
        t = 0.0
        for path in self.paths:
            t += self.path_follow.get_total_path_time(path)
            if path is target_path:
                break
        else:
            print("The path you wanted a time until is not in this routine.")
        return t

    def get_transition_speed(
        self, first_path: Path, unified_trajectory: trajectory.Trajectory
    ) -> float:
        """
        Get the speed at which the robot would have moved between the supplied path and the
        next one if the entire routine was a single trajectory
        """
        sample_time = self.cumulative_time_to_path(first_path)
        state = unified_trajectory.sample(sample_time)
        return state.velocity

    def populate_transition_speeds(self) -> None:
        """
        Fill in the transition velocities for all of our paths
        """
        unified_trajectory = self.generate_unified_trajectory()
        for i, path in enumerate(self.paths):
            speed = self.get_transition_speed(path, unified_trajectory)
            self.paths[i].transition_velocity = speed


class Showoff(RunAndGun):
    MODE_NAME = "Showoff"

    def setup(self):
        self.paths = [
            Path(
                [
                    Pose2d(8.382, 3.528, -math.pi / 2),
                    Translation2d(8.212, 2.491),
                    Translation2d(7.278, 2.375),
                    Translation2d(6.858, 3.048),
                    Translation2d(6.439, 3.750),
                    Translation2d(5.604, 3.706),
                    Translation2d(5.334, 3.048),
                    Pose2d(5.018, 2.310, math.pi),
                ],
                reversed=False,
                intake=True,
            ),
            Path(
                [
                    Pose2d(5.018, 2.310, math.pi),
                    Translation2d(3.875, 2.359),
                    Translation2d(3.745, 3.583),
                    Translation2d(5.035, 3.636),
                    Translation2d(5.334, 3.048),
                    Translation2d(5.116, 2.237),
                    Pose2d(2.826, 2.296, math.pi),
                ],
                reversed=False,
                shoot=True,
                stop_to_fire=False,
            ),
            Path(
                [
                    Pose2d(2.826, 2.296, math.pi),
                    Translation2d(2.286, 1.524),
                    Translation2d(2.754, 0.761),
                    Translation2d(3.537, 0.846),
                    Translation2d(3.800, 1.524),
                    Translation2d(4.373, 2.267),
                    Translation2d(5.118, 2.091),
                    Translation2d(5.334, 1.524),
                    Translation2d(5.756, 0.888),
                    Translation2d(6.637, 0.935),
                    Translation2d(6.858, 1.524),
                    Translation2d(7.429, 2.273),
                    Translation2d(8.217, 2.025),
                    Pose2d(8.382, 1.524, -math.pi / 2),
                ],
                reversed=False,
                intake=True,
                shoot=True,
                stop_to_fire=False,
            ),
        ]
        super().setup()
