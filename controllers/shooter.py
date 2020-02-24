import math

from magicbot import feedback, StateMachine, state, will_reset_to
import wpilib.geometry

from components.chassis import Chassis
from components.indexer import Indexer
from components.shooter import Shooter
from components.range_finder import RangeFinder
from components.turret import Turret
from components.target_estimator import TargetEstimator
from components.led_screen import LEDScreen


class ShooterController(StateMachine):
    """Statemachine for high level control of the shooter and injector"""

    chassis: Chassis
    indexer: Indexer
    shooter: Shooter
    turret: Turret
    target_estimator: TargetEstimator
    led_screen: LEDScreen
    range_finder: RangeFinder

    fire_command = will_reset_to(False)

    TARGET_RADIUS = (3 * 12 + 3.25) / 2 * 0.0254  # Circumscribing radius of target
    BALL_RADIUS = 7 / 2 * 0.0254
    # convert from freedom units
    CENTRE_ACCURACY = (
        0.1  # maximum distance the centre of the ball will be from our target point
    )
    TOTAL_RADIUS = BALL_RADIUS + CENTRE_ACCURACY
    OFFSET = TOTAL_RADIUS / math.sin(math.pi / 3)
    TRUE_TARGET_RADIUS = TARGET_RADIUS - OFFSET

    TARGET_POSITION = wpilib.geometry.Pose2d(
        0, -2.404, wpilib.geometry.Rotation2d(math.pi)
    )
    # in field co ordinates

    def __init__(self) -> None:
        super().__init__()
        self.spin_command = False
        self.distance = None

    def execute(self) -> None:
        super().execute()
        self.update_LED()

    def update_LED(self) -> None:
        if self.shooter.is_ready():
            self.led_screen.set_bottom_row(0, 255, 0)
        else:
            self.led_screen.set_bottom_row(255, 0, 0)

        if self.indexer.is_ready():
            self.led_screen.set_middle_row(0, 255, 0)
        else:
            self.led_screen.set_middle_row(255, 0, 0)

        if self.target_estimator.is_ready():
            if self.turret.is_ready():
                self.led_screen.set_top_row(0, 255, 0)
            else:
                self.led_screen.set_top_row(255, 128, 0)
        else:
            self.led_screen.set_top_row(255, 0, 0)

    @state(first=True)
    def searching(self) -> None:
        """
        The vision system does not have a target, we try to find one using odometry
        """
        if self.target_estimator.is_ready():
            # means no data is available
            # print(f"searching -> tracking {self.vision.get_vision_data()}")
            self.next_state("tracking")
        else:
            # TODO test this
            # pose: wpilib.geometry.Pose2d = self.chassis.get_pose()
            # rel: wpilib.geometry.Pose2d = self.TARGET_POSITION.relativeTo(pose)
            # rel_heading = rel.rotation().radians()
            # self.turret.scan(rel_heading)

            self.turret.scan(0)  # TODO remove this

    @state
    def tracking(self) -> None:
        """
        Aiming towards a vision target and spining up flywheels
        """
        # collect data only once per loop
        if not self.target_estimator.is_ready():
            self.next_state("searching")
            # print(f"tracking -> searching {self.vision.get_vision_data()}")
        else:
            target_data = self.target_estimator.get_data()
            if abs(target_data.angle) > self.find_allowable_angle(target_data.distance):
                # print(f"Telling turret to slew by {delta_angle}")
                self.turret.slew(target_data.angle)
            if self.turret.is_ready():
                self.shooter.set_range(target_data.distance)
            if self.ready_to_fire() and self.fire_command:
                self.next_state("firing")

    @state(must_finish=True)
    def firing(self, initial_call) -> None:
        """
        Positioned to fire, inject and expel a single ball
        """
        if initial_call:
            self.shooter.fire()
        elif not self.shooter.is_firing():
            self.next_state("tracking")

    def fire_input(self) -> None:
        """
        Called by robot.py to indicate the fire button has been pressed
        """
        self.fire_command = True

    @feedback
    def ready_to_fire(self) -> bool:
        return (
            self.shooter.is_ready()
            and self.turret.is_ready()
            and self.indexer.is_ready()
        )

    def find_allowable_angle(self, dist: float) -> float:
        """
        Find the maximum angle by which the turret can be misaligned to still score a hit
        Currently does not consider angle from target
        dist: planar distance from the target
        """
        dist = min(dist, 14.0)
        angle = math.atan(self.TRUE_TARGET_RADIUS / dist)
        # print(f"angle tolerance +- {angle} true target radius{self.TRUE_TARGET_RADIUS}")
        return angle
