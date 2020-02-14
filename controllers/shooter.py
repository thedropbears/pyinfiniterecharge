import math

# from magicbot import StateMachine, state
from magicbot import feedback

from components.chassis import Chassis
from components.indexer import Indexer
from components.shooter import Shooter
from components.turret import Turret
from components.vision import Vision
from components.led_screen import LEDScreen


# class ShooterController(StateMachine):
class ShooterController:
    """Statemachine for high level control of the shooter and injector"""

    chassis: Chassis
    indexer: Indexer
    shooter: Shooter
    turret: Turret
    vision: Vision
    led_screen: LEDScreen

    TARGET_RADIUS = (3 * 12 + 3.25) / 2 * 0.0254  # Circumscribing radius of target
    BALL_RADIUS = 7 / 2 * 0.0254
    # convert from freedom units
    CENTRE_ACCURACY = (
        0.1  # maximum distance the centre of the ball will be from our target point
    )
    TOTAL_RADIUS = BALL_RADIUS + CENTRE_ACCURACY
    OFFSET = TOTAL_RADIUS / math.sin(math.pi / 3)
    TRUE_TARGET_RADIUS = TARGET_RADIUS - OFFSET

    VISON_CONVERSION_FACTOR = 0.5  # a magic number for the vision angle
    # TODO fix vision so this isn't nessecary, requires tuning

    def __init__(self) -> None:
        # super().__init__()
        self.state = self.searching
        self.input_command = False
        self.spin_command = False
        self.distance = None
        self.initial_call = True

    def execute(self) -> None:
        """
        tempoary replacement of magicbot statemachine
        """
        self.state()
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

        if self.vision.is_ready():
            if self.turret.is_ready():
                self.led_screen.set_top_row(0, 255, 0)
            else:
                self.led_screen.set_top_row(255, 128, 0)
        else:
            self.led_screen.set_top_row(255, 0, 0)

    # @state(first=True)
    def searching(self) -> None:
        """
        The vision system does not have a target, we try to find one using odometry
        """
        heading = self.chassis.get_pose().rotation().radians()
        self.turret.scan(heading)

        if self.vision.get_data() is not None:
            # means no data is available
            # self.next_state("tracking")
            self.state = self.tracking

    # @state
    def tracking(self) -> None:
        """
        Aiming towards a vision target and spining up flywheels
        """
        vision_data = self.vision.get_data()
        # collect data only once per loop
        if vision_data is None:
            # self.next_state("searching")
            self.state = self.searching
        else:
            current_turret_angle = self.turret.get_azimuth()
            old_turret_angle = self.turret.azimuth_at_time(vision_data.timestamp)
            if old_turret_angle is None:
                # data has timed out
                self.state = self.searching
                return
            delta_since_vision = current_turret_angle - old_turret_angle
            target_angle = (
                vision_data.angle - delta_since_vision * self.VISON_CONVERSION_FACTOR
            )
            if abs(target_angle) > self.find_allowable_angle(vision_data.distance):
                # print(f"Telling turret to slew by {delta_angle}")
                self.turret.slew(target_angle)
            if self.ready_to_spin():
                # self.next_state("firing")
                self.distance = vision_data.distance
                self.state = self.spining_up

    def spining_up(self) -> None:
        if self.initial_call:
            self.shooter.set_range(self.distance)
            self.initial_call = False
        if self.ready_to_fire() and self.input_command:
            self.distance = None
            self.initial_call = True
            # print(f"spining_up -> firing {self.vision.get_vision_data()}")
            self.state = self.firing

    # @state
    def firing(self) -> None:
        """
        Positioned to fire, inject and expel a single ball
        """
        self.shooter.fire()
        # self.next_state("tracking")
        self.state = self.tracking

    def driver_input(self, command: bool) -> None:
        """
        Called by robot.py to indicate the fire button has been pressed
        """
        self.input_command = command

    def spin_input(self) -> None:
        """
        Called by robot.py to indicate the fire button has been pressed
        """
        self.spin_command = not self.spin_command

    @feedback
    def ready_to_fire(self) -> bool:
        return (
            self.shooter.is_ready()
            and self.indexer.is_ready()
            and self.turret.is_ready()
        )

    @feedback
    def ready_to_spin(self) -> bool:
        return self.indexer.is_ready() and self.turret.is_ready() and self.spin_command

    def find_allowable_angle(self, dist: float) -> float:
        """
        Find the maximum angle by which the turret can be misaligned to still score a hit
        Currently does not consider angle from target
        dist: planar distance from the target
        """
        angle = math.atan(self.TRUE_TARGET_RADIUS / dist)
        # print(f"angle tolerance +- {angle} true target radius{self.TRUE_TARGET_RADIUS}")
        return angle
