from magicbot import StateMachine, state

from components.indexer import Indexer
from components.shooter import Shooter
from components.turret import Turret
from components.vision import Vision


# class ShooterController(StateMachine):
class ShooterController:
    """Statemachine for high level control of the shooter and injector"""

    indexer: Indexer
    shooter: Shooter
    turret: Turret
    vision: Vision

    def __init__(self) -> None:
        # super().__init__()
        self.state = self.searching

    def execute(self) -> None:
        """
        tempoary replacement of magicbot statemachine
        """
        self.state()

    # @state(first=True)
    def searching(self) -> None:
        """
        The vision system does not have a target, we try to find one using odometry
        """
        # currently just waits for vision
        if self.vision.get_vision_data()[2] != -1:  # -1 means no data is available
            # print(f"searching -> tracking {self.vision.get_vision_data()}")
            # self.next_state("tracking")
            self.state = self.tracking

    # @state
    def tracking(self) -> None:
        """
        Aiming towards a vision target and spining up flywheels
        """
        dist, delta_angle, timestamp = self.vision.get_vision_data()
        # collect data only once per loop
        if timestamp == -1:
            # self.next_state("searching")
            # print(f"tracking -> searching {self.vision.get_vision_data()}")
            self.state = self.searching
        else:
            self.shooter.set_range(dist)
            self.turret.slew(delta_angle)
            if self.ready_to_fire() and self.input_command:
                # self.next_state("firing")
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

    def ready_to_fire(self) -> bool:
        return (
            self.shooter.is_ready()
            and self.shooter.is_in_range()
            and self.indexer.is_ball_ready()
            and self.turret.isReady()
        )
