from magicbot import StateMachine, state

from components.indexer import Indexer
from components.shooter import Shooter
from components.turret import Turret
from components.vision import Vision


class ShooterController(StateMachine):
    """Statemachine for high level control of the shooter and injector"""

    indexer: Indexer
    shooter: Shooter
    turret: Turret
    vision: Vision

    def __init__(self):
        super().__init__()

    @state(first=True)
    def searching(self):
        """
        The vision system does not have a target, we try to find one using odometry
        """
        # currently just waits for vision
        if self.vision.get_vision_data()[0] != -1:  # -1 means no data is available
            self.next_state("tracking")

    @state
    def tracking(self):
        """
        Aiming towards a vision target and spining up flywheels
        """
        (
            dist,
            delta_angle,
            timestamp,
        ) = self.vision.get_vision_data()  # collect data only once per loop
        if dist != -1:
            self.next_state("searching")
        else:
            self.shooter.set_range(dist)
            self.turret.slew(delta_angle)
            if (
                self.shooter.is_ready()
                and self.shooter.is_in_range()
                and self.indexer.is_ball_ready()
                and self.input_command
                and self.turret.is_aimed()
            ):
                self.next_state("firing")

    @state
    def firing(self):
        """
        Positioned to fire, inject and expel a single ball
        """
        self.shooter.fire()
        self.next_state("tracking")

    def driver_input(self, command: bool):
        """
        Called by robot.py to indicate the fire button has been pressed
        """
        self.input_command = command
