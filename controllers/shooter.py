# from magicbot import StateMachine, state
from components.indexer import Indexer


# class ShooterController(StateMachine):
class ShooterController:
    """Class to control the entire shooter conglomerate"""

    indexer: Indexer

    def __init__(self) -> None:
        self.state = "intake_cells"

    def execute(self) -> None:
        """Executed every cycle"""
        if self.state == "intake_cells":
            self.intake_cells()
        elif self.state == "eject_cells":
            self.eject_cells()
        elif self.state == "shoot_cells":
            self.shoot_cells()

    # @state()
    def eject_cells(self) -> None:
        """Powers all motors in reverse to eject all cells out front"""
        for motor in self.indexer.indexer_motors:
            motor.set(-1)

    # @state()
    def shoot_cells(self) -> None:
        """Powers all motors to eject all cells into the shooter"""
        for motor in self.indexer.indexer_motors:
            motor.set(1)

    # @state(first=True)
    def intake_cells(self) -> None:
        """Powers motors based on the limit switches to intake and store balls at the back of the indexer"""
        motor_states = [True] * len(self.indexer.indexer_motors)
        switch_results = [switch.get() for switch in self.indexer.indexer_switches]
        if not switch_results[
            0
        ]:  # Test the back switch (because all others require 2 switches)
            motor_states[0] = False

        for i in range(
            1, len(motor_states)
        ):  # Disable motor if switch and previous switch are pressed
            if not switch_results[i] and not switch_results[i - 1]:
                motor_states[i] = False

        for i in range(len(motor_states)):  # Set all motors to required states
            if motor_states[i]:
                self.indexer.indexer_motors[i].set(1)
            else:
                self.indexer.indexer_motors[i].stopMotor()
