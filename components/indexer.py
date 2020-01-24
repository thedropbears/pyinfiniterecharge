import wpilib


class Indexer:
    indexer_motors: list
    indexer_switches: list

    def on_enable(self) -> None:
        self.indexing = True

    def execute(self) -> None:
        if self.indexing:
            motor_states = [True] * len(self.indexer_motors)
            switch_results = [switch.get() for switch in self.indexer_switches]
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
                    self.indexer_motors[i].set(1)
                else:
                    self.indexer_motors[i].stopMotor()
        else:
            for motor in self.indexer_motors:
                motor.stopMotor()

    def enable_indexing(self) -> None:
        self.indexing = True

    def disable_indexing(self) -> None:
        self.indexing = False

    def balls_loaded(self) -> int:
        return sum(not switch.get() for switch in self.indexer_switches)

    def is_ball_ready(self) -> bool:
        return not self.indexer_switches[0].get()

