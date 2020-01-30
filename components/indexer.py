class Indexer:
    indexer_motors: list
    indexer_switches: list

    def on_enable(self) -> None:
        self.indexing = True

    def execute(self) -> None:
        if self.indexing:
            motor_states = [True] * len(self.indexer_motors)
            switch_results = [switch.get() for switch in self.indexer_switches]
            # Test the back switch (because all others require 2 switches)
            if not switch_results[0]:
                motor_states[0] = False

            # Disable motor if switch and previous switch are pressed
            for i in range(1, len(motor_states)):
                if not switch_results[i] and not switch_results[i - 1]:
                    motor_states[i] = False

            # Set all motors to required states
            for motor_state, motor in zip(motor_states, self.indexer_motors):
                if motor_state:
                    motor.set(1)
                else:
                    motor.stopMotor()
        else:
            for motor in self.indexer_motors:
                motor.stopMotor()

    def enable_indexing(self) -> None:
        self.indexing = True

    def disable_indexing(self) -> None:
        self.indexing = False

    def balls_loaded(self) -> int:
        return sum(not switch.get() for switch in self.indexer_switches)

    def is_ready(self) -> bool:
        return not self.indexer_switches[0].get()
