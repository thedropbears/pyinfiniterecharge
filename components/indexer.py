from magicbot import feedback


class Indexer:
    indexer_motors: list
    indexer_switches: list

    def on_enable(self) -> None:
        for motor in self.indexer_motors:
            motor.setInverted(True)
        self.indexing = True

    def execute(self) -> None:
        if self.indexing:
            for motor, switch in zip(
                self.indexer_motors, [switch.get() for switch in self.indexer_switches]
            ):
                if switch:
                    motor.set(0.3)
                else:
                    motor.stopMotor()
        else:
            for motor in self.indexer_motors:
                motor.stopMotor()

    def enable_indexing(self) -> None:
        self.indexing = True

    def disable_indexing(self) -> None:
        self.indexing = False

    @feedback
    def balls_loaded(self) -> int:
        return sum(not switch.get() for switch in self.indexer_switches)

    @feedback
    def is_ready(self) -> bool:
        return not self.indexer_switches[0].get()
