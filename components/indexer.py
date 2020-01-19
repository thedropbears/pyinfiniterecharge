import wpilib


class Indexer:
    indexer_motors: list
    indexer_switches: list

    def __init__(self):
        pass

    def setup(self):
        pass

    def on_enable(self):
        for motor in self.indexer_motors:
            motor.stopMotor()

    def execute(self):
        pass
