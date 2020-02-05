from magicbot import feedback
import ctre
import wpilib


class Indexer:
    indexer_motors: list
    indexer_switches: list
    piston_switch: wpilib.DigitalInput
    injector_switch: wpilib.DigitalInput
    injector_master_motor: ctre.TalonSRX
    injector_slave_motor: ctre.TalonSRX

    def setup(self):
        for motor in self.indexer_motors:
            motor.setInverted(True)

        self.speed = 0.2
        self.NUMBER_OF_MOTORS = len(self.indexer_motors)
        self.injector_speed = self.speed * 2

        self.injector_slave_motor.follow(self.injector_master_motor)
        self.injector_slave_motor.setInverted(ctre.InvertType.OpposeMaster)

    def on_enable(self) -> None:
        self.indexing = True

    def execute(self) -> None:
        # handle the injector motors
        if not self.injector_switch.get():
            if not self.piston_switch.get():
                self.injector_master_motor.set(self.injector_speed)
            else:
                self.injector_master_motor.stopMotor()

        # handle indexer motors
        for i in range(0, self.NUMBER_OF_MOTORS):
            if self.indexer_switches[i].get():
                self.indexer_motors[i].set(self.speed)

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
