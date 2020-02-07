from magicbot import feedback
import ctre
import wpilib


class Indexer:
    indexer_motors: list
    indexer_switches: list
    piston_switch: wpilib.DigitalInput
    injector_switch: wpilib.DigitalInput
    injector_master_motor: ctre.WPI_TalonSRX
    injector_slave_motor: ctre.WPI_TalonSRX

    def setup(self):
        for motor in self.indexer_motors:
            motor.setInverted(True)

        self.indexer_speed = 0.2
        self.NUMBER_OF_MOTORS = len(self.indexer_motors)
        self.injector_speed = 0.4

        self.injector_slave_motor.follow(self.injector_master_motor)
        self.injector_slave_motor.setInverted(ctre.InvertType.OpposeMaster)

    def on_enable(self) -> None:
        self.indexing = True

    def execute(self) -> None:
        if self.indexing:
            # handle the injector motors
            if self.injector_switch.get() and not self.piston_switch.get():
                self.injector_master_motor.set(self.injector_speed)
            else:
                self.injector_master_motor.stopMotor()

            # handle indexer motors
            for motor, switch in zip(self.indexer_motors, self.indexer_switches):
                if switch.get():
                    motor.set(self.indexer_speed)
                else:
                    motor.stopMotor()
        else:
            self.injector_master_motor.stopMotor()
            for motor in self.indexer_motors:
                motor.stopMotor()

    def enable_indexing(self) -> None:
        self.indexing = True

    def disable_indexing(self) -> None:
        self.indexing = False

    @feedback
    def balls_loaded(self) -> int:
        balls = sum(not switch.get() for switch in self.indexer_switches)
        if not self.injector_switch.get():
            balls += 1
        return balls

    @feedback
    def is_ready(self) -> bool:
        return not self.injector_switch.get()
