import ctre
import wpilib
from magicbot import will_reset_to


class Hang:
    winch_motor_master: ctre.WPI_TalonSRX
    winch_motor_slave: ctre.WPI_TalonSRX
    kracken_hook_latch: wpilib.DoubleSolenoid

    WINCH_SPEED = 1.0

    winch_desired_output = will_reset_to(0.0)

    def __init__(self) -> None:
        self.fire_hook = False

    def setup(self) -> None:
        # setup master slave motor config
        self.winch_motor_slave.follow(self.winch_motor_master)

    def on_disable(self) -> None:
        # stop motors
        self.winch_motor_master.stopMotor()

    def execute(self) -> None:

        # solenoid
        if self.fire_hook:
            self.kracken_hook_latch.set(wpilib.DoubleSolenoid.Value.kForward)

        # drive winch
        self.winch_motor_master.set(self.winch_desired_output)

    def raise_hook(self) -> None:
        # fire solenoid
        self.fire_hook = True

    def winch(self) -> None:
        # drive motor
        self.winch_desired_output = self.WINCH_SPEED
