import ctre
import wpilib
from magicbot import tunable, will_reset_to


class Hang:
    winch_motor: ctre.WPI_TalonFX
    kracken_hook_latch: wpilib.Solenoid

    winch_speed = tunable(0.5)

    winch_desired_output = will_reset_to(0.0)

    def __init__(self) -> None:
        self.fire_hook = False

    def setup(self) -> None:
        self.winch_motor.setInverted(True)

        sendable_reg = wpilib.SendableRegistry.getInstance()
        for dev in (self.winch_motor, self.kracken_hook_latch):
            sendable_reg.setSubsystem(dev, "Hang")

    def on_disable(self) -> None:
        # stop motors
        self.winch_motor.stopMotor()

    def execute(self) -> None:

        # solenoid
        if self.fire_hook:
            self.kracken_hook_latch.set(True)

        # drive winch
        self.winch_motor.set(self.winch_desired_output)

    def raise_hook(self) -> None:
        # fire solenoid
        self.fire_hook = True

    def winch(self) -> None:
        # drive motor
        self.winch_desired_output = self.winch_speed

    def pay_out(self) -> None:
        # drive motor backwards to release cable in pits
        self.winch_desired_output = -self.winch_speed
