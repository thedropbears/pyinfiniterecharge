import wpilib
import rev
import rev.color


class Spinner:
    spinner_motor: wpilib.Spark
    spinner_solenoid: wpilib.DoubleSolenoid
    colour_sensor: rev.color.ColorSensorV3

    def __init__(self):
        pass

    def setup(self):
        pass

    def on_enable(self):
        self.spinner_motor.stopMotor()
        self.spinner_solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

    def execute(self):
        pass

    def read_colour(self):
        return (self.colour_sensor.getColor(), self.colour_sensor.getIR())

    def piston_up(self):
        self.spinner_solenoid.set(wpilib.DoubleSolenoid.Value.kForward)

    def piston_down(self):
        self.spinner_solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
