import wpilib
import rev
import rev.color


class Colour:
    def __init__(self, red: float, green: float, blue: float):
        self.red = red
        self.green = green
        self.blue = blue

    def dist(self, c) -> tuple:
        return (
            abs(self.red - c.red) + abs(self.green - c.green) + abs(self.blue - c.blue)
        )


class Spinner:
    spinner_motor: wpilib.Spark
    spinner_solenoid: wpilib.Solenoid
    colour_sensor: rev.color.ColorSensorV3

    MAX_COLOUR_DIST = 0.2  # Chosen without experimentation, should be tuned
    COLOUR_OFFSET = 1  # The offset of the colour sensor, in 45 degree increments

    WHEEL_COLOURS = {  # Colours read in real world
        "R": Colour(82 / 255, 116 / 255, 55 / 255),
        "B": Colour(55 / 255, 120 / 255, 80 / 255),
        "G": Colour(61 / 255, 126 / 255, 66 / 255),
        "Y": Colour(72 / 255, 131 / 255, 50 / 255),
    }

    WHEEL_ORDER = ["R", "Y", "B", "G"]

    POSITION_SPIN_FACTOR = 0.3

    ROTATION_SPIN_FACTOR = 20
    ROTATION_MAX_SPEED = 1
    ROTATION_TARGET_ROTATIONS = 3.5

    def __init__(self):
        pass

    def setup(self):
        self.spinner_motor.set(0)
        self.piston_up()
        self.rotations = 0
        self.state = None
        self.piston_state = "down"

    def on_enable(self) -> None:
        self.spinner_motor.set(0)
        self.piston_up()

    def piston_up(self) -> None:
        self.spinner_solenoid.set(False)
        self.piston_state = "up"

    def piston_down(self) -> None:
        self.spinner_solenoid.set(True)
        self.piston_state = "down"

    def get_current_colour(self) -> str:
        self.sensed_colour = self.colour_sensor.getColor()

        distances = {}
        for col_name, col_value in self.WHEEL_COLOURS.items():
            distances[col_name] = col_value.dist(self.sensed_colour)

        if (
            distances[min(distances, key=lambda col: distances[col])]
            < self.MAX_COLOUR_DIST
        ):
            return min(distances, key=lambda col: distances[col])

    def get_wheel_dist(self) -> int:
        current_colour = (
            self.get_current_colour()
        )  # only called when running position control
        if current_colour == None:
            return 0
        distance = (
            self.WHEEL_ORDER.index(self.required_colour)
            + self.COLOUR_OFFSET
            - self.WHEEL_ORDER.index(current_colour)
        )
        if distance > 2:
            return distance - 4
        return distance

    def go_to_colour(self, colour: str):
        self.state = "position"
        distance = self.get_wheel_dist()
        self.spinner_motor.set(distance * self.POSITION_SPIN_FACTOR)
        if distance == 0 and self.pistonState == "down":
            self.state = None
            self.piston_up()

    def do_rotation(self) -> None:
        self.state = "rotation"
        spin_speed = (
            self.ROTATION_SPIN_FACTOR ** (x - self.ROTATION_TARGET_ROTATIONS + 1)
            + self.ROTATION_MAX_SPEED
        )
        self.spinner_motor.set(spin_speed)
        # speed slows down as it gets closer to required revolutions
        current_colour = self.get_current_colour()
        if self.lastCol != current_colour:
            self.rotations += 1 / 8
        self.lastCol = current_colour
        if self.rotations >= 3.5 and self.pistonState == "down":
            self.state = None
            self.spinner_motor.set(0)
            self.piston_up()

    def is_complete(self) -> None:
        return self.state == None

    def execute(self):
        pass
