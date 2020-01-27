import wpilib
import rev
import rev.color


class Colour:  # class for storing rgb colours
    def __init__(self, red: float, green: float, blue: float):
        self.red = red
        self.green = green
        self.blue = blue

    def dist(self, c: Colour) -> int:
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

    WHEEL_SPIN_FACTOR = 0.3

    MAX_SESNOR_RANGE = 40

    def __init__(self) -> None:
        self.rotations = 0
        self.state = None
        self.lastCol = None
        self.required_colour = None
        self.pistonState = "up"

    def on_enable(self) -> None:
        self.spinner_motor.stopMotor()
        self.piston_up()
        self.rotation = 0
        self.state = None
        self.lastCol = None
        self.required_colour = None

    # Methods to be called by controller
    def is_complete(self) -> bool:
        return self.state == None

    def do_rotation_control(self) -> None:  # to start the rotation routine
        self.state = "rotation"
        self.rotations = 0
        self.lastCol = None
        self.piston_down()

    def go_to_colour(self, colour: str) -> None:  # to start the position routine
        self.state = "position"
        self.required_colour = colour
        self.piston_down()

    def piston_up(self) -> None:  # as functions to make swapping easier
        self.spinner_solenoid.set(True)
        self.pistonState = "up"

    def piston_down(self) -> None:
        self.spinner_solenoid.set(False)
        self.pistonState = "down"

    def read_colour(self) -> tuple:
        col, ir = self.colour_sensor.getColor()
        if ir > MAX_SENSOR_RANGE: #currently redudndant check to see if in range
            return col
        else:
            return col

    def get_wheel_dist(
        self,
    ) -> int:  # finds the colour wheels distance from required colour in segments
        current_colour = self.get_current_colour()
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

    def get_current_colour(
        self,
    ) -> float:  # gets colour as a letter ("R", "Y", "B", "G")
        sensed_colour, _ = self.read_colour()

        distances = {}
        for col_name, col_value in self.WHEEL_COLOURS.items():
            distances[col_name] = col_value.dist(sensed_colour)

        if (
            distances[min(distances, key=lambda col: distances[col])]
            < self.MAX_COLOUR_DIST
        ):
            return min(distances, key=lambda col: distances[col])

    def run_rotation(self) -> None:  # rotation control to be called every 20ms
        self.spinner_motor.set(
            1 - self.rotations / 6
        )  # speed slows down as it gets closer to required revolutions
        current_colour = self.get_current_colour()
        if self.lastCol != current_colour:
            self.rotations += 1 / 8
        self.lastCol = current_colour
        if self.rotations >= 3.5 and self.pistonState == "down":
            self.state = None
            self.spinner_motor.set(0)
            self.piston_up()

    def run_position(self) -> None:  # position control to be called every 20ms
        distance = self.get_wheel_dist()
        self.spinner_motor.set(distance * self.WHEEL_SPIN_FACTOR)
        if distance == 0 and self.pistonState == "down":
            self.state = None
            self.piston_up()

    def execute(self) -> None:
        if self.state == "rotation":
            self.run_rotation()
        if self.state == "position":
            self.run_position()
        wpilib.SmartDashboard.putString("Read Colour", str(self.lastCol))
        wpilib.SmartDashboard.putString(
            "Colour wheel state", str(self.state)
        )  # convert to string beacuse it is None sometimes
