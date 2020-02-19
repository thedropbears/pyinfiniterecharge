import wpilib
import rev
import rev.color


class Colour:
    def __init__(self, red: float, green: float, blue: float):
        self.red = red
        self.green = green
        self.blue = blue

    def dist(self, c) -> float:
        # finds the "distance" of one colour from another
        # the "distance" is the sum of the differences in rgb value
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

    POSITION_SPIN_FACTOR = 0.3  # position control will run at up to twice this speed

    # rotation control uses the formula: 1 - ROTATION_SPIN_FACTOR ** (rotations - ROTATION_TARGET_ROTATIONS)  for wheel speed
    ROTATION_SPIN_FACTOR = 20  # increase this to recude slow down
    ROTATION_MAX_SPEED = 1
    ROTATION_TARGET_ROTATIONS = 3.5

    def __init__(self):
        pass

    def setup(self):
        self.spinner_motor.set(0)
        self.raise_wheel()
        self.rotations = 0
        self.state = self.idle
        self.piston_state = "down"
        self.required_colour = "B"
        self.lastCol = "Y"
        self.motor_speed = 0

    def on_enable(self) -> None:
        self.spinner_motor.set(0)
        self.motor_speed = 0
        self.raise_wheel()

    def raise_wheel(self) -> None:  # moves wheel up
        self.spinner_solenoid.set(False)
        self.piston_state = "up"

    def lower_wheel(self) -> None:  # moves wheel down
        self.spinner_solenoid.set(True)
        self.piston_state = "down"

    def get_current_colour(self) -> str:  # reads and catagorizes colour
        self.sensed_colour = self.colour_sensor.getColor()

        distances = {}
        for col_name, col_value in self.WHEEL_COLOURS.items():
            distances[col_name] = col_value.dist(self.sensed_colour)

        smallest_dist = min(distances, key=lambda col: distances[col])
        if (
            distances[smallest_dist]
            < self.MAX_COLOUR_DIST
        ):
            return smallest_dist

    def get_wheel_dist(self) -> int:
        # find the distance of the colour wheel from desired state in segments
        # reuturns between -2 and 2 with 0 being the colour desired
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

    def position_run(self):  # do position control
        self.lower_wheel()
        self.required_colour = colour
        distance = self.get_wheel_dist()
        self.motor_speed = distance * self.POSITION_SPIN_FACTOR

        if distance == 0:
            self.state = self.idle

    def rotation_run(self):
        self.lower_wheel()
        self.motor_speed = 1 - (
            self.ROTATION_SPIN_FACTOR
            ** (self.rotations - self.ROTATION_TARGET_ROTATIONS)
            * self.ROTATION_MAX_SPEED
        )
        # speed slows down as it gets closer to required revolution

        current_colour = self.get_current_colour()
        if self.lastCol != current_colour:  # if it has moved along in segments
            self.rotations += 1 / 8
        self.lastCol = current_colour

        if self.rotations >= 3.5:
            self.rotations = 0
            self.state = self.idle

    def go_to_colour(self, colour: str):
        self.state = self.position_run
        self.required_colour = colour

    def do_rotation_control(self):
        self.state = self.position_run

    def is_complete(self) -> bool:
        return self.state == self.idle

    def idle(self):
        self.raise_wheel()
        self.motor_speed = 0

    def execute(self):
        self.state()
        self.spinner_motor.set(self.motor_speed)
