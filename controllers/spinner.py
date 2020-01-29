# from magicbot import StateMachine, state
from components.spinner import Spinner


class Colour:
    def __init__(self, red: float, green: float, blue: float):
        self.red = red
        self.green = green
        self.blue = blue

    def dist(self, c) -> tuple:
        return (
            abs(self.red - c.red) + abs(self.green - c.green) + abs(self.blue - c.blue)
        )


# class SpinnerController(StateMachine):
class SpinnerController:
    spinner: Spinner

    MAX_COLOUR_DIST = 0.2  # Chosen without experimentation, should be tuned
    COLOUR_OFFSET = 1  # The offset of the colour sensor, in 45 degree increments

    WHEEL_COLOURS = {
        "R": Colour(82 / 255, 116 / 255, 55 / 255),
        "B": Colour(55 / 255, 120 / 255, 80 / 255),
        "G": Colour(61 / 255, 126 / 255, 66 / 255),
        "Y": Colour(72 / 255, 131 / 255, 50 / 255),
    }

    WHEEL_ORDER = ["R", "Y", "B", "G"]

    WHEEL_SPIN_FACTOR = -0.1

    def run(self, test=False, task=None):
        self.test = test
        self.task = task
        if self.state:
            self.state = None
            self.spinner.piston_up()
            self.spinner.spinner_motor.stopMotor()
        else:
            self.state = "select_task"
            # self.next_state("select_task")

    def setup(self):
        self.state = None
        self.spinner.piston_up()
        self.spinner.spinner_motor.stopMotor()
        self.task = "position"
        self.required_colour = "R"

    # @state(first=True)
    def select_task(self):
        if not self.test:
            self.task = "position"  # TODO Implement FMS Connection
            self.required_colour = "R"

        if self.task == "position":
            self.required_colour = "R"
            self.state = "position"
            # self.next_state("position")
        else:
            self.state = "rotation"
            # self.next_state("rotation")

    # @state()
    def position(self):
        distance = self.get_wheel_dist()
        self.spinner.spinner_motor.set(distance * self.WHEEL_SPIN_FACTOR)

    # @state()
    def rotation(self):
        pass

    def execute(self):
        if self.state == "position":
            self.position()
        elif self.state == "rotation":
            self.rotation()
        elif self.state == "select_task":
            self.select_task()

    def get_current_colour(self):
        sensed_colour, _ = self.spinner.read_colour()

        distances = {}
        for col_name, col_value in self.WHEEL_COLOURS.items():
            distances[col_name] = col_value.dist(sensed_colour)

        if (
            distances[min(distances, key=lambda col: distances[col])]
            < self.MAX_COLOUR_DIST
        ):
            return min(distances, key=lambda col: distances[col])

    def get_wheel_dist(self):
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
