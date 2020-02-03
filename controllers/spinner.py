# from magicbot import StateMachine, state
from components.spinner import Spinner


# class SpinnerController(StateMachine):
class SpinnerController:
    spinner: Spinner

    def setup(self):
        self.state = None
        self.spinner.raise_wheel()
        self.task = None
        self.required_colour = "R"

    # @state()
    def position_control(self):
        self.spinner.lower_wheel()
        self.state = "position"
        self.required_colour = "R"  # get colour from fms

    def rotation_control(self):
        self.spinner.lower_wheel()
        self.state = "rotation"

    def execute(self):
        if self.state == "position":  # TODO get FMS info
            if self.spinner.go_to_colour(self.required_colour):
                self.state = None
        elif self.state == "rotation":
            if self.spinner.do_rotation():
                self.state = None
