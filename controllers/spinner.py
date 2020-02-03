# from magicbot import StateMachine, state
from components.spinner import Spinner


# class SpinnerController(StateMachine):
class SpinnerController:
    spinner: Spinner

    def setup(self):
        self.state = None
        self.spinner.piston_up()
        self.task = "position"
        self.required_colour = "R"

    # @state()
    def position_control(self):
        self.spinner.piston_down()
        self.state = "position"
        self.spinner.state = "position"
        self.required_colour = "R"  # get colour

    def rotation_control(self):
        self.spinner.piston_down()
        self.state = "rotation"
        self.spinner.state = "rotation"

    def execute(self):
        if self.state == "position":  # TODO get FMS info
            if self.spinner.go_to_colour(self.required_colour):
                self.state = None
        elif self.state == "rotation":
            if self.spinner.do_rotation():
                self.state = None
