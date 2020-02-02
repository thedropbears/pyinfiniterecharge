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
        self.required_colour = "R"  # get colour

    def rotation_control(self):
        self.spinner.piston_down()
        self.state = "rotation"

    def execute(self):
        if self.state == "position":  # TODO get FMS info
            self.spinner.go_to_colour(self.required_colour)
        elif self.state == "rotation":
            self.spinner.do_rotation()

        if self.spinner.is_complete():
            self.state == None