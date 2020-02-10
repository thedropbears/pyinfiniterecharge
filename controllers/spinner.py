# from magicbot import StateMachine, state
from components.spinner import Spinner


# class SpinnerController(StateMachine):
class SpinnerController:
    spinner: Spinner

    def setup(self):
        self.spinner.raise_wheel()
        self.task = None
        self.required_colour = "R"

    # @state()
    def position_control(self):
        self.required_colour = "R"  # get colour from fms
        self.spinner.go_to_colour()

    def rotation_control(self):
        self.spinner.do_rotation_control()

    def execute(self):
        pass