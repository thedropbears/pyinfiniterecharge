# from magicbot import StateMachine, state
from components.spinner import Spinner


# class SpinnerController(StateMachine):
class SpinnerController:
    spinner: Spinner

    def setup(self, testing = False):
        self.spinner.raise_wheel()
        self.task = None
        self.required_colour = "R"
        self.testing  = testing

    # @state()
    def position_control(self, test = self.testing):
        if test:
            self.required_colour = "R"
        else:
            self.required_colour = wpilib.DriverStation.getGameSpecificMessage() # gets the required colour from fms (not tested)
        self.spinner.go_to_colour(self.required_colour)

    def rotation_control(self):
        self.spinner.do_rotation_control()

    def execute(self):
        pass