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
<<<<<<< HEAD
        self.spinner.piston_down()
=======
>>>>>>> 9cba054243d070358ffdcf8075670a9af6e68956
        self.state = "position"
        self.required_colour = "R"  # get colour

    def rotation_control(self):
<<<<<<< HEAD
        self.spinner.piston_down()
=======
>>>>>>> 9cba054243d070358ffdcf8075670a9af6e68956
        self.state = "rotation"

    def execute(self):
        if self.state == "position":  # TODO get FMS info
<<<<<<< HEAD
            self.spinner.go_to_colour(self.required_colour)
        elif self.state == "rotation":
            self.spinner.do_rotation()

        if self.spinner.is_complete():
            self.state == None
=======
            self.spinner.got_to_colour(self.required_colour)
        elif self.state == "rotation":
            self.spinner.do_rotation()
>>>>>>> 9cba054243d070358ffdcf8075670a9af6e68956
