
# from magicbot import StateMachine, state
from components.spinner import Spinner
import wpilib

# class SpinnerController(StateMachine):
class SpinnerController:
	spinner: Spinner


	def run(self, test=False, task=None):
		self.test = test
		self.task = task
		if self.spinner.is_complete():
			self.state = "select_task"
		else:
			self.spinner.state = None
		self.select_task()

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

		elif self.task == "position":
			self.required_colour = "R"
			self.state = "position"
			self.spinner.go_to_colour("R")
			# self.next_state("position")

		elif self.task == "rotation":
			self.state = "rotation"
			self.spinner.do_rotation_control()
			# self.next_state("rotation")


	def execute(self):
		pass