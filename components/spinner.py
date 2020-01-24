import wpilib
import rev
import rev.color

class Colour:
    def __init__(self, red: float, green: float, blue: float):
        self.red = red
        self.green = green
        self.blue = blue

    def dist(self, c) -> tuple:
        return (
            abs(self.red - c.red) + abs(self.green - c.green) + abs(self.blue - c.blue)
        )

class Spinner:
	spinner_motor: wpilib.Spark
	spinner_solenoid: wpilib.Solenoid
	colour_sensor: rev.color.ColorSensorV3


	MAX_COLOUR_DIST = 0.2  # Chosen without experimentation, should be tuned
	COLOUR_OFFSET = 1  # The offset of the colour sensor, in 45 degree increments

	WHEEL_COLOURS = {
		"R": Colour(82 / 255, 116 / 255, 55 / 255),
		"B": Colour(55 / 255, 120 / 255, 80 / 255),
		"G": Colour(61 / 255, 126 / 255, 66 / 255),
		"Y": Colour(72 / 255, 131 / 255, 50 / 255),
	}

	WHEEL_ORDER = ["R", "Y", "B", "G"]

	WHEEL_SPIN_FACTOR = 0.3

	def __init__(self):
		self.rotations = 0
		self.state = None
		self.lastCol = None
		self.required_colour = None

	def on_enable(self):
		self.spinner_motor.stopMotor()
		self.spinner_solenoid.set(False)
		self.rotation = 0
		self.state = None
		self.lastCol = None
		self.required_colour = None




	# Methods
	def is_complete(self):
		return self.state == None
	def do_rotation_control(self):
		self.state = "rotation"
		self.rotations = 0
		self.lastCol = None
		self.piston_down()

	def go_to_colour(self, colour):
		self.state = "position"
		self.required_colour = colour
		self.piston_down()





	def piston_up(self):
		self.spinner_solenoid.set(True)
	def piston_down(self):
		self.spinner_solenoid.set(False)

	def read_colour(self):
		return (self.colour_sensor.getColor(), self.colour_sensor.getIR())

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

	def get_current_colour(self):
		sensed_colour, _ = self.read_colour()

		distances = {}
		for col_name, col_value in self.WHEEL_COLOURS.items():
			distances[col_name] = col_value.dist(sensed_colour)

		if (
			distances[min(distances, key=lambda col: distances[col])]
			< self.MAX_COLOUR_DIST
		):
			return min(distances, key=lambda col: distances[col])


	def run_rotation(self):
		self.spinner_motor.set(1-self.rotations/6)
		current_colour = self.get_current_colour()
		if(self.lastCol != current_colour):
			self.rotations += 1/8
		self.lastCol = current_colour
		if self.rotations >= 3.5 and self.read_colour()[1] > 35:
			self.state = None
			self.spinner_motor.set(0)
			self.piston_up()

	def run_position(self):
		distance = self.get_wheel_dist()
		self.spinner_motor.set(distance * self.WHEEL_SPIN_FACTOR)
		if distance == 0  and self.read_colour()[1] > 35:
			self.state = None
			self.piston_up()

	def execute(self):
		if self.state == "rotation":
			self.run_rotation()
		if self.state == "position":
			self.run_position()
		wpilib.SmartDashboard.putString("Read Colour", str(self.lastCol))