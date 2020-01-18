# #!/usr/bin/env python3

# import wpilib
# import ctre
# import math

# class Colour:
# 	def __init__(self, red, green, blue):
# 		self.red = red
# 		self.green = green
# 		self.blue = blue

# class WheelOfFortune:
#     """
#         This class handles reading and moving the color wheel for the 2020
#         FRC Competition Infinite Recharge.
#     """
    
#     def __init__(self):
#     	#colours from the real world
# 		self.blue = Colour(55, 120, 80)
# 		self.green = Colour(61, 126, 66)
# 		self.red = Colour(82, 116, 55)
# 		self.yellow = Colour(72, 131, 50)
# 		self.colorSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)

# 		#defining components
# 		self.frontLeft = ctre.TalonSRX(1)
# 		self.rearLeft = ctre.TalonSRX(2)
# 		self.left = wpilib.SpeedControllerGroup(self.frontLeft, self.rearLeft)

# 		self.frontRight = ctre.TalonSRX(3)
# 		self.rearRight = ctre.TalonSRX(4)
# 		self.right = wpilib.SpeedControllerGroup(self.frontRight, self.rearRight)

# 		self.drive = DifferentialDrive(self.left, self.right)

# 		spinMotor = ctre.TalonSRX(0)
# 		spinMotor.setNeutralMode(ctre.NeutralMode.Brake)
# 		XboxController =  wpilib.XboxController(0)
# 		self.state = "up"
# 		self.spinSpeed = 0.3
# 		self.colsRead = 0
# 		self.lastCol = "none"

# 		self.piston = wpilib.DoubleSolenoid(1, 0)
# 		"""
# 		self.spinMotor = ctre.TalonSRX(0)
# 		self.joystick = wpilib.Joystick(0)


# 	def robotPeriodic(self):
# 		# pnumatics code


# 		# colour sensing code
# 		"""
# 		detectedColor = self.colorSensor.getColor()

# 		ir = self.colorSensor.getIR()

# 		#find distance to each colour
# 		bDist = colDist(self.blue, detectedColor)
# 		gDist = colDist(self.green, detectedColor)
# 		rDist = colDist(self.red, detectedColor)
# 		yDist = colDist(self.yellow, detectedColor)

# 		#find which is smallest and which one that it
# 		closestDist = min([bDist, gDist, rDist, yDist])
# 		if closestDist == bDist:
# 			closestCol = "blue"
# 			closestColPlot = 0
# 		elif closestDist == gDist:
# 			closestCol = "green"
# 			closestColPlot = 1
# 		elif closestDist == rDist:
# 			closestCol = "red"
# 			closestColPlot = 2
# 		elif closestDist == yDist:
# 			closestCol = "yellow"
# 			closestColPlot = 3
# 		closestColPlot += closestCol/1000

# 		#send stuff to smart dahsboard
# 		wpilib.SmartDashboard.putString("read colour", closestCol)
# 		wpilib.SmartDashboard.putNumber("closestColPlot", closestColPlot)

# 		wpilib.SmartDashboard.putNumber("Red", detectedColor.red)
# 		wpilib.SmartDashboard.putNumber("Green", detectedColor.green)
# 		wpilib.SmartDashboard.putNumber("Blue", detectedColor.blue)
# 		wpilib.SmartDashboard.putNumber("IR", ir)

# 		proximity = self.colorSensor.getProximity()
# 		wpilib.SmartDashboard.putNumber("Proximity", proximity)

# 		"""
# 		up------ 
# 		|a		|b
# 		look   count
# 		|		|
# 		stop----
# 		|a or b
# 		up

# 		"""
# 		#position and rotation control code
# 		if self.state == "up":
# 			self.piston.set(2)
# 			spinMotor.set(ctre.ControlMode.PercentOutput, 0)
# 			if XboxController.getAButtonReleased():
# 				self.state = "look"
# 			if XboxController.getBButtonReleased():
# 				self.state =  "count"

# 		if self.state == "look":
# 			self.piston.set(1)
# 			spinMotor.set(ctre.ControlMode.PercentOutput, self.spinSpeed)
# 			if XboxController.getAButtonReleased():
# 				self.state = "stop"
# 			if closestCol == "red":
# 				spinMotor.set(ctre.ControlMode.PercentOutput, -self.spinSpeed)
# 				self.state = "stop"

# 		if self.state == "count":
# 			self.piston.set(1)
# 			if self.lastCol != closestCol:
# 				self.colsRead += 1
# 			self.lastCol = closestCol
# 			if self.colsRead >= 24:
# 				self.state = "stop"
# 				self.colsRead = 0
# 			if XboxController.getBButtonReleased():
# 				self.state = "stop"


# 		if self.state == "stop":
# 			spinMotor.set(ctre.ControlMode.PercentOutput, 0)
# 			self.piston.set(1)
# 			if XboxController.getAButtonReleased() or XboxController.getBButtonReleased():
# 				self.state = "up"

# 		if self.state == "spin": # not used currently
# 			spinMotor.set(ctre.ControlMode.PercentOutput, self.spinSpeed)
# 			if XboxController.getAButtonReleased():
# 				self.state = "look"
# 			if XboxController.getBButtonReleased():
# 				self.state = "count"

# 		if XboxController.getXButtonReleased():
# 			self.state = up
# 			spinMotor.set(ctre.ControlMode.PercentOutput, 0)
# 			self.piston
# 		wpilib.SmartDashboard.putNumber("spins", self.colsRead/8)

# 	def colDist(self, a, b):
# 		dist = (abs(a.red - b.red) + abs(a.green - b.green) + abs(a.blue - b.blue))/3
# 		return dist
