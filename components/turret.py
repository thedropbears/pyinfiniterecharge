import wpilib
import magicbot

class Turret:
    motor: wpilib.Spark
    left_index: wpilib.DigitalInput
    centre_index: wpilib.DigitalInput
    right_index: wpilib.DigitalInput
    
  

    def reset_ticks(self) -> None:
        # prototype starts at full speed 
        self.motor_speed = 1.0
        self.seeking = False
        self.tick_count = 0
        self.starting_max_ticks = 2
        self.max_ticks = self.starting_max_ticks
        self.max_ticks_factor = -2
        self.tick_increment = 1

    def on_enable(self) -> None:
        self.motor.stopMotor()
        self.reset_ticks()

    def setup(self) -> None:
        pass

    def execute(self) -> None:
        # Are we there yet? If so, stop the motor and stop seeking
        if self.centre_index.get(): 
            self.logger.info("Yep, we are there!!!")
            self.motor.stopMotor()
            self.reset_ticks()
            return
        
        # Start seeking right
        if self.seeking == False:
            self.logger.info("starting to seek")
            self.motor.set(self.motor_speed)
            self.tick_count = 0
            self.max_ticks = self.starting_max_ticks
            self.seeking = True
            return
        
        self.tick_count = self.tick_count + self.tick_increment
        if self.tick_count == self.max_ticks:
            self.logger.info("hitting the limit, turning around")
            self.motor.stopMotor()
            self.motor_speed = -self.motor_speed
            self.motor.set(self.motor_speed)
            self.tick_increment = self.tick_increment * -1
            self.max_ticks = self.max_ticks * self.max_ticks_factor
        
        
             
        
