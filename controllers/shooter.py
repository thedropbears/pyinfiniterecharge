from components.indexer import Indexer


class ShooterController:
    """Class to control the entire shooter conglomerate"""

    indexer: Indexer
    
    def execute(self):
        """Executed every cycle"""

        motor_states = [True] * len(self.indexer.indexer_motors)
        switch_results = [switch.get() for switch in self.indexer.indexer_switches]
        if not switch_results[0]: # Test the back switch (because all others require 2 switches)
            motor_states[0] = False 
        
        for i in range(1, len(motor_states)): # Disable motor if switch and previous switch are pressed
            if not switch_results[i] and not switch_results[i - 1]:
                motor_states[i] = False

        for i in range(len(motor_states)): # Set all motors to required states
            if motor_states[i]:
                self.indexer.indexer_motors[i].set(0.5)
            else:
                self.indexer.indexer_motors[i].stopMotor()