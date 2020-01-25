from magicbot import StateMachine, state

from components.turret import Turret
from components.indexer import Indexer
from components.shooter import Shooter

class ShooterController(StateMachine):
    """Statemachine for high level control of the shooter and injector"""

    turret: Turret
    indexer: Indexer
    shooter: Shooter

    @state
    def searching(self):
        """
        The vision system does not have a target, we try to find one using odometry
        """
        pass

    @state
    def tracking(self):
        """
        Aiming towards a vision target and spining up flywheels
        """
        pass

    @state
    def firing(self):
        """
        Positioned to fire, inject and expel a single ballball
        """
        pass
