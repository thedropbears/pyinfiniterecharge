from magicbot import StateMachine, state, timed_state

from components.hang import Hang
from components.indexer import Indexer
from components.shooter import Shooter
from components.turret import Turret


class KrakenController(StateMachine):
    """Statemachine for high-level control of the Kraken"""

    hang: Hang
    indexer: Indexer
    shooter: Shooter
    turret: Turret

    def __init__(self) -> None:
        super().__init__()

    def execute(self) -> None:
        super().execute()

    @state(first=True, must_finish=True)
    def disable_shooter(self) -> None:
        self.shooter.disabled = True
        self.next_state("clear_turret")

    @state(must_finish=True)
    def clear_turret(self) -> None:
        self.turret.park_and_disable()
        if self.turret.is_parked():
            self.next_state("lower_intake")

    @timed_state(must_finish=True, duration=1, next_state="release_the_kraken")
    def lower_intake(self) -> None:
        self.indexer.lower_intake()

    @state(must_finish=True)
    def release_the_kraken(self) -> None:
        self.hang.raise_hook()
        self.done()
