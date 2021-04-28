from magicbot import StateMachine, state

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
        self.next_state("prepare")

    @state(must_finish=True)
    def prepare(self, state_tm, initial_call) -> None:
        if initial_call:
            self.turret.park_and_disable()
            self.indexer.lower_intake()
        if self.turret.is_parked() and state_tm > 1:
            self.next_state("release_the_kraken")

    @state(must_finish=True)
    def release_the_kraken(self) -> None:
        self.hang.raise_hook()
        self.done()
