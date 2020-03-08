import math

from magicbot import feedback, StateMachine, state, timed_state

from components.hang import Hang
from components.indexer import Indexer
from components.turret import Turret


class KrakenController(StateMachine):
    """Statemachine for high-level control of the Kraken"""

    hang: Hang
    indexer: Indexer
    turret: Turret

    def __init__(self) -> None:
        super().__init__()

    def execute(self) -> None:
        super().execute()

    @state(first=True)
    def clear_turret(self) -> None:
        self.turret.disabled = True
        if self.turret.is_ready():
            self.next_state("lower_intake")

    @timed_state(must_finish=True, duration=1, next_state="release_the_kraken")
    def lower_intake(self) -> None:
        self.indexer.lower_intake()

    @state
    def release_the_kraken(self) -> None:
        self.hang.raise_hook()
        self.done()
