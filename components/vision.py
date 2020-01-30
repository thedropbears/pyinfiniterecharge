from networktables import NetworkTables
from typing import Tuple


class Vision:
    def __init__(self) -> None:
        self.nt = NetworkTables
        self.table = self.nt.getTable("/vision")
        self.entry = self.table.getEntry("data")

    def get_vision_data(self) -> Tuple[float, float, float]:
        """Returns a tuple containing the distance (metres), angle (radians), and timestamp (time.monotonic)
        If it can't get info, it returns [None, None, None]
        """
        return self.entry.getDoubleArray([None, None, None])
