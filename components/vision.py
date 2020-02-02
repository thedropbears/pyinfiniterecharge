from networktables import NetworkTables
from typing import Tuple
import time
import math


class Vision:
    def __init__(self) -> None:
        self.nt = NetworkTables
        self.table = self.nt.getTable("/vision")
        self.entry = self.table.getEntry("data")

    def get_vision_data(self) -> Tuple[float, float, float]:
        """Returns a tuple containing the distance (metres), angle (radians), and timestamp (time.monotonic)
        If it can't get info, it returns [None, None, None]
        """
        self.data = self.entry.getDoubleArray([None, None, None])
        return self.data

    def is_ready(self) -> int:
    	if self.data[2]-time.monotonic() > 0.5:
    		return 0 # no target
    	elif self.data[1] < math.degrees(5):
    		return 1 #target out of alignment
    	else: return 2 #aligned

