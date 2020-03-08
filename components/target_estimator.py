from components.chassis import Chassis
from components.range_finder import RangeFinder
from components.turret import Turret
from components.vision import Vision, VisionData

from utilities.functions import constrain_angle

import math
import time

from typing import Optional

from magicbot import feedback


def _test_for_downrange(heading: float, azimuth: float) -> bool:
    # Calculate the turret azimuth in field coordinates
    turret_in_field = constrain_angle(heading + azimuth)
    return abs(turret_in_field) < math.pi / 2.0


class TargetEstimator:

    ALPHA = 0.1  # Exponential smoothing fraction
    CAMERA_TO_LIDAR = 0.15

    chassis: Chassis
    range_finder: RangeFinder
    turret: Turret
    vision: Vision

    def __init__(self) -> None:
        self.previous_azimuth = 0.0
        self.previous_heading = 0.0
        self._pointing_downrange = False
        self._init()

    def _init(self):
        self.angle_to_target: Optional[float] = None
        self.vision_range: Optional[float] = None
        self.previous_vision_data: Optional[VisionData] = None

    def on_enable(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._init()
        heading = self.chassis.get_heading()
        azimuth = self.turret.get_azimuth()
        self._pointing_downrange = _test_for_downrange(heading, azimuth)
        self.previous_heading = heading
        self.previous_azimuth = azimuth

    @feedback
    def is_ready(self) -> bool:
        return (
            self.angle_to_target is not None
            and self.is_pointing_downrange()
            and self.has_vision()
        )

    def has_vision(self) -> bool:
        return self.vision.is_ready()

    @feedback
    def is_pointing_downrange(self) -> bool:
        return self._pointing_downrange

    def get_data(self) -> VisionData:
        # Clients should check is_ready before calling, so this is just
        # defensive in case they don't
        if not self.is_ready():
            return VisionData(0.0, 0.0, 0.0)
        assert self.angle_to_target is not None  # mypy is silly
        if abs(self.angle_to_target) < math.radians(5.0):
            vd = VisionData(
                self.range_finder.get_distance() - self.CAMERA_TO_LIDAR,
                self.angle_to_target,
                time.monotonic(),
            )
        else:
            assert self.vision_range is not None
            vd = VisionData(self.vision_range, self.angle_to_target, time.monotonic())
        return vd

    def execute(self) -> None:
        # Each timestep we need to adjust our estimate for turret movement.
        # We also check for new vision data.
        # If we have it, we add it to our current running average.
        if not self.is_ready():
            self.reset()
        if self.previous_heading is None:
            self.previous_heading = self.chassis.get_heading()

        # First, correct our estimate
        current_azimuth = self.turret.get_azimuth()
        current_heading = self.chassis.get_heading()
        if self.angle_to_target is not None:
            # print(f"prior angle to target {self.angle_to_target}")
            delta = constrain_angle(current_azimuth - self.previous_azimuth)
            self.angle_to_target -= delta
            # print(f"angle to target minus azimuth delta {self.angle_to_target}")
            delta = constrain_angle(current_heading - self.previous_heading)
            self.angle_to_target -= delta
            # print(f"angle to target minus heading delta {self.angle_to_target}")
        self.previous_azimuth = current_azimuth
        self.previous_heading = current_heading

        # Check for new vision data
        # We only add it if it could possibly be a target
        self._pointing_downrange = _test_for_downrange(current_heading, current_azimuth)
        if not self._pointing_downrange:
            # If the turret is not pointing downfield we cannot be getting a valid vision target
            return

        vision_data = self.vision.get_data()
        if vision_data is not None:
            # Is it new?
            if (
                self.previous_vision_data is None
                or vision_data.timestamp != self.previous_vision_data.timestamp
            ):
                turret_movement = constrain_angle(
                    current_azimuth - self.turret.azimuth_at_time(vision_data.timestamp)
                )
                corrected_angle = vision_data.angle - turret_movement
                if self.angle_to_target is None:
                    self.angle_to_target = corrected_angle
                else:
                    self.angle_to_target = (
                        self.ALPHA * corrected_angle
                        + (1.0 - self.ALPHA) * self.angle_to_target
                    )
                if self.vision_range is None:
                    self.vision_range = vision_data.distance
                else:
                    self.vision_range = (
                        self.ALPHA * vision_data.distance
                        + (1.0 - self.ALPHA) * self.vision_range
                    )
                self.previous_vision_data = vision_data
