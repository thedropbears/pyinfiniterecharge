from components.range_finder import RangeFinder
from components.turret import Turret
from components.vision import Vision, VisionData

import math
import time


class TargetEstimator:

    ALPHA = 0.1  # Exponential smoothing fraction
    CAMERA_TO_LIDAR = 0.15

    range_finder: RangeFinder
    turret: Turret
    vision: Vision

    def setup(self) -> None:
        self.reset()

    def on_enable(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.angle_to_target: float = 0.0
        self.vision_range: float = 0.0
        self.previous_azimuth: float = 0.0
        self.previous_vision_data: VisionData = VisionData(0.0, 0.0, 0.0)

    def is_ready(self) -> bool:
        return self.vision.is_ready()

    def get_data(self) -> VisionData:
        if abs(self.angle_to_target) < math.radians(5.0):
            vd = VisionData(
                self.range_finder.get_distance() - self.CAMERA_TO_LIDAR,
                self.angle_to_target,
                time.monotonic(),
            )
        else:
            vd = VisionData(self.vision_range, self.angle_to_target, time.monotonic(),)
        return vd

    def execute(self) -> None:
        # Each timestep we need to adjust our estimate for turret movement.
        # We also check for new vision data.
        # If we have it, we add it to our current running average.
        if not self.is_ready():
            self.reset()

        # First, correct our estimate
        current_azimuth = self.turret.get_azimuth()
        delta = current_azimuth - self.previous_azimuth
        self.angle_to_target -= delta
        self.previous_azimuth = current_azimuth

        # Check for new vision data
        vision_data = self.vision.get_data()
        if vision_data is not None:
            # Is it new?
            if vision_data.timestamp != self.previous_vision_data.timestamp:
                turret_movement = (
                    self.turret.azimuth_at_time(vision_data.timestamp) - current_azimuth
                )
                corrected_angle = vision_data.angle - turret_movement
                self.angle_to_target = (
                    self.ALPHA * corrected_angle
                    + (1.0 - self.ALPHA) * self.angle_to_target
                )
                self.vision_range = (
                    self.ALPHA * vision_data.distance
                    + (1.0 - self.ALPHA) * self.vision_range
                )
                self.previous_vision_data = vision_data
