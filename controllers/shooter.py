import math
import time

from typing import Optional

from magicbot import feedback, StateMachine, state, will_reset_to

from components.chassis import Chassis
from components.indexer import Indexer
from components.shooter import Shooter
from components.range_finder import RangeFinder
from components.turret import Turret
from components.target_estimator import TargetEstimator
from components.led_screen import LEDScreen


class ShooterController(StateMachine):
    """Statemachine for high level control of the shooter and injector"""

    VERBOSE_LOGGING = True

    chassis: Chassis
    indexer: Indexer
    shooter: Shooter
    turret: Turret
    target_estimator: TargetEstimator
    led_screen: LEDScreen
    range_finder: RangeFinder

    fire_command = will_reset_to(False)

    MAX_MISALIGNMENT = 0.2  # m from centre of target

    MIN_SCAN_PERIOD = 3.0
    TARGET_LOST_TO_SCAN = 0.5

    CAMERA_TO_LIDAR = 0.15

    def __init__(self) -> None:
        super().__init__()
        self.spin_command = False
        self.distance = None
        self.time_of_last_scan: Optional[float] = None
        self.time_target_lost: Optional[float] = None
        self.disabled_flash: int = 0
        self.fired_count: int = 0

    def execute(self) -> None:
        super().execute()
        self.update_LED()

    def on_disable(self):
        self.stop()
        return super().on_disable()

    def update_LED(self) -> None:
        # Flash if turret and shooter are disabled
        if self.shooter.disabled:
            if self.disabled_flash < 25:
                self.led_screen.set_bottom_row(255, 0, 0)
                self.led_screen.set_middle_row(255, 0, 0)
                self.led_screen.set_top_row(255, 0, 0)
            else:
                self.led_screen.set_bottom_row(0, 0, 0)
                self.led_screen.set_middle_row(0, 0, 0)
                self.led_screen.set_top_row(0, 0, 0)
            self.disabled_flash = (self.disabled_flash + 1) % 50
            return

        if self.shooter.is_ready():
            self.led_screen.set_bottom_row(0, 255, 0)
        else:
            self.led_screen.set_bottom_row(255, 0, 0)

        if self.indexer.is_ready():
            self.led_screen.set_middle_row(0, 255, 0)
        else:
            self.led_screen.set_middle_row(255, 0, 0)

        if self.target_estimator.is_ready():
            if self.aimed_at_target():
                self.led_screen.set_top_row(0, 255, 0)
            else:
                self.led_screen.set_top_row(255, 128, 0)
        else:
            self.led_screen.set_top_row(255, 0, 0)

    @state(first=True)
    def startup(self, initial_call) -> None:
        """
        Wait for the turret to complete it's inital slew before doing anything
        """
        if initial_call:
            # target_angle = self.chassis.find_power_port_angle()
            self.turret.slew(0)
        if self.turret.is_ready():
            self.next_state("searching")

    @state
    def searching(self) -> None:
        """
        The vision system does not have a target, we try to find one using odometry
        """
        if self.target_estimator.is_ready():
            # print(f"searching -> tracking {self.vision.get_vision_data()}")
            self.time_target_lost = None
            self.next_state("tracking")
        else:
            # Scan starting straight downrange.
            time_now = time.monotonic()
            # Start a scan only if it's been a minimum time since we lost the target
            if (
                self.time_target_lost is None
                or (time_now - self.time_target_lost) > self.TARGET_LOST_TO_SCAN
            ):
                # Start a scan only if it's been a minimum time since we started
                # a scan. This allows the given scan to grow enough to find the
                # target so that we don't just start the scan over every cycle.
                if (
                    self.time_of_last_scan is None
                    or (time_now - self.time_of_last_scan) > self.MIN_SCAN_PERIOD
                ):
                    self.turret.scan(-self.chassis.get_heading() + math.pi)
                    self.time_of_last_scan = time_now
            # TODO test this
            # target_angle = self.chassis.angle_to_target()
            # self.turret.scan(target_angle)

    def stop(self) -> None:
        self.done()
        self.shooter.stop()
        self.turret.slew(0)

    @state
    def tracking(self) -> None:
        """
        Aiming towards a vision target and spinning up flywheels
        """
        # collect data only once per loop
        if not self.target_estimator.is_ready():
            self.time_target_lost = time.monotonic()
            self.next_state("searching")
            # print(f"tracking -> searching {self.vision.get_vision_data()}")
        else:
            target_data = self.target_estimator.get_data()
            if abs(target_data.angle) > self.find_allowable_angle():
                # print(f"Telling turret to slew by {target_data.angle}")
                self.turret.slew(target_data.angle)
            if self.turret.is_ready():
                self.shooter.set_range(target_data.distance)
            if self.ready_to_fire() and self.fire_command:
                self.next_state("firing")

    @state(must_finish=True)
    def firing(self, initial_call) -> None:
        """
        Positioned to fire, inject and expel a single ball
        """
        if initial_call:
            self.shooter.fire()
            self.fired_count += 1
        elif not self.shooter.is_firing():
            self.next_state("tracking")

    def fire_input(self) -> None:
        """
        Called by robot.py to indicate the fire button has been pressed
        """
        self.fire_command = True

    @feedback
    def ready_to_fire(self) -> bool:
        return (
            self.shooter.is_ready()
            and self.aimed_at_target()
            and self.indexer.is_ready()
            and self.target_estimator.is_ready()
            and self.turret.is_ready()
        )

    @feedback
    def aimed_at_target(self) -> bool:
        target_data = self.target_estimator.get_data()
        return abs(target_data.angle) < self.find_allowable_angle()

    def find_allowable_angle(self) -> float:
        """
        Find the maximum angle by which the turret can be misaligned to still score a hit
        Currently does not consider angle from target, assumes we are at max distance
        """
        # dist: the maximum value of our lookup table
        dist = self.shooter.ranges[-1]
        angle = math.atan(self.MAX_MISALIGNMENT / dist)
        return angle
