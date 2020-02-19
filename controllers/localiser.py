from components.chassis import Cassis
from components.vision import Vision
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from collections import deque
import numpy as np
import time


class Localiser:
    chassis: Chassis
    vision: Vision

    reject_delta_limit = 1  # m

    # process noise divided by the robot's velocity,
    # P/v - we modulate P based on the robot's velocity
    process_noise_vel_P = np.array([[1, 0, 0.0], [0, 1, 0.0], [0, 0, 1.0]])

    pnp_noise = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    loading_bay_offset = np.array([[0.0], [0.0], [0.0]])

    def __init__(self):

        self.ukf = UnscentedKalmanFilter(
            dim_x=3, dim_z=2, dt=1 / 50.0, fx=predict_f
        )

        # to make sigma point objects for different measurement functions,
        # keep all but the first argument the same, which should be the dimensionality
        # of the measurement space
        self.sigma_position = MerweScaledSigmaPoints(3, 1e-3, 2, 0)

    def setup(self):
        self.last_pose = chassis.get_pose()
        self.history = deque()
        self.last_t = time.monotonic()

    def execute(self):

        t = time.monotonic()
        dt = t - self.last_t
        self.last_t = dt

        pose = chassis.get_pose()
        delta_trans = (pose - last_pose).translation()
        delta = np.array([delta_trans.X(), delta_trans.Y(), delta_trans.rotation().radians()])

        v = (self.chassis.get_left_velocity() + self.chassis.get_right_velocity()) / 2.0
        noise = self.process_noise_vel_P * v

        self.history.push([t, dt, delta, noise])

        self.predict_to_now()

    def update(
        self,
        measurement,
        measurement_cov,
        measurement_tm,
        hx,
        sigmas,
        residual_z=np.subtract,
        **hx_args
    ):
        # nb this approach is problematic if a second camera's update comes after the first's
        # but has a smaller timestamp
        while not self.history.is_empty():
            [t, dt, delta_np, noise] = self.history.popleft()
            self.ukf.Q = noise
            self.ukf.predict(dt=dt)
            if t < measurement_tm:
                # filterpy has a 'bug' which does not allow you to update the residual types
                # for different types of measurements
                self.ukf.residual_z = residual_z
                self.ukf.update(measurement, R=mesaurement_cov, hx=hx, **hx_args)
                break
        self.predict_to_now()

    def predict_to_now(self):
        self.current_state = np.copy(sulf.ukf.x)
        for d in self.delta_deque:
            self.current_state += d

    def position_update(self, measurement, measurement_tm):
        self.update(
            measurement,
            self.pnp_noise,
            measurement_tm,
            h_position,
            self.sigma_position,
            residual_z=residual_position,
            offset=self.loading_bay_offset,
        )


# predict model, x_t+1 = x + delta_x (where delta_x = [dx; dy])
def predict_f(x, dt, u):
    return x + u

# observation model, identity as we are observing the output from SolvePnP
def h_position(x, offset=np.zeros(1, 3)):
    return x - offset

# third argument is an angle
def residual_position(a, b):
    diff = a - b
    diff[2] = np.atan2(np.sin(diff[2]), np.cos(diff[2]))
    return diff