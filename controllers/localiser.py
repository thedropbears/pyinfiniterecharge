from components.chassis import Cassis
from components.vision import Vision
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from collections import deque
import numpy as np
import time


class Localiser:
    chassis: Chassis
    vision: Vision

    # process noise divided by the robot's velocity,
    # P/v - we modulate P based on the robot's velocity
    process_noise_vel_P = np.array([[1, 0, 0.0], [0, 1, 0.0], [0, 0, 1.0]])

    # covariance matrix for the output of solvePnP algorithm
    # idea: experiment with modulating based on distance, if required
    pnp_cov = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    loading_bay_offset = np.array([[0.0], [0.0], [0.0]])
    
    # the filter's default starting position and covariance
    default_start_pos = np.array([[0], [0], [0]])
    default_start_cov = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    def __init__(self):

        # dimensions in the filter are [x, y, theta]
        self.ukf = UnscentedKalmanFilter(
            dim_x=3, dim_z=2, dt=1 / 50.0, fx=predict_f, residual_x=residual_position
        )

        # to make sigma point objects for different measurement functions,
        # keep all but the first argument the same, which should be the dimensionality
        # of the measurement space
        self.sigma_position = MerweScaledSigmaPoints(3, 1e-3, 2, 0)

    def setup(self):
        self.last_pose = chassis.get_pose()
        self.history = deque()
        self.last_t = time.monotonic()
        self.reset()

    def reset(self, start_pos:np.ndarray = None, start_cov:np.ndarray = None):
        """ Reset the filter state.

        Arguments:
            start_pos: starting position of the filter.
            start_cov: starting covariance of the filter.
        """
        if start_pos is None:
            start_pos = self.default_start_pos
        if start_cov is None:
            start_cov = self.default_start_cov
        self.ukf.x = start_pos
        self.ukf.P = start_cov

    def execute(self):

        t = time.monotonic()
        dt = t - self.last_t
        self.last_t = dt

        pose = chassis.get_pose()
        delta_trans = (pose - last_pose).translation()
        delta = np.array([delta_trans.X(), delta_trans.Y(), delta_trans.rotation().radians()])
        self.last_pose = pose

        v = (self.chassis.get_left_velocity() + self.chassis.get_right_velocity()) / 2.0
        noise = self.process_noise_vel_P * v

        self.history.push([t, dt, delta, noise])

        # prevent unbounded growth of the queue size in absence of vision update calls
        if len(self.history) > 25: # max history len, ~0.5sec
            [t, dt, delta_np, noise] = self.history.popleft()
            self.ukf.Q = noise
            self.ukf.predict(dt=dt)

        self.predict_to_now()

    def update(
        self,
        measurement: np.ndarray,
        measurement_tm: float,
        measurement_cov: np.ndarray,
        hx: typing.callable,
        sigmas: filterpy.kalman.MerweScaledSigmaPoints,
        residual_z:typing.callable=np.subtract,
        **hx_args
    ):
        """ Perform a UKF update on a general measurement type.

        The function predicts forward the current measurement history to the time of the measurement, and discards the history up to this point.

        Arguments:
            measurement: the measurement.
            measurement_tm: the time the measurement was taken, should be relative to the roboRIOs monotonic clock.
            measurement_cov: covariance associated with the measurement, in the measurement space.
            hx: the function mapping the UKF state to the measurement space.
            sigmas: the sigma point object to be used by the UKF.
            residual_z: residual function.
            **hx_args: 
        """
        # nb this approach is problematic if a second camera's update comes after the first's
        # but is further back in time.
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
        """ Propogates forward the state to now based on the odemetry history temporarily each timestep.
        """
        self.current_state = np.copy(self.ukf.x)
        for [t, dt, delta_np, noise] in self.history:
            self.current_state += delta_np
        # wraps the angle
        self.current_state = residual_position(self.current_state, 0.)

    def position_update(self, measurement: np.ndarray, measurement_tm: float, measurement_cov: None):
        """ Perform an update from a position measurement on the field - intended to be used on the output of solvePnP
            Arguments:
                measurement: the measurement.
                measurement_tm: the time the measurement was taken, should be relative to the roboRIOs monotonic clock.
                measurement_cov: covariance associated with the measurement, in the measurement space.
        """
        if measurement_cov is None:
            measurement_cov = self.pnp_cov
        self.update(
            measurement,
            self.pnp_cov,
            measurement_tm,
            h_position,
            self.sigma_position,
            residual_z=residual_position,
            offset=self.loading_bay_offset,
        )


# predict model, x_t+1 = x + delta_x (where delta_x = [dx; dy])
def predict_f(x, dt, u):
    return x + u

# observation model, identity minus a constant offset as we are observing the output from SolvePnP
def h_position(x, offset=np.zeros(1, 3)):
    return residual_position(x, offset)

# third argument is an angle, so wrap to +- pi
def residual_position(a, b):
    diff = a - b
    diff[2] = np.atan2(np.sin(diff[2]), np.cos(diff[2]))
    return diff