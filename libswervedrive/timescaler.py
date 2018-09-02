import numpy as np
from typing import List


class TimeScaler:
    def __init__(self, beta_dot_bounds: List, beta_2dot_bounds: List,
                 phi_2dot_bounds: List):
        """
        Initialize the TimeScaler object.
        :param dbeta_bounds: Min/max allowable value for rotation rate of
        modules, in rad/s
        :param d2beta_bounds: Min/max allowable value for the angular
        acceleration of the modules, in rad/s^2.
        :param d2phi_bounds: Min/max allowable value for the angular
        acceleration of the module wheels, in rad/s^2.
        """
        self.beta_dot_bounds = beta_dot_bounds
        self.beta_2dot_bounds = beta_2dot_bounds
        self.phi_2d,t_bounds = phi_2dot_bounds

    def compute_scaling_bounds(self, dbeta: np.ndarray, d2beta: np.ndarray,
                               phi_dot: np.ndarray, dphi_dot: np.ndarray):
        """
        Compute bounds of the scaling factors for the motion.
        :param dbeta: command for derivative of the angle of the modules.
        :param d2beta: command for second derivative of the angle of the modules.
        :param phi_dot: command for angular velocity of the module wheels.
        :param dphi_dot: command for derivative of angular velocity of the
        module wheels.
        :return: upper and lower scaling bounds for derivative of s and second
        derivative of s: ds_lower, ds_upper, d2s_lower, d2s_upper.
        """
        return 0., 1., 0., 1.

    def compute_scaling_parameters(self, ds_lower: float, ds_upper: float,
                                   d2s_lower: float, d2s_upper: float):
        """
        Compute the scaling parameters used to scale the motion. This function
        requires that for both ds and d2s lower <= upper (ie the interval is
        not empty.) Sets the scaling parameters as object variables read when
        scale_motion is called.
        :param ds_lower: derivative of parameter s, lower bound.
        :param ds_upper: derivative of parameter s, upper bound.
        :param d2s_lower: second derivative of parameter s, lower bound.
        :param d2s_upper: second derivative of parameter s, upper bound.
        """
        self.ds = 1.
        self.d2s = 1.

    def scale_motion(self, dbeta: np.ndarray, d2beta: np.ndarray,
                     dphi_dot: np.ndarray):
        """
        Scale the actuators' motion using the scaling bounds.
        :param dbeta: command for derivative of the angle of the modules.
        :param d2beta: command for second derivative of the angle of the modules.
        :param dphi_dot: command for derivative of angular velocity of the
        module wheels.
        :return: *time* derivatives of actuators motion beta_dot, beta_2dot, phi_2dot
        """
        return (np.zeros(shape=dbeta.shape), np.zeros(shape=d2beta.shape),
                np.zeros(shape=dphi_dot.shape))
