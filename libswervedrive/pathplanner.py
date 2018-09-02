import numpy as np
from typing import List


class PathPlanner:
    def __init__(self, modules_alpha: np.ndarray, modules_l: np.ndarray,
                 phi_dot_bounds: List, k_lmda: float, k_mu: float):
        """
        Initialize the PathPlanner object. The order in the arrays
        must be preserved throughout all arguments passed to this object.
        :param modules_alpha: array containing the angle to each of the modules,
        measured counter clockwise from the x-axis.
        :param modules_l: distance to the axis of rotation of each module from
        the origin of the chassis frame
        :param dphi_bounds: Min/max allowable value for rotation rate of
        module wheels, in rad/s
        :param k_lmda: Proportional gain for the movement of lmda. Must be >=1
        :param k_mu: Proportional gain for movement of mu. Must be >=1
        """
        self.alpha = modules_alpha
        self.l = modules_l
        self.dphi_bounds = phi_dot_bounds
        self.k_lmda = k_lmda
        self.k_mu = k_mu

    def compute_chassis_motion(self, lmda_d: np.ndarray, lmda_e: np.ndarray,
                               mu_d: float, mu_e: float, k_b: float):
        """
        Compute the path to the desired state and implement control laws
        required to produce the motion.
        :param lmda_d: The desired ICR.
        :param lmda_e: Estimate of the current ICR.
        :param mu_d: Desired motion about the ICR.
        :param mu_e: Estimate of the current motion about the ICR.
        :param k_b: Backtracking constant.
        :return: (derivative of lmda, 2nd derivative of lmda, derivative of mu)
        """
        return np.zeros(shape=(3, 1)), np.zeros(shape=(3, 1)), 0
