import numpy as np
import math


class KinematicModel:
    def __init__(self, modules_alpha: np.ndarray, modules_l: np.ndarray,
                 modules_b: np.ndarray, k_beta: float):
        """
        Initialize the KinamaticModel object. The order in the following arrays
        must be preserved throughout all arguments passed to this object.
        :param modules_alpha: array containing the angle to each of the modules,
        measured counter clockwise from the x-axis.
        :param modules_l: distance to the axis of rotation of each module from
        the origin of the chassis frame
        :param modules_b: distance from the axis of rotation of each module to
        it's contact with the ground.
        :param k_beta: the speed at which wheel reconfiguration must occur.
        """
        self.alpha = modules_alpha
        self.l = modules_l
        self.b = modules_b
        self.n_modules = len(self.alpha)
        self.k_beta = k_beta

        self.a = np.zeros(shape=(3, self.n_modules))
        self.a_orth = np.zeros(shape=(3, self.n_modules))
        self.s = np.zeros(shape=(3, self.n_modules))
        self.l_v = np.zeros(shape=(3, self.n_modules))
        for i in range(self.n_modules):
            self.a[:,i] = np.array([math.cos(self.alpha[i]),
                                    math.sin(self.alpha[i]),
                                    0])
            self.a_orth[:,i] = np.array([-math.sin(self.alpha[i]),
                                         math.cos(self.alpha[i]),
                                         0])
            self.s[:,i] = np.array([self.l[i]*math.cos(self.alpha[i]),
                                    self.l[i]*math.sin(self.alpha[i]),
                                    1])
            self.l_v[:,i] = np.array([0, 0, self.l[i]])

    def compute_actuators_motion(self, dlmda: np.ndarray, d2lmda: np.ndarray,
                                 dmu: float):
        """
        Compute the motion of the actuators based on the chassis motion commands.
        :param dlmda: first derivative of lmda, as a column vector.
        :param d2lmda: second derivative of lmda, as a column vector.
        :param dmu: derivative of mu.
        :return: first and second derivatives of the actuators' positions, as
        arrays: (dbeta, d2beta, phi_dot, dphi_dot) (phi_dot is a already a time
        derivative as the relevant constraints are applied in the path planner).
        """
        return np.zeros(shape=(self.n_modules)) * 4


    def reconfigure_wheels(self, beta_d: np.ndarray, beta_e: np.ndarray):
        """
        Perform a wheel reconfiguration to a desired state.
        This method must be called *instead* of compute actuators motion when
        the ICR is on a structural singularity, as in that case there are no
        valid wheel configurations.
        :beta_d: array of desired beta values
        :beta_e: array of measured beta values.
        :returns: Array of dbeta values.
        """
        return np.zeros(shape=(self.n_modules,))
