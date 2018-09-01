import numpy as np
import math


class ICREstimator:

    # constants used in the lmda estimation algo
    eta_lmda: float = 1e-3 # TODO: figure out what values this should be
    eta_delta: float = 1e-3 # TODO: figure out what values this should be
    max_iter = 3 # TODO: figure out what value should be

    def __init__(self, epsilon_init: np.ndarray, modules_alpha: np.ndarray, modules_l: np.ndarray,
                 modules_b: np.ndarray):
        """
        Initialize the ICREstimator object. The order in the following arrays
        must be preserved throughout all arguments passed to this object.
        :param epsilon_init: the starting position estimate for the robot
        position. Form (x, y, theta)^T.
        :param modules_alpha: array containing the angle to each of the modules,
        measured counter clockwise from the x-axis.
        :param modules_l: distance to the axis of rotation of each module from
        the origin of the chassis frame
        :param modules_b: distance from the axis of rotation of each module to
        it's contact with the ground.
        """

        self.epsilon = epsilon_init

        self.alpha = modules_alpha
        self.l = modules_l
        self.b = modules_b
        self.n_modules = len(self.alpha)
        self.epsilon_init = epsilon_init

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

    def compute_odometry(self, lmda_e: np.ndarray, mu_e: float, delta_t: float):
        """
        Update our estimate of epsilon (twist position) based on the new ICR
        estimate.
        :param lmda_e: the estimate of the ICR in h-space.
        :param mu_e: estimate of the position of the robot about the ICR.
        :param delta_t: time since the odometry was last updated.
        """

    def estimate_mu(self, phi_dot: np.ndarray, lmda_e):
        """
        Find the rotational position of the robot about the ICR.
        :param phi_dot: array of angular velocities of the wheels.
        :param lmda_e: the estimate of the ICR in h-space.
        :return: the estimate of mu (float).
        """
        # this requires solving equation (22) from the control paper, i think
        # we may need to look into whether this is valid for a system with no
        # wheel coupling
        return 0.

    def estimate_lmda(self, q: np.ndarray):
        """
        Find the ICR given the steering angles.
        :param q: list of angles beta between representing the steer angle
        (measured relative to the orientation orthogonal to the line to the
        chassis frame origin.)
        :return: our estimate of ICR as the array (u, v, w)^T.
        """
        starting_points = self.select_starting_points(q)
        found = False
        closest_lmda = None
        closest_dist = None
        for lmda_start in starting_points:
            lmda = lmda_start
            if closest_lmda is None:
                closest_lmda = lmda_start
                closest_dist = np.linalg.norm(q - self.S(lmda_t))
            if np.linalg.norm(q - self.S(lmda)) < self.eta_delta:
                found = True
            else:
                for i in range(self.max_iter):
                    (S_u, S_v) = self.compute_derivatives(lmda)
                    (delta_u, delta_v) = self.solve(S_u, S_v, q, lmda)
                    lmda_t = self.update_parameters(lmda, delta_u, delta_v)
                    lmda_t = self.handle_singularities(lmda_t)
                    if np.linalg.norm(q - self.S(lmda_t)) > np.linalg.norm(q - self.S):
                        # appears the algorithm has diverged as we are not
                        # improving
                        found = False
                        break
                    else:
                        found = np.linalg.norm(lmda - lmda_t) > self.eta_lmda
                        distance = np.linalg.norm(q - self.S(lmda_t))
                        if distance < closest_dist:
                            closest_lmda = lmda_t
                            closest_dist = distance
                    lmda = lmda_t
            if found:
                return lmda
        return closest_lmda

    def select_starting_points(self, q: np.ndarray):
        """
        Find the starting points for the Newton-Raphson algorithm. This
        implementation places them at the intersection of the propulsion axis
        and orders them according to their distance to the input point.
        :param q: list of angles beta between representing the steer angle
        (measured relative to the orientation orthogonal to the line to the
        chassis frame origin.)
        :return: List of the top three starting points ordered according to
        their distance to the input length.
        """
        return np.zeros(shape=(3, 3))

    def compute_derivatives(self, lmda: np.ndarray):
        """
        Compute the derivateves of the constraining surface at the current
        estimate of the point.
        :param lmda: position of the ICR estimate
        :return: np.ndarray with (S_u, S_v). S_u and S_v are the vectors
        containing the derivatives of each steering angle in q with respect
        u and v, respectively.
        """
        S_u = np.zeros(shape=(self.n_modules,))
        S_v = np.zeros(shape=(self.n_modules,))
        lmda = lmda.T # computations require lambda as a row vector
        for i in range(self.n_modules):
            # equations 16 and 17 in the paper
            a = column(self.a, i)
            a_orth = column(self.a_orth, i)
            l = column(self.l_v, i)
            delta = lmda.dot(a-l)
            omega = lmda.dot(a_orth)
            # equation 18 excluding ∂lmda/∂u
            gamma_top = (omega*(a-l) + delta*a_orth)
            gamma_bottom = lmda.dot(delta*(a-l) - omega*a_orth)
            # equation 19
            du = np.array([1, 0, -lmda[0,0]/lmda[0,2]]).reshape(1, 3)
            dv = np.array([0, 1, -lmda[0,1]/lmda[0,2]]).reshape(1, 3)
            beta_u = du.dot(gamma_top) / gamma_bottom
            beta_v = dv.dot(gamma_top) / gamma_bottom
            S_u[i] = beta_u
            S_v[i] = beta_v
        return (S_u, S_v)

    def solve(self, S_u: np.ndarray, S_v: np.ndarray, q: np.ndarray,
              lmda: np.ndarray):
        """
        Solve the system of linear equations to find the free parameters
        delta_u and delta_v.
        :param S_u: derivative of constraining surface wrt u (vector).
        :param S_v: derivative of constraining surface wrt v (vector).
        :param q: list of angles beta between representing the steer angle
        (measured relative to the orientation orthogonal to the line to the
        chassis frame origin.)
        :param lmda: position of the ICR estimate.
        :return: the free parameters in the form (delta_u, delta_v).
        """
        a_u = S_u.dot(S_u)
        a_c = S_u.dot(S_v)
        a_v = S_v.dot(S_v)
        A = np.array([[a_u, a_c], [a_c, a_v]])
        p_zero = self.S(lmda)
        diff = (q - p_zero).reshape((1, -1))
        b = np.array([diff.dot(S_u.T), diff.dot(S_v.T)])
        x = np.linalg.solve(A, b)
        return x

    def update_parameters(self, lmda: np.ndarray, delta_u: float, delta_v: float):
        """
        Move our estimate of the ICR based on the free parameters delta_u and
        delta_v. If invalid parameters are produced rescale them so they lie
        within the sphere. If the algorithm has diverged backtrack if possible
        :param lmda: current position of the ICR estimate.
        :param delta_u: free parameter defining how much to move the ICR
        estimate in the direction S_u.
        :param delta_v: free parameter defining how much to move the ICR
        estimate in the direction S_v.
        :return: the new ICR estimate, a flag indicating divergence of the
        algorithm for this starting point.
        """
        return np.zeros(shape=(3, 1))

    def handle_singularities(self, lmda: np.ndarray):
        """
        Handle the structural singularities that may have been produced when
        the parameters were updated (when the ICR lies on a steering axis).
        :param lmda: the ICR estimate after the parameters were updated.
        :return: the ICR estimate with the singularity corrected.
        """
        return np.zeros(shape=(3, 1))

    def S(self, lmda: np.ndarray):
        """
        Compute the point in the joint space (space of all beta steering angle
        values) associated with a particular ICR.
        :param lmda: the ICR to compute the point for.
        :return: row vector expressing the point.
        """
        S = np.zeros(shape=(self.n_modules,))
        lmda = lmda.T # computations require lambda as a row vector
        for i in range(self.n_modules):
            # equations 16 and 17 in the paper
            a = column(self.a, i)
            a_orth = column(self.a_orth, i)
            l = column(self.l_v, i)
            delta = lmda.dot(a-l)
            omega = lmda.dot(a_orth)
            norm = np.linalg.norm([delta, omega])
            sin_beta = np.sign(delta) * omega / norm
            cos_beta = np.sign(delta) * delta / norm
            S[i] = math.atan2(sin_beta, cos_beta)
        return S


def column(mat, row_i):
    """
    Grab a column from a vector as a numpy column vector.
    :param row_i: row index
    :return: the column vector (shape (n, 1))
    """
    return mat[:,row_i:row_i+1]
