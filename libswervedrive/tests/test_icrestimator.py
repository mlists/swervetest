import numpy as np
import math
from icrestimator import ICREstimator

def init_icre(alphas, ls, bs):
    alphas = np.array(alphas)
    ls = np.array(ls)
    bs = np.array(bs)
    epsilon = np.zeros(shape=(3, 1))
    icre = ICREstimator(epsilon, alphas, ls, bs)
    return icre

def test_joint_space_conversion():
    icre = init_icre([math.pi/4], [1], [0])
    lmda = np.array([0, 0, -1]).reshape(-1, 1)
    beta_target = np.array([0])
    assert np.allclose(beta_target, icre.S(lmda))
    lmda = np.array([0, -1, 0]).reshape(-1, 1)
    beta_target = np.array([math.pi/4])
    assert np.allclose(beta_target, icre.S(lmda))

def test_solve():
    # for now, check only for runtime errors until compute_derivatives works
    icre = init_icre([math.pi/4, -math.pi/4, math.pi], [1, 1, 1], [0, 0, 0])
    lmda = np.array([0, -1, 0]).reshape(-1, 1)
    S_u = np.array([1/math.sqrt(2), 1/math.sqrt(2), 0])
    S_v = np.array([0, 0, 1])
    q = np.array([0, 0, 0])
    icre.solve(S_u, S_v, q, lmda)

def test_compute_derivatives():
    # for now, check only for runtime errors
    icre = init_icre([math.pi/4, -math.pi/4, math.pi], [1, 1, 1], [0, 0, 0])
    lmda = np.array([0, 0, -1]).reshape(-1, 1)
    S_u, S_v = icre.compute_derivatives(lmda)

def test_handle_singularities():
    icre = init_icre([0, math.pi/2, math.pi], [1, 1, 1], [0, 0, 0])
    # icr on wheel 0 on the R^2 plane
    icr = np.array([1, 0, 1]).reshape(-1, 1)
    lmda = icr * 1/np.linalg.norm(icr)
    singularity, wheel_number = icre.handle_singularities(lmda)
    assert singularity
    assert wheel_number is 0
    icr = np.array([100, 0, 1]).reshape(-1, 1)
    lmda = icr * 1/np.linalg.norm(icr)
    singularity, wheel_number = icre.handle_singularities(lmda)
    assert not singularity
    assert wheel_number is None

def test_update_parameters():
    icre = init_icre([0, math.pi/2, math.pi], [1, 1, 1], [0, 0, 0])
    q = np.zeros(shape=(3,)) # ICR on the robot's origin
    desired_lmda = np.array([0, 0, -1]).reshape(-1, 1)
    u, v = -0.1, -0.1 # ICR estimate too negative
    lmda_estimate = np.array([u, v,
                              math.sqrt(1-np.linalg.norm([u, v]))]).reshape(-1, 1)
    delta_u, delta_v = 0.1, 0.1
    lmda_t, worse = icre.update_parameters(lmda_estimate, delta_u, delta_v,
                                           q)
    # ignore w coordinate in comparison due to antipodal points
    assert np.allclose(lmda_t[:2], desired_lmda[:2])
    assert not worse
    delta_u, delta_v = -0.1, -0.1
    lmda_t, worse = icre.update_parameters(lmda_estimate, delta_u, delta_v,
                                           q)
    assert worse
