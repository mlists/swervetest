from icrcontroller import ICRController
import numpy as np

def test_icrc_init():
    icrc = ICRController(np.array([0]),
                         np.array([1]),
                         np.array([0.1]),
                         np.zeros(shape=(3, 1)),
                         [-1, 1], [-1, 1], [-1, 1],
                         [-1, 1], [-1, 1])