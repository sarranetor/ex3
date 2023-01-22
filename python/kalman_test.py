import numpy as np
import matplotlib.pyplot as plt
import math


class RadarSim():
    """ .. """
    # True position in (x,y) plain at each tk time
    _x = 0
    _y = 0
    # delta t between the sampled positions
    _detlta_t = 0
    
    def __init__(self, x, y, delta_t):
        self._x = x
        self._y = y
        self._detlta_t = delta_t

    """ var_err: standard deviation of the error expressed in .. """
    def get_range(self, var_err):
        r = math.sqrt(self._x**2 + self._y**2) + var_err * np.random.randn()
        return r

    """ var_err: standard deviation of the error expressed in radiant """
    def get_angle(self, var_err):
        epsilon = np.arctan(self._y / self._x) + var_err * np.random.randn()