from math import sqrt
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import ExtendedKalmanFilter
from numpy import eye, array, asarray
import numpy as np
from numpy.random import randn
import math
import matplotlib.pyplot as plt

def HJacobian_at(x):
    """ compute Jacobian of H matrix at x """

    horiz_dist = x[0]
    altitude   = x[2]
    denom = sqrt(horiz_dist**2 + altitude**2)
    return array ([[horiz_dist/denom, 0., altitude/denom, 0.],
                   [1/(1+(altitude/horiz_dist)**2) * (- altitude/horiz_dist**2), 0., 1/(1+(altitude/horiz_dist)**2) * (1/horiz_dist), 0.]])

def hx(x):
    """ compute measurement for slant range and angle that
    would correspond to state x.
    """
    
    return [(x[0]**2 + x[2]**2) ** 0.5, np.arctan(x[2] / x[0])]

class RadarSim:
    """ Simulates the radar signal returns from an object
    flying at a constant altityude and velocity in 1D. 
    """
    
    def __init__(self, dt, pos, vel, alt):
        self.pos = pos
        self.vel = vel
        self.alt = alt
        self.dt = dt

    def update_velocity(self, vel):
        self.vel = vel
        
    def get_range(self):
        """ Returns slant range to the object. Call once 
        for each new measurement at dt time from last call.
        """
        
        # add some process noise to the system
        self.vel = self.vel  + .1*randn()
        self.alt = self.alt + .1*randn()
        self.pos = self.pos + self.vel*self.dt
    
        # add measurement noise
        err = self.pos * 0.05*randn()
        slant_dist = math.sqrt(self.pos**2 + self.alt**2)
        
        return slant_dist + err

    def get_angle(self):
        self.vel = self.vel  + .1*randn()
        self.alt = self.alt + .1*randn()
        self.pos = self.pos + self.vel*self.dt

        angle = np.arctan(self.alt / self.pos)
        err = angle * 0.01*randn()

        return angle * err


dt = 0.05
rk = ExtendedKalmanFilter(dim_x=4, dim_z=2)

radar = RadarSim(dt, pos=0., vel=100., alt=1000.)

# make an imperfect starting guess
rk.x = array([radar.pos-100, radar.vel+100, radar.alt+1000, 10])

rk.F = eye(4) + array([[0, 1, 0, 0],
                       [0, 0, 0, 0],
                       [0, 0, 0, 1],
                       [0, 0, 0, 0]]) * dt

range_std = 5. # meters
rk.R = np.diag([range_std**2], 1)
rk.R[1,1] = 0.01

rk.Q[0:2, 0:2] = Q_discrete_white_noise(2, dt=dt, var=0.1)
rk.Q[2,2] = 0.1
rk.P *= 50

xs, track = [], []
for i in range(int(20/dt)):
    radar.update_velocity(radar.vel)

    r = radar.get_range()
    angle = radar.get_angle()
    track.append((radar.pos, radar.vel, radar.alt))
    
    rk.update(array([r, angle]), HJacobian_at, hx)
    xs.append(rk.x)
    rk.predict()

xs = asarray(xs)
track = asarray(track)
time = np.arange(0, len(xs)*dt, dt)

#ekf_internal.plot_radar(xs, track, time)
plt.figure()
plt.title("track")
plt.plot(time, xs[:,0])
plt.plot(time, track[:,0], "-.")

plt.figure()
plt.title("velocity")
plt.plot(time, xs[:,1])
plt.plot(time, track[:,1], "-.")

plt.figure()
plt.title("altitude")
plt.plot(time, xs[:,2])
plt.plot(time, track[:,2], "-.")
plt.show()

