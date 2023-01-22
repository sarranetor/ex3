import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt

def ekfilter(z, updateNumber):
    dt = 1.0
    j = updateNumber

    # Initialize State
    if updateNumber == 0: # First Update

        # compute position values from measurements
        # x = r*sin(b)
        temp_x = z[0][j]*np.sin(z[1][j]*np.pi/180)
        # y = r*cos(b)
        temp_y = z[0][j]*np.cos(z[1][j]*np.pi/180)

        # state vector
        # - initialize position values
        ekfilter.x = np.array([[temp_x],
                            [temp_y],
                            [0],
                            [0]])

        # state covariance matrix
        # - initialized to zero for first update
        ekfilter.P = np.array([[0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0]])

        # state transistion matrix
        # - linear extrapolation assuming constant velocity
        ekfilter.A = np.array([[1, 0, dt, 0],
                             [0, 1, 0, dt],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])


        # measurement covariance matrix
        ekfilter.R = z[2][j]

        # system error matrix
        # - initialized to zero matrix for first update
        ekfilter.Q = np.array([[0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0]])

        # residual and kalman gain
        # - not computed for first update
        # - but initialized so it could be output
        residual = np.array([[0, 0],
                      [0, 0]])
        K = np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0]])

    # Reinitialize State
    if updateNumber == 1: # Second Update

        prev_x = ekfilter.x[0][0]
        prev_y = ekfilter.x[1][0]


        # x = r*sin(b)
        temp_x = z[0][j]*np.sin(z[1][j]*np.pi/180)
        # y = r*cos(b)
        temp_y = z[0][j]*np.cos(z[1][j]*np.pi/180)
        temp_xv = (temp_x - prev_x)/dt
        temp_yv = (temp_y - prev_y)/dt

        # state vector
        # - reinitialized with new position and computed velocity
        ekfilter.x = np.array([[temp_x],
                            [temp_y],
                            [temp_xv],
                            [temp_yv]])

        # state covariance matrix
        # - initialized to large values
        # - more accurate position values can be used based on measurement
        #   covariance but this example does not go that far
        ekfilter.P = np.array([[100, 0, 0, 0],
                                 [0, 100, 0, 0],
                                 [0, 0, 250, 0],
                                 [0, 0, 0, 250]])

        # state transistion matrix
        # - linear extrapolation assuming constant velocity
        ekfilter.A = np.array([[1, 0, dt, 0],
                             [0, 1, 0, dt],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])

        # measurement covariance matrix
        # - provided by the measurment source
        ekfilter.R = z[2][j]

        # system error matrix
        # - adds 4.5 meter std dev in x and y position to state covariance
        # - adds 2 meters per second std dev in x and y velocity to state covariance
        # - these values are not optimized but work for this example
        var_v = 2
        ekfilter.Q = var_v * np.array([[20, 0, 0, 0],
                                        [0, 20, 0, 0],
                                        [0, 0, 4, 0],
                                        [0, 0, 0, 4]])

        
        #ekfilter.Q = np.array([[20, 0, dt, 0],
        #                                [0, 20, 0, dt],
        #                               [0, 0, 5, 0],
        #                                [0, 0, 0, 5]])

        # residual and kalman gain
        # - not computed for first update
        # - but initialized so it could be output
        residual = np.array([[0, 0],
                      [0, 0]])
        K = np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0]])
    if updateNumber > 1: # Third + Updates

      # Predict State Forward
      x_prime = ekfilter.A.dot(ekfilter.x)

      # Predict Covariance Forward
      P_prime = ekfilter.A.dot(ekfilter.P).dot(ekfilter.A.T) + ekfilter.Q

      # state to measurement transition matrix
      x1 = x_prime[0][0]
      y1 = x_prime[1][0]
      x_sq = x1*x1
      y_sq = y1*y1
      den = x_sq+y_sq
      den1 = np.sqrt(den)
      ekfilter.H = np.array([[  x1/den1,    y1/den1, 0, 0],
                           [y1/den, -x1/den, 0, 0]])

      ekfilter.HT = np.array([[x1/den1, y1/den],
                              [y1/den1, -x1/den],
                              [0, 0],
                              [0, 0]])

      # measurement covariance matrix
      ekfilter.R = z[2][j]

      # Compute Kalman Gain
      S = ekfilter.H.dot(P_prime).dot(ekfilter.HT) + ekfilter.R
      K = P_prime.dot(ekfilter.HT).dot(np.linalg.inv(S))

      # Estimate State
      # temp_z = current measurement in range and azimuth
      temp_z = np.array([[z[0][j]],
                         [z[1][j]]])

      # compute the predicted range and azimuth
      # convert the predicted cartesian state to polar range and azimuth
      pred_x = x_prime[0][0]
      pred_y = x_prime[1][0]

      sumSquares = pred_x*pred_x + pred_y*pred_y
      pred_r = np.sqrt(sumSquares)
      pred_b = np.arctan2(pred_x, pred_y) * 180/np.pi
      h_small = np.array([[pred_r],
                       [pred_b]])

      # compute the residual
      # - the difference between the state and measurement for that data time
      residual = temp_z - h_small

      # Compute new estimate for state vector using the Kalman Gain
      ekfilter.x = x_prime + K.dot(residual)

      # Compute new estimate for state covariance using the Kalman Gain
      ekfilter.P = P_prime - K.dot(ekfilter.H).dot(P_prime)

    return [ekfilter.x[0], ekfilter.x[1], ekfilter.x[2], ekfilter.x[3], ekfilter.P, K, residual, updateNumber];
# End of ekfilter

def exponential_average(x, y, alpha_x, alpha_y, n):
    x_av = np.zeros(n)
    y_av = np.zeros(n)

    x_av[0] = x[0]
    y_av[0] = y[0]
    for i in range(1,n):
        x_av[i] = x[i]*(1-alpha_x) + x_av[i-1]*alpha_x
        y_av[i] = y[i]*(1-alpha_y) + y_av[i-1]*alpha_y

    return x_av, y_av

def get_rmserror(xt, yt, xf, yf, percentage=1):
    x_rmse = np.sqrt(sum((xt - xf)**2) / len(xt))
    y_rmse = np.sqrt(sum((yt - yf)**2) / len(xt))

    return x_rmse, y_rmse
    

def getMeasurements(n, dt=1):
    t = np.arange(0, n*dt, dt)
    xinit = 20000
    yinit = 2000
    xvel = 20
    yvel = 0

    x = np.zeros(n)
    y = np.zeros(n)
    for i in range(n):
        #x[i] = xinit + xvel * t[i] + i*2
        #y[i] = yinit + 2e4*np.cos(4*i/n)

        # STO COGLIONE !
        x[i] = xinit + xvel * t[i] #ASSE VERTIVALE
        y[i] = yinit #ASSE ORIZZONTALE

    # misura e aggiungi errore
    r_stdev = 50 #m
    r_err = r_stdev * np.array([ np.random.randn() for i in range(n)])
    r = np.sqrt(x**2 + y**2) + r_err

    # misura e aggiungi errore
    angle_stdev = 0.01 #oppure questa è gia la varianza ..
    angle_err = angle_stdev * 180 / np.pi * np.array([ np.random.randn() for i in range(n)])
    angle = np.arctan2(x, y)
    angle = 180 / np.pi * angle + angle_err

    R = []
    for i in range(n):
        rr = np.array([[1, 0.0001], [0.0001, 1]])
        coef = 1
        rr[0,0] = r_stdev**2
        rr[1,1] = (angle_stdev * 180 / np.pi)**2
        R.append(rr)

    return [r, angle, R, 0], x, y, t

def get_possition(r, angle):
    x = r*np.sin(angle * np.pi/180) #ASSE VERTICALE
    y = r*np.cos(angle * np.pi/180) #ASSE ORIZZONTALE
    return x, y

n = 250
z, x, y, t = getMeasurements(n)
xf = []
yf = []
k = []

x_no_kalman, y_no_kalman = get_possition(z[0], z[1])
x_eav, y_eav = exponential_average(x_no_kalman, y_no_kalman, 0.2, 0.2, n)

"""zx_eav, zy_eav = exponential_average(z[0], z[1], 0.1, 0.1, n)
z[0] = zx_eav
z[1] = zy_eav"""

for iii in range(0, len(z[0])):
    # for each measurement, call the Extended Kalman Filter function
    ff = ekfilter(z, iii)
    xf.append(ff[0]) #ma sono invertiti??
    yf.append(ff[1])
    k.append(ff[5])

#compute error
xf_rmse, yf_rmse = get_rmserror(x, y, np.array(xf).reshape(-1), np.array(yf).reshape(-1))
xeav_rmse, yeav_rmse = get_rmserror(x, y, x_eav, y_eav)
xnf, ynf = get_rmserror(x, y, x_no_kalman, y_no_kalman)
print("kalman error (x, y): " + str(xf_rmse) + "  " + str(yf_rmse))
print("exp filter error (x, y): " + str(xeav_rmse) + "  " + str(yeav_rmse))
print("no filter error (x, y): " + str(xnf) + "  " + str(ynf))

f1, ax = plt.subplots()
plt.title("ship radar tracking")
plt.plot(x_no_kalman, y_no_kalman, "-.", label="from measure", linewidth=0.5)
plt.plot(x_eav, y_eav,  "r-.", label="eav", linewidth=1)
plt.plot(xf, yf, "-.", label="kalman", linewidth=2)
plt.plot(x, y, "-.", label="truth", linewidth=3)
ax.annotate(str(round(t[int(n/2)]/3600, 1)) + "h", xy=(x[int(n/2)], y[int(n/2)]))
ax.annotate(str(round(t[int(n-1)]/3600, 1)) + "h", xy=(x[int(n-1)], y[int(n-1)]))
plt.xlabel("x [meter]")
plt.ylabel("y [meter]")
plt.legend()

plt.figure()
plt.plot(np.array(xf).reshape(-1) - x, ".")


#f2, ax = plt.subplots()
#plt.plot(k[0])
#plt.plot(k[1])

plt.show()