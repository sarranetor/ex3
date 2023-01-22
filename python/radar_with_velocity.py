import numpy as np
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
        # vel_x
        temp_vel_x = z[10][j]*np.sin(z[1][j]*np.pi/180)
        # vel_y
        temp_vel_y = z[10][j]*np.sin(z[1][j]*np.pi/180)

        # state vector
        # - initialize position values
        ekfilter.x = np.array([[temp_x],
                            [temp_y],
                            [temp_vel_x],
                            [temp_vel_y]])

        # state covariance matrix
        # - initialized to zero for first update
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
        ekfilter.R = z[2][j]

        # system error matrix
        # - initialized to zero matrix for first update
        ll = 0
        ekfilter.Q = np.array([[20, ll, ll, ll],
                                 [ll, 20, ll, ll],
                                 [ll, ll, 4, ll],
                                 [ll, ll, ll, 4]])

        # residual and kalman gain
        # - not computed for first update
        # - but initialized so it could be output
        residual = np.array([[0, 0],
                            [0, 0]])
        K = np.array([[0, 0, 0],
                      [0, 0, 0],
                      [0, 0, 0],
                      [0, 0, 0]])

    # Reinitialize State
        #no

    if updateNumber >= 1: # Third + Updates

      # Predict State Forward
      x_prime = ekfilter.A.dot(ekfilter.x)

      # Predict Covariance Forward
      P_prime = ekfilter.A.dot(ekfilter.P).dot(ekfilter.A.T) + ekfilter.Q

      # state to measurement transition matrix
      x1 = x_prime[0][0]
      y1 = x_prime[1][0]
      vx1 = x_prime[2][0]
      vy1 = x_prime[3][0]
      x_sq = x1*x1
      y_sq = y1*y1
      den = x_sq+y_sq
      den1 = np.sqrt(den)
      den_v = np.sqrt(vx1**2 + vy1**2)
      ekfilter.H = np.array([[  x1/den1,    y1/den1, 0, 0],
                           [y1/den, -x1/den, 0, 0],
                           [ 0, 0, vx1/den_v, vy1/den_v]])

      ekfilter.HT = np.array([[x1/den1, y1/den, 0],
                              [y1/den1, -x1/den, 0],
                              [0, 0, vx1/den_v],
                              [0, 0, vy1/den_v]])

      # measurement covariance matrix
      ekfilter.R = z[2][j]

      # Compute Kalman Gain
      S = ekfilter.H.dot(P_prime).dot(ekfilter.HT) + ekfilter.R
      K = P_prime.dot(ekfilter.HT).dot(np.linalg.inv(S))

      # Estimate State
      # temp_z = current measurement in range and azimuth
      temp_z = np.array([[z[0][j]],
                         [z[1][j]],
                         [z[10][j]]])

      # compute the predicted range and azimuth
      # convert the predicted cartesian state to polar range and azimuth
      pred_x = x_prime[0][0]
      pred_y = x_prime[1][0]
      pred_vx1 = x_prime[2][0]
      pred_vy1 = x_prime[3][0]

      sumSquares = pred_x*pred_x + pred_y*pred_y
      pred_r = np.sqrt(sumSquares)
      pred_b = np.arctan2(pred_x, pred_y) * 180/np.pi
      pred_v = np.sqrt(pred_vx1*pred_vx1 + pred_vy1*pred_vy1)
      h_small = np.array([[pred_r],
                       [pred_b],
                       [pred_v]])

      # compute the residual
      # - the difference between the state and measurement for that data time
      residual = temp_z - h_small

      # Compute new estimate for state vector using the Kalman Gain
      ekfilter.x = x_prime + K.dot(residual)

      # Compute new estimate for state covariance using the Kalman Gain
      ekfilter.P = P_prime - K.dot(ekfilter.H).dot(P_prime)

    return [ekfilter.x[0], ekfilter.x[1], ekfilter.P, ekfilter.x[2], ekfilter.x[3], K, residual];
# End of ekfilter


def gerMeasurements(n):
    # m are taken 1 time per second
    t = np.linspace(0, n, num=n)
    numOfMeasurements = len(t)
    # define x and y initial condition
    x = 3000
    y = 3000
    #velocity m/s 
    vel_x = 22
    vel_y = 0
    # create storage arrays for true position data
    t_time = []
    t_x = []
    t_y = []
    t_r = []
    t_b = []
    t_v = []
    # compute real position data
    for i in range(0, numOfMeasurements):
        # set delta 1 sec
        dT = 1.0
        t_time.append(t[i])
        # compute position in cartesian
        x = x + dT*vel_x 
        y = y + dT*vel_y
        #store
        t_x.append(x)
        t_y.append(y)
        t_v.append(np.sqrt(vel_x**2 + vel_y**2))
        # compute r and azimuth (or bearing) in degree
        temp = x*x + y*y
        r = np.sqrt(temp)
        t_r.append(r)
        b = np.arctan2(x,y) * 180/np.pi
        t_b.append(b)
    
    # create storage containers for polar measurements data
    m_r = []
    m_b = []
    m_vel = []
    m_cov = []
    # bearing standard deviation = 9 milliradiants in degree
    sig_b = 0.009*180/np.pi
    sig_r = 30 #range standard deviation = 30 meters
    sig_vel = 1 #m/2
    # storage containers for cartesian measurements
    m_x = []
    m_y = []
    for ii in range(0, len(t_time)):
        # compute error for each measurement
        # by taking the max bet .25 of defined std and 
        # the randomly generated normal error, it guarantiees an error
        temp_sig_b = sig_b * np.random.randn()
        temp_sig_r = sig_r * np.random.randn()
        temp_sig_vel = sig_vel * np.random.randn()
        # save the measurement as a function of the true value and the error
        temp_b = t_b[ii] + temp_sig_b
        temp_r = t_r[ii] + temp_sig_r
        temp_vel = t_v[ii] + temp_sig_vel
        # save
        m_b.append(temp_b)
        m_r.append(temp_r)
        m_vel.append(temp_vel)
        """m_cov.append(np.array([[temp_sig_r*temp_sig_r, 0],
                                [0, temp_sig_b*temp_sig_b]]))"""

        m_cov.append(np.array([[sig_r*sig_r, 0, 0],
                                [0, sig_b*sig_b, 0],
                                [0, 0, sig_vel**sig_vel]]))
        m_x.append(temp_r * np.sin(temp_b*np.pi/180))
        m_y.append(temp_r*np.cos(temp_b*np.pi/180))

    return [m_r, m_b, m_cov, t_r, t_b, t_time, t_x, t_y, m_x, m_y, m_vel] #velocity added last cause of time


def get_rmserror(xt, yt, xf, yf, start_percentage=0):
    l = len(xt)
    st_p_i = int(start_percentage * l)
    x_rmse = np.sqrt(sum((xt[st_p_i:l] - xf[st_p_i:l] )**2) / len(xt[st_p_i:l] ))
    y_rmse = np.sqrt(sum((yt[st_p_i:l]  - yf[st_p_i:l] )**2) / len(xt[st_p_i:l] ))

    return x_rmse, y_rmse

f_x = []
f_y = []
f_x_sig = []
f_y_sig = []
f_xv = []
f_yv = []
f_xv_sig = []
f_yv_sig = []

z = gerMeasurements(30)
for iii in range(0, len(z[0])):
    f = ekfilter(z, iii)
    f_x.append(f[0])
    f_y.append(f[1])
    f_xv.append(f[3])
    f_yv.append(f[4])
    f_x_sig.append(np.sqrt(f[2][0][0]))
    f_y_sig.append(np.sqrt(f[2][1][1]))


plot1 = plt.figure()
plt.grid(True)
plt.plot(z[5], z[3])
plt.scatter(z[5], z[0])
plt.title('Actual Range vs Measured Range')
plt.legend(['Ship Actual Range', 'Ship Measured Range'])
plt.ylabel('Range (mters)')
plt.xlabel('Update Number')

plot2 = plt.figure()
plt.grid(True)
plt.plot(z[5], z[4])
plt.scatter(z[5], z[1])
plt.title('Actual Azimut vs Measured Azimut')
plt.legend(['Ship Actual Azimuth', 'Ship Measured Azimuth'])
plt.ylabel('Azimuth (degrees)')
plt.xlabel('Update Number')

plot3 = plt.figure()
plt.grid(True)
plt.plot(z[5], f_xv)
plt.plot(z[5], f_yv)
plt.title('Velocity Estimate On Each Measurement Update \n', fontweight="bold") 
plt.legend(['x Velocity Estimate', 'Y Velocity estimate'])

# Compute Range error
e_x_err = []
e_x_3sig = []
e_x_3sig_neg = []
e_y_err = []
e_y_3sig = []
e_y_3sig_neg = []
for m in range(0, len(z[0])):
    e_x_err.append(f_x[m] - z[6][m])
    e_x_3sig.append(3*f_x_sig[m])
    e_x_3sig_neg.append(-3*f_x_sig[m])
    e_y_err.append(f_y[m] - z[7][m])
    e_y_3sig.append(3*f_y_sig[m])
    e_y_3sig_neg.append(-3*f_y_sig[m])


plot4 = plt.figure()
plt.grid(True)
line1 = plt.scatter(z[5], e_x_err)
line2, = plt.plot(z[5], e_x_3sig, color='green')
plt.plot(z[5], e_x_3sig_neg, color='green')
plt.ylabel('Position Error (meters)')
plt.xlabel('Update Number')
plt.title('X Position Estimate Error Containment \n', fontweight='bold')
plt.legend([line1, line2,], ['XPosition Error', '3 Sigme Error Bound'])

plot5 = plt.figure()
plt.grid(True)
yline1 = plt.scatter(z[5], e_y_err)
yline2, = plt.plot(z[5], e_y_3sig, color='green')
plt.plot(z[5], e_y_3sig_neg, color='green')
plt.ylabel('Position Error (meters)')
plt.xlabel('Update Number')
plt.title('Y Position Estimate Error Containment \n', fontweight='bold')
plt.legend([yline1, yline2,], ['Y Position Error', '3 Sigme Error Bound'])

plot6 = plt.figure()
plt.grid(True)
plt.plot(f_x, f_y, "-.", label='kalman')
plt.plot(z[6], z[7], "-.", label='truth')
plt.plot(z[8], z[9], "-.", label='measured')
plt.legend()
plt.title('xy kalman vs xy measurement vs xy truth \n', fontweight='bold')

f_x = np.array(f_x).reshape(-1)
f_y = np.array(f_y).reshape(-1)
x_tr = np.array(z[6]).reshape(-1)
y_tr = np.array(z[7]).reshape(-1)
x_m = np.array(z[8]).reshape(-1)
y_m = np.array(z[9]).reshape(-1)

print("radius data: " + str(z[0]))
print("azimuth data: " + str(z[1]))

print(str(get_rmserror(f_x, f_y, x_tr, y_tr, start_percentage=0.2)) + " --> kalman rmse against truth")
print(str(get_rmserror(x_m, y_m, x_tr, y_tr, start_percentage=0.2)) + " --> meaure no filter rmse against truth")


plt.show()