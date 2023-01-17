# Kalman Filter Applied to Radar Signal

## Executing progpam
Eigen library is used for Matrix operations, it is a code dependency. Eigen is a header only library, thus needs only its source files to be copied in code directory to run the program. 

    jhdsjssdskdhskjdh

# Introduction



## Problem Description 
Compute position (x,y) and velocity (x_vel, y_vel) of an object in motion thanks to a radar with Kalman filter. In **Appendix A** a more in depth description on Kalman theory is provided.

Radar System:
![Radar System](/cpp/kalman/figs/fig1.png)
Radar measurement output each T time:
- Radius r (meter): distance in meter between the radar and the object. Computed thanks the time to arrival.
- Azimut az (degree): angle between the object and the radar internal x axes.
- Velocity v (m/s): velocity of the object in the radial direction computed thanks to dopler effect.  

The signals coming from the radar are assumed to have a certain error compared to the true radius, azimuth and velocity. 

This error is assumed to be gaussian with a certain varience: **var_r**,  **var_az**,  **var_v**.

# Model Description

To implement the Kalman filter is necessary to work out the process model and the observations model. In **Appendix A** a more in depth description on Kalman theory is provided.

## Process Model
A constant velocity model is assumed. Thus:

$x = x_0 + v*dt$


State Variables: $x_k = [y_k, x_k, y'_k, x'_k]$
where yk and xk is the position in cartesian coordinates and y'k and x'k is the velocity in cartesian coordinates.

State transition equation:

$x[k+1] = A  x[k] + w_k$

Thus the state transition matrix A is

    A = [1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1]

Where $w_k$ is the process noise. it is assumed gaussian.

## Observation Model

$z_k = [radius, azimuth, velocity]$

$z_k = h(x_k) + v_k$ where $z_k$ is the measure at time k, $x_k$ is the state at time k, $v_k$ is the measurement error, it is assumed gaussian.

$radius = \sqrt(x^2 + y^2)$

$azimuth = arctan(y/x)$

$velocity = \sqrt(x.vel^2 + y.vel^2)$

The relation is non linear thus the **Exstended** kalman filter needs to be used. This means to linearise the relationship using the taylor series with the first derivative. H will be the **Jacobian** matrix of $h(.)$ given the state variable.

    H = [y/sqrt(y*y + x*x), x/sqrt(y*y + x*x), 0, 0,
         x/(y*y + x*x),   - y/(y*y + x*x),     0, 0,
         0,     0,     y_vel/sqrt(y_vel^2+ x_vel^2), x_vel/sqrt(y_vel^2+ x_vel^2)]

## Kalman filtering

Algorithm:

- Presiction Step:
    - Given $x^-$[k-1] state estimation at time k-1 and given $P^-$[k-1] error covariance matrix estimation at time k-1
    - Project state ahead $x[k] = A  x[k-1]$
    - Project error covariance ahead $P[k] = A  P^-[k-1] A^t + Q$
  

-  Update Step:
   - Update H_k .. *Exstended part* ..  
   -  Update kalman gains $K_k = P_k H^t (H P_k H^t + R)^-1$
   -  Update estimate with $z_k$ measurement at time k: $x^-[k]  = x_k + K_k(z_k - Hx_k)$ 
   -  Update error covariance matrix $P^-[k]=(I - K_k H) P_k$


Where **R** is the measurement noise covariance matrix, usually measured prior the operation of the filter and dependent on the measurement system error. In this case the measurement error of our radar system, thus R is set to:

     R = [var_r, 0, 0,
          0, var_az, 0,
          0, 0, var_vel] 

Where **Q** is the process error covariance matrix. It provides and indication on how the prediction process model can be wrong between step k and k+1. In this example Q is set to:

     Q = [20, 0, 0, 0,  // hp ..
          0, 20, 0, 0,
          0, 0, 4, 0,  // hp ..
          0, 0, 0, 4] 

# Results 

The designed filter will be tested with different simulated measures and the filtered signal will be compared with the non-filtered one.

# only velx 22 m/s x0=3000 y0=3000 n=100
(6.743953761836919, 18.67847050290694) --> kalman rmse against truth
(35.639784238092545, 34.57783895063594) --> meaure no filter rmse against truth

# only vely 22 m/s x0=3000 y0=3000 n=100
(19.706376061088925, 3.9981673069769688) --> kalman rmse against truth
(43.44241856775875, 35.81681433507567) --> meaure no filter rmse against truth

# both velx 12 m/s vely 8 m/s x0=3000 y0=3000 n=50
(16.62784941084549, 24.243956950332368) --> kalman rmse against truth
(36.530692004235455, 42.73814882119287) --> meaure no filter rmse against truth

# accx np.random.rand()*5 vely 5 x0 3000 y0 3000 n=50
(26.747478897271915, 17.847742212429917) --> kalman rmse against truth
(33.462418327117206, 35.955291694819856) --> meaure no filter rmse against truth

# Conclusions

# References

# Appendix A