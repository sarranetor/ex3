# Kalman Filter Applied to Radar Signal

## Executing progpam
Eigen library is used for Matrix operations, it is a code dependency. Eigen is a header only library, thus needs only its source files to be copied in code directory to run the program. 

    jhdsjssdskdhskjdh

## README Sections
1. d
2. fdf

# 1 Introduction



## 1.1 Problem Description 
Compute position (x,y) and velocity (x_vel, y_vel) of an object in motion thanks to a radar with Kalman filter. In **Appendix A** a more in depth description on Kalman theory is provided.

Radar System:
![Radar System](/cpp/kalman/figs/fig1.png)
Radar measurement output each T time:
- Radius r (meter): distance in meter between the radar and the object. Computed thanks the time to arrival.
- Azimut az (degree): angle between the object and the radar internal x axes.
- Velocity v (m/s): velocity of the object in the radial direction computed thanks to dopler effect.  

The signals coming from the radar are assumed to have a certain error compared to the true radius, azimuth and velocity. 

This error is assumed to be gaussian with a certain varience: **var_r**,  **var_az**,  **var_v**.

# 2 Model Description

To implement the Kalman filter is necessary to work out the process model and the observations model. In **Appendix A** a more in depth description on Kalman theory is provided.

## 2.1 Process Model
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

## 2.2 Observation Model

$z_k = [radius, azimuth, velocity]$

$z_k = h(x_k) + v_k$ where $z_k$ is the measure at time k, $x_k$ is the state at time k, $v_k$ is the measurement error, it is assumed gaussian.

$radius = \sqrt(x^2 + y^2)$

$azimuth = arctan(y/x)$

$velocity = \sqrt(x.vel^2 + y.vel^2)$

The relation is non linear thus the **Exstended** kalman filter needs to be used. This means to linearise the relationship using the taylor series with the first derivative. H will be the **Jacobian** matrix of $h(.)$ given the state variable.

    H = [y/sqrt(y*y + x*x), x/sqrt(y*y + x*x), 0, 0,
         x/(y*y + x*x),   - y/(y*y + x*x),     0, 0,
         0,     0,     y_vel/sqrt(y_vel^2+ x_vel^2), x_vel/sqrt(y_vel^2+ x_vel^2)]

## 2.3 Kalman filtering

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

# 3 Results 

The designed filter will be tested with different simulated measures and the filtered signal will be compared with the non-filtered one.

1. only vel_x=22 m/s - init_position=(x0=3000, y0=3000) - n_iteration=100

![Radar System](/cpp/kalman/figs/onlyvelx_xy_100p.png)
- Root Mean Square Error Kalman Filtered signal (x_err,y_err)=(6.74, 18.67) 
- Root Mean Square Error non filtered signal (x_err,y_err)=(35.63, 34.57)

![Radar System](/cpp/kalman/figs/onlyvelx_vxvy_100p.png)


2. only vel_y=22 m/s - init_position=(x0=3000, y0=3000) - n_iteration=100

![Radar System](/cpp/kalman/figs/onlyvely_xy_100p.png)
- Root Mean Square Error Kalman Filtered signal (x_err,y_err)=(19.70, 3.99) 
- Root Mean Square Error non filtered signal (x_err,y_err)=(43.44, 35.81)

![Radar System](/cpp/kalman/figs/onlyvely_vxvy_100p.png)


3. both vel vel_x=12 m/s  vel_y= 8 m/s - init_position=(x0=3000, y0=3000) - n_iteration=50

![Radar System](/cpp/kalman/figs/cothvelxy_xy_100p.png)
- Root Mean Square Error Kalman Filtered signal (x_err,y_err)=(16.62, 24.24) 
- Root Mean Square Error non filtered signal (x_err,y_err)=(36.53, 42.73)

![Radar System](/cpp/kalman/figs/cothvelxy_xy_50p.png)


4. only vel_y=5 m/s with random gaussian acceleration on x with 5m/s of standard deviation - init_position=(x0=3000, y0=3000) - n_iteration=100 

![Radar System](/cpp/kalman/figs/randaccx_xy_50p.png)
- Root Mean Square Error Kalman Filtered signal (x_err,y_err)=(26.74, 17.84) 
- Root Mean Square Error non filtered signal (x_err,y_err)=(33.46, 35.95)

![Radar System](/cpp/kalman/figs/randaccx_vxvy_50p.png)

# 4 Conclusions

An exstended kalman filter prototype for filtering data from a radar application has been designed and tested. Prototype performance have been tested with simulated signals of moving object in different situations. The prototype has been codeed in cpp using the Eigen library and tested.

The filter has been shown improvements over the measured data in all reported tests, though more simulations and more scenarions need to be tested to properly assess the filter performance.

The observation model law was non linear, which made necessary the usage of the exstended kalman filter. Future possible work could be implementing an uscended kalman kilter to tackle the non linearity and compare performance with the exstended kalman implementation. Also a constant acceleration process model could be implemented instead of a constant velocity one and results could be compared.

# 5 References

# 6 Appendix A

TODO