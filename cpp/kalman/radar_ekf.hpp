#include <cmath>
#include "../../eigen/Eigen/Dense"
#include "types.hpp"

#ifndef RADAR_EKF_H
#define RADAR_EKF_H

/* 
   RadarEKF is a class for applying Exstended Kalman Filter to Radar data.
   It is assumed that the radar measures radius (radial distance to the object), 
   azimith angle and velocity in radial direction. 
   ... state desctiption data desctiprion ..
*/
class RadarEKF 
{
  // measurement sampling time interval [sec]
  double _dt;
  // prior estimate error covariance matrix      
  Matrix44d _Pk;
  // predicted error covariance matrix      
  Matrix44d _Pk_1;
  // measurement noise covariancematrix
  Matrix33d _R;
  // process noise covariance matrix
  Matrix44d _Q;
  // state to measurement Jacobian matrix
  Matrix34d _Hk;
  // state transition matrix assuming constant velocity
  Matrix44d _A; 
  // kamlan gains at step k
  Matrix43d _K; 
  Matrix31d _residual; 
  Matrix33d _S;
  Matrix31d _zk_prediction;

  Matrix41d _xk_posteriori; 
  // previous state (y, x, y_vel, x_vel)
  Matrix41d _xk;   
  // system model next (y, x, y_vel, x_vel)
  Matrix41d _xk_1;  

  /* Recompute state to measurement Jacobian matrix for step k */
  void _update_H();

  /* 
    State to measurement non liner transformation
    @param xk 
    @return zk computed
  */
  Matrix31d _h(Matrix41d &xk);


  public:

  /*
    RadarEKF constructor
    @param dt in seconds
  */
  RadarEKF(double dt);

  /*
    RadarEKF constructor
    @param P
    @param Q
    @param R
    @param dt in seconds
  */
  RadarEKF(Matrix44d &P, Matrix44d &Q, Matrix33d &R, double dt); 

  // void set_p(Matrix44d P);
  // void set_q(Matrix44d Q);
  // void set_r(Matrix33d R);

  /* 
    Initialize state matrix fot time zero measurement data
    @param zk time zero data (radiud in meter, azimith in degree, radial velocity in m/s)
    @return x0 state vector at time zero (y, x, y_vel, x_vel)
  */
  Matrix41d initialize(Matrix31d zk);

  /* 
    Compute and predict a posteriori state based on computed Kalman gains for step k
    @param zk measurement data at time k (radiud in meter, azimith in degree, radial velocity in m/s)
    @return xk a posteriori state vector based on computed Kalman gains for step k (y, x, y_vel, x_vel)
  */
  Matrix41d predict_xk(Matrix31d zk); 
};

#endif