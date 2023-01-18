#include <cmath>
#include "../eigen/Eigen/Dense"
#include "types.hpp"

#ifndef RADAR_EKF_H
#define RADAR_EKF_H

/* 
  RadarEKF is a class for applying Exstended Kalman Filter to Radar data.
  It is assumed that the radar measures radius r (radial distance to the object), 
  azimith az angle and velocity vel in radial direction. 
   
  - meaure vector zk = [r, az, vel] in [m , degree, m/s] units
  - state vector xk = [y, x, y_vel, x_vel] in [m, m, m/s, m/s] units
*/
class RadarEKF 
{
  // measurement sampling time interval [sec]
  double _dt;
  // prior estimate error covariance matrix      
  Matrix44d _Pk;
  // predicted error covariance matrix      
  Matrix44d _Pk_1;
  // measurement noise covariance matrix
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
  Matrix31d _h(const Matrix41d &xk);


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
  RadarEKF(const Matrix44d &P, const Matrix44d &Q, const Matrix33d &R, double dt); 

  /* Default destructor */
  ~RadarEKF() = default;

  /* Set error covariance matrix P */
  void set_p(Matrix44d &P);

  /* Set process noise covariance matrix Q */
  void set_q(Matrix44d &Q);

  /* Set measurement noise covariance matrix R */
  void set_r(Matrix33d &R);

  /* 
    Initialize state matrix fot time zero measurement data
    @param zk time zero data (radiud in meter, azimith in degree, radial velocity in m/s)
    @return x0 state vector at time zero (y, x, y_vel, x_vel)
  */
  Matrix41d initialize(const Matrix31d &zk);

  /* 
    Compute and predict a posteriori state based on computed Kalman gains for step k
    @param zk measurement data at time k (radiud in meter, azimith in degree, radial velocity in m/s)
    @return xk a posteriori state vector based on computed Kalman gains for step k (y, x, y_vel, x_vel)
  */
  Matrix41d predict_xk(const Matrix31d &zk); 
};

RadarEKF::RadarEKF(double dt) 
{ 
  _dt = dt;

  _A << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

  _Hk.setZero(3,4);
  _Pk.setZero(4,4);
  _Pk_1.setZero(4,4);
  _S.setZero(3, 3);
  _K.setZero(4, 3);
};

RadarEKF::RadarEKF(const Matrix44d &P, const Matrix44d &Q, const Matrix33d &R, double dt)
  : RadarEKF(dt) 
{ 
  _Pk = P; 
  _Q = Q;
  _R = R;     
};

void RadarEKF::set_p(Matrix44d &P) { 
  _Pk = P;    
};

void RadarEKF::set_q(Matrix44d &Q) { 
  _Q = Q;    
};

void RadarEKF::set_r(Matrix33d &R) { 
  _R = R;     
};

Matrix41d RadarEKF::initialize(const Matrix31d &zk) 
{
  double r = zk(0);
  double azimuth = zk(1);
  double velocity = zk(2);

  double y = r * std::sin(azimuth * PI / 180);
  double x = r * std::cos(azimuth * PI / 180);
  double y_vel = velocity * std::sin(azimuth* PI / 180);
  double x_vel = velocity * std::cos(azimuth* PI / 180);

  // initialize state
  _xk << y, x, y_vel, x_vel;

  return _xk;
};

Matrix41d RadarEKF::predict_xk(const Matrix31d &zk) 
{
  /* Time Update/Predict */
  // project the state ahead
  _xk_1 = _A*_xk;
  // project the error covariance ahead
  _Pk_1 = _A*_Pk* _A.transpose() + _Q;

  /* Measurement Update */
  // recompute Jacobian matrix
  _update_H();
  // Compute Kalman Gains
  _S = _Hk*_Pk_1*_Hk.transpose() + _R;
  _K = _Pk_1*_Hk.transpose()*_S.inverse();

  // get predicted zk 
  _zk_prediction = _h(_xk_1);
  // Update estimate with measurement zk
  _xk_posteriori = _xk_1 + _K*(zk - _zk_prediction);
  // Update the error covariance 
  _Pk = _Pk_1 - _K*_Hk*_Pk_1;

  // Update k state with current valeus
  _xk = _xk_posteriori;
  
  return _xk_posteriori;
};

void RadarEKF::_update_H() 
{
  double y = _xk_1(0,0);
  double x = _xk_1(1,0);
  double y_vel = _xk_1(2,0);
  double x_vel = _xk_1(3,0);

  // radius state-to-measurement Jacobian
  _Hk(0,0) = y / std::sqrt(y*y + x*x);
  _Hk(0,1) = x / std::sqrt(y*y + x*x);

  // azimuth state-to-measurement Jacobian
  _Hk(1,0) = x / (y*y + x*x);
  _Hk(1,1) = - y / (y*y + x*x);

  // velocity detected by radar state-to-measurement Jacobian
  _Hk(2,2) = y_vel / std::sqrt(y_vel*y_vel + x_vel*x_vel);
  _Hk(2,3) = x_vel / std::sqrt(y_vel*y_vel + x_vel*x_vel);
};

Matrix31d RadarEKF::_h(const Matrix41d &xk) 
{
  double y = xk(0,0);
  double x = xk(1,0);
  double y_vel = xk(2,0);
  double x_vel = xk(3,0);

  double radius = std::sqrt(y*y + x*x);
  double azimuth = std::atan2(y,x) * 180 / PI;
  double velocity = std::sqrt(y_vel*y_vel + x_vel*x_vel);

  return Matrix31d(radius, azimuth, velocity);
};
 

#endif