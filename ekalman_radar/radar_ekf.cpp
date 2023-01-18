#include "radar_ekf.hpp"


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

RadarEKF::RadarEKF(Matrix44d &P, Matrix44d &Q, Matrix33d &R, double dt)
  : RadarEKF(dt) 
{ 
  _Pk = P; 
  _Q = Q;
  _R = R;     
};

Matrix41d RadarEKF::initialize(Matrix31d zk) 
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

Matrix41d RadarEKF::predict_xk(Matrix31d zk) 
{
  /* Time Update/Predict */
  // project the state ahead
  _xk_1 = _A*_xk;
  // project the error covariance ahead
  _Pk_1 = _A*_Pk* _A.transpose() + _Q;

  /* Measurement Update*/
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

Matrix31d RadarEKF::_h(Matrix41d &xk) 
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
 