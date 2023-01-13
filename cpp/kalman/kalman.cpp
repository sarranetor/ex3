#include <iostream>
#include <math>
#include "data.hpp"
#include "Eigen/Dense"
#include "Eigen/LU"

// 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
#define PI 3.14159265
 
using Eigen::MatrixXd;
typedef Eigen::Matrix<double, 4, 4> Matrix44d;
typedef Eigen::Matrix<double, 4, 1> Matrix41d;
typedef Eigen::Matrix<double, 3, 3> Matrix33d;
typedef Eigen::Matrix<double, 3, 1> Matrix31d;

class RadarKalman {
  Eigen::Matrix<double, 4, 4> _Pk; // typedef can be used yeah
  Eigen::Matrix<double, 4, 4> _Pk_1;
  Eigen::Matrix<double, 3, 3> _R;
  Eigen::Matrix<double, 4, 4> _Q;
  Eigen::Matrix<double, 4, 4> _Hk;
  Eigen::Matrix<double, 4, 4> _S; // I don't know its dimention
  Eigen::Matrix<double, 4, 4> _A; 
  Eigen::Matrix<double, 4, 3> _K; 
  Eigen::Matrix<double, 3, 1> _residual; 

  Eigen::Matrix<double, 4, 1> _xk_posteriori; 
  Eigen::Matrix<double, 4, 1> _xk;   
  Eigen::Matrix<double, 4, 1> _xk_1;  

  double _dt;

  //
  void _update_H();
  //
  MatrixXd _h(MatrixXd &xk);

  public:
  RadarKalman(double dt);
  RadarKalman(MatrixXd P, MatrixXd Q, MatrixXd R, double dt);

  void set_p(MatrixXd P);
  void set_q(MatrixXd Q);
  void set_r(MatrixXd R);

  MatrixXd initialize(MatrixXd &zk);
  MatrixXd predict_xk(MatrixXd &zk); //returns the state 

};

MatrixXd RadarKalman::predict_xk(MatrixXd &zk) {

  _xk_1 = _A*_xk;
  _Pk_1 = _A*_Pk* _A.transpose() + _Q;

  _update_h();

  _S = _Hk*_Pk_1*_Hk.transpose() + _R;
  _K = _pk_1*_Hk.transpose()*_S.inverse();

  Matrix31d zk_prediction = _h(_xk_1);
  _xk_posteriori = _xk_1 + _K*(zk - zk_prediction);

  _Pk = _Pk_1 - K*_Hk*_Pk_1;

  return _xk_posteriori;
};

void RadarKalman::_update_H() {
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

MatrixXd RadarKalman::_h(MatrixXd &xk) {
  double y = xk(0,0);
  double x = xk(1,0);
  double y_vel = xk(2,0);
  double x_vel = xk(3,0);

  double radius = std::sqrt(y*y + x*x);
  double azimuth = std::atan2(y,x) * 180 / PI;
  double velocity = std::sqrt(y_vel*y_vel + x_vel*x_vel);

  return MatrixXd(radius, azimuth, velocity);
};
 
int main()
{

}