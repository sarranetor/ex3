#include <iostream>
#include <cmath>
#include <vector>
#include "../eigen/Eigen/Dense"
#include "types.hpp"
#include "data.hpp"
#include "radar_ekf.hpp"
// Should be added to the compiler command instead. It is ok though for the purpose of this demo.
#include "radar_ekf.cpp" 

/* get measurement noise covariance */
Matrix33d get_R();

/* process noise covariance */
Matrix44d get_Q();

/* get error covariance matrix at time zero */
Matrix44d get_P0();


int main()
{
  // measurement sampling time
  double dt = 1.0;
  Matrix33d R = get_R();
  Matrix44d Q = get_Q();
  Matrix44d P0 = get_P0();
  
  // create storage matrix for a posteriori predicted states 
  Eigen::MatrixXd x_out;
  x_out.resize(4, get_n_measurements());
  // get measurements
  Eigen::MatrixXd zk = get_zk();
  // instanciate RadarEKF class with P0, Q, R, dt values
  RadarEKF Ekalman_filter(P0, Q, R, dt);

  /* Start Filtering Signal */
  // first measure is needed to initialize the filter
  x_out.col(0) = Ekalman_filter.initialize(zk.col(0));
  
  // feed to the filter measurement data sample by sample
  for (int i=1; i<get_n_measurements(); i++) 
  {
    // a posteriori predicted state at each k step
    x_out.col(i) = Ekalman_filter.predict_xk(zk.col(i));
  }

  // print filter results for every step k
  std::cout << "Xk states EKFiltering Output: " << std::endl;
  std::cout << "y: " << x_out(0, Eigen::all) <<std::endl;
  std::cout << "x: " << x_out(1, Eigen::all) <<std::endl;
  std::cout << "vy: " << x_out(2, Eigen::all) <<std::endl;
  std::cout << "vx: " << x_out(3, Eigen::all) <<std::endl;

  return 0;
}


Matrix33d get_R() 
{
  double var_r = measure_stdv.radius * measure_stdv.radius;
  double var_az = measure_stdv.azimuth * measure_stdv.azimuth;
  double var_vel = measure_stdv.velocity * measure_stdv.velocity;

  Matrix33d R;
  R << var_r, 0, 0,
     0, var_az, 0,
     0, 0, var_vel; 

  return R;
};

Matrix44d get_Q() 
{
  Matrix44d Q;
  Q << 20, 0, 0, 0, // hp 4.5 m std
       0, 20, 0, 0,
       0, 0, 4, 0, // hp 2 m/s std
       0, 0, 0, 4; 

  return Q;
};

Matrix44d get_P0() 
{
  Matrix44d P0;
  P0 << 100, 0, 0, 0,
        0, 100, 0, 0,
        0, 0, 250, 0,
        0, 0, 0, 250;

  return P0;
};