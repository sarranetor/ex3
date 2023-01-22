#include <iostream>
#include "Eigen/Dense"
#include "Eigen/LU"
 
using Eigen::MatrixXd;
 
int main()
{
  MatrixXd m(2,2);
  MatrixXd m_inv(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  //std::cout << "m" << m << std::endl;
 
  std::cout << "m_inv" << m.inverse() << std::endl;

  // std::cout << "I" << m  << std::endl;

  MatrixXd m2 = MatrixXd::Random(3,3);
  std::cout << "size" << m2.size() << std::endl;
  std::cout << "Here is the matrix m2:" << std::endl << m2 << std::endl;
  std::cout << "Its inverse is:" << std::endl << m2.inverse()*m2 << std::endl;
  
  return 0;
}