#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // Checking the validity of the following inputs:
  //  * The estimation vector size should not be zero
  //  * The estimation vector size should equal ground truth vector size
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // Accumulating squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Calculating the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);

  // Recovering state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Pre-computing a set of terms to avoid repeated calculation
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = (c1 * c2);

  // Checking division by zero
  if(fabs(c1) < 0.0001) {
	  cout << "CalculateJacobian () - Error - Division by Zero" << endl;
	  return Hj;
  }

  // Computing the Jacobian matrix
  Hj << (px / c2), (py / c2), 0, 0,
        -(py / c1), (px / c1), 0, 0,
        py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

  return Hj;
}

VectorXd Tools::ConverFromPolarToCartesian(const VectorXd& polar) {
  float rho = polar[0];
  float phi = polar[1];
  float rho_dot = polar[2];

  VectorXd cartesian(4);
  // Converting the polar coordinates to cartesian
  cartesian << rho * cos(phi), rho * sin(phi), rho_dot * cos(phi), rho_dot * sin(phi);

  return cartesian;
}
