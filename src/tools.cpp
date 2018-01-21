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

    // Checking the validity of the following inputs:
    //  * The estimation vector size should not be zero
    //  * The estimation vector size should equal ground truth vector size
    if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
        cerr << "Invalid estimation or ground_truth data" << endl;
        rmse << -1, -1, -1, -1; // -1 to indicate an undefined value
        return rmse;
    }

    rmse << 0, 0, 0, 0;

    // Accumulating squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i) {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    // Calculating the mean
    rmse = rmse / estimations.size();

    // Calculating the squared root
    rmse = rmse.array().sqrt();

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
    if (c1 < 0.0001) {
        throw "CalculateJacobian () - Error - Division by Zero";
    }

    // Computing the Jacobian matrix
    Hj << (px / c2), (py / c2), 0, 0,
          -(py / c1), (px / c1), 0, 0,
          py * (vx * py - vy * px) / c3, px * (vy * px - vx * py) / c3, px / c2, py / c2;

    return Hj;
}

VectorXd Tools::ConvertFromPolarToCartesian(const VectorXd& polar) {
    float rho = polar[0];
    float phi = polar[1];
    float rho_dot = polar[2];

    VectorXd cartesian(4);
    // Converting the polar coordinates to cartesian
    cartesian << rho * cos(phi), rho * sin(phi), rho_dot * cos(phi), rho_dot * sin(phi);

    return cartesian;
}

VectorXd Tools::ConvertFromCartesianToPolar(const VectorXd& cartesian, const VectorXd& polar_base) {
    float px = cartesian[0];
    float py = cartesian[1];
    float vx = cartesian[2];
    float vy = cartesian[3];

    float rho = sqrt(px * px + py * py);

    // Checking division by zero
    if (rho < 0.0001) {
        throw "ConvertFromCartesianToPolar () - Error - Division by Zero";
    }

    VectorXd polar(3);
    polar << rho,
             NormalizeAngle(atan2(py, px), polar_base[1]),
             (px * vx + py * vy) / rho;

    return polar;
}

float Tools::NormalizeAngle(const float& angle, const float& base_angle) {
    float norm_angle = angle;

    // Reducing the angle by 2*PI (a full lap) incremeps
    // if radial distance from angle to base angle is bigger than pi
    while (norm_angle - base_angle > M_PI) {
        norm_angle -= 2 * M_PI;
    }
    // Augmenting the angle by 2*PI (a full lap) increments
    // if radial distance from base angle to angle is bigger than pi
    while (base_angle - norm_angle > M_PI) {
        norm_angle += 2 * M_PI;
    }

    return norm_angle;
}
