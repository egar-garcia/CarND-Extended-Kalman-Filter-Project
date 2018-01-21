#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
                        Eigen::MatrixXd &Q_in, Eigen::MatrixXd &H_in,
                        Eigen::MatrixXd &R_laser_in, Eigen::MatrixXd &R_radar_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Q_ = Q_in;
  R_laser_ = R_laser_in;
  R_radar_ = R_radar_in;
}

void KalmanFilter::Predict() {
  // Applying Kalman filter prediction algorithm
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  DoMeasurementUpdate(y, H_, R_laser_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd y = z - tools.ConvertFromCartesianToPolar(x_, z);
  MatrixXd Hj = tools.CalculateJacobian(x_);
  DoMeasurementUpdate(y, Hj, R_radar_);
}

void KalmanFilter::DoMeasurementUpdate(const VectorXd &y, const MatrixXd &H, const MatrixXd &R) {
  // Applying Kalman filter measurement update algorithm
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd K = P_ * Ht * S.inverse();

  // New estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H) * P_;
}
