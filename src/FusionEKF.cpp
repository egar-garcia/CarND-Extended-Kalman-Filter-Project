#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // Setting the acceleration noise components
  noise_ax_ = 9;
  noise_ay_ = 9;

  // Setting the measurement matrix H
  ekf_.H_ = MatrixXd(2, 4);
  ekf_.H_ << 1, 0, 0, 0,
             0, 1, 0, 0;

  // Setting the measurement covariance matrix R
  // for laser and radar
  ekf_.R_laser_ = MatrixXd(2, 2);
  ekf_.R_laser_ << 0.0225, 0,
                   0, 0.0225;

  ekf_.R_radar_ = MatrixXd(3, 3);
  ekf_.R_radar_ << 0.09, 0, 0,
                   0, 0.0009, 0,
                   0, 0, 0.09;

  // The 4D state vector
  ekf_.x_ = VectorXd(4);

  // Initial state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  // Initial transition matrix F
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // Initial process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // Initializing the state ekf_.x_ with the first measurement.

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Converting radar from polar to cartesian coordinates and initializing state.
      VectorXd cartesian = tools.ConvertFromPolarToCartesian(measurement_pack.raw_measurements_);
      ekf_.x_ << cartesian[0], cartesian[1], cartesian[2], cartesian[3];
    }
    else /* if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) */ {
      // Initializing state directly from the messure.
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // Done with initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Computing the time elapsed between the current and previous measurements
  // dt - expressed in seconds
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;

  // Calculating some powers of -dt- to save computations
	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	// Updating the (state transition) F matrix integrating the time difference
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	// Updating the (process noise covariance) Q matrix integrating the time difference
  ekf_.Q_(0, 0) = dt_4 / 4 * noise_ax_;
  ekf_.Q_(0, 2) = dt_3 / 2 * noise_ax_;
  ekf_.Q_(1, 1) = dt_4 / 4 * noise_ay_;
  ekf_.Q_(1, 3) = dt_3 / 2 * noise_ay_;
  ekf_.Q_(2, 0) = dt_3 / 2 * noise_ax_;
  ekf_.Q_(2, 2) = dt_2 * noise_ax_;
  ekf_.Q_(3, 1) = dt_3 / 2 * noise_ay_;
  ekf_.Q_(3, 3) = dt_2 * noise_ay_;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  // Using the sensor type to perform the update step.
  // Updating the state and covariance matrices.

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    VectorXd z(3);
    z << measurement_pack.raw_measurements_[0],
         measurement_pack.raw_measurements_[1],
         measurement_pack.raw_measurements_[2];
    ekf_.UpdateEKF(z);
  } else {
    VectorXd z(2);
    z << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1];
    ekf_.Update(z);
  }

  // Commenting next 2 lines to save time by avoiding I/O, but feel free to uncomment them to debug
  //cout << "x_ = " << endl << ekf_.x_ << endl;
  //cout << "P_ = " << endl << ekf_.P_ << endl;
}
