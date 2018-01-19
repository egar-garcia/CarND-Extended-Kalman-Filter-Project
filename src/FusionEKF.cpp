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

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  R_radar_ = MatrixXd(3, 3);

  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
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

  // Setting the acceleration noise components
	noise_ax_ = 9;
	noise_ay_ = 9;
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
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    //ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      VectorXd cartesian = tools.ConverFromPolarToCartesian(measurement_pack.raw_measurements_);
      cout << "** RADAR Measurement: " << endl; // TODO: Remove this egarg@
      ekf_.x_ << cartesian[0], cartesian[1], cartesian[2], cartesian[3];
    }
    else /* if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) */ {
      /**
      Initialize state.
      */
      cout << "** LASER Measurement: " << endl; // TODO: Remove this egarg@
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // TODO: Remove this egarg@
    cout << "init x_ = " << ekf_.x_ << endl;
    cout << "init P_ = " << ekf_.P_ << endl;

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // Computing the time elapsed between the current and previous measurements
  // dt - expressed in seconds
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;

  // Calculating some powers of -dt- to save computations
	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	// Modifying the F matrix integrating the time difference
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	// Modifying the Q matrix integrating the time difference
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

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    cout << "** RADAR Measurement: " << endl; // TODO: Remove this egarg@
    //VectorXd cartesian = tools.ConverFromPolarToCartesian(measurement_pack.raw_measurements_);
    //ekf_.x_ << cartesian[0], cartesian[1], cartesian[2], cartesian[3];
    VectorXd cartesian = tools.ConverFromPolarToCartesian(measurement_pack.raw_measurements_);
    VectorXd z(2);
    z << cartesian[0], cartesian[1];
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(z);
  } else {
    // Laser updates
    cout << "** LASER Measurement: " << endl; // TODO: Remove this egarg@
    //ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    VectorXd z(2);
    z << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1];
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
