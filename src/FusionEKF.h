#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
  public:
    /**
     * Constructor.
     */
    FusionEKF();

    /**
     * Destructor.
     */
    virtual ~FusionEKF();

    /**
     * Runs the whole flow of the Kalman Filter from here.
     */
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    /**
     * Kalman Filter update and prediction math lives in here.
     */
    KalmanFilter ekf_;

  private:
    // Check whether the tracking toolbox was initialized or not (first measurement)
    bool is_initialized_;

    // Previous timestamp
    long long previous_timestamp_;

    // Tool object used to do conversions between polar and cartesian coorinates
    Tools tools;

    // acceleration noise components
    float noise_ax_;
    float noise_ay_;
};

#endif /* FusionEKF_H_ */
