#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
  * A helper method to convert from polar to cartesian coordinates.
  */
  VectorXd ConvertFromPolarToCartesian(const VectorXd& polar);

  /**
  * A helper method to convert from cartesian to polar coordinates.
  * Using a base polar cordinate to make sure than the angle has a radial distance
  * not greather than PI radians with respect to this one.
  */
  VectorXd ConvertFromCartesianToPolar(const VectorXd& cartesian, const VectorXd& polar_base);

  /**
  * Normalizes and angle with respect to a base angle,
  * to avoid that the radial distance to the base one is greather than PI radians.
  */
  float NormalizeAngle(const float& angle, const float& base_angle);
};

#endif /* TOOLS_H_ */
