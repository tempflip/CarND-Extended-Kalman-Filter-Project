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
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  ekf_.Init();
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  Tools tools;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      cout << "## init a RADER" << endl;
      VectorXd polar = VectorXd(3);
      polar << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), measurement_pack.raw_measurements_(2);
      VectorXd cartesian = tools.PolarToCartesian(polar);
      ekf_.x_ = cartesian;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      cout << "## init a LIDAR" << endl;
      ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1) ,0,0;
    }

    is_initialized_ = true;
    return;
  }

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) /1000000.;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.UpdateQ(dt);
  ekf_.UpdateF(dt);
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.Update_R(measurement_pack.raw_measurements_);
  } else {
    ekf_.Update_L(measurement_pack.raw_measurements_);
  }

}
