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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  ekf_.Init();


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  Tools tools;

  //cout << "::EKF: " << measurement_pack.sensor_type_ << " | " << measurement_pack.timestamp_ << "\n" << measurement_pack.raw_measurements_ << "\n\n" << endl;


  // some dummy test stuff
  ///////////////////
  /*
  ekf_.x_ = VectorXd(4);
  if (measurement_pack.sensor_type_ == 0) { // laser
    ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1) ,9,9;
  } else { 
    VectorXd polar = VectorXd(3);
    polar << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), measurement_pack.raw_measurements_(2);
    VectorXd cartesian = tools.polarToCartesian(polar);

    ekf_.x_ = cartesian;

  }
  */
  
  //////////////////
  ////////////////

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    previous_timestamp_ = measurement_pack.timestamp_;
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      VectorXd polar = VectorXd(3);
      polar << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), measurement_pack.raw_measurements_(2);
      VectorXd cartesian = tools.polarToCartesian(polar);

      ekf_.x_ = cartesian;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1) ,9,9;
    }

    // done initializing, no need to predict or update
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

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) /100000.;
  previous_timestamp_ = measurement_pack.timestamp_;
  cout << dt << endl;

  ekf_.UpdateQ(dt);
  ekf_.UpdateF(dt);
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
    ekf_.Update_R(measurement_pack.raw_measurements_);
  } else {
    ekf_.Update_L(measurement_pack.raw_measurements_);
    // Laser updates
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
