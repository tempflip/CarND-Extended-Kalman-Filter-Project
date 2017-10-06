#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  Eigen::MatrixXd Hx_;

  // measurement covariance matrix

  Eigen::MatrixXd R_;
  Eigen::MatrixXd Rr_;
  Eigen::MatrixXd Rl_;

  Eigen::VectorXd z_;
  Eigen::MatrixXd I_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  void Predict();

  void Update_R(const Eigen::VectorXd &z);
  void Update_L(const Eigen::VectorXd &z);


  void UpdateEKF();

  void UpdateQ(float dt);

  void UpdateF(float dt);

  void Init();
};

#endif /* KALMAN_FILTER_H_ */
