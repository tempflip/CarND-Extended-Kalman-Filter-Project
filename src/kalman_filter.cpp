#include "kalman_filter.h"
using namespace std;
#include <iostream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init() {
  P_ = MatrixXd(4,4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  F_ = MatrixXd(4,4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  Rl_ = MatrixXd(2,2);
  Rl_ << 0.0225, 0,
        0, 0.0225;

  Rr_ = MatrixXd(3,3);
  Rr_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  I_ = MatrixXd(4,4);
  I_ << 1, 0, 0, 0,
        0, 1, 0, 0, 
        0, 0, 1, 0,
        0, 0, 0, 1;

}


void KalmanFilter::Predict() {

  //std::cout << "## Before Q: \n" << Q_ << std::endl;
  //std::cout << "## Before F: \n" << F_ << std::endl;
  //std::cout << "## Before P: \n" << P_ << std::endl;

  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  //std::cout << "## PREDICTED P: \n" << P_ << std::endl;
  //std::cout << "## PREDICTED x: \n" << x_ << std::endl;

}


void KalmanFilter::Update_L(const VectorXd &z) {
  z_ = z;
  R_ = Rl_;

  H_ = MatrixXd(2, 4);
  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;
  Hx_ = H_ * x_;
  UpdateEKF();
}


void KalmanFilter::Update_R(const VectorXd &z) {

  Tools tools;
  z_ = z;
  R_ = Rr_;
  H_ = tools.CalculateJacobian(tools.PolarToCartesian(z));
  Hx_ = tools.CartesianToPolar(x_);
  UpdateEKF();
}

void KalmanFilter::UpdateEKF() {

  //std::cout << "## UPDATE z: " << z_ << std::endl;
  //std::cout << "## UPDATE H: " << H_ << std::endl;
  //std::cout << "## UPDATE Hx_: " << Hx_ << std::endl;
  //std::cout << "## UPDATE P: " << P_ << std::endl;
  //std::cout << "## UPDATE R_: " << R_ << std::endl;

  MatrixXd y = z_ - Hx_;
  MatrixXd PHt = P_ * H_.transpose();
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  if (y.size() == 3) y(1) = atan2(sin(y(1)), cos(y(1)));

  // update
  ////////////////
  x_ = x_ + K * y;
  P_ = (I_ - K * H_) * P_;
  
  std::cout << "!! UPDATED P: " << P_ << std::endl;
  std::cout << "!! UPDATED x: \n" << x_ << std::endl;

}


void KalmanFilter::UpdateQ(float dt) {
        float dt2 = dt * dt;
        float dt3 = dt * dt2;
        float dt4 = dt * dt3;

        float noise_ax = 9;
        float noise_ay = 9;

        float r11 = dt4 * noise_ax / 4;
        float r13 = dt3 * noise_ax / 2;
        float r22 = dt4 * noise_ay / 4;
        float r24 = dt3 * noise_ay / 2;
        float r31 = dt3 * noise_ax / 2;
        float r33 = dt2 * noise_ax;
        float r42 = dt3 * noise_ay / 2;
        float r44 = dt2 * noise_ay;

        Q_ = MatrixXd(4,4);
        
        Q_ <<      r11, 0, r13, 0,
                   0, r22, 0, r24,
                   r31, 0, r33, 0,
                   0, r42, 0, r44;


}

void KalmanFilter::UpdateF(float dt) {
  F_(0, 2) = dt;
  F_(1, 3) = dt;
}

