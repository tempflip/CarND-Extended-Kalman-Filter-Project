#include "kalman_filter.h"
using namespace std;
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}

/* PYTHON

        dt2 = dt * dt
        dt3 = dt * dt2
        dt4 = dt * dt3

        x, y = 5,5

        r11 = dt4 * x / 4
        r13 = dt3 * x / 2
        r22 = dt4 * y / 4
        r24 = dt3 * y /  2
        r31 = dt3 * x / 2 
        r33 = dt2 * x
        r42 = dt3 * y / 2
        r44 = dt2 * y

        self.Q = np.array([[r11, 0, r13, 0],
                      [0, r22, 0, r24],
                      [r31, 0, r33, 0], 
                      [0, r42, 0, r44]])   


*/
void KalmanFilter::UpdateQ(float dt) {
        float dt2 = dt * dt;
        float dt3 = dt * dt2;
        float dt4 = dt * dt3;

        float noise_ax = 9;
        float noise_ay = 9;

        float r00 = dt4 * noise_ax / 4;
        float r02 = dt3 * noise_ax / 2;
        float r11 = dt4 * noise_ay / 4;
        float r13 = dt3 * noise_ay /  2;
        float r20 = dt3 * noise_ax / 2;
        float r22 = dt2 * noise_ax;
        float r31 = dt3 * noise_ay / 2;
        float r33 = dt2 * noise_ay;

        Q_ = MatrixXd(4,4);
        Q_(0, 0) = r00;
        Q_(0, 1) = 0;
        Q_(0, 2) = r02;
        Q_(0, 3) = 0;

        Q_(1, 0) = 0;
        Q_(1, 1) = r11;
        Q_(1, 2) = 0;
        Q_(1, 3) = r13;

        Q_(2, 0) = r20;
        Q_(2, 1) = 0;
        Q_(2, 2) = r22;
        Q_(2, 3) = 0;        

        Q_(3, 0) = 0;
        Q_(3, 1) = r31;
        Q_(3, 2) = 0;
        Q_(3, 3) = r33;
        
        cout << "updateQ" << Q_ << " >>>> <<<<" << endl;


}
