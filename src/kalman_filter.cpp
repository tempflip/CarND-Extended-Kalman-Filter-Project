#include "kalman_filter.h"
using namespace std;
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init() {
  P_ = MatrixXd(4,4);
  P_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  F_ = MatrixXd(4,4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  Rl_ = MatrixXd(2,2);
  Rl_ << 0.01, 0,
        0, 0.01;

  Rr_ = MatrixXd(3,3);
  Rr_ << 0.01, 0, 0,
        0, 1.0e-6, 0,
        0, 0, 0.01;

  I_ = MatrixXd(4,4);
  I_ << 1, 0, 0, 0,
        0, 1, 0, 0, 
        0, 0, 1, 0,
        0, 0, 0, 1;

}


/*
PYTHON
self.x = np.dot(self.F, self.x) #+ self.u # new state : movement matrix X current state + movement
        self.P = np.dot(np.dot(self.F, self.P), self.F.transpose()) + self.Q # adjusting covar matrix
*/
void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  cout << "## Predict " << endl;
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update_R(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

}

/*
H = np.array([[1, 0, 0, 0] # X to measurement mapping matrix (H * x = measurement)
                     ,[0, 1, 0, 0]])
        R = np.array([[0.01, 0], 
                     [0, 0.01]])
        
        Hx = H.dot(self.x)
    
        self.process_meas(z, H, Hx, R)

*/
void KalmanFilter::Update_L(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  z_ = z;
  H_ = MatrixXd(2, 4);
  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;
  Hx_ = H_ * x_;
  R_ = Rl_;
  UpdateEKF();
}

/*
 def process_meas(self, z, H, Hx, R):

        y = z - Hx # error : previous state - measurement X measurement matrix
        S = H.dot(self.P).dot(H.transpose()) + R
        K = self.P.dot(H.transpose()).dot(np.linalg.inv(S)) # kalman gain
        self.x = self.x + K.dot(y) # new state : x + kalman gain X error
        self.P = np.dot((self.I - K.dot(H)), self.P) # + self.Q # adjusting uncertainty covaria       
*/
void KalmanFilter::UpdateEKF() {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  MatrixXd y = z_ - Hx_;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // update
  ////////////////
  x_ = x_ + K * y;
  P_ = (I_ - K * H_) * P_;
  cout << "##### UpdateEKF" << S << endl;
}


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
        
        //cout << "updateQ" << Q_ << " >>>> <<<<" << endl;

}

void KalmanFilter::UpdateF(float dt) {
  F_(0, 2) = dt;
  F_(1, 3) = dt;
  cout << ":: F" << F_ << endl;
}

