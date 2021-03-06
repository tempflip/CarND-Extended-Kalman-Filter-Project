#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
  VectorXd RMSE = VectorXd(4);
  RMSE << 0,0,0,0;

  
  for (int i = 0; i < estimations.size(); ++i) {
  	VectorXd diff = estimations[i] - ground_truth[i];
  	diff = diff.array() * diff.array();
  	diff = diff.array().sqrt();
  	RMSE = RMSE + diff;
  }

  RMSE = RMSE / estimations.size(); 
  return RMSE;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj = MatrixXd(3,4);
  float THRESH = 0.00001;
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);


  
  float px2_plus_py2 = px*px + py*py;
  float on_sqrt = sqrt(px2_plus_py2);
  float on_3_per_2 = pow(px2_plus_py2, 3/2);

  float vi1 = py * (vx*py - vy*px) / on_3_per_2;
  float vi2 = px * (vy*px - vx*py) / on_3_per_2;

  Hj << px / on_sqrt,        py / on_sqrt,      0,             0,
        -py / px2_plus_py2,  px / px2_plus_py2, 0,             0,
        vi1,                 vi2,               px / on_sqrt,  py / on_sqrt;

  return Hj;
}

VectorXd Tools::PolarToCartesian(const VectorXd& polar) {
	VectorXd cartesian = VectorXd(4);

	float rho = polar(0);
	float phi = polar(1);
	float drho = polar(2);

	float px = rho * cos(phi);
	float py = rho * sin(phi);
	float vx = drho * cos(phi);
	float vy = drho * sin(phi);

	cartesian << px, py, vx, vy;
	return cartesian;
}


VectorXd Tools::CartesianToPolar(const VectorXd& cartesian) {
  VectorXd polar = VectorXd(3);
  float THRESH = 0.0001;
  float px = cartesian(0);
  float py = cartesian(1);
  float vx = cartesian(2);
  float vy = cartesian(3);

  float rho = sqrt(px*px + py*py);
  float phi = atan2(py, px);
  float drho = 0;
  if (rho > THRESH) {
    drho = (px*vx + py*vy) / rho;
  }

  polar << rho, phi, drho;
  return polar;
}
