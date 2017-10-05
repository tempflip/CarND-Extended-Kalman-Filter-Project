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
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}


/* PYTHON : 
def polar_to_cartesian(polar_x):
    rho, phi, drho = polar_x[0], polar_x[1], polar_x[2]
    
    px = rho * math.cos(phi)
    py = rho * math.sin(phi)
    vx = drho * math.cos(phi)
    vy = drho * math.sin(phi)
    
    return np.array([px, py, vx, vy])
*/
VectorXd Tools::polarToCartesian(const VectorXd& polar) {

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
