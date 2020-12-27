#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
	VectorXd rmse = VectorXd(4);
	rmse << 0, 0, 0, 0;

	if(estimations.size() != ground_truth.size() ||
	   estimations.size() == 0) {
		std::cout << "Error in calculating RMSE, invalid estiamtions or ground_truth" << std::endl;
		return rmse; 
	}

	for(int i = 0; i < estimations.size(); ++i) {
		VectorXd error = estimations[i] - ground_truth[i];
		error = error.array() * error.array();
		rmse += error;
	}
	// calculate mean
	rmse = rmse / estimations.size();

	// calculate sqrt
	rmse = rmse.array().sqrt();

	// return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
	float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = c1 * c2;
    MatrixXd jacobian = MatrixXd(3,4);
    // Jacobian << 0, 0, 0, 0,
    // 			0, 0, 0, 0,
    // 			0, 0, 0, 0;
    if(fabs(c1) < 0.0001) {
    	std::cout << "Dividing by 0 in Jacobian calculation" << std::endl;
    	return jacobian;
    }
    jacobian << (px/c2), (py/c2), 0, 0,
               -(py/c1), (px/c1), 0, 0,
                py * (vx * py - vy * px) / c3, px * (vy * px - vx * py) / c3, (px/c2), (py/c2);
    return jacobian;
}
