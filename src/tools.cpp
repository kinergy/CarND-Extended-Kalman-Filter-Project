#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
	rmse << 0,0,0,0;

	// // check the validity of the following inputs:
	// //  * the estimation vector size should not be zero
	// //  * the estimation vector size should equal ground truth vector size
	// if(estimations.size() != ground_truth.size()
	// 		|| estimations.size() == 0){
	// 	cout << "Invalid estimation or ground_truth data" << endl;
	// 	return rmse;
	// }

	// //accumulate squared residuals
	// for(unsigned int i=0; i < estimations.size(); ++i){

	// 	VectorXd residual = estimations[i] - ground_truth[i];

	// 	//coefficient-wise multiplication
	// 	residual = residual.array()*residual.array();
	// 	rmse += residual;
	// }

	// //calculate the mean
	// rmse = rmse/estimations.size();

	// //calculate the squared root
	// rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3,4);
	// //recover state parameters
	// float px = x_state(0);
	// float py = x_state(1);
	// float vx = x_state(2);
	// float vy = x_state(3);

	// //TODO: YOUR CODE HERE 
	// float d_1 = pow(px, 2) + pow(py, 2);
	// float d_1_2 = pow(d_1, 0.5);
	// float d_3_2 = pow(d_1, 1.5);

	// //check division by zero
	// if (fabs(d_1) < 0.0001) {
	//     cout << "CalculateJacobian() - Error - Divison by zero" << endl;
	//     return Hj;
	// }
	
	// //compute the Jacobian matrix
	// Hj << px/d_1_2, py/d_1_2, 0, 0,
	//       -py/d_1, px/d_1, 0, 0,
	//       py*(vx*py-vy*px)/d_3_2, px*(vy*px-vx*py)/d_3_2, px/d_1_2, py/d_1_2;

	return Hj;
}
