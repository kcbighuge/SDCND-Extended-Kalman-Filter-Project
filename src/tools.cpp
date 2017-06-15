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
  TO_DID:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

	// check the validity of the following inputs:
	// * the estimation vector size should not be zero
	// * the estimation vector size should equal ground truth vector size
	if (estimations.size() == 0) {
		//cout << "Estimation vector size be zero!" << endl;
		return rmse;
	};

	if (estimations.size() != ground_truth.size()) {
		//cout << "Estimation and ground truth vector sizes not sames!" << endl;
		return rmse;
	};

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){

        VectorXd residual = estimations[i] - ground_truth[i];
				residual = residual.array().square();
        rmse += residual;
	}

	//calculate the mean
	rmse /= estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TO_DID:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//my neanderthalic variable creations
	float px2 = pow(px, 2);
	float py2 = pow(py, 2);
	float px_div = px / pow(px2+py2, 0.5);
	float py_div = py / pow(px2+py2, 0.5);

	//check division by zero
	float zero_check = 0.00001;
	if (fabs(px)<0.00001 && fabs(py)<0.00001) {
		px = zero_check;
		py = zero_check;
	};

	//compute the Jacobian matrix
	Hj << px_div, py_div, 0, 0,
		 -py / (px2+py2), px / (px2+py2), 0, 0,
		 py*(vx*py - vy*px) / pow(px2+py2, 1.5), px*(vy*px - vx*py) / pow(px2+py2, 1.5), px_div, py_div;

	return Hj;
}
