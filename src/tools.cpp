#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth) {

	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
	if (estimations.size() == 0) {
		cout << "CalculateRMSE() - Error - estimations size should not be zero.";
	}
	else {
		if (estimations.size() != ground_truth.size()) {
			cout << "CalculateRMSE() - Error - Size of estimations and ground_truth should be equal.";
		}
	}

	//accumulate squared residuals
	for (int i = 0; i < estimations.size(); ++i) {
		VectorXd errors = estimations[i] - ground_truth[i];
		errors = errors.array()*errors.array();
		rmse += errors;
	}

	//calculate the mean
	rmse /= estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3, 4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//check division by zero
	// if (px == 0 || py == 0) {
	// 	cout << "CalculateJacobian () - Error - Division by Zero" << endl;
 //    return Hj;
	// }

  // pre-compute
  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  // c3 is equal to pow((pow(px, 2) + pow(py, 2)), 3 / 2);

  //check division by zero
  if(fabs(c1) < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

	//compute the Jacobian matrix
	float j1 = px / c2;
	float j2 = py / c2;
	float j3 = 0;
	float j4 = 0;
	float j5 = -py / c1;
	float j6 = px / c1;
	float j7 = 0;
	float j8 = 0;
  float j9 = (py*(vx*py - vy*px)) / c3;
	float j10 = (px*(vy*px - vx*py)) / c3;
	float j11 = px / c2;
	float j12 = py / c2;

	Hj << j1, j2, j3, j4,
		j5, j6, j7, j8,
		j9, j10, j11, j12;

	cout << "Hj is:\n" << Hj << endl;
	return Hj;
}
