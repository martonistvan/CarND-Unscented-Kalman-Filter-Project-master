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
	VectorXd red;
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() == 0) {
		cout << "The estimation vector size should not be zero" << endl;
	}
	else if (estimations.size() != ground_truth.size()) {
		cout << "The estimation vector size should equal ground truth vector size" << endl;
	}
	else {
		//accumulate squared residuals
		for(int i=0; i < estimations.size(); ++i){
			// ... your code here
			red = estimations[i]-ground_truth[i];
			red = red.array()*red.array();
			rmse += red;
		}

		//calculate the mean
		rmse = rmse / estimations.size();
		//calculate the squared root
		rmse = rmse.array().sqrt();
	}
	//return the result
	return rmse;
}
