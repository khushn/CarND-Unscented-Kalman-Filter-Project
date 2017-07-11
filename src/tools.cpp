#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {
	outfile_.open("data/obj_pose-laser-radar-ukf-output.txt");	
	outfile2_.open("data/full-output.txt");
}

Tools::~Tools() {
	outfile_.close();
	outfile2_.close();
}

ofstream* Tools::getOStream() {
	return &outfile_;
}

ofstream* Tools::getOStream2() {
	return &outfile2_;
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
	rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	if (estimations.size() == 0) {
	    cerr << "Nothing to estimate. Vector empty" << endl;
	    return rmse;
	}
	
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
	if (estimations.size() != ground_truth.size()) {
	    cerr << "The vector sizes differ!" << endl;
	}

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        for(int j=0; j<4; j++) {
            float d = estimations[i][j] - ground_truth[i][j];
		    rmse[j] += d*d;
        }
        
	}

	//calculate the mean
	// ... your code here
	rmse = rmse/estimations.size();
	

	//calculate the squared root
	// ... your code here
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}