#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include <fstream>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  ofstream* getOStream();
  ofstream* getOStream2();

  
private: 
  ofstream outfile_;
  ofstream outfile2_;
};

#endif /* TOOLS_H_ */