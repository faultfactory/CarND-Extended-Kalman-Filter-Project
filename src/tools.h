#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
private:

  // Storage for prior Jacobian Matrix
  Eigen::MatrixXd Hj_prev_;

public:
  /**
  * Constructor.
  */
  Tools(){
    MatrixXd Hj_prev_; 
    Hj_prev_ << 0,0,0,0,
      0,0,0,0,
      0,0,0,0,
      0,0,0,0;
  };
  

  /**
  * Destructor.
  */
  virtual ~Tools();

   /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

};

#endif /* TOOLS_H_ */
