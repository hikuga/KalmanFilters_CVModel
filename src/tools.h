#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
    const double APPROX_ZERO = 0.0001;

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

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);
  /*
  * convert cartesian to polar.
  */
  VectorXd CartesianPolar(const Eigen::VectorXd x);
  void PolarCartesian(const double phi, const double r, double &px, double &py);
};

#endif /* TOOLS_H_ */
