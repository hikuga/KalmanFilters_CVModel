#include <iostream>
#include <algorithm>
#include <numeric>
#include <math.h>

#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  
    * Calculate the RMSE here.
  */
    VectorXd rmse2(4);
    rmse2 << 0,0,0,0;
    if(estimations.size() != ground_truth.size()
       || estimations.size() == 0){
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse2;
    }
    
    auto ground_truth_itr = begin(ground_truth);
    return  (accumulate( begin(estimations), end(estimations), rmse2, [&](VectorXd  ans, VectorXd  est_itm){
        VectorXd diff = ( est_itm - (*ground_truth_itr) );
        diff = diff.array() * diff.array();
        ++ground_truth_itr;
        ans += diff;
        return ans;
    }) / estimations.size()).array().sqrt();
    
   }

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    //pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);
    
    //check division by zero
    if(fabs(c1) < 0.0000001){
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }
    
    //compute the Jacobian matrix
    Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
    
    return Hj;
}

void Tools::PolarCartesian(const double phi, const double r, double &px, double &py) {
    
    
    px = r * cos(phi);
    py = r * sin(phi);
}

Eigen::VectorXd Tools::CartesianPolar(const Eigen::VectorXd x) {
    VectorXd z_pred(3);
    
    // Unpack the state vector
    double px = x(0);
    double py = x(1);
    double vx = x(2);
    double vy = x(3);
    
    if (fabs(px) < APPROX_ZERO) {
        px = APPROX_ZERO;
    }
    
    // Convert from cartesian to polar
    double px2 = px * px;
    double py2 = py * py;
    double rho = sqrt(px2 + py2);
    
    // Avoid division by zero
    if (fabs(rho) < APPROX_ZERO) {
        rho = APPROX_ZERO;
    }
    
    z_pred[0] = rho;
    z_pred[1] = atan2(py, px);
    z_pred[2] = (px * vx + py * vy) / rho;
    
    return z_pred;
}
