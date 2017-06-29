#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z, const Eigen::MatrixXd & H_laser, const Eigen::MatrixXd & R_laser) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    H_ = H_laser;
    R_ = R_laser;
    VectorXd z_pred = H_ * x_;
    doUpdate(z, z_pred);
    }

void KalmanFilter::UpdateEKF(const VectorXd &z, const Eigen::MatrixXd & R_radar) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    R_ = R_radar;
    H_ = tools.CalculateJacobian(x_);
    VectorXd z_pred = tools.CartesianPolar(x_);
    doUpdate(z, z_pred);
}
double SNormalizeAngle(double phi)
{
    const double Max = M_PI;
    const double Min = -M_PI;
    
    return phi < Min
    ? Max + std::fmod(phi - Min, Max - Min)
    : std::fmod(phi - Min, Max - Min) + Min;
}

void KalmanFilter::doUpdate(const VectorXd &z, const VectorXd &z_pred){
    VectorXd y = z - z_pred;
    
    y[1] = SNormalizeAngle(y[1]);
    
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;

}
