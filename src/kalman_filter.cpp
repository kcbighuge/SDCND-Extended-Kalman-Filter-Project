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
  TO_DID:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TO_DID:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  //new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - (K * H_)) * P_;

  //KF Prediction step
  //VectorXd u = 0, 0, 0, 0;
  //x_ = (F_ * x_) + u;
  //MatrixXd Ft = F_.transpose();
  //P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TO_DID:
    * update the state by using Extended Kalman Filter equations
  */
  //state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  //check if position values equal zero
  if (px==0 && py==0) {
    px = 0.001;
    py = 0.001;
  };

  float ro = sqrt(px*px + py*py);
  float theta = atan2(py,px);
  float ro_dot = (px*vx + py*vy) / ro;
  VectorXd z_pred = VectorXd(3);
  z_pred << ro, theta, ro_dot;

  VectorXd y = z - z_pred;

  // check if angle phi in y vector is between -pi and pi
  if (y(1) > M_PI) {
    y(1) -= 2*M_PI;
  }
  if (y(1) < M_PI) {
    y(1) += 2*M_PI;
  }

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  //new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - (K * H_)) * P_;

  //KF Prediction step
  //VectorXd u = 0, 0, 0, 0;
  //x_ = (F_ * x_) + u;
  //MatrixXd Ft = F_.transpose();
  //P_ = F_ * P_ * Ft + Q_;  

}
