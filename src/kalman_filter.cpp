#include <iostream>
#include "kalman_filter.h"
#define SMALLVAL 0.001

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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
  //cout << "\nF:\n" << F_ << "\n";
  //cout << "x:\n" << x_ << "\n";
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  //cout << "P:\n" << P_ << "\n";
  //cout << "Ft:\n" << Ft << "\n";
  //cout << "Q:\n" << Q_ << "\n";
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  GenericUpdate(y); 
}

void KalmanFilter::GenericUpdate(const VectorXd &y) {
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

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  if (x_[0] > -SMALLVAL && x_[0] < SMALLVAL) {
    return;
  }
  VectorXd hx = VectorXd(3);
  hx << sqrt(pow(x_[0],2) + pow(x_[1],2)),
        atan2(x_[1], x_[0]),
        (x_[0]* x_[2] + x_[1]* x_[3])/sqrt(pow(x_[0],2) + pow(x_[1],2));
  
  VectorXd y = z - hx;
  //cout << "y:\n" << y << "\n";
  GenericUpdate(y);
}
