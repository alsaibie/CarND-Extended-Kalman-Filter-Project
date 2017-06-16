#include "kalman_filter.h"
#include <iostream>
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
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    MatrixXd Ht = H_.transpose();
    VectorXd y = z - H_ * x_;
    MatrixXd S_ = H_ * P_ * Ht + R_;
    MatrixXd K = P_ * Ht * S_.inverse();
    I_ = Eigen::Matrix4d::Identity();
    x_ = x_ + K * y;
    P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

    auto r = sqrt(pow(x_(0), 2) + pow(x_(1), 2));
    auto phi = atan2(x_(1), x_(0));
    double rdot;
    if (fabs(r) < 0.0001) {
        rdot = 0;
    } else {
        rdot = (x_(0) * x_(2) + x_(1) * x_(3)) / r;
    }

    VectorXd z_est(3);
    z_est << r, phi, rdot;

    MatrixXd Ht = H_.transpose();
    VectorXd y = z - z_est;
    while (y(1) > M_PI) y(1) -= 2. * M_PI;
    while (y(1) < -M_PI) y(1) += 2. * M_PI;
    MatrixXd S_ = H_ * P_ * Ht + R_;
    MatrixXd K = P_ * Ht * S_.inverse();


    I_ = Eigen::Matrix4d::Identity();


    x_ = x_ + K * y;

    P_ = (I_ - K * H_) * P_;
}
