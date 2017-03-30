#include <iostream>
#include "kalman_filter.h"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in) {

  SetStateEstimationVector(x_in);

  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
}

void KalmanFilter::SetStateEstimationVector(VectorXd &x_in) {
  /**
   * Set the estimation vector value and also
   * precalculates the Identity matrix, which is used
   * both on Predict() and Update()
  **/
    x_ = x_in;
    long size = x_.size();
    I_ = MatrixXd::Identity(size, size);
}

VectorXd KalmanFilter::GetStateEstimationVector() {
  return x_;
}

MatrixXd KalmanFilter::GetStateCovarianceMatrix() {
  return P_;
}

void KalmanFilter::Predict(const float dt) {
  /**
    * Predict the state
  */

  // Modify the F matrix so that the time is integrated
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Set the acceleration noise components
  float noise_ax = 9.0;
  float noise_ay = 9.0;

  // Set the process covariance matrix Q
  MatrixXd Q = MatrixXd(4, 4);
  Q <<  dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
              0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
              dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
              0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q;
}

void KalmanFilter::Update(const VectorXd &z, const MatrixXd &R) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // New estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const MatrixXd &R) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
  MatrixXd Hj = Tools::CalculateJacobian(x_);

  VectorXd y = z - Tools::fromCartesianToPolar(x_);
  MatrixXd Ht = Hj.transpose();

  MatrixXd S = Hj * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // New estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * Hj) * P_;

}
