#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in,
      Eigen::MatrixXd &F_in, Eigen::MatrixXd &H_in);

  /**
   * Set state vector value
   * @param x_in Initial state
   */
  void SetStateEstimationVector(Eigen::VectorXd &x_in);

  /**
   * Get state vector value
   * @return current state x_
   */
  Eigen::VectorXd GetStateEstimationVector();

  /**
   * Return current state covariane
   * @return current state covariance P_ matrix
   */
  Eigen::MatrixXd GetStateCovarianceMatrix();

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(const float dt);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   * @param R Measurement covariance matrix
   */
  void Update(const Eigen::VectorXd &z, const Eigen::MatrixXd &R);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   * @param R Measurement covariance matrix
   */
  void UpdateEKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &R);

private:
    // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transistion matrix
  Eigen::MatrixXd F_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // identity matrix
  Eigen::MatrixXd I_;

};

#endif /* KALMAN_FILTER_H_ */
