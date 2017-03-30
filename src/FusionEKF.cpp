#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // Initialize the Measurement covariance matrix - radar
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // Initialize the Measurement covariance matrix - laser
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // Measurement matrix
  MatrixXd H_in = MatrixXd(2, 4);
  H_in << 1, 0, 0, 0,
          0, 1, 0, 0;

  // The initial transition matrix F_
  MatrixXd F_in = MatrixXd(4, 4);
  F_in << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;

  VectorXd x_init = VectorXd(4);
  MatrixXd P_init = MatrixXd(4, 4);
  ekf_.Init(x_init, P_init, F_in, H_in);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    // cout << "Initializating EKF: " << endl;
    VectorXd x_init = VectorXd(4);

    // Initialize state.
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float ro = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];

      // Convert radar from polar to cartesian coordinates
      float p1 = ro * cos(phi);
      float p2 = ro * sin(phi);
      x_init << p1, p2, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      float p1 = measurement_pack.raw_measurements_[0];
      float p2 = measurement_pack.raw_measurements_[1];
      x_init << p1, p2, 0, 0;
    } else {
      // This is a save exit. It shouldn't happen.
      return;
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.SetStateEstimationVector(x_init);
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.Predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_, R_radar_);
  } else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_, R_laser_);
  }

  // cout << "x_ = " << ekf_.GetStateEstimationVector() << endl;
  // cout << "P_ = " << ekf_.GetStateCovarianceMatrix() << endl;
}

VectorXd FusionEKF::GetStateEstimationVector()
{
    return ekf_.GetStateEstimationVector();
}
