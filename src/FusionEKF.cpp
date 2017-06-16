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

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0005, 0,
            0, 0.0005;

    //measurement covariance matrix - radar
    R_radar_ << .005, 0, 0,
            0, .009, 0,
            0, 0, .01;

    /**
    TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
    */
    // State Standard Deviation

    auto sigma_x0 = sqrt(0.1);

    MatrixXd P_ = Eigen::Vector4d(.5, .5, 10, 10).asDiagonal();
    P_ *= (sigma_x0 * sigma_x0);
    ekf_.P_ = P_;

    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    MatrixXd F_ = Eigen::Matrix4d::Identity();
    F_.topRightCorner(2, 2) = Eigen::Matrix2d::Identity();
    ekf_.F_ = F_;





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
        /**
        TODO:
          * Initialize the state ekf_.x_ with the first measurement.
          * Create the covariance matrix.
          * Remember: you'll need to convert radar from polar to cartesian coordinates.
        */
        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;
        VectorXd measurement = measurement_pack.raw_measurements_;


        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            */
            auto r = measurement(0);
            auto phi = measurement(1);
            auto rdot = measurement(2);
            ekf_.x_(0) = r * cos(phi);
            ekf_.x_(1) = r * sin(phi);
            ekf_.x_(2) = rdot * cos(phi);
            ekf_.x_(3) = rdot * sin(phi);

        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state.
            */
            ekf_.x_(0) = measurement(0);
            ekf_.x_(1) = measurement(1);
        }

        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
     TODO:
       * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
       * Update the process noise covariance matrix.
       * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    double sigma_w0 = sqrt(.1);

    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
    ekf_.Q_ = Q_(sigma_w0, dt);
    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     TODO:
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */
    VectorXd x_ = measurement_pack.raw_measurements_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.R_ = R_radar_;
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.H_ = Hj_;
        ekf_.UpdateEKF(x_);

    } else {
        // Laser updates
        ekf_.R_ = R_laser_;
        ekf_.H_ = H_laser_;
        ekf_.Update(x_);
    }

    // print the output
    //cout << "x_ = " << ekf_.x_ << endl;
    //cout << "P_ = " << ekf_.P_ << endl;
}

Eigen::MatrixXd FusionEKF::Q_(const double &sigma_w, const double dt) {

    auto dt2 = pow(dt, 2);
    auto dt3 = pow(dt, 3);
    auto dt4 = pow(dt, 4);
    auto w = sigma_w;
    MatrixXd Q(4, 4);
    Q << dt4 / 4 * w, 0, dt3 / 2 * w, 0,
            0, dt3 / 4 * w, 0, dt3 / 2 * w,
            dt3 / 2 * w, 0, dt2 * w, 0,
            0, dt3 / 2 * w, 0, dt2 * w;

    return Q;
}


