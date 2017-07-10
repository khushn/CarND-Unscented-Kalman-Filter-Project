#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  is_initialized_ = false;
  P_ << 1, 0, 0, 0,
      0, 1, 0, 0, 
      0, 0, 1, 0, 
      0, 0, 0, 1;

   //set state dimension
  n_x_ = 5;

  // the augmentation matrix dimention, when we add the noise part
  n_aug_ = 7;

  //define spreading parameter
  lambda_ = 3 - n_x_;
}

UKF::~UKF() {}


/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

   if (!is_initialized_) {
     /**
      Initialize state.
      */
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float rho_dot = meas_package.raw_measurements_[2];
      //cout << "rho: " << rho << ", phi: " << phi << ", rho dot: " << rho_dot << endl;
      float px = rho * cos(phi);
      float py = rho * sin(phi);
      x_ << px, py;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0;
    }

    time_us_ = meas_package.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt - expressed in seconds
  time_us_ = meas_package.timestamp_;
  //cout << "dt: " << dt << endl;
  if (dt <.05) {
    cout << "Skipping prediction step as dt (small): " << dt << endl;
    // TODO: YOUR CODE HERE
  } else {
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  MatrixXd Xsig_aug = MatrixXd();

  // 1st step: Generate the sigma points
  // NOTE: This get the augmented points i.e. 2 extra in the bottom right for 
  // process noise
  AugmentedSigmaPoints(&Xsig_aug);

  
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

//create augmented mean vector
VectorXd x_aug = VectorXd(n_aug_);

//create augmented state covariance
MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

//create sigma point matrix
MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

//create augmented mean state
x_aug.head(n_x_) = x_;
x_aug(n_x_) = 0;
x_aug(n_x_+1) = 0;

//create augmented covariance matrix
P_aug.fill(0.0);
P_aug.topLeftCorner(n_x_,n_x_) = P_;
P_aug(n_x_,n_x_) = std_a_*std_a_;
P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;

//create square root matrix
MatrixXd L = P_aug.llt().matrixL();

//create augmented sigma points
Xsig_aug.col(0)  = x_aug;
for (int i = 0; i< n_aug_; i++)
{
  Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
  Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
}
  

//print result
//std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

//write result
*Xsig_out = Xsig_aug;

}