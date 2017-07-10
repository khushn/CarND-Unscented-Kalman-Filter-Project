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

  //set state dimension
  n_x_ = 5;
  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

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

  

  // the augmentation matrix dimention, when we add the noise part
  n_aug_ = 7;

  //define spreading parameter
  lambda_ = 3 - n_x_;

  // We compute the weights only once
  weights_ = VectorXd(2*n_aug_+1);
  double weight_0 = lambda_/(lambda_+n_aug_);
 
  weights_[0] = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_[i] = weight;
  }
}

UKF::~UKF() {}


/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  

  if ( (meas_package.sensor_type_ == MeasurementPackage::LASER && !use_laser_) ||
      (meas_package.sensor_type_ == MeasurementPackage::RADAR && !use_radar_) ) {
    // If either radar or lidar is disabled 
    // check and return 
    return;
  }
  

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

  // Do the prediction part of the UKF roadmap
  Prediction(dt);

  // Do the Update part of the UKF roadmap
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {}

    // As a first step of the radar update, we need to 
    // transform the predicted state as per radar data
    VectorXd z_out = VectorXd(3);
    MatrixXd S_out = MatrixXd(3, 3);
    ukf.PredictRadarMeasurement(&z_out, &S_out);
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

  // Step 1: Generate the sigma points
  // NOTE: This get the augmented points i.e. 2 extra in the bottom right for 
  // process noise
  AugmentedSigmaPoints(&Xsig_aug);

  // Step 2: Predict the sigma points
  Xsig_pred_ = MatrixXd();
  SigmaPointPrediction(delta_t, Xsig_aug, &Xsig_pred_);

  // Step 3: Predict mean and covariance
  PredictMeanAndCovariance(&x_, &P_);

  
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

void UKF::SigmaPointPrediction(double delta_t, MatrixXd& Xsig_aug, MatrixXd* Xsig_out) {

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }

  //print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  //write result
  *Xsig_out = Xsig_pred;

}

void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) {

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x = x+ weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }


  //print result
  std::cout << "Predicted state" << std::endl;
  std::cout << x << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P << std::endl;

  //write result
  *x_out = x;
  *P_out = P;
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out) {

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;

  
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  std::cout << "S: " << std::endl << S << std::endl;

  //write result
  *z_out = z_pred;
  *S_out = S;
}