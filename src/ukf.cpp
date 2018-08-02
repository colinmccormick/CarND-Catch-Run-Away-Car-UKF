#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
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
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // Set to false to begin; set true after initialization with first measurement
  is_initialized_ = false;

  // Time that state is valid (microseconds)
  time_us_ = 0.0;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1); 

  // Weights matrix
  weights_ = VectorXd(2 * n_aug_ + 1);
  double weight_0_ = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0_;
  double w_ = 0.5 / (n_aug_ + lambda_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    weights_(i) = w_;
  }

  // Lidar noise matrix
  R_laser_ = MatrixXd(2,2);
  R_laser_ << std_laspx_ * std_laspx_, 0,
              0, std_laspy_ * std_laspy_;

  // Radar noise matrix
  R_radar_ = MatrixXd(3,3);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0, std_radrd_ * std_radrd_;

  // Process noise matrix (augmented)
  R_process_ = MatrixXd(2,2);
  R_process_ << std_a_ * std_a_, 0,
                0, std_yawdd_ * std_yawdd_;

  // Projection matrix for lidar measurement
  H_laser_ = MatrixXd(2, 5);
  H_laser_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;              

  // Overshoot factor
  overshoot_ = 1.25;

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

  // Check if we are using this measurement type
  if ( (meas_package.sensor_type_ == MeasurementPackage::LASER && !use_laser_) ||
       (meas_package.sensor_type_ == MeasurementPackage::RADAR && !use_radar_) ) {
    return;
  }

  // If filter isn't initialized, use measurement for initialization
  if (!is_initialized_) {

    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);

    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

      const double rho_   = meas_package.raw_measurements_(0);
      const double theta_ = meas_package.raw_measurements_(1);
      x_(0) = rho_ * cos(theta_);
      x_(1) = rho_ * sin(theta_);

    }

    // Finish initializing state
    x_(2) = 1;
    x_(3) = 1;
    x_(4) = 0.1;

    // Initialize covariance matrix
    // Use std_laspx_ and std_laspy_ for (0,0) and (1,1) entries since we are 
    // initializing from measurements. Otherwise use 1 (default value).

    P_ << 0.15,    0, 0, 0, 0,
             0, 0.15, 0, 0, 0,
             0,    0, 1, 0, 0,
             0,    0, 0, 1, 0,
             0,    0, 0, 0, 1;

    // Set time and initialize
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;

    return;

  } 

  // Calculate time interval since last measurement (in seconds)
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  // Predict state
  Prediction(delta_t);

  // Use sensor-appropriate update function
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
  } 
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
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

  // Create augmented state
  VectorXd x_aug_(n_aug_);
  x_aug_.head(5) = x_;
  x_aug_(5) = 0.0;
  x_aug_(6) = 0.0;

  // Create augmented covariance matrix
  MatrixXd P_aug_(n_aug_,n_aug_);
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5,5) = P_;

  // Add process noise
  P_aug_.bottomRightCorner(2,2) = R_process_;

  // Calculate square root matrix
  MatrixXd A_ = P_aug_.llt().matrixL();  

  // Generate augmented sigma points
  MatrixXd Xsig_aug_(n_aug_, 2 * n_aug_ + 1);
  double sl_ = sqrt(lambda_ + n_aug_);
  Xsig_aug_.col(0) = x_aug_;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i + 1)        = x_aug_ + sl_ * A_.col(i);
    Xsig_aug_.col(i + n_aug_ + 1) = x_aug_ - sl_ * A_.col(i);
  }

  // Predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    // Extract sigma point values
    const double px_            = Xsig_aug_(0,i);
    const double py_            = Xsig_aug_(1,i);
    const double v_             = Xsig_aug_(2,i);
    const double yaw_           = Xsig_aug_(3,i);
    const double yaw_dot_       = Xsig_aug_(4,i);
    const double nu_a_          = Xsig_aug_(5,i);
    const double nu_psi_dotdot_ = Xsig_aug_(6,i);

    // Calculate helpful quantities
    const double s_yaw_ = sin(yaw_);
    const double c_yaw_ = cos(yaw_);
    const double dt2_ = delta_t * delta_t;

    // Calculate new sigma point
    VectorXd new_sp_(n_x_);
    new_sp_.fill(0.0);

    // Avoid division by zero
    if (fabs(yaw_dot_) > 0.001) {
      new_sp_(0) = px_ + v_ / yaw_dot_ * (sin(yaw_ + yaw_dot_ * delta_t) - s_yaw_);
      new_sp_(1) = py_ + v_ / yaw_dot_ * (-cos(yaw_ + yaw_dot_ * delta_t) + c_yaw_);
    } else {
      new_sp_(0) = px_ + v_ * c_yaw_ * delta_t;
      new_sp_(1) = py_ + v_ * s_yaw_ * delta_t;
    }
    new_sp_(2) = v_;
    new_sp_(3) = yaw_ + yaw_dot_ * delta_t;
    new_sp_(4) = yaw_dot_;

    // Add noise
    new_sp_(0) += 0.5 * dt2_ * c_yaw_ * nu_a_;
    new_sp_(1) += 0.5 * dt2_ * s_yaw_ * nu_a_;
    new_sp_(2) += delta_t * nu_a_;
    new_sp_(3) += 0.5 * dt2_ * nu_psi_dotdot_;
    new_sp_(4) += delta_t * nu_psi_dotdot_;

    Xsig_pred_.col(i) = new_sp_;
    
  }

  // Calculate state mean prediction
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // Calculate state covariance prediction
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    VectorXd x_diff_ = Xsig_pred_.col(i) - x_;

    while (x_diff_(3) > M_PI) x_diff_(3) -= 2*M_PI;
    while (x_diff_(3) < -M_PI) x_diff_(3) += 2*M_PI;

    P_ += weights_(i) * x_diff_ * x_diff_.transpose();

  }

  return;

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
  
  // Use Kalman filter equations (linear system)
  VectorXd z_pred_ = H_laser_ * x_;
  VectorXd z_diff_ = meas_package.raw_measurements_ - z_pred_;
  MatrixXd Ht_ = H_laser_.transpose();
  MatrixXd PHt_ = P_ * Ht_;
  MatrixXd S_ = H_laser_ * PHt_ + R_laser_;
  MatrixXd Si_ = S_.inverse();
  MatrixXd K_ = PHt_ * Si_;

  // Update state and covariance
  x_ = x_ + (K_ * z_diff_);
  long x_size_ = x_.size();
  MatrixXd I_ = MatrixXd::Identity(x_size_,x_size_);
  P_ = (I_ - K_ * H_laser_) * P_;

  return;

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

  // Declare dimension of radar measurement space
  int n_z_ = 3;

  // Transform sigma points into radar measurement space
  MatrixXd Zsig_(n_z_,2*n_aug_+1);
  Zsig_.fill(0.0); 

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    // Extract sigma point
    const double px_  = Xsig_pred_(0,i);
    const double py_  = Xsig_pred_(1,i);
    const double v_   = Xsig_pred_(2,i);
    const double psi_ = Xsig_pred_(3,i);

    // Map to radar measurement coordinates
    const double rho_ = sqrt(px_ * px_ + py_ * py_);
    Zsig_(0,i) = rho_;
    Zsig_(1,i) = atan2(py_,px_);

    // If measurement isn't too close to origin use the velocity value
    if (rho_ > 0.001) {
      Zsig_(2,i) = (px_ * cos(psi_) * v_ + py_ * sin(psi_) * v_) / rho_;
    } else {
      Zsig_(2,i) = 0.0;
    }
  }

  // Calculate mean state in measurement space
  VectorXd z_pred_(n_z_);
  z_pred_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred_ += weights_(i) * Zsig_.col(i);
  }

  // Calculate state covariance in measurement space
  MatrixXd S_(n_z_,n_z_);
  S_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    VectorXd z_diff_ = Zsig_.col(i) - z_pred_;
    while (z_diff_(1) > M_PI) z_diff_(1) -= 2*M_PI;
    while (z_diff_(1) < -M_PI) z_diff_(1) += 2*M_PI;

    S_ += weights_(i) * z_diff_ * z_diff_.transpose();
  }

  // Add measurement noise covariance
  S_ += R_radar_;

  // Calculate cross correlation matrix
  MatrixXd Tc_(n_x_,n_z_);
  Tc_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    VectorXd x_diff_ = Xsig_pred_.col(i) - x_;
    while (x_diff_(3) > M_PI) x_diff_(3) -= 2*M_PI;
    while (x_diff_(3) < -M_PI) x_diff_(3) += 2*M_PI;

    VectorXd z_diff_ = Zsig_.col(i) - z_pred_;
    while (z_diff_(1) > M_PI) z_diff_(1) -= 2*M_PI;
    while (z_diff_(1) < - M_PI) z_diff_(1) += 2*M_PI;

    Tc_ += weights_(i) * x_diff_ * z_diff_.transpose();
  }

  // Calculate Kalman gain
  MatrixXd K_ = Tc_ * S_.inverse();

  // Update state mean and covariance
  VectorXd z_pred_diff_ = meas_package.raw_measurements_ - z_pred_;
  while (z_pred_diff_(1) > M_PI) z_pred_diff_(1) -= 2*M_PI;
  while (z_pred_diff_(1) < -M_PI) z_pred_diff_(1) += 2*M_PI;
  x_ += K_ * z_pred_diff_;
  P_ -= K_ * S_ * K_.transpose();

  return;

}

