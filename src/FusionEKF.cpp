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
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
        0, 1, 0, 0;
    
  //create a 4D state vector, we don't know yet the values of the x state
  ekf_.x_ = VectorXd(4);
    
  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
    
  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
  ekf_.Q_ = MatrixXd(4, 4);
    
  //set the acceleration noise components
  noise_ax = 2;
  noise_ay = 2;
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
    * Initialize the state ekf_.x_ with the first measurement.
    * Create the covariance matrix.
    * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    if (measurement_pack.sensor_type_ == measurement_pack.LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_, 0, 0;
    } else {
      double rho = measurement_pack.raw_measurements_(0);
      double phi = measurement_pack.raw_measurements_(1);
      double rho_dot = measurement_pack.raw_measurements_(2);
      double px = rho * cos(phi);
      double py = rho * sin(phi);
      double vx = 0;
      double vy = 0;
      ekf_.x_  << px, py, vx, vy;
    }
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }
  // first measurement
  cout << "EKF: " << endl;
  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/ 100000;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;
    
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
    
  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
    
  //set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4*noise_ax/4,                0,  dt_3*noise_ax/2,                0,
                            0,  dt_4*noise_ay/4,                0,  dt_3*noise_ay/2,
              dt_3*noise_ax/2,                0,    dt_2*noise_ax,                0,
                            0,  dt_3*noise_ay/2,                0,    dt_2*noise_ay;
    
  //predict
  ekf_.Predict();
  std::cout << "~~x_= " << ekf_.x_ << std::endl;
  ekf_.H_ = H_laser_;
  
  if (measurement_pack.sensor_type_ == measurement_pack.RADAR) {
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.Hj_ = Hj_;
    ekf_.R_ = R_radar_;
    
    //measurement update
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.R_ = R_laser_;
    
    //measurement update
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  
    
  std::cout << "x_= " << ekf_.x_ << std::endl;
  std::cout << "P_= " << ekf_.P_ << std::endl;
}
