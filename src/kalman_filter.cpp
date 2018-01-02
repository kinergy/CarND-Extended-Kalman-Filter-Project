#include "kalman_filter.h"
// #include <iostream>
// using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;  // state vector
  P_ = P_in;  // uncertainty covariance
  F_ = F_in;  // state transition matrix
  H_ = H_in;  // measurement function (map the state vector, x,  into the measurement space of the sensor)
  R_ = R_in;  // covariance of the measurement noise
  Q_ = Q_in;  // covariance of the process noise
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;

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
  // to calculate y, use h(x') to map the predicted x' from Cartesian coordinates to polar
  // to calculate S, K, P, use Hj instead of H
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  float rho = sqrt(px * px + py * py);
  float phi = atan2(py, px);
  float rho_dot = (px * vx + py * vy) / rho;

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;
	VectorXd y = z - z_pred;

  // cout << y[1] << " | ";

  // phi, y[1], needs to be normalized so that its angle is between -Pi and Pi. This is done by adding or subtracting 2*Pi until the value is in the correct range.

  while (y[1] > M_PI) {
    // cout << "subtracting ";
    y[1] -= 2*M_PI;
  }

  while (y[1] < -M_PI) {
    // cout << "adding ";
    y[1] += 2*M_PI;
  }

  // cout << y[1] << endl;

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
