#include "kalman_filter.h"

#include <iostream>

#include <math.h> 


using std::cout;
using std::endl;
using Eigen::MatrixXd;
using Eigen::VectorXd;


/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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

VectorXd h(VectorXd x) {
  	float eps = 0.0001;
	double rho = sqrt(x(0)*x(0) + x(1)*x(1));
    double phi = atan2(x(1) , x(0) + eps);
    double rho_dot = (x(0)*x(2) + x(1)*x(3)) / (rho + eps);
    VectorXd h = VectorXd(3);
    h << rho, phi, rho_dot;
  	return h;
}

void normalize(VectorXd &y) {
  while (y(1) < -M_PI) {y(1) += 2 * M_PI;}
  while (y(1) > M_PI) {y(1) -= 2 * M_PI;}
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  MatrixXd I = MatrixXd::Identity(4, 4);
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {  
  VectorXd y = z - h(x_);
  normalize(y);
  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  MatrixXd I = MatrixXd::Identity(4, 4);
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}
