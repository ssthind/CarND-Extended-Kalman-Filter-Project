#include "kalman_filter.h"
#include <iostream>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  cout << "now in init function " << endl;
  int x_size = x_.size();
}

void KalmanFilter::Predict() {
  /**

    * predict the state
  */
  	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
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
	int x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**

    * update the state by using Extended Kalman Filter equations
  */

	float rho, rhodot, phi;
	rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
	//cout << "check in UpdateEKF 1 "  << endl;
	phi = atan2(x_(1), x_[0]);
	rhodot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
	//rhodot = sqrt(x_(2)*x_(2) + x_(3)*x_(3));
	
	//cout << "check in UpdateEKF 3 "  << endl;
	Eigen::VectorXd z_pred_polar = VectorXd(3);
	z_pred_polar << rho, phi, rhodot;
	//cout << "check in UpdateEKF 4 z_pred_polar: "<< z_pred_polar << endl;

	VectorXd y = z - z_pred_polar;
/* 	if (y(1) > atan(1)*4){
		y(1) -= atan(1)*8;
		//cout << "check in UpdateEKF 5 y(1): " << y(1) << endl;
	}
	else if ((y(1) < (-1)*atan(1)*4)){
		y(1) += atan(1)*8;
		//cout << "check in UpdateEKF 5.1 y(1): " << y(1) << endl;
	} */
	y(1) = atan2(sin(y(1)), cos(y(1)));
	
	
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	int x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
  