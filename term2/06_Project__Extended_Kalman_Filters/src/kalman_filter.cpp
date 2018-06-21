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
  float _px = x_(0);
  float _py = x_(1);
  float _vx = x_(2);
  float _vy = x_(3);
	float _rho = sqrt(_px*_px+_py*_py);
	if(fabs(_px) < 0.0001){
		cout << "UpdateEKF() - Error - Division by Zero: px" << endl;
	}
	else if(fabs(_rho) < 0.0001){
		cout << "UpdateEKF() - Error - Division by Zero: rho" << endl;
	}
	else{
    float _phi = atan2(_py,_px);
    float _rho_dot = (_px*_vx+_py*_vy)/_rho;
    VectorXd z_pred(3);
    z_pred << _rho, _phi, _rho_dot;
  	VectorXd y = z - z_pred;
    while(y(1)<=-M_PI)y(1)+=2*M_PI;
    while(y(1)>M_PI)y(1)-=2*M_PI;
  	MatrixXd Ht = H_.transpose();
  	MatrixXd S = H_ * P_ * Ht + R_;
  	MatrixXd Si = S.inverse();
  	MatrixXd PHt = P_ * Ht;
  	MatrixXd K = PHt * Si;

  	x_ = x_ + (K * y);
  	long x_size = x_.size();
  	MatrixXd I = MatrixXd::Identity(x_size, x_size);
  	P_ = (I - K * H_) * P_;
  }
}
