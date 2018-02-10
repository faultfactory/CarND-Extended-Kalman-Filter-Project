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
  
  //cout<<endl<<"Prediction Started"<<endl;
  //cout<<endl<<"F"<<endl<<F_<<endl<<"x"<<endl<<x_<<endl;
  
  x_ = F_ * x_;
	//cout<<endl<<"Prediction step complete";
  
  MatrixXd Ft = F_.transpose();
  //cout<<endl<<"Ft written"<<endl;
  //cout<<endl<<"P"<<endl<<P_<<endl<<"Q_"<<endl<<Q_<<endl;

	P_ = F_ * P_ * Ft + Q_;

  //cout<<endl<<"Process Noise Completed"<<endl;
}

void KalmanFilter::Update(const VectorXd &z,const MatrixXd &H_laser_, const MatrixXd &R_laser_) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  cout<<endl<<"Laser Update"<<endl;
  VectorXd z_pred = H_laser_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_laser_.transpose();
	MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
  
  x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_laser_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z, const MatrixXd &Hj_, const MatrixXd &R_radar_) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  cout<<endl<<"Radar Update"<<endl;
  // convert predicted state to polar coords
  
  x_r_ = VectorXd(3);
  x_r_(0) = sqrt(pow(x_(0),2)+pow(x_(1),2));  
  if(x_r_(0)<0000.1){
    return;
  };
  
  x_r_(1) = atan2(x_(1),x_(0));
  x_r_(2) = (x_(0)*x_(2)+x_(1)*x_(3))/x_r_(0);
  
  //cout<<endl<<"x_r_"<<endl<<x_r_<<endl;
  //cout<<endl<<"Hj_"<<endl<<Hj_<<endl;
  //cout<<endl<<"z"<<endl<<z<<endl;

  VectorXd z_pred = x_r_;
	VectorXd y = z - z_pred;

	MatrixXd Ht = Hj_.transpose();  
	MatrixXd S = Hj_ * P_ * Ht + R_radar_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
  

  //cout<<endl<<"y"<<endl<<y<<endl;
  //cout<<endl<<"K"<<endl<<K<<endl;

  x_ = x_ + (K * y);
  //cout<<endl<<"state update completed"<<endl;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj_) * P_;
  
  //cout<<endl<<"noise update completed"<<endl;

}
