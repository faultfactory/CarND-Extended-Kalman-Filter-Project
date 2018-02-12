#include "kalman_filter.h"
#include <iostream>
#include "math.h"

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
    * predict the state - COMPLETE
  */

  
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z,const MatrixXd &H_laser_, const MatrixXd &R_laser_) {
  /**
  TODO:
    * update the state by using Kalman Filter equations - COMPLETE
  */

  //cout<<endl<<"Laser Update"<<endl;
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
  //cout<<endl<<"Radar Update"<<endl;
  // convert predicted state to polar coords
  float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

  x_r_ = VectorXd(3);
  x_r_(0) = sqrt((px*px)+(py*py));  
  x_r_(1) = (px != 0 ? atan2(py,px) : M_PI/2.0);
  x_r_(2) = (x_r_(0) != 0 ? ((px*vx)+(py*vy))/x_r_(0) : 0);
  


  VectorXd z_pred = x_r_;     
	VectorXd y = z - z_pred;

  if(fabs(y(1))>M_PI){
    cout<<endl<<"AngleLimExceeded!!!"<<endl;
    if(y(1)>0){
      y(1)-=(2*M_PI);
    } else {
      y(1)+=(2*M_PI);
    }
  } 

	MatrixXd Ht = Hj_.transpose();  
	MatrixXd S = Hj_ * P_ * Ht + R_radar_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
 
  x_ = x_ + (K * y);
 

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj_) * P_;
  

}
