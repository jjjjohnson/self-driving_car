#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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

void KalmanFilter::Update_routine(const VectorXd &y){
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd Si = S.inverse();
    
    MatrixXd K = PHt * Si;
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  Update_routine(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float phi_pred = 0.0;


    float rho = sqrt(px*px + py*py);
    //Avoid deviding by zero
    if(rho < 0.0001) rho = 0.0001;

  if((fabs(px) < 1e-3) && (fabs(py) < 1e-3)){
    px = 1e-3;
      py = 1e-3;
  }
  else if(fabs(px) < 1e-3){
      px = 1e-3;
  }


    phi_pred = atan2(py, px);
    VectorXd z_pred(3);
    z_pred << rho, phi_pred, (px*vx + py*vy)/rho;

    VectorXd y = z - z_pred;

    while(y(1) < -PI)
        y(1) += 2 * PI;
    while(y(1) > PI)
        y(1) -= 2 * PI;

    Update_routine(y);
}
