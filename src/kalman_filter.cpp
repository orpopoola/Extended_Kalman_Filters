#include "kalman_filter.h"

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

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
   x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
   VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  UpdateCommon(y);
}
//To check and correct range of angle
void NormalizeAngle(double& phi)
{
phi = atan2(sin(phi), cos(phi));
}
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    //recover state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  // Equations for h_ekf below
  float rho = sqrt(px * px + py * py);

  //check division by zero
  if(rho <0.00000001) {
    px += .000001;
    py += .000001;
    rho = sqrt(px*px+py*py);
  }
  float phi = atan2(py,px);
  float rho_dot = (px*vx+py*vy)/rho;   

  //Feed in equations above
  VectorXd H_ekfi(3);
  H_ekfi << rho, phi, rho_dot;
  VectorXd y = z - H_ekfi;
  // Check the angle
  
  NormalizeAngle(y(1));
//continue with KF algorithm
  UpdateCommon(y);
}
void KalmanFilter::UpdateCommon(const VectorXd &y){
const MatrixXd PHt = P_ * H_.transpose();
const MatrixXd S = H_ * PHt + R_;
const MatrixXd K = PHt*S.inverse();
x_ += K*y;
P_ -= K * H_ * P_;
}