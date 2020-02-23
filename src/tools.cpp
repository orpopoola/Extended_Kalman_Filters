#include "tools.h"
#include <iostream>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: accumulate squared residuals
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 
   float pxy = sqrt(px*px+py*py);
   float pxy2 = pxy*pxy;
   float pxy3 = pxy2*pxy;
   float pvx = vx*py-vy*px;
   float pvy = vy*px-vx*py;
  if(pxy < .00000001) {
    px += .000001;
    py += .000001;
    pxy = sqrt(px*px+py*py);
  }
    else {
        Hj << px/pxy, py/pxy, 0, 0,
             -py/pxy2, px/pxy2, 0, 0,
             py*pvx/pxy3, px*pvy/pxy3,px/pxy,py/pxy;
    }
  // check division by zero
  
  // compute the Jacobian matrix

  return Hj;
}
