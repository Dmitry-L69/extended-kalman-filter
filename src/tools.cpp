#include "tools.h"
#include <iostream>

#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
    VectorXd rmse(4);
    rmse << 0,0,0,0;
  
    if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
      return rmse;
    }
  
    for (int i=0; i < estimations.size(); ++i) {
      VectorXd residual = estimations[i] - ground_truth[i];
      residual = residual.array().pow(2);
      rmse += residual;
    }

    rmse /= estimations.size();
    rmse = rmse.array().sqrt();

    return rmse;
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  float sqrSum = pow(px, 2) + pow(py, 2);
  
  MatrixXd J(3,4);
  J << 	(px / sqrt(sqrSum)), (py / sqrt(sqrSum)), 0, 0,
  		(-py / sqrSum), (px / sqrSum), 0, 0,
  		(py*(vx*py - vy*px)/(sqrSum*sqrt(sqrSum))), (px*(px*vy - py*vx)/(sqrSum*sqrt(sqrSum))), (px/sqrt(sqrSum)), (py/sqrt(sqrSum));
  
  return J;

}
