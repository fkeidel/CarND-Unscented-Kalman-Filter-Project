#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() != ground_truth.size()
		|| estimations.size() == 0) {
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse / estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

VectorXd Tools::ConvCart2Polar(const VectorXd& x_state)
{
  //recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  double rho = sqrt(px*px + py*py);
  double phi = atan2(py, px);
  double rho_dot = (rho > 0) ? (px*vx + py*vy) / rho : 0;

  VectorXd x_polar(3);
  x_polar << rho, phi, rho_dot;

  return x_polar;
}

VectorXd Tools::ConvPolar2Cart(const VectorXd& x_polar)
{
  //recover state parameters
  double rho = x_polar(0);
  double phi = x_polar(1);
  double rho_dot = x_polar(2);

  double px = rho*cos(phi);
  double py = rho*sin(phi);
  double vx = rho_dot*cos(phi);
  double vy = rho_dot*sin(phi);

  VectorXd x_state(4);
  x_state << px, py, vx, vy;

  return x_state;
}