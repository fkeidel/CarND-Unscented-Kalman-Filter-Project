#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * Helper methods to convert between cartesian and polar coordinates
  */
  VectorXd ConvCart2Polar(const VectorXd& x_state);
  VectorXd ConvPolar2Cart(const VectorXd& x_polar);
};

#endif /* TOOLS_H_ */