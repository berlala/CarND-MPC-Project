#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs,vector<double>& xvals_plan, vector<double>& yvals_plan);
 private:
  double polyeval(Eigen::VectorXd coeffs, double x);

};

#endif /* MPC_H */
