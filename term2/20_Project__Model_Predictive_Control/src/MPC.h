#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

void change_ref_v(double v);
void update_current_control(double cdelta, double ca);

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  // vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, vector<double> &xs, vector<double> &ys);
};

#endif /* MPC_H */
