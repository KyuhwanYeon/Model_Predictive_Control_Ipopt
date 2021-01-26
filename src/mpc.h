#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

// weights
#define w_cte 6
#define w_epsi 10000
#define w_verr 6
#define w_delta 100
#define w_a 10
#define w_d_delta 10
#define w_d_a 100

using namespace std;
// too
class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // return actuations, throttle/steer
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
