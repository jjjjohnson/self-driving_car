#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;
// Weight factors for cost optimization

struct MPC_Weights {
    const double w_cte = 4000;
    const double w_epsi = 4000;
    const double w_v = 1;
    const double w_angle = 100;
    const double w_accel = 100;
    const double w_angle_jerk = 50000;
    const double w_accel_jerk = 1000;
    const double w_norm = 1 / (w_cte + w_epsi + w_v + w_angle + w_accel + w_angle_jerk + w_accel_jerk);
} ;


class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
