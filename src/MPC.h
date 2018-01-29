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
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  //Convert vehicle position and orientation from map coordinates to vehicle coordinates
  vectorXd transformCoord_MapToVehicle(Eigen::VectorXd state_m);

  //Convert vehicle position and orientation from vehicle coordinates to map coordinates
  vectorXd transformCoord_VehicleToMap(Eigen::VectorXd state_m);


};

#endif /* MPC_H */
