#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <string>

using namespace std;

class MPC {
 public:
  MPC();
  //Set the latency value. 
  const int latency = 100;  //in ms
  // This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
  const double Lf = 2.67;
  const double max_acc = 3.9;  //Maximum estimated acceleration of vehicle, in m.s-2
  const double min_acc = 7.7;  //Absolute value of minimum negative acceleration ie. max deceleration under braking, in m.s-2. This value must be >0

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  void printVector(vector<double> x, vector<double> y, std::string labelText);

  void printVector(Eigen::VectorXd x, Eigen::VectorXd y, std::string labelText);

  void printVector(CPPAD_TESTVECTOR(double) x, CPPAD_TESTVECTOR(double) y, std::string labelText);

  void printVector(vector<double> x, std::string labelText);

  void printVector(Eigen::VectorXd x, std::string labelText);

  void printVector(CPPAD_TESTVECTOR(double) x, std::string labelText);
};

#endif /* MPC_H */
