#include "MPC.h"
#include "Eigen-3.3/Eigen/Core"
#include <math.h>
#include <assert.h>

using CppAD::AD;

//Set the timestep length and duration
const size_t N = 7;
const double dt = 0.15;
//Geometric constant
const double  Lf = 2.67;

//Target speed
const double ref_v = 50 * 1.609 / 3.6;  //in m.s-1

//Define aliases for starting indices of each state variable within fg[] (convenience)
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    
    // ## Cost calculation ##
    fg[0] = 0;

    //Cost of reference state
    for (size_t t = 0; t < N; ++t) {
      fg[0] += 10 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 10 * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += 10 * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    //Minimize actuator inputs (smoother trajectory changes)
    for (size_t t = 0; t < N - 1; ++t) {
      fg[0] += 10 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 5 * CppAD::pow(vars[a_start + t], 2);
    }
    //Minimize actuator input gradients (smoother inputs)
    for (size_t t = 0; t < N - 2; ++t) {
      fg[0] += 10 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 1 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // ## Constraints definition ##

    //Initial constraints. Since fg[0] contains the cost function, each of the constraint starting indices
    //  is incremented by 1.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    //Subsequent constraints
    for (size_t t = 0; t < N - 1; ++t) {
      //State at time t
      AD<double> x0 = vars[x_start + t];
      AD<double> y0 = vars[y_start + t];
      AD<double> psi0 = vars[psi_start + t];
      AD<double> v0 = vars[v_start + t];
      AD<double> cte0 = vars[cte_start + t];
      AD<double> epsi0 = vars[epsi_start + t];

      //State at time t + 1
      AD<double> x1 = vars[x_start + t + 1];
      AD<double> y1 = vars[y_start + t + 1];
      AD<double> psi1 = vars[psi_start + t +1];
      AD<double> v1 = vars[v_start + t + 1];
      AD<double> cte1 = vars[cte_start + t + 1];
      AD<double> epsi1 = vars[epsi_start + t + 1];

      //Actuations (only at time t)
      AD<double> delta0 = vars[delta_start + t];
      AD<double> a0 = vars[a_start + t];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * pow(x0, 2));

      //Set up constraints. For instance x(t+1) = x(t) + v(t) * cos(psi(t)) * dt so
      //  we set fg[2 + x_start + t] = x(t+1) - x(t) - v(t) * cos(psi(t)) * dt, 
      //  then later we will set fg[1 + x_start + t] to 0 (upper bound == 0, lower bound == 0).
      fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  //assert(4 * N * dt >= latency / 1000.0);  //N * dt must be significantly greater than latency
  
  //State has 6 elements and actuator input has 2. For actuations,
  //  we don't have any values for the last time step. So the total
  //  number of variables is 6 N + 2 (N-1)
  size_t n_vars = 6 * N + 2 * (N - 1);
  size_t n_constraints = 6 * N;  //One constraint per element of the state vector per timestep

  
  // Initial value of the independent variables.
  // Should be 0 besides initial state.
  Dvector vars(n_vars);

  for (size_t i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }


  //Initial state values
  double x_ini = state[0];
  double y_ini = state[1];
  double psi_ini = state[2];
  double v_ini = state[3];
  double cte_ini = state[4];
  double epsi_ini = state[5];


  vars[x_start] = x_ini;
  vars[y_start] = y_ini;
  vars[psi_start] = psi_ini;
  vars[v_start] = v_ini;
  vars[cte_start] = cte_ini;
  vars[epsi_start] = epsi_ini;

  //Set lower and upper bounds for variables
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  for (size_t i = 0; i < delta_start; ++i)
  //Set state variable bounds to very low and very high values
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (size_t i = delta_start; i < a_start; ++i)
  //Steering inputs (+/-25deg converted to radians)
  {
    vars_lowerbound[i] = -25. * M_PI / 180.;
    vars_upperbound[i] = 25. * M_PI / 180.;
  }

  for (size_t i = a_start; i < n_vars; ++i)
  //Throttle inputs; bound between -1 and 1
  {
    vars_lowerbound[i] = -min_acc;  //Here we take the actual min and max acceleration values
    vars_upperbound[i] = max_acc;   //  not their mapped throttle values
  }
  
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  for (size_t i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  //Initial state:
  constraints_lowerbound[x_start] = state[0];
  constraints_lowerbound[y_start] = state[1];
  constraints_lowerbound[psi_start] = state[2];
  constraints_lowerbound[v_start] = state[3];
  constraints_lowerbound[cte_start] = state[4];
  constraints_lowerbound[epsi_start] = state[5];

  constraints_upperbound[x_start] = state[0];
  constraints_upperbound[y_start] = state[1];
  constraints_upperbound[psi_start] = state[2];
  constraints_upperbound[v_start] = state[3];
  constraints_upperbound[cte_start] = state[4];
  constraints_upperbound[epsi_start] = state[5];

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.1\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  //Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`. We return the first value of each variable and discard the other N-1

  vector<double> result;

  
  //Collect calculated actuations for the 1st timestep that occurs later than latency
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  for (size_t i = 0; i < N - 1; ++i)
  //Collect predicted x and y positions for future timesteps, so we can plot the trajectory in the sim
  {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }

  return result;
}


//Print vectors of different types (one vector or two vectors of identical lengths)
void MPC::printVector(vector<double> x, vector<double> y, std::string labelText)
//For debugging
{
  assert(x.size() == y.size());
  std::cout << "\n" << labelText << "\n";
  for (size_t i = 0; i < x.size(); ++i)
  {
    std::cout << x[i] << "\t";
    std::cout << y[i] << "\n";
  }
}

void MPC::printVector(Eigen::VectorXd x, Eigen::VectorXd y, std::string labelText)
//For debugging
{
  assert(x.size() == y.size());
  std::cout << "\n" << labelText << "\n";
  for (size_t i = 0; i < x.size(); ++i)
  {
    std::cout << x[i] << "\t";
    std::cout << y[i] << "\n";
  }
}

void MPC::printVector(CPPAD_TESTVECTOR(double) x, CPPAD_TESTVECTOR(double) y, std::string labelText)
//For debugging
{
  assert(x.size() == y.size());
  std::cout << "\n" << labelText << "\n";
  for (size_t i = 0; i < x.size(); ++i)
  {
    std::cout << x[i] << "\t";
    std::cout << y[i] << "\n";
  }
}

void MPC::printVector(vector<double> x, std::string labelText)
//For debugging
{
  std::cout << "\n" << labelText << "\n";
  for (size_t i = 0; i < x.size(); ++i)
  {
    std::cout << x[i] << "\t";
  }
  std::cout << std::endl;
}

void MPC::printVector(Eigen::VectorXd x, std::string labelText)
//For debugging
{
  std::cout << "\n" << labelText << "\n";
  for (size_t i = 0; i < x.size(); ++i)
  {
    std::cout << x[i] << "\t";
  }
  std::cout << std::endl;
}

void MPC::printVector(CPPAD_TESTVECTOR(double) x, std::string labelText)
//For debugging
{
  std::cout << "\n" << labelText << "\n";
  for (size_t i = 0; i < x.size(); ++i)
  { 
    std::cout << x[i] << "\t";
  }
  std::cout << std::endl;
}

