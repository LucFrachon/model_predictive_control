#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); ++j) {
    for (int i = 0; i < order; ++i) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Waypoints
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          // Current state of vehicle
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];  
          v = v * 1.609 / 3.6;  //convert speed to SI units

          //Initial actuation inputs
          double steer_value = j[1]["steering_angle"];  
          steer_value *= -1;  //In the simulator, right-hand turns are positive, unlike in cartesian coordinates
          
          double throttle_value = j[1]["throttle"];
          //Map throttle value to actual acceleration / deceleration (assuming 2 piecewise linear relationships)
          if (throttle_value >= 0)
            throttle_value *=  mpc.max_acc;
          else
            throttle_value *= mpc.min_acc;
          
          //To take latency into account, we predict the new vehicle position under current actuation inputs
          px = px + v * cos(psi) * mpc.latency / 1000.;
          py = py + v * sin(psi) * mpc.latency / 1000. ;
          psi = psi + v * steer_value / mpc.Lf * mpc.latency / 1000.;
          v = v + throttle_value * mpc.latency / 1000.;
         
          //Convert waypoints to vehicle coordinates
          for (unsigned int i = 0; i < ptsx.size(); ++i)
          {
            double ptsx_temp = ptsx[i];
            ptsx[i] = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
            ptsy[i] = -(ptsx_temp - px) * sin(psi) + (ptsy[i] - py) * cos(psi);
          }         
          
          //polyfit takes a VectorXd but ptsx and ptsy are C++ vectors so we need to convert them
          double* ptrx = &ptsx[0];
          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, ptsx.size());
          double* ptry = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, ptsy.size());         

          //mpc.printVector(ptsx_transform, ptsy_transform, "ptsx_transform, ptsy_transform");

          //Fit a third order polynomial to the waypoints
          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

          mpc.printVector(coeffs, "coeffs");
          
          // Current error values
          double cte = polyeval(coeffs, 0);  //py is zero in the vehicle's coordinate system
          double epsi = -atan(coeffs[1]);  //psi and px are zero in the vehicle's coordinate system, which simplifies the formula to this
          std::cout << "Current error values:\t" << cte << "\t" << epsi << std:: endl;

          //State vector to pass to solver
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;  //px, py and psi are 0 in car coordinates

          //Compute lowest-cost solution
          auto solution = mpc.Solve(state, coeffs);
          //mpc.printVector(solution, "Computed solution: ");

          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          steer_value = solution[0] / deg2rad(25);  //Scaled to [-1, 1]

          //Map back computed steering angle to [-1, 1]
          if (solution[1] >= 0)
            throttle_value = solution[1] / mpc.max_acc;
          else
            throttle_value = solution[1] / mpc.min_acc;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          for (size_t i = 2; i < solution.size(); ++i)
          {
            if (i % 2 == 0)
            {
              mpc_x_vals.push_back(solution[i]);
            }
            else
            {
              mpc_y_vals.push_back(solution[i]);
            }
          }

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          const double poly_inc = 2.5;
          const size_t num_points = 10;

          for (size_t i = 1; i < num_points; ++i)
          {
            next_x_vals.push_back(poly_inc * i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
          }

          
          //Send messages back to simulator
          json msgJson;
          msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"] = throttle_value;

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
         
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(mpc.latency));  //Set to 100ms
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
