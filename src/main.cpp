#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "PID.h"

#include "ParameterSearch.h"
#include "json.hpp"
// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

constexpr bool PARAMETER_OPTIMIZATION_ENABLED = false;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  constexpr double TARGET_SPEED = 30;

  // initial pid parameters (best from last run of optimization)
  std::vector<double> p{0.15592, 0.069404, 0.05};
  // initial delta parameters for optimization algorithm
  std::vector<double> dp{0.0018432, 0.000576, 0.000512};
  // last error of parameter search
  double err_init = 1e40;

  ParameterSearch ps(p, dp, err_init);

  // pid for steering angle
  PID pid(p);

  // pid for velocity
  PID pid_v(0.3, 0.05, 0.005);

  // sum of error terms for parameter search
  double cte_int = 0;
  bool first_start = true;

  Logger measure_log("meas_" + std::to_string(p[0]) + "_" +
                     std::to_string(p[1]) + "_" + std::to_string(p[2]) +
                     "2.csv");

  h.onMessage([&pid, &pid_v, &ps, &cte_int, &first_start, &measure_log](
                  uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      static int startcnt = 0;
      startcnt++;

      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          if (first_start) {
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            first_start = false;
          }
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          // steering value in range [-1, 1] from PID controller
          double steer_value = 0.;

          // throttle setting from velocity pid controller
          double throttle = 0.;

          // Wait after reset and ignore possible old values
          constexpr int RESET_TIME = 10;
          if (startcnt > RESET_TIME) {
            steer_value = pid.calc(cte);
            throttle = pid_v.calc(speed - TARGET_SPEED);
          }

          // logging
          std::vector<double> measurements{cte, steer_value, angle, speed,
                                           throttle};
          measure_log.log(std::to_string(pid.Kp_) + "_" +
                              std::to_string(pid.Kd_) + "_" +
                              std::to_string(pid.Ki_),
                          measurements);

          // test reset of simulator with message
          if (PARAMETER_OPTIMIZATION_ENABLED) {
            static int runningcounter = 0;

            // sum of errors for parameter optimization
            // cte^2 and steer_value^2 should be minimized
            constexpr double ERROR_WEIGHT = 0.15;
            cte_int += ERROR_WEIGHT * cte * cte +
                       (1.0 - ERROR_WEIGHT) * steer_value * steer_value;

            // change of steer_value sign should be penalized
            static double steer_value_old = 0;
            static double oscillation_error = 0;
            if (((steer_value > 0) && (steer_value_old < 0)) ||
                ((steer_value < 0) && (steer_value_old > 0))) {
              constexpr double OSCERRDELTA = 0.6;
              cte_int += OSCERRDELTA;
              oscillation_error += OSCERRDELTA;
            }

            steer_value_old = steer_value;

            // run for maximum number of steps or till the car leaves the road
            // cte might be high since some old messages from the last try can
            // still arrive
            constexpr int RUNNINGCOUNTERMAX = 1500;
            if ((runningcounter++ > RUNNINGCOUNTERMAX) ||
                ((runningcounter > 20) && (fabs(cte) > 3.))) {
              // reset simulator
              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

              if (fabs(cte) > 3.)
                cte_int *= (RUNNINGCOUNTERMAX - runningcounter) * 1e20;

              // next parameter set
              pid = ps.next(cte_int);

              std::cout << "Oscillation error = " +
                               std::to_string(oscillation_error)
                        << std::endl;

              // reset velocity pid and accumulated error
              pid_v.reset();
              cte_int = 0;
              oscillation_error = 0;
              runningcounter = 0;
              startcnt = 0;
            }
          } else {
            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                      << std::endl;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
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
  // program doesn't compile :-(
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
    if (!PARAMETER_OPTIMIZATION_ENABLED)
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
