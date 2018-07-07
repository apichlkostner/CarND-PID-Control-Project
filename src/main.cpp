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

constexpr bool PARAMETER_OPTIMIZATION_ENABLED = true;

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

  // initial pid parameters
  // std::vector<double> p{0.29614, 0.22, 0.130195};
  std::vector<double> p{0.215795, 0.0994473, 0.1131};
  // initial delta parameters for optimization algorithm
  // std::vector<double> dp{0.00819321, 0.0310672, 0.0131173};
  std::vector<double> dp{0.0235795, 0.0117406, 0.0175385};

  // double err_init = 184.077;
  double err_init = 100000;

  ParameterSearch ps(p, dp, err_init);
#if 1
  PID pid(p);
#else
  PID pid(0.08, 0.1, 0.02);
#endif
  PID pid_v(0.3, 0.05, 0.005);
  double cte_int = 0;

  h.onMessage([&pid, &pid_v, &ps, &cte_int](uWS::WebSocket<uWS::SERVER> ws,
                                            char *data, size_t length,
                                            uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = pid.calc(cte);

          double throttle = pid_v.calc(speed - TARGET_SPEED);

          // currently not used
          (void)angle;

          // test reset of simulator with message
          if (PARAMETER_OPTIMIZATION_ENABLED) {
            static int mycnt = 0;

            // sum of errors for parameter optimization
            cte_int += cte * cte * cte * cte;

            if (mycnt++ > 1500) {
              // reset simulator
              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

              // next parameter set
              pid = ps.next(cte_int);

              // reset velocity pid and accumulated error
              pid_v.reset();
              cte_int = 0;

              mycnt = 0;
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
