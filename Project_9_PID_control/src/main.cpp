#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
using namespace std;
// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  vector<double> pid_param = {0.2,0.0001,5.5};
  vector<double> dp = {1, 0.001, 1};
  pid.Init(pid_param[0],pid_param[1],pid_param[2], false);

  int counter = 0;
  double best_error = 0;
  double cumm_error = 0;
  int switchTweeker = 0;
  bool up, optimize;
  optimize = false; // flag to use twiddle find best parameter or not

  h.onMessage([&pid, &counter, &best_error, &cumm_error, &pid_param, &dp, &switchTweeker, &up, &optimize]\
(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      int steps = 500;
      if (length && length > 2 && data[0] == '4' && data[1] == '2') {
        auto s = hasData(std::string(data).substr(0, length));
        if (s != ""){
          auto j = json::parse(s);
          std::string event = j[0].get<std::string>();
          if (event == "telemetry" ) {
            // j[1] is the data JSON object
            double cte = std::stod(j[1]["cte"].get<std::string>());
            double speed = std::stod(j[1]["speed"].get<std::string>());
            double angle = std::stod(j[1]["steering_angle"].get<std::string>());
            double steer_value;

            pid.UpdateError(cte);
            steer_value = pid.TotalError();

            // DEBUG
//            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;


            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          if (optimize){
            counter += 1;
//            cout << "counter:" << counter << endl;
            cumm_error += cte * cte;

            double dp_sum = dp[0] + dp[1] + dp[2];
            if (counter == steps) {
              if (!pid.is_initialized) {
                best_error = cumm_error;
                pid.is_initialized = true;
                up = true;
              } else if (cumm_error < best_error) {
                // improvement
                best_error = cumm_error;
                if (switchTweeker == 0) dp[switchTweeker] *= 1.1;
                else if (switchTweeker == 1) dp[switchTweeker] *= 1.1;
                else dp[switchTweeker] *= 1.1;
                switchTweeker = (switchTweeker + 1) % 3;
                up = true;
              } else {
                if (up == true) up = false;
                else {
                  if (switchTweeker == 0) {
                    pid_param[switchTweeker] += dp[switchTweeker];
                    dp[switchTweeker] *= 0.9;
                  } else if (switchTweeker == 1) {
                    pid_param[switchTweeker] += dp[switchTweeker];
                    dp[switchTweeker] *= .9;
                  } else {
                    pid_param[switchTweeker] += dp[switchTweeker];
                    dp[switchTweeker] *= .9;
                  }
                  switchTweeker = (switchTweeker + 1) % 3;
                  up = true;
                }
              }
              if (switchTweeker == 0) {
                if (up) pid_param[switchTweeker] += dp[switchTweeker];
                else pid_param[switchTweeker] -= 2 * dp[switchTweeker];
              }
              if (switchTweeker == 1) {
                if (up) pid_param[switchTweeker] += dp[switchTweeker];
                else pid_param[switchTweeker] -= 2 * dp[switchTweeker];
              }
              if (switchTweeker == 2) {
                if (up) pid_param[switchTweeker] += dp[switchTweeker];
                else pid_param[switchTweeker] -= 2 * dp[switchTweeker];
              }
              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              counter = 0;
              cumm_error = 0;
              pid.Init(pid_param[0], pid_param[1], pid_param[2], true);
              cout << "best_error: " << best_error << ", Kp: " << pid_param[0] << ", Ki: " << pid_param[1] << ", Kd: "
                   << pid_param[2] << endl;
            }
          }
          }

            } else {
              // Manual driving
              std::string msg = "42[\"manual\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }

        }
    });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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

