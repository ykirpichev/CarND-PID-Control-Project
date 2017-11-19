#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <algorithm>

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

void sendReset(uWS::WebSocket<uWS::SERVER>* ws)
{
    std::string msg = "42[\"reset\", {}]";
    ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}



class Twiddler {
public:
    Twiddler(PID* pid) :
        pid_(pid),
        bestError_(2000),
        counter_(0),
        twiddleStep_(0),
        params_({pid->Kp, pid->Kd, pid->Ki}),
        dParams_({1.0, 1.0, 0.1}),
        paramsIndex_(0),
        finished_(false)
    {}

    void checkTwiddle(uWS::WebSocket<uWS::SERVER>* ws) {
        ++counter_;

        int twiddleCounterLimit = 10000;
        if (counter_ > std::min(twiddleCounterLimit / 2, 500)) {
            accumulatedError_ += pid_->p_error * pid_->p_error;
        }

        if (counter_ >= twiddleCounterLimit || accumulatedError_ >= bestError_) {
            std::cout << "Twiddle: "
                      << " accumulated error: " << accumulatedError_
                      << " best error: " << bestError_ << std::endl;
            std::cout << "Twiddle: parameters: "
                      << " Kp = " << params_[0]
                      << " Kd = " << params_[1]
                      << " Ki = "<< params_[2] << std::endl;
            std::cout << "Twiddle: dparameters: "
                      << " dKp = " << dParams_[0]
                      << " dKd = " << dParams_[1]
                      << " dKi = "<< dParams_[2] << std::endl;
            twiddleOnce();
            sendReset(ws);
        }
    }

    void twiddleNextParam() {
        twiddleStep_ = 0;
        paramsIndex_ = (paramsIndex_ + 1) % dParams_.size();
        twiddleOnce();
    }

    void twiddleOnce() {
        if (twiddleStep_ == 0) {
            params_[paramsIndex_] += dParams_[paramsIndex_];
            twiddleStep_ = 1;
        } else if (twiddleStep_ == 1) {
            if (accumulatedError_ < bestError_) {
                bestError_ = accumulatedError_;
                dParams_[paramsIndex_] *= 1.3;

                twiddleNextParam();
            } else {
                twiddleStep_ = 2;
                params_[paramsIndex_] -= 2 * dParams_[paramsIndex_];
            }
        } else if (twiddleStep_ == 2) {
            if (accumulatedError_ < bestError_) {
                bestError_ = accumulatedError_;
                dParams_[paramsIndex_] *= 1.3;

                twiddleNextParam();
            } else {
                params_[paramsIndex_] += dParams_[paramsIndex_];
                dParams_[paramsIndex_] *= 0.7;

                twiddleNextParam();
            }
        }

        pid_->Init(params_[0], params_[1], params_[2]);
        counter_ = 0;
        accumulatedError_ = 0;
    }

    bool twiddleFinished() {
        return finished_;
    }

    void checkTwiddleFinished() {
        if (std::accumulate(dParams_.begin(), dParams_.end(), 0.0) < 0.0001) {
            finished_ = true;
        }
    }

private:
    PID* pid_;
    int counter_;
    int paramsIndex_;
    int twiddleStep_;
    std::vector<double> params_;
    std::vector<double> dParams_;
    double accumulatedError_;
    double bestError_;
    bool finished_;
};


int main()
{
    uWS::Hub h;

    PID pid;
//    pid.Init(0.15, 1.5, 0.0001);
//    pid.Init(1, 10.0, 0.01);
    pid.Init(0.2, 3, 0.002);


//    Twiddle: parameters:  Kp = 0.755967 Kd = 29.9241 Ki = 0.00566257
    pid.Init(0.755967, 29.9241, 0.00566257);

    Twiddler twiddler(&pid);

    h.onMessage([&pid, &twiddler](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    double speed = std::stod(j[1]["speed"].get<std::string>());
                    double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                    pid.UpdateError(cte);
                    double steer_value = std::min(std::max(pid.TotalError(), -1.0), 1.0);
                    /*
                    * [-1, 1].
                    * NOTE: Feel free to play around with the throttle and speed. Maybe use
                    * another PID controller to control the speed!
                    */

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = 0.3;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    if (!twiddler.twiddleFinished()) {
                        // twiddler.checkTwiddle(&ws);
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
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
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
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
