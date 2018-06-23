#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include<chrono>

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
  for (int i = 0; i < coeffs.size(); i++) {
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

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

class MyTime {
  std::chrono::system_clock::time_point start_time;
public:
  void tick(){
    start_time = std::chrono::system_clock::now();
  };
  void tack(string txt){
    std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast< std::chrono::milliseconds >(end_time - start_time).count();
    std::cout << txt << ": duration = " << elapsed << "msec.\n";
  };
  void tack(void){
    tack(string("Time"));
  }
  double measure(){
    std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now();
    return std::chrono::duration_cast< std::chrono::milliseconds >(end_time - start_time).count();
  };
};


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
    MyTime mytime;
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          Eigen::VectorXd ptsx_car_coodinate(ptsx.size());
          Eigen::VectorXd ptsy_car_coodinate(ptsy.size());
          // Converting to car coodinate
          for(size_t i=0; i<ptsx.size(); i++){ //TODO. To be modified
            ptsx_car_coodinate[i] =  (ptsx[i]-px)*cos(psi) + (ptsy[i]-py)*sin(psi);
            ptsy_car_coodinate[i] = -(ptsx[i]-px)*sin(psi) + (ptsy[i]-py)*cos(psi);
          }
          auto coeffs = polyfit(ptsx_car_coodinate, ptsy_car_coodinate, 3);

          double cte = polyeval(coeffs, 0) - 0;
          double epsi = 0 - atan(coeffs[1]);
          //double epsi = psi - atan(coeffs[1]);
          cout << "cte= "<<cte<<",\tepsi="<<epsi<<endl;
          double max_v = 40.0;
          double min_v = 30.0;
          // if(cte>1.5){change_ref_v(min_v);}
          // else{change_ref_v(max_v);}

          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          std::vector<double> x_vals = {state[0]};
          std::vector<double> y_vals = {state[1]};
          double steer_value;
          double throttle_value;

          mytime.tick();
          auto vars = mpc.Solve(state, coeffs);
          x_vals.push_back(vars[0]);
          y_vals.push_back(vars[1]);

          steer_value = vars[6] * (-1.0);
          throttle_value = vars[7];
          //if(cte>1.3 && throttle_value>0.0) throttle_value=0;

          state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];

          mytime.tack(string("solver"));

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals = x_vals; //TODO?
          vector<double> mpc_y_vals = y_vals; //TODO?

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          next_x_vals.clear();
          next_y_vals.clear();
          for(int i=0; i<20; i++){ //TODO. To be modified
            double x_car = ((double)i)*4.0;
            double y_car = polyeval(coeffs, x_car);
            next_x_vals.push_back(x_car);
            next_y_vals.push_back(y_car);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
