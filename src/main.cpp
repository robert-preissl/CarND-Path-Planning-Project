#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include <unordered_map>
#include "trajectory.h"
#include "behavior.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  std::unordered_map< std::string, vector<double> > map_data;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // for wrapping around the track
  map_waypoints_x.push_back(map_waypoints_x[0]);
  map_waypoints_y.push_back(map_waypoints_y[0]);
  map_waypoints_s.push_back(MAX_S);
  map_waypoints_dx.push_back(map_waypoints_dx[0]);
  map_waypoints_dy.push_back(map_waypoints_dy[0]);

  map_data["x"] = map_waypoints_x;
  map_data["y"] = map_waypoints_y;
  map_data["s"] = map_waypoints_s;
  map_data["dx"] = map_waypoints_dx;
  map_data["dy"] = map_waypoints_dy;

  // create the trajectory planner and behavior analyzer objects
  Trajectory trajectory_planner(map_data);
  Behavior behavior_analyzer;

  double prev_car_speed = 0.0;
  double prev_car_s = 0.0;
  double prev_car_d = 0.0;

  h.onMessage([&trajectory_planner,&behavior_analyzer,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &prev_car_speed, &prev_car_s, &prev_car_d](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {

            printf("\n \n ===== new cycle ===== \n \n");

        	  // Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

            // Previous path's end s and d values
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            // Previous path data given to the Planner
            vector<double> previous_path_x = j[1]["previous_path_x"];
            vector<double> previous_path_y = j[1]["previous_path_y"];

            int prev_path_size = previous_path_x.size();

            traffic own_car;
            if( prev_path_size == 0 ) {
              own_car = traffic(-1,
                                car_x,
                                car_y,
                                car_speed,
                                car_s,
                                car_d,
                                car_yaw );
            } else {
              own_car = traffic(-1,
                                previous_path_x[prev_path_size-1],
                                previous_path_y[prev_path_size-1],
                                prev_car_speed,
                                end_path_s,
                                end_path_d,
                                car_yaw );
            }
            // printf("\n Our car: "); own_car.print();

          	json msgJson;

            // only react to slow traffic (via potential lane changes) when we are
            //  about to recalculate the trajectory, which is done if the prev.
            //  path size falls under N_PREV_PATH
            if(prev_path_size < N_PREV_PATH) {
              behavior_analyzer.sense_traffic(j[1]["sensor_fusion"], car_s);

              behavior_analyzer.check_slowdown(own_car.s, own_car.d, own_car.vel);
            }

            vector<double> next_x_vals;
          	vector<double> next_y_vals;
            trajectory_planner.calculate_trajectory(own_car,
                                                    behavior_analyzer.get_state(),
                                                    behavior_analyzer.get_goal_d(),
                                                    previous_path_x,
                                                    previous_path_y,
                                                    next_x_vals,
                                                    next_y_vals,
                                                    prev_car_speed,
                                                    prev_car_s,
                                                    prev_car_d);

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

            /*printf("\n \n trajectory: \n");
            for( int i = 0; i < next_x_vals.size(); ++i ) {
              printf( "x = %f / y = %f \n", next_x_vals[i], next_y_vals[i] );
            }
            printf("\n");*/

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            behavior_analyzer.clear_surrounding_traffic();

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
