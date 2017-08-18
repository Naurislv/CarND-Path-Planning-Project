#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "external/json.hpp"
#include "helpers.h"
#include "vehicle.h"
#include "map.h"
#include "states.h"
#include "predict.h"

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

	Map map;
	map.load_data("../data/highway_map.csv");

	Vehicle vehicle = Vehicle(map);
	States states = States();
	Predict predict = Predict(map);

	vehicle.lane = 1;
	vehicle.ref_vel = 48.9; // 48.9

	predict.max_front_distance = 30;
	predict.min_front_distance = 12;
	predict.safe_back_distance = 10;

    h.onMessage([&vehicle, &states, &predict](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);

		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);
			if (s != "") {
				auto j = json::parse(s);
				
				string event = j[0].get<string>();
				if (event == "telemetry") {
					// j[1] is the data JSON object
				
					// Main car's localization Data
					states.car_x = j[1]["x"];
					states.car_y = j[1]["y"];
					states.car_s = j[1]["s"];
					states.car_d = j[1]["d"];
					states.car_yaw = j[1]["yaw"];
					states.car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					states.set_previous_path_x(j[1]["previous_path_x"]);
					states.set_previous_path_y(j[1]["previous_path_y"]);
					
					// Previous path's end s and d values 
					states.end_path_s = j[1]["end_path_s"];
					states.end_path_d = j[1]["end_path_d"];

					if (states.prev_size > 0) {
						states.car_s = states.end_path_s;
					}

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					states.set_sensor_fusion(j[1]["sensor_fusion"]);
					states.set_prev_size();  // store prevous path size for later use

					vector<double> next_x_vals;
					vector<double> next_y_vals;

					vehicle.states = states; // set states for vehicle class
					predict.states = states; // set states for predict class

					predict.movement(vehicle.goal_vel, vehicle.ref_vel, vehicle.lane);

					vehicle.apply_path(
						next_x_vals,
						next_y_vals
					);

					json msgJson;
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
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
















































































