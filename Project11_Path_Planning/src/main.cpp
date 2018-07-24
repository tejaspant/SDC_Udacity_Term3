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
#include "spline.h"
#include "vehicle.h"
#include <map>
#include "input.h"
#include "track.h"

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

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  // initial lane ID
  int lane = 1;

  // Reference velocity
  double ref_vel = 0.0;

  //initialize map
  Track track;
  track.read_map_and_interpolate(map_file_);

  bool start_sim = true;

   h.onMessage([&track,&ref_vel,&lane,&start_sim](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);


      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
			// This is in global coordinate system
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

            int prev_size = previous_path_x.size();
			// prev_size = previous points not eaten up by simulator in one time step

			if (prev_size > 0){
				car_s = end_path_s;
			}

			//////////////////////////////////////////////////////////// PREDICTIONS NON-EGO VEHICLES ////////////////////////////////////////////////
			map<int ,vector<Vehicle> > non_ego_predictions;
			vector<Vehicle> non_ego_vehicles;

			bool car_on_the_left = false;
			bool car_on_the_right = false;
			bool car_ahead = false;

			//////////////////////////////////////////////////////////// SENSOR FUSION ////////////////////////////////////////////////
			for (int i = 0; i < sensor_fusion.size(); i++){
				int v_id = sensor_fusion[i][0];
                //cout << "v_id = " << v_id << endl;
				double d_car = sensor_fusion[i][6];
                //int lane_num = track.get_lane_num_from_d(d_car);
                //cout << "i = " << i << "lane_num = " << lane_num << "d_car = "<<d_car<< endl;
				double vx = sensor_fusion[i][3];
			    double vy = sensor_fusion[i][4];
			    double check_speed = sqrt(vx * vx + vy * vy);
				double check_car_s = sensor_fusion[i][5];
				int lane_num_non_ego = track.get_lane_num_from_d(d_car);
				//cout << "i = " << i << "lane_num = " << lane_num_non_ego << "d_car = "<<d_car<< endl;
				//Prediction of non ego vehicle
				check_car_s += ((double)prev_size * 0.02 * check_speed);
				if (lane == lane_num_non_ego){
					if ((check_car_s > car_s) && ((check_car_s - car_s) < SAFETY_DT_S)){
						//cout << "WARNING: Car in the front, slow down" << endl;
						// NEED TO WORK IN LANES INSTEAD OF D
						car_ahead = true;
						}
				} else if (lane - 1 == lane_num_non_ego){
					if (fabs(check_car_s - car_s) < SAFETY_DT_S){
						//cout << "WARNING: Car to the left cannot turn left" << endl
						car_on_the_left = true;
						}
				} else if (lane + 1 == lane_num_non_ego){
					if (fabs(check_car_s - car_s) < SAFETY_DT_S){
						//cout << "WARNING: Car to the right cannot turn right" << endl;
						car_on_the_right = true;
						}
				}
			}
			/////////////////////////////////////////// BEHAVIOR //////////////////////////////////////////////////////////////////
			double change_speed = 0;
			if(car_ahead) {
				//if(lane != TOTAL_NUM_LANES - 1 && !car_on_the_right){
				if(lane == 0 && !car_on_the_right){
					lane += 1;
					change_speed += MAX_ACC;
				} else if (lane != 0 && !car_on_the_left){
					lane -= 1;
					change_speed += MAX_ACC;
				} else if (!car_on_the_left && !car_on_the_right) {
					lane -= 1;
					change_speed += MAX_ACC;
				} else { //Stay in lane
					change_speed -= MAX_ACC;
				}
			} else if (lane == TOTAL_NUM_LANES - 1){
				if (!car_on_the_left) {
					lane -= 1;
					change_speed += MAX_ACC;
				}
			} else{
				change_speed += MAX_ACC;
			}

			vector<double> ptsx;
			vector<double> ptsy;

			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = car_yaw;

			vector<double> next_x_vals;
          	vector<double> next_y_vals;

            // previous path points used in generating the spline
			if (prev_size < 2) {
				// helps create tangent to path of car. This way transition will be smooth
				double prev_car_x = car_x - cos(car_yaw);
				double prev_car_y = car_y - sin(car_yaw);

				ptsx.push_back(prev_car_x);
				ptsx.push_back(car_x);

				ptsy.push_back(prev_car_y);
				ptsy.push_back(prev_car_y);
			}
			else{
				ref_x = previous_path_x[prev_size-1];
				ref_y = previous_path_y[prev_size-1];

				double ref_x_prev = previous_path_x[prev_size-2];
				double ref_y_prev = previous_path_y[prev_size-2];
				ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

				ptsx.push_back(ref_x_prev);
				ptsx.push_back(ref_x);

				ptsy.push_back(ref_y_prev);
				ptsy.push_back(ref_y);
			}

			// Need 3 additional points for constructing the spline
			vector<double> way_pt0 = track.getXYFromSpline(car_s + 30, (2+4*lane));
			vector<double> way_pt1 = track.getXYFromSpline(car_s + 60, (2+4*lane));
			vector<double> way_pt2 = track.getXYFromSpline(car_s + 90, (2+4*lane));

			ptsx.push_back(way_pt0[0]);
			ptsx.push_back(way_pt1[0]);
			ptsx.push_back(way_pt2[0]);

			ptsy.push_back(way_pt0[1]);
			ptsy.push_back(way_pt1[1]);
			ptsy.push_back(way_pt2[1]);

			// transform to car cordinates for generating path
			// if prev_size => 2 we assume car is at the last point of previous point and use it as as origin for transormation
			for (int i = 0; i < ptsx.size(); i++){
				double shift_x = ptsx[i] - ref_x;
				double shift_y = ptsy[i] - ref_y;

				ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
				ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
			}

			tk::spline s;
			s.set_points(ptsx,ptsy);

			// add remaining points from previous path into next values
			for (int i = 0; i < prev_size; i++){
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			double target_x = 30;
			double target_y = s(target_x);
			double target_dt = sqrt((target_x*target_x) + (target_y*target_y));

			double x_add_on = 0;

			for(int i = 0; i <= 50 - prev_size; i++){
				ref_vel += change_speed;
				if ( ref_vel > MAX_SPEED ) {
					ref_vel = MAX_SPEED;
					} 
				double N = (target_dt / (0.02*ref_vel/2.24));
				double x_point = x_add_on + (target_x / N);
				double y_point = s(x_point);

				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;

                //Convert from car coordinates to map coordinates
				x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
				y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

				x_point += ref_x;
			    y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);

			}


			// Straight line and car remains along path without spline functions
			/*
			double dist_inc = 0.4;
			for(int i = 0; i < 50; i++)
			{
				double next_s = car_s + dist_inc*(i+1);
				double next_d = 6;
				vector<double> next_xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                next_x_vals.push_back(next_xy[0]);
				next_y_vals.push_back(next_xy[1]);
				//next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
				//next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
				}
			*/

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
