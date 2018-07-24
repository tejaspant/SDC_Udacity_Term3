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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
//gives the closest waypoint to the argument x, y value
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_s)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = maps_s[0];
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  //double max_s = 6945.554;

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

  // initial lane ID
  int lane = 1;

  // Reference velocity
  double ref_vel = 0.0;

  //initialize map
  Track track;
  track.read_map_and_interpolate(map_file_);

   h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&ref_vel,&lane,&track](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
			// previous path points in global coordinates
			if (prev_size > 0){
				car_s = end_path_s;
			}

			//////////////////////////////////////////////////////////// INTERPOLATION OF WAYPOINTS ////////////////////////////////////////////////

			// Need to interpolate waypoints to get accurate Frenet coordinates

			// Get the anchor points for spline interpolation of way points
			int total_waypoints = map_waypoints_x.size();
			vector<double> map_waypoints_x_for_inter, map_waypoints_y_for_inter, map_waypoints_s_for_inter;
			vector<double> map_waypoints_dx_for_inter, map_waypoints_dy_for_inter;

			int next_waypoint_near_ego = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_x);
			//cout << "waypoint = " << next_waypoint_near_ego << "ego_s = " << car_s << endl;

			for (int i = -INTERP_POINTS_BEHIND_EGO; i < INTERP_POINTS_AHEAD_EGO; i++){
				int way_pt_index = (next_waypoint_near_ego + i) % total_waypoints;

				// correction for last few points
				if (way_pt_index < 0) {
					way_pt_index += total_waypoints;
				}

				double s_index = map_waypoints_s[way_pt_index];
				double s_next_way_pt = map_waypoints_s[next_waypoint_near_ego];

				// make changes at the start and end of circuit loop
				if (i < 0 && s_index > s_next_way_pt){
					s_index -=  MAX_S;
				}
				if (i > 0 && s_index < s_next_way_pt){
					s_index +=  MAX_S;
				}

				map_waypoints_x_for_inter.push_back(map_waypoints_x[way_pt_index]);
				map_waypoints_y_for_inter.push_back(map_waypoints_y[way_pt_index]);
				map_waypoints_s_for_inter.push_back(s_index);
				map_waypoints_dx_for_inter.push_back(map_waypoints_dx[way_pt_index]);
				map_waypoints_dy_for_inter.push_back(map_waypoints_dy[way_pt_index]);
			}

			// Carry out interpolation
			cout << "car_s = " << car_s << "s_begin = " <<map_waypoints_s_for_inter[0] << "s_end = "<< map_waypoints_s_for_inter[map_waypoints_s_for_inter.size()-1] << endl;

			vector<double> map_waypoints_x_fine, map_waypoints_y_fine, map_waypoints_s_fine;
			vector<double> map_waypoints_dx_fine, map_waypoints_dy_fine;

			int interp_anchor_pts_size = map_waypoints_x_for_inter.size();
			int num_interp_pts = ceil((map_waypoints_s_for_inter[interp_anchor_pts_size - 1] - map_waypoints_s_for_inter[0]) / INTERP_DS);

		    tk::spline spline_x;
			tk::spline spline_y;
		    tk::spline spline_dx;
			tk::spline spline_dy;
			spline_x.set_points(map_waypoints_s_for_inter,map_waypoints_x_for_inter); // X spline
			spline_y.set_points(map_waypoints_s_for_inter,map_waypoints_y_for_inter); // Y spline
			spline_dx.set_points(map_waypoints_s_for_inter,map_waypoints_dx_for_inter); // dx spline
			spline_dy.set_points(map_waypoints_s_for_inter,map_waypoints_dy_for_inter); // dy spline

			// generate interpolate waypoints
			for(int i = 0; i < num_interp_pts; i++){
                double s_interp = map_waypoints_s_for_inter[0] + i * INTERP_DS;
				//cout << "car_s = " << car_s << "s_interp = " << s_interp << endl;
				map_waypoints_s_fine.push_back(s_interp);
				map_waypoints_x_fine.push_back(spline_x(s_interp));
				map_waypoints_y_fine.push_back(spline_y(s_interp));
			    map_waypoints_dx_fine.push_back(spline_dx(s_interp));
				map_waypoints_dy_fine.push_back(spline_dy(s_interp));
				//cout<<"s_interp = "<< s_interp << "x = " << spline_x(s_interp) << "y = " << spline_y(s_interp) << "dx = " << spline_dx(s_interp) << "dy = " << spline_dy(s_interp) << endl;
			}
            //cout << "s start = " << map_waypoints_s_fine[0] << "s end = " << map_waypoints_s_fine[map_waypoints_s_fine.size()-1] << "y start = " << map_waypoints_y_fine[0] << "y end = " << map_waypoints_x_fine[map_waypoints_s_fine.size()-1] << endl;
			//cout << "s_begin = " <<num_interp_pts << endl;

			bool too_close = false;

            //std::cout << "sensor size = " <<sensor_fusion.size()<< std::endl;
		    // get lane number of ego vehicle
			int lane_num_ego = track.get_lane_num_from_d(car_d);

			// initialize ego vehicle
			Vehicle ego_vehicle = Vehicle();
			ego_vehicle.s = car_s;
			ego_vehicle.d = car_d;
			ego_vehicle.v = car_speed;
			ego_vehicle.state = "KL";
			ego_vehicle.lane = lane_num_ego;

			//predictions of non-ego vehicles
			//////////////////////////////////////////////////////////// PREDICTIONS NON-EGO VEHICLES ////////////////////////////////////////////////
			map<int ,vector<Vehicle> > non_ego_predictions;
			vector<Vehicle> non_ego_vehicles;

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

				Vehicle non_ego = Vehicle(check_car_s, d_car, check_speed);
				non_ego_vehicles.push_back(non_ego);
                //cout << "t_pred = " << T_PRED << endl;
				vector<Vehicle> non_ego_pred = non_ego.generate_predictions(T_PRED);
			    /*
				if (i == 0){
                    cout << "speed" << check_speed << endl;
                    cout << "check_car_s = " << check_car_s << endl;
					cout << "pred_size = " << non_ego_pred.size() << endl;
                    cout << "pred_s1 = " << non_ego_pred[0].s<<endl;
					cout << "pred_s2 = " << non_ego_pred[1].s<<endl;
					cout << "pred_s3 = " << non_ego_pred[2].s<<endl;
					cout << "pred_s4 = " << non_ego_pred[3].s<<endl;
                }
                */

                non_ego_predictions[v_id] = non_ego_pred;
				//cout << "i = " << i << "check_car_s = " << check_car_s << "non-ego = " << non_ego.s << endl;
			}

			bool car_on_the_left = false;
			bool car_on_the_right = false;
			bool car_ahead = false;



			for (int i = 0; i<non_ego_vehicles.size(); i++){
				Vehicle non_ego_vehicle = non_ego_vehicles[i];
				double diff_s_ego_non_ego = fabs(ego_vehicle.s - non_ego_vehicle.s);
				double diff_d_ego_non_ego = fabs(ego_vehicle.d - non_ego_vehicle.d);
				int lane_num_non_ego = track.get_lane_num_from_d(non_ego_vehicle.d);
				//cout<<"i = "<<i<<"diff_s = "<<diff_s_ego_non_ego<<endl;
				if (diff_s_ego_non_ego < SAFETY_DT_S){
					if (ego_vehicle.lane == lane_num_non_ego){
						//cout << "WARNING: Car in the front, slow down" << endl;
						// NEED TO WORK IN LANES INSTEAD OF D
						car_ahead = true;
					}
					if (ego_vehicle.lane - 1 == lane_num_non_ego){
						//cout << "WARNING: Car to the left cannot turn left" << endl
						car_on_the_left = true;
					}
					if (ego_vehicle.lane + 1 == lane_num_non_ego){
						//cout << "WARNING: Car to the right cannot turn right" << endl;
						car_on_the_right = true;
					}

				}
			}

			// identify the possible successor states
			vector<string> ego_vehicle_available_next_states;
			ego_vehicle_available_next_states = ego_vehicle.successor_states(car_ahead, car_on_the_left, car_on_the_right);


			//Boundary conditions for generating JMT trajectory
			double si, si_dot, si_dot_dot;
			double di, di_dot, di_dot_dot;
			double xi_jmt, xi_min_1_jmt, yi_jmt, yi_min_1_jmt, yaw_jmt;

			int start_pt_JMT = min(prev_size, START_POINT_TRAJ);

			if (start_pt_JMT < 2){
				si = car_s;
				si_dot = car_speed;
				si_dot_dot = 0.0;
				di = car_d;
				di_dot = 0.0;
				di_dot_dot = 0.0;
			}
			// not selecting the end point helps smoother transition and no jerks
			else{
				xi_jmt = previous_path_x[start_pt_JMT-1];
				yi_jmt = previous_path_y[start_pt_JMT-1];

			    xi_min_1_jmt = previous_path_x[start_pt_JMT-2];
				yi_min_1_jmt = previous_path_y[start_pt_JMT-2];

				yaw_jmt = atan2(yi_jmt - yi_min_1_jmt, xi_jmt - xi_min_1_jmt);

				//vector<double> frenet_s_d_old =  getFrenet(xi_jmt, yi_jmt, yaw_jmt, map_waypoints_x, map_waypoints_y, map_waypoints_s);
				//vector<double> frenet_s_d_new =  getFrenet(xi_jmt, yi_jmt, yaw_jmt, map_waypoints_x_fine, map_waypoints_y_fine, map_waypoints_s_fine);

				//vector<double> xy_old = getXY(frenet_s_d_old[0], frenet_s_d_old[1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
				//vector<double> xy_new = getXY(frenet_s_d_new[0], frenet_s_d_new[1], map_waypoints_s_fine, map_waypoints_x_fine, map_waypoints_y_fine);


				//cout << "waypoint size = " << map_waypoints_x.size()<<endl;
				//cout <<"x = "<< xi_jmt<<"y = "<< yi_jmt <<"frenet cordinates_old1 = "<< frenet_s_d_old[0] << "frenet cordinates_old1 = " << frenet_s_d_old[1] << endl;
				//cout <<"x = "<< xi_jmt<<"y = "<< yi_jmt <<"frenet cordinates_new1 = "<< frenet_s_d_new[0] << "frenet cordinates_new1 = " << frenet_s_d_new[1] << endl;
				//cout <<"x = "<< xi_jmt<<"y = "<< yi_jmt <<"xy_old0 = "<< xy_old[0] << "xy_old0 = " << xy_old[1] << endl;
				//cout <<"x = "<< xi_jmt<<"y = "<< yi_jmt <<"xy_new0 = "<< xy_new[0] << "xy_new1 = " << xy_new[1] << endl;

				/*
                double ref_x_prev = previous_path_x[prev_size-2];
				double ref_y_prev = previous_path_y[prev_size-2];


				ptsx.push_back(ref_x_prev);
				ptsx.push_back(ref_x);

				ptsy.push_back(ref_y_prev);
				ptsy.push_back(ref_y);
                */
			}


			/*
            for (int i = 0; i<ego_vehicle_available_next_states.size(); i++){
				cout<<"Next possible state = " << ego_vehicle_available_next_states[i] << endl;
			}*/

			for (int i = 0; i < sensor_fusion.size(); i++){
				// check lane of car
				float d = sensor_fusion[i][6];
				if (d < (2+4*lane+2) && d > (2+4*lane-2)){
					double vx = sensor_fusion[i][3];
					double vy = sensor_fusion[i][4];
					double check_speed = sqrt(vx * vx + vy * vy);
					double check_car_s = sensor_fusion[i][5];

					// predict position of car based on current speed/
                    // position predeicted in frenet
                    check_car_s += ((double)prev_size * 0.02 * check_speed); // NOT SURE. EXPERIMENT WITH THIS
					// NEED TO PREDICT IN THE FUTURE FOR GRADUATE DECELERATION AND NOT IMMEDIATE BRAKING

					if ((check_car_s > car_s) && ((check_car_s - car_s)<SAFETY_DT_S)) {
						// reduce velocity if car is in the same lane
                        //ref_vel = 29.5;
                       /*
					   if (lane > 0){
                            //lane = 0;
							lane -= 1;
                        }
					   */
                        too_close = true;
					}
				}
			}

           // decelerate and follow car
            if(too_close){
                ref_vel -= 0.224;
            }
            else if(ref_vel < 49.5){
                ref_vel += 0.224;
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
				ptsy.push_back(car_y);
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
			vector<double> way_pt0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> way_pt1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> way_pt2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

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
