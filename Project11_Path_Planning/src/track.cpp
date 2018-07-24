#include <iostream>
#include "track.h"
#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include "input.h"


/**
 * Initializes Map
 */
Track::Track() {}

Track::~Track() {}

int Track::get_lane_num_from_d(double d){
	// gives lane number based on value of d
	int lane_num = floor(d/LANE_WIDTH);
	return lane_num;
}

// read map and interpolate for Frenet to XY transformation
void Track::read_map_and_interpolate(string map_file_){
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

	// Setup splines
    spline_x.set_points(map_waypoints_s,map_waypoints_x); // X spline
	spline_y.set_points(map_waypoints_s,map_waypoints_y); // Y spline
	spline_dx.set_points(map_waypoints_s,map_waypoints_dx); // dx spline
	spline_dy.set_points(map_waypoints_s,map_waypoints_dy); // dy spline

	// generate interpolate waypoints
	for(double s = 0; s < floor(MAX_S); s++){
		map_waypoints_s_fine.push_back(s);
		map_waypoints_x_fine.push_back(spline_x(s));
		map_waypoints_y_fine.push_back(spline_y(s));
	    map_waypoints_dx_fine.push_back(spline_dx(s));
		map_waypoints_dy_fine.push_back(spline_dy(s));
	}
	//cout << "s_start = " << map_waypoints_s_fine[0] << "s_end = " << map_waypoints_s_fine[map_waypoints_s_fine.size()-1] << endl;
	//cout << "s_start = " << map_waypoints_x_fine[0] << "s_end = " << map_waypoints_x_fine[map_waypoints_s_fine.size()-1] << endl;
	//cout << "x = " << map_waypoints_x_fine[3] << "y = " << map_waypoints_y_fine[3] << "s = " << map_waypoints_s_fine[3] << "dx = " << map_waypoints_dx_fine[3] << "dy = " << map_waypoints_dy_fine[3] << endl;
	//cout<<"s_interp = "<< s_interp << "x = " << spline_x(s_interp) << "y = " << spline_y(s_interp) << "dx = " << spline_dx(s_interp) << "dy = " << spline_dy(s_interp) << endl;
}

//get euclidean distance
double Track::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int Track::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

int Track::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
    angle = min(2*M_PI - angle, angle);

	if(angle > M_PI/4){
		closestWaypoint++;
		if (closestWaypoint == maps_x.size()){
			closestWaypoint = 0;
		}
	}

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Track::getFrenet(double x, double y, double theta){
	// use the new map waypoints for Frenet

	const vector<double> &maps_x = this->map_waypoints_x_fine;
	const vector<double> &maps_y = this->map_waypoints_y_fine;
	const vector<double> &maps_s = this->map_waypoints_s_fine;

	/*
	const vector<double> &maps_x = this->map_waypoints_x;
	const vector<double> &maps_y = this->map_waypoints_y;
	const vector<double> &maps_s = this->map_waypoints_s;
	*/

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
vector<double> Track::getXY(double s, double d){
	// use the new map waypoints for Frenet
	/*
	const vector<double> &maps_x = this->map_waypoints_x_fine;
	const vector<double> &maps_y = this->map_waypoints_y_fine;
	const vector<double> &maps_s = this->map_waypoints_s_fine;
	*/


	const vector<double> &maps_x = this->map_waypoints_x;
	const vector<double> &maps_y = this->map_waypoints_y;
	const vector<double> &maps_s = this->map_waypoints_s;


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

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

vector<double> Track::getXYFromSpline(double s, double d){
	// use the new map waypoints for Frenet
	s = fmod(s,MAX_S);

	double x = spline_x(s) + d * spline_dx(s);
	double y = spline_y(s) + d * spline_dy(s);

	return {x,y};
}

void Track::testInterpError(double car_x, double car_y, double car_yaw){
	//TEST Interpolation
	vector<double> frenet_s_d_new =  getFrenet(car_x, car_y, car_yaw);
	vector<double> xy_new = getXYFromSpline(frenet_s_d_new[0], frenet_s_d_new[1]);
	double error = distance(car_x, car_y, xy_new[0], xy_new[1]);
	cout << "error = " << error << endl;
}
