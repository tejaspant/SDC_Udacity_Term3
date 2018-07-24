#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"
#include "spline.h"

using namespace std;

class Track {
public:
    // read from map
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;
	
	// after interpolation
	vector<double> map_waypoints_x_fine;
	vector<double> map_waypoints_y_fine;
	vector<double> map_waypoints_s_fine;
	vector<double> map_waypoints_dx_fine;
	vector<double> map_waypoints_dy_fine;
	
	tk::spline spline_x;
	tk::spline spline_y;
	tk::spline spline_dx;
	tk::spline spline_dy;

    /**
  	* Constructor
  	*/
  	Track();

  	/**
  	* Destructor
  	*/
  	virtual ~Track();
	
	void read_map_and_interpolate(string map_name);
	double distance(double x1, double y1, double x2, double y2);
	int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
	int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
	vector<double> getFrenet(double x, double y, double theta);
	vector<double> getXY(double s, double d);
	vector<double> getXYFromSpline(double s, double d);
	void testInterpError(double car_x, double car_y, double car_yaw);
	int get_lane_num_from_d(double d);

};
