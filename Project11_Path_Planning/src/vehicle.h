#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "input.h"

using namespace std;

class Vehicle {
public:

/*
  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;

  int s;

  float v;

  float a;

  float target_speed;

  int lanes_available;

  float max_acceleration;

  int goal_lane;

  int goal_s;

 */
  double s;
  double d;
  double v;
  string state;
  int lane;
  
  double si, si_dot, si_dot_dot;
  double sf, sf_dot, sf_dot_dot;
  double di, di_dot, di_dot_dot;
  double df, df_dot, df_dot_dot;
  
  //double target_speed;
  //double max_acceleration;
  //int target_lane;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(double s, double d, double v, int lane, string state="CS");

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<Vehicle> generate_predictions(int horizon);

  double position_at(double dt);

  vector<string> successor_states(bool car_ahead, bool car_on_the_left, bool car_on_the_right);
  
  /*void generate_end_conditions_jmt(string state, map<int,vector<Vehicle> > non_ego_predictions);
  
  double get_kinematics(map<int, vector<Vehicle>> non_ego_predictions, int lane);
  
  bool get_vehicle_ahead(map<int, vector<Vehicle>> non_ego_predictions, int lane, Vehicle & rVehicle);

  bool get_vehicle_behind(map<int, vector<Vehicle>> non_ego_predictions, int lane, Vehicle & rVehicle);
*/
   
};  

#endif
