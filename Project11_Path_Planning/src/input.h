#ifndef INPUTS
#define INPUTS

// parameters of simulator
const double DT_SIMUL = 0.02; // time step size between two path points, sec

// for prediction of non-ego vehicles
const int HORIZON_PTS_NON_EGO = 50; // number of points in horizon for prediction of non-ego vehicles

// parameters for defining safety limits
const double SAFETY_DT_S = 25; // distance to be maintained ahead or behind a car in s coordinate
const double SAFETY_DT_D = 2; // distance to be maintained to left or right of a car in d coordinate
const double MAX_SPEED = 49.5; //maximum permissible speed of ego vehicle, m/s
const double MAX_ACC = 0.224; //maximum permissible acceleration for ego vehicle, m/s^2
const double MAX_JERK = 10; //maximum permissible jerk for ego vehicle, m/s^3

//parameters of the road and map
const int TOTAL_NUM_LANES = 3; // total number of lanes on the road
const double LANE_WIDTH = 4; // width of a single lane, m
const double MAX_S = 6945.554; // maximum frenet

//JMT trajectory parameters
const int START_POINT_TRAJ = 25; // point from end of previous trahectort that will be used as the starting point for JMT trajectory
const double T_JMT = 2; // duration into the future the prediction is made for generating JMT trajectory, sec

// For spline interporlation of map waypoints
const int INTERP_POINTS_BEHIND_EGO = 5; // number of points behind the ego car which are achor points for the spline
const int INTERP_POINTS_AHEAD_EGO = 5; // number of points ahead of the ego car which are achor points for the spline
const double INTERP_DS = 1.0; // change in ds for interpolation
#endif
