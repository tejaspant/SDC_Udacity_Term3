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

using namespace std;

	struct JMT_traject{
	vector<double> x_traj;
	vector<double> y_traj;
	vector<double> s_traj;
	vector<double> d_traj;
	};
	
	struct JMT_traject_BC{
	double si, si_dot, si_dot_dot;
	double sf, sf_dot, sf_dot_dot;
	double di, di_dot, di_dot_dot;
	double df, df_dot, df_dot_dot;
	};


class JMT{
	
	/**
  	* Constructor
  	*/
  	JMT(string state, JMT_traject_BC BC, map<int ,vector<Vehicle> > &predictions);

  	/**
  	* Destructor
  	*/
  	~JMT(){};

	//JMT_traject generate_trajectory_JMT();
};
