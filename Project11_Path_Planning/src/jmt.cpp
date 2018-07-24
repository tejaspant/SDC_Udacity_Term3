#include <iostream>
#include "jmt.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include "input.h"


/**
 * Initializes Map
 */
JMT::JMT(string state, JMT_traject_BC BC, map<int ,vector<Vehicle> > &predictions) {
	/*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
	JMT_traject trajectory;
	vector<Vehicle> trajectory;
    if (state.compare("KL") == 0) {
        cout << "Will use keep lane" << endl;
		//trajectory = keep_lane_trajectory(predictions);
	} 
	else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        //trajectory = lane_change_trajectory(state, predictions);
	}

    //return trajectory;
}




