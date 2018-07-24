#include "vehicle.h"

Vehicle::Vehicle(){}

Vehicle::Vehicle(double s, double d, double v, int lane, string state) {

    this->s = s;
	this->d = d;
    this->v = v;
	this->lane = lane;
	this->state = state;
}

Vehicle::~Vehicle(){}

double Vehicle::position_at(double t) {
	return this->s + this->v*t;
}

// POSSIBLY INCORRECT NEED TO CHANGE THE DURATION.
// DURATION SHOULD BE POSSIBLY 1sec = 50 points * 0.02 sec
vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
    //assuming v remains constant for prediction purposes and there is no lane change
	vector<Vehicle> predictions;
    for(int i = 0; i < horizon; i++) {
      double next_s = position_at(i*DT_SIMUL);
	  /*
      cout << "dt_pred = " << dt_pred << endl;
	  cout <<"i = "<<i<<"s = "<<this->s<<"v = "<<this->v<<"next_s = "<<next_s<<endl;
	  */
      /*
      double next_v = 0;
      if (i < NSTEPS_PRED-1) {
        next_v = position_at(2*dt_pred) - next_s;
      }
      */
      //predictions.push_back(Vehicle(next_s, this->d, next_v));
      predictions.push_back(Vehicle(next_s, this->d, this->v, this->lane));
  	}
    return predictions;

}

vector<string> Vehicle::successor_states(bool car_ahead, bool car_on_the_left, bool car_on_the_right){
	vector<string> states;
    states.push_back("KL");
    if(car_ahead) {
		if(this->lane == 1 && !car_on_the_right){
			states.push_back("LCR");
		}
		else if (this->lane == TOTAL_NUM_LANES && !car_on_the_left){
			states.push_back("LCL");
		}
		else if (!car_on_the_left && !car_on_the_right) {
			states.push_back("LCL");
			states.push_back("LCR");
		}
    }
	else if (car_on_the_left) {
        if (this->lane != TOTAL_NUM_LANES && !car_on_the_right) {
            states.push_back("LCR");
        }
    }
	else if (car_on_the_right) {
        if (this->lane != 1 && !car_on_the_left) {
            states.push_back("LCL");
        }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}
/*
void Vehicle::generate_end_conditions_jmt(string state, map<int,vector<Vehicle>> non_ego_predictions){
	//Calculates sf, sf_dot, sf_dot_dot, df, df_dot_dot
	// Identify the targets based on the state

	//First set the quantities which are zero
	this->sf_dot_dot = 0.0;
	this->df_dot = 0.0;
	this->df_dot_dot = 0.0;

	//Df
    int target_lane;
	if(state.compare("KL") == 0){
		target_lane = this->lane;
		this->df = this->di;
	}

	if(state.compare("LCL") == 0){
		target_lane = this->lane - 1;
		this->df = this->di - LANE_WIDTH;
	}

	if(state.compare("LCR") == 0){
		target_lane = this->lane + 1;
		this->df = this->di + LANE_WIDTH;
	}

	//Sf and sf_dot
	double target_sf_dot = min((this->si_dot + 0.8 * MAX_ACC * T_JMT), MAX_SPEED);
	// target speed based on predictions
	double target_speed_based_on_predictions = get_kinematics(non_ego_predictions, target_lane);

	this->sf_dot = min(target_sf_dot,target_speed_based_on_predictions); // NOT SURE ABOUT THIS
	this->sf = this->si + this->sf_dot * T_JMT;
}
*/
/*
double Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int target_lane) {

    //float max_velocity_accel_limit = this->max_acceleration + this->v;
    //float new_position;
    float new_velocity;
    //float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, target_lane, vehicle_ahead)) {

        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
        } else {
            new_velocity = MAX_SPEED;
        }
    }
    return new_velocity;

*/
/*
bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    int min_s = SAFETY_DT_S;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}
*/
/*
bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}
*/
