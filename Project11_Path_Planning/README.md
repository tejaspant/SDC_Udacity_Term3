## Project: Path Planning 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)

[image1]: ./write_up_images/simulator.png "simulator"

## Overview
---
In this project, the goal is to design a path planner that is able to generate smooth and safe paths for an autonomous car to follow along a 3 lane highway with traffic
![alt text][image1]

## Dependencies
---
* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4
* [Eigen library](http://eigen.tuxfamily.org/index.php?title=Main_Page) 

## Instructions to Run
---
* Clone this repo.
* Make a build directory: mkdir build && cd build
* Compile: cmake .. && make
* Run Simulator: Clone [Term 3 Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) and run it
* Run Code: ./path_planning

## Approach
---
The initial approach considered was the use of finite state machine (FSM) with cost functions to determine the best possible next state of the car along with using the minimum jerk trajectory. However because of shortage of time, the spline approach for generating the trajectory as discussed the walkthrough has been implemented. Instead of using cost functions, the next state of the car is determined deterministically. The bulk of the code is implmented in the main.cpp.

Sensor Fusion
---
Line 108-148
Using the sensor fusion data, the path of the non-ego vehicles is predicted and it is determined whether there is a car in the current lane or the adjacent lanes

Behavior Planning
---
Line 149-172
Based on the sensor fusion data, in the behavior planning section one of the next possible states of the car among, keep in lane, lane change left or lane change right is determined. Further imporvement can be made here by using cost functions to determine which state is more feasible based on the cost of each state.

#Trajectory Generation
---
line 174-273
Once the next state of the autonomous car is determined in the behavior planning section, a spline trajectory is generated using the [spline.h](http://kluge.in-chemnitz.de/opensource/spline/) header file. This approach of generating a trajectory is more straightforward in comparison to using the minimum jerk trajectory approach which requires the initial and final position, velocity and acceleration in both of the Frenet co-ordinates. 

## Result
---
Here is the [link](https://www.youtube.com/watch?v=RKndu2MFpvU) to a video showing that the car is able to navigate atleast 4.32 miles without any incident

## Future Work
---
There are a number of ways in which the current the path planner can be included like:
* Using a FSM approach with cost functions to determine the best possible next state
* Use a multi-modal estimation algorithm for prediction of behavior of non-ego vehicles to make it more robust to even city driving conditions
* Use the minimum jerk trajectory to determine the path
