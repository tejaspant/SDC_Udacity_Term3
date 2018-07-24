## Project: Path Planning 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview
---
In this project, the goal is to design a path planner that is able to generate smooth and safe paths for an autonomous car to follow along a 3 lane highway with traffic

[//]: # (Image References)

[image1]: ./write_up_images/simulator.png "simulator"

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

## Result
---
Here is the [link](https://www.youtube.com/watch?v=RKndu2MFpvU) to a video showing that the car is able to navigate atleast 4.32 miles without any incident

## Future Work
---
There are a number of ways in which the current the path planner can be included like:
* Using a Finite State Machine approach with cost functions to determine the best possible next state
* Use a multi-modal estimation algorithm for prediction of behavior of non-ego vehicles to make it more robust to even city driving conditions
* Use the minimum jerk trajectory to determine the path