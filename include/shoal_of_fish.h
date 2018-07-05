// Node to create a moving "shoal of fish" or releasing the shoal of fish by GPS and moving by new GPS points
// Creating a circle that defines the shoal, and a pre set path for the shoal to follow

#ifndef SHOAL_OF_FISH_H
#define SHOAL_OF_FISH_H

#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include "ros/ros.h"
#include <iostream>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "following_algorithm/ShoalCoordinates.h"

using namespace std;

class ShoalOfFish
{

public: 
	ShoalOfFish(); // Constructor
	~ShoalOfFish(); // Deconstructor

	void distanceStart(); // Start a desired distance in front of the boat
	void pointOne(); // Publish waypoint one with predefined gps coordinates
	void pointTwo(); // Publish waypoint two
	void pointThree(); // Publish waypoint three

private: 
	ros::NodeHandle nh_;
	ros::Publisher activate_point_pub_;
	ros::Publisher point_pub_;

};

#endif 