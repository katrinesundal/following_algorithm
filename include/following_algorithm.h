#ifndef Following_algorithm_H
#define Following_algorithm_H

#include "ros/ros.h"
#include "math.h"
#include "Eigen/Dense"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float64MultiArray.h"
#include "following_algorithm/get_max_surge_force.h"


class FollowingAlgorithm
{
public: 
	FollowingAlgorithm(); // Constructor
	~FollowingAlgorithm(); // Deconstructor

private: 
	ros::NodeHandle nh_; 
	
}