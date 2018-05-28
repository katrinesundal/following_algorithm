// Node to create a moving "shoal of fish" 
// Creating a circle that defines the shoal, and a pre set path for the shoal to follow

#ifndef SHOAL_OF_FISH_H
#define SHOAL_OF_FISH_H
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "math.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include "following_algorithm/guiObjectUpdate.h"
#include "following_algorithm/AnchorCoordinates.h"

class ShoalOfFish
{

public: 
	ShoalOfFish(); // Constructor
	~ShoalOfFish(); // Deconstructor

private: 
	ros::NodeHandle nh_;
	ros::Subscriber shoal_coordinates_sub_;
	// Activate and deactivate shoal?? 
	ros::Publisher draw_shoal_pub_;

	void startShoalByGPS(const following_algorithm::ShoalCoordinates::ConstPtr& shoal_starting_point); // Releases shoal of fish at a specific coordinates defined when the shoal is started, activates the following algorithm
	void setShoalParameter(const following_algorithm::setShoalParameter::ConstPtr& shoal_parameter);

};

#endif 