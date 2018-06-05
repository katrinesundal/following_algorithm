#include "shoal_of_fish.h"
using namespace std; 

ShoalOfFish::ShoalOfFish()
{

	activate_coordinates_sub_ = nh_.subscribe("following_algorithm/activate_by_coordinates", 0, &ShoalOfFish::startShoalByGPS, this); // Subscribing to message from starting the shoal of fishes by coordinates
	draw_shoal_pub_ = nh_.advertise<following_algorithm::guiObjectUpdate>("following_algorithm/position", 1000); // Publishing shoal of fish position to GUI for drawing
	deactivate_sub_ = nh_.subscribe("following_algorithm/deactivate", 0, &ShoalOfFish::removeShoal, this);// Subscribing to message from removing the shoal of fish
	set_param_sub_ = nh_.subscribe("following_algorithm/set_anchor_parameter", 0, &ShoalOfFish::setShoalParameter, this);
}

ShoalOfFish::~ShoalOfFish()
{

}



int main(int argc, char* argv[])
{
	ros::init(argc, argv, "shoal_of_fish");
	ros::start();
	ROS_INFO("Started node shoal_of_fish.");

	ShoalOfFish shoal_of_fish;
	
	ros::spin();

	ROS_INFO("exit node shoal_of_fish");
	ros::shutdown();
	return 0;
}