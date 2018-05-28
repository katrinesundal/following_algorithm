#include "shoal_of_fish.h"
using namespace std; 

ShoalOfFish::ShoalOfFish()
{

	shoal_coordinates_sub_ = nh_.subscribe("following_algorithm/activate_by_coordinates", 0, &ShoalOfFish::startShoalByGPS, this); // Subscribing to message from starting the shoal of fishes by coordinates
	draw_shoal_pub_ = nh_.advertise<following_algorithm::guiObjectUpdate>("following_algorithm/position", 1000); // Publishing shoal of fish position to GUI for drawing

}

ShoalOfFish::~ShoalOfFish()
{

}

void ShoalOfFish::startShoalByGPS(const following_algorithm::ShoalCoordinates::ConstPtr& shoal_starting_point)
{
	shoal_gps_position_.latitude = shoal_starting_point->latitude;
	shoal_gps_position_.longitude = shoal_starting_point->longitude;
	shoal_of_fishes_detected_ = true;

	following_algorithm::guiObjectUpdate shoal;
	shoal.msgDescriptor ="position_update";
	shoal.objectDescriptor = "shoal";
	shoal.objectID = "shoal_1";
	shoal.size = 10.0;
	shoal.longitude = shoal_gps_position_.longitude;
	shoal.latitude = shoal_gps_position_.latitude;
	shoal.heading = 0.0;
	draw_shoal_pub_.publish(shoal);

	ROS_INFO("latitude = %f", shoal_gps_position_.latitude);
	ROS_INFO("longitude = %f", shoal_gps_position_.longitude);
}

void ShoalOfFish::setShoalParameter(const following_algorithm::SetShoalParameter::ConstPtr& shoal_parameter)
{
	if(shoal_parameter->name == "set_shoal_size_" || shoal_parameter->name == "set_shoal_size")
	{
		set_shoal_size_ = shoal_parameter->value;
	}
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