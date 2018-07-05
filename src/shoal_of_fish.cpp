#include "shoal_of_fish.h"
using namespace std; 

ShoalOfFish::ShoalOfFish()
{

	activate_point_pub_ = nh_.advertise<std_msgs::Float64>("following_algorithm/activate_by_distance",0); //!< Publishing message to activate the virtual anchor by a distance from the boat to the anchor
	point_pub_ = nh_.advertise<following_algorithm::ShoalCoordinates>("following_algorithm/activate_by_coordinates", 0);//!< Publishing message to activate the virtual anchor by desired coordinates

}

ShoalOfFish::~ShoalOfFish()
{

}

void ShoalOfFish::distanceStart(){
	std_msgs::Float64 activate_point;
	activate_point_pub_.publish(activate_point);
	ROS_INFO("lenght = yaml	");
}

void ShoalOfFish::pointOne(){
	following_algorithm::ShoalCoordinates coord;

	coord.latitude = 59.43387;
	coord.longitude = 10.47543;
	point_pub_.publish(coord);

}

void ShoalOfFish::pointTwo(){
	following_algorithm::ShoalCoordinates coord;

	coord.latitude = 59.43583;
	coord.longitude = 10.47793;
	point_pub_.publish(coord);
	ROS_INFO("Point Two");
}

void ShoalOfFish::pointThree(){
	following_algorithm::ShoalCoordinates coord;

	coord.latitude = 59.43438;
	coord.longitude = 10.48241;
	point_pub_.publish(coord);

	ROS_INFO("Point Three");
}

int main(int argc, char* argv[])
{
	int it = 0;
	ros::init(argc, argv, "shoal_of_fish");
	ros::start();
	ROS_INFO("shoal_of_fish started");
 	
 	ShoalOfFish shoal_of_fish;
	
	ros::Duration(4).sleep();
	shoal_of_fish.distanceStart();
	ros::spinOnce();
	ros::Rate r(1);
	while(ros::ok())
	{
		it ++;
		if(it == 180){
		shoal_of_fish.pointOne();
		}if(it ==360){
		shoal_of_fish.pointTwo();
		}if(it ==540){
		shoal_of_fish.pointThree();
		it = 0;
		}
		ros::spinOnce();
		r.sleep();

	}
	ros::shutdown();
	return 0;
}